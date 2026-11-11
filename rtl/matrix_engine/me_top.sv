// ============================================================================
// File: me_top.sv
// Block: Matrix engine - top level (Stage 1): the six blocks wired together
//
// Wiring only: no logic of its own beyond the handshake taps the row store
// watches. Inside:
//
//   weight ring ──ent, ring reads, frees──> tile walker ──beats──> array
//   row store   ──window reads──────────────^      │                 │
//        ^  ^                                      scales, tags      tiles
//        │  └── constants, staging, residual, K/V ── drain <─────────┘
//        └── loader, dump ── Stage-1 command path ── command (walker + drain)
//
//   me_weight_ring    holds the weight stream: 1,024 slots
//   me_tile_walker    runs a GEMM command: ring + window -> array beats,
//                     column scales and tile tags to the drain
//   me_array          4 x 8 x 16 DSP48E1
//   me_drain          tiles -> scaled outputs into staging / the residual
//   me_row_store      window, staging, residual, own K/V; window loader, dump
//   me_s1_cmd         debug registers: window load, GEMM, dump, step context,
//                     class table
//
// Outside ports (the memory side and the control side):
//   - weight stream into the ring: run descriptors and weight beats from the
//     memory side;
//   - window-loader stream (from memory) and dump stream (to memory);
//   - the memory requests from the command path (rq_* read, wq_* write,
//     wt_* weights);
//   - the quantiser's window-fill port (fq_*, stage 2; tie off before);
//   - the debug registers (csr_*, from the CSR block) and the status.
// One clock (the core clock) and its soft reset; any clock crossing to the
// memory side is outside.
//
// The row store's staging and residual read data reach the drain through one
// register stage here, and the drain runs with RD_LAT = 3: alone, each block
// meets 125 MHz with 2 ns for the other side, but together the store's 36-bank
// read mux and the drain's ACC add and saturation took 8.3 ns in one cycle
// (whole-engine run, 2026-10-22: -0.358 ns). The drain issues its ACC reads
// one cycle earlier, so a tile still takes 16 cycles (verified at RD_LAT 3).
//
// Not connected in Stage 1 (later stages): the array's bypass port and push
// mode (attention), the own K/V read port (attention), the drain's K/V
// output (driven 0 until stage 2).
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module me_top
  import me_if_pkg::*;
#(
  parameter bit USE_PRIM = 1'b1      // ring and row store on RAMB primitives (0: behavioural, faster sims)
) (
  input  wire logic              clk,
  input  wire logic              rst_n,          // the core's soft reset, synchronous, active low

  // ---- debug registers (the CSR block) ----
  input  wire logic              csr_we,
  input  wire logic [11:0]       csr_waddr,
  input  wire logic [31:0]       csr_wdata,
  input  wire logic [11:0]       csr_raddr,
  output logic [31:0]            csr_rdata,

    // ---- weight stream into the ring (run descriptors and beats from the memory side) ----
  input  wire logic              run_valid,
  output logic                   run_ready,
  input  wire logic [5:0]        run_layer,
  input  wire logic [4:0]        run_tag,
  input  wire logic [1:0]        run_kind,
  input  wire logic [15:0]       run_n,
  input  wire logic [15:0]       run_k,
  input  wire logic [23:0]       run_beats,
  input  wire logic              w_valid,
  output logic                   w_ready,
  input  wire logic [511:0]      w_data,

  // ---- window-loader stream (from memory) ----
  input  wire logic              ld_valid,
  output logic                   ld_ready,
  input  wire logic [511:0]      ld_data,

  // ---- dump stream (to memory) ----
  output logic                   dp_valid,
  input  wire logic              dp_ready,
  output logic [511:0]           dp_data,

  // ---- the memory requests ----
  output logic                   rq_valid,
  input  wire logic              rq_ready,
  output logic [31:0]            rq_addr,
  output logic [15:0]            rq_beats,
  output logic                   wq_valid,
  input  wire logic              wq_ready,
  output logic [31:0]            wq_addr,
  output logic [15:0]            wq_beats,
  input  wire logic              wq_done,
  output logic                   wt_valid,
  input  wire logic              wt_ready,
  output logic [5:0]             wt_layer,
  output logic [4:0]             wt_tag,

  // ---- the quantiser's window fill (stage 2; tie off before) ----
  input  wire logic              fq_start,
  input  wire logic [3:0]        fq_cls,
  input  wire logic              fq_wr,
  input  wire logic [4:0]        fq_row,
  input  wire logic [6:0]        fq_chunk,
  input  wire logic [127:0]      fq_data,
  input  wire logic              fq_rc_wr,
  input  wire logic [4:0]        fq_rc_row,
  input  wire logic [14:0]       fq_absmax,
  input  wire logic [4:0]        fq_shift,
  input  wire logic              fq_done,

  // ---- status ----
  output logic                   busy,           // the command path runs an operation
  output logic [5:0]             err             // sticky: array overrun, array bypass, ring free,
                                                 // store fill, store port, drain fault
);

  // ==========================================================================
  // nets between the blocks
  // ==========================================================================
  // step context and class table (command path)
  logic [5:0]   n_layers;   logic [4:0] step_rows;   logic [3:0] step_logit_rows;
  logic [39:0]  logit_map;  logic [127:0] cls_e, cls_w;  logic cnt_clr;
  // command path <-> walker
  logic         cmd_valid, cmd_ready, cmd_done, cmd_fault;
  logic [63:0]  cmd_word;   logic [5:0] cmd_id;   logic [7:0] cmd_fault_code;
  logic         w_busy;     logic [31:0] w_perf_beats, w_perf_scales, w_bubbles, w_stall_credit;
  // command path <-> drain
  logic         dc_valid;   logic [5:0] dc_id;    logic [63:0] dc_word;
  logic         cmd_fin;    logic [5:0] cmd_fin_id;
  logic         d_fault, d_fault_pulse, d_busy;  logic [7:0] d_fault_code;  logic [5:0] d_fault_cmd;
  logic [31:0]  d_sat_cnt, d_perf_tiles, d_perf_outs;
  // command path <-> row store
  logic         ld_start;   logic [3:0] ld_cls;   logic [4:0] ld_nrows;   logic [6:0] ld_kch;
  logic         fill_ready, ld_busy, win_valid;   logic [3:0] win_cls;
  logic         dp_start, dp_res, dp_busy;  logic [15:0] dp_first, dp_beats;
  // ring <-> walker
  logic         ent_valid, ent_ready;  logic [5:0] ent_layer;  logic [4:0] ent_tag;  logic [1:0] ent_kind;
  logic [15:0]  ent_n, ent_k;
  logic [10:0]  ring_wptr;  logic ring_rd;  logic [9:0] ring_raddr;  logic [1023:0] ring_rdata;
  logic         ring_free;  logic [6:0] ring_free_n;
  logic         r_err_free; logic [31:0] r_perf_slots, r_perf_full;
  // row store <-> walker
  logic         win_rd;     logic [9:0] win_raddr;  logic [511:0] win_rdata;
  // walker -> drain
  logic         scale_valid;  logic [10:0] scale_grp;  logic [511:0] scale_data;
  logic         tile_tag_valid;  mei_tag_t tile_tag;
  // walker -> array
  logic         op_valid, op_first, op_last, op_bank;
  mei_w_bus_t   op_w;  mei_dp_bus_t op_dp;  mei_x_bus_t op_x;
  // array <-> drain
  logic         tile_valid, tile_ready;  mei_tile_bus_t tile_acc;
  logic         byp_valid;  mei_byp_bus_t byp_sum;  mei_btag_t byp_tag;
  logic         a_err_overrun, a_err_bypass;  logic [31:0] a_perf_beats, a_perf_tiles, a_perf_byp;
  // drain <-> row store
  logic [4:0]   drn_row;    logic [14:0] drn_absmax;  logic [4:0] drn_shift;
  logic         stg_we, stg_rd;  logic [15:0] stg_waddr, stg_raddr;  logic [1:0] stg_whe;
  logic [31:0]  stg_wdata, stg_rdata, stg_rdata_q;   // _q: through the register stage to the drain
  logic         res_we, res_rd;  logic [12:0] res_waddr, res_raddr;  logic [1:0] res_wee;
  logic [63:0]  res_wdata, res_rdata, res_rdata_q;
  logic         kvo_we;     logic [10:0] kvo_waddr;  logic [7:0] kvo_wbe;  logic [63:0] kvo_wdata, kvo_rdata;
  logic         s_err_fill, s_err_port;

  // ==========================================================================
  // the Stage-1 command path
  // ==========================================================================
  me_s1_cmd u_cmd (
    .clk, .rst_n,
    .csr_we, .csr_waddr, .csr_wdata, .csr_raddr, .csr_rdata, .busy,
    .n_layers, .step_rows, .step_logit_rows, .logit_map, .cls_e, .cls_w, .cnt_clr,
    .cmd_valid, .cmd_ready, .cmd_word, .cmd_id, .cmd_done, .cmd_fault, .cmd_fault_code,
    .perf_beats(w_perf_beats),
    .dc_valid, .dc_id, .dc_word, .cmd_fin, .cmd_fin_id,
    .drn_fault(d_fault), .drn_fault_code(d_fault_code), .drn_fault_cmd(d_fault_cmd), .drn_sat_cnt(d_sat_cnt),
    .ld_start, .ld_cls, .ld_nrows, .ld_kch, .fill_ready, .ld_busy, .win_valid,
    .dp_start, .dp_res, .dp_first, .dp_beats, .dp_busy,
    .rq_valid, .rq_ready, .rq_addr, .rq_beats,
    .wq_valid, .wq_ready, .wq_addr, .wq_beats, .wq_done,
    .wt_valid, .wt_ready, .wt_layer, .wt_tag);

  // ==========================================================================
  // the weight ring
  // ==========================================================================
  me_weight_ring #(.USE_PRIM(USE_PRIM)) u_ring (
    .clk, .rst_n,
    .run_valid, .run_ready, .run_layer, .run_tag, .run_kind, .run_n, .run_k, .run_beats,
    .in_valid(w_valid), .in_ready(w_ready), .in_data(w_data),
    .ent_valid, .ent_ready, .ent_layer, .ent_tag, .ent_kind, .ent_n, .ent_k,
    .ring_wptr, .ring_rd, .ring_raddr, .ring_rdata, .ring_free, .ring_free_n,
    .err_free(r_err_free), .perf_slots(r_perf_slots), .perf_full(r_perf_full));

  // ==========================================================================
  // the tile walker (the ring and the window both answer 2 cycles after a read)
  // ==========================================================================
  me_tile_walker #(.W_LAT(2), .X_LAT(2)) u_walker (
    .clk, .rst_n,
    .n_layers, .step_rows, .step_logit_rows,
    .cmd_valid, .cmd_ready, .cmd_word, .cmd_id, .cmd_done, .cmd_fault, .cmd_fault_code, .busy(w_busy),
    .ent_valid, .ent_ready, .ent_layer, .ent_tag, .ent_kind, .ent_n, .ent_k,
    .ring_wptr, .ring_rd, .ring_raddr, .ring_rdata, .ring_free, .ring_free_n,
    .win_valid, .win_cls, .win_rd, .win_raddr, .win_rdata,
    .scale_valid, .scale_grp, .scale_data,
    .op_valid, .op_first, .op_last, .op_bank, .op_w, .op_dp, .op_x,
    .tile_valid, .tile_ready, .tile_tag_valid, .tile_tag,
    .cnt_clr, .perf_beats(w_perf_beats), .perf_scales(w_perf_scales), .bubbles(w_bubbles),
    .perf_stall_credit(w_stall_credit));

  // ==========================================================================
  // the array (GEMM only in Stage 1: no bypass beats)
  // ==========================================================================
  me_array u_array (
    .clk, .rst_n,
    .op_valid, .op_first, .op_last, .op_bank, .op_bypass(1'b0), .op_btag('0), .op_w, .op_dp, .op_x,
    .tile_valid, .tile_ready, .tile_acc,
    .byp_valid, .byp_sum, .byp_tag,
    .err_overrun(a_err_overrun), .err_bypass(a_err_bypass),
    .perf_beats(a_perf_beats), .perf_tiles(a_perf_tiles), .perf_byp(a_perf_byp));

  // ==========================================================================
  // the drain
  // ==========================================================================
  me_drain #(.RD_LAT(3)) u_drain (                     // the store's 2 cycles + the register stage
    .clk, .rst_n, .logit_map, .cnt_clr, .cls_e, .cls_w,
    .dc_valid, .dc_id, .dc_word,
    .scale_valid, .scale_grp, .scale_data,
    .tile_valid, .tile_ready, .tile_acc, .tile_tag_valid, .tile_tag,
    .drn_row, .drn_absmax, .drn_shift,
    .stg_we, .stg_waddr, .stg_whe, .stg_wdata, .stg_rd, .stg_raddr, .stg_rdata(stg_rdata_q),
    .res_we, .res_waddr, .res_wee, .res_wdata, .res_rd, .res_raddr, .res_rdata(res_rdata_q),
    .kvo_we, .kvo_waddr, .kvo_wbe, .kvo_wdata,
    .cmd_fin, .cmd_fin_id, .fault(d_fault), .fault_code(d_fault_code), .fault_cmd(d_fault_cmd),
    .fault_pulse(d_fault_pulse), .busy(d_busy),
    .sat_cnt(d_sat_cnt), .perf_tiles(d_perf_tiles), .perf_outs(d_perf_outs));

  // the register stage on the store's read data to the drain (no reset: data only)
  always_ff @(posedge clk) begin
    stg_rdata_q <= stg_rdata;
    res_rdata_q <= res_rdata;
  end

  // ==========================================================================
  // the row store (it watches the walker's entry pop and the tile handshake:
  // signals the walker and the array already drive)
  // ==========================================================================
  me_row_store #(.USE_PRIM(USE_PRIM)) u_store (
    .clk, .rst_n,
    .win_rd, .win_raddr, .win_rdata, .win_valid, .win_cls,
    .fq_start, .fq_cls, .fill_ready, .fq_wr, .fq_row, .fq_chunk, .fq_data,
    .fq_rc_wr, .fq_rc_row, .fq_absmax, .fq_shift, .fq_done,
    .walker_busy(w_busy), .ent_pop(ent_valid && ent_ready), .tile_hs(tile_valid && tile_ready),
    .tile_last(tile_tag.last),
    .drn_row, .drn_absmax, .drn_shift,
    .stg_we, .stg_waddr, .stg_whe, .stg_wdata, .stg_rd, .stg_raddr, .stg_rdata,
    .res_we, .res_waddr, .res_wee, .res_wdata, .res_rd, .res_raddr, .res_rdata,
    .kvo_we, .kvo_waddr, .kvo_wbe, .kvo_wdata, .kvo_rd(1'b0), .kvo_raddr('0), .kvo_rdata,
    .ld_start, .ld_cls, .ld_nrows, .ld_kch, .ld_valid, .ld_ready, .ld_data, .ld_busy,
    .dp_start, .dp_res, .dp_first, .dp_beats, .dp_valid, .dp_ready, .dp_data, .dp_busy,
    .err_fill(s_err_fill), .err_port(s_err_port));

  assign err = {d_fault, s_err_port, s_err_fill, r_err_free, a_err_bypass, a_err_overrun};

endmodule

`default_nettype wire
