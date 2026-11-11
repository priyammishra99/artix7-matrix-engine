// ============================================================================
// File: bd_shell.sv
// Block: AC701 board shell (matrix engine only) - me_top with its board-side
//        host control, memory and check, on one clock
//
//   bd_ctrl ──csr_*, soft reset──> me_top <──ld_*, run_*/w_*── bd_mem_side
//      ^  case table, script ROM      │ rq_*/wt_*/wq_* ──────────> │ ROMs: window,
//      └── results <── checker ◄── dp_* (dump) ──────────────────► │ weights, reference
//
// No block of the matrix engine is changed: me_top is instantiated as it is
// (USE_PRIM = 1; the array's switches come from the build's defines, as for
// the me_top project). The quantiser's port is tied off (stage 2).
// The dump RAM (DUMP_RAM = 1) keeps the checked output beats of the last run
// for a look through VIO (and a UART later); dump_raddr reads it (latency 2).
// dbg: the handshakes and the checker's lanes, for the ILA.
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module bd_shell #(
  parameter string WT_INIT     = "bd_wt.mem",
  parameter string WIN_INIT    = "bd_win.mem",
  parameter string REF_INIT    = "bd_ref.mem",
  parameter string CASE_INIT   = "bd_case.mem",
  parameter string SCRIPT_INIT = "bd_script.mem",
  parameter int    WT_DEPTH    = 10404,
  parameter int    WIN_DEPTH   = 962,
  parameter int    REF_DEPTH   = 1908,
  parameter int    TIMEOUT     = 1 << 20,
  parameter bit    DUMP_RAM    = 1'b1
) (
  input  wire logic          clk,
  input  wire logic          rst_n,          // synchronous, active low
  input  wire logic          start,          // pulse
  input  wire logic [2:0]    sel,
  input  wire logic          rep,
  input  wire logic          gm,             // with start: GEMM-only mode
  input  wire logic [9:0]    dump_raddr,
  output logic [511:0]       dump_rdata,

  output logic               running, done_any, last_pass, fail_any,
  output logic [2:0]         cur_sel,
  output logic               cur_rep, cur_gm,
  output logic [31:0]        gemm_count, loop_cycles,
  output logic [7:0]         fail_code, fail_pc,
  output logic [31:0]        run_count, pass_count, fail_count,
  output logic [31:0]        gemm_cycles, run_cycles, sat_count, arr_beats,
  output logic [31:0]        chk_checked, chk_mism,
  output logic [15:0]        chk_first_k,
  output logic [4:0]         chk_first_lane,
  output logic [15:0]        chk_first_got, chk_first_exp,
  output logic [5:0]         me_err,
  output logic [5:0]         ms_err,
  output logic [127:0]       dbg
);

  // ---- nets ----
  logic         core_rst_n, run_clr;
  logic         csr_we;  logic [11:0] csr_waddr, csr_raddr;  logic [31:0] csr_wdata, csr_rdata;
  logic         run_valid, run_ready;  logic [5:0] run_layer;  logic [4:0] run_tag;  logic [1:0] run_kind;
  logic [15:0]  run_n, run_k;  logic [23:0] run_beats;
  logic         w_valid, w_ready;    logic [511:0] w_data;
  logic         ld_valid, ld_ready;  logic [511:0] ld_data;
  logic         dp_valid, dp_ready;  logic [511:0] dp_data;
  logic         rq_valid, rq_ready;  logic [31:0] rq_addr;  logic [15:0] rq_beats;
  logic         wq_valid, wq_ready;  logic [31:0] wq_addr;  logic [15:0] wq_beats;  logic wq_done;
  logic         wt_valid, wt_ready;  logic [5:0] wt_layer;  logic [4:0] wt_tag;
  logic         me_busy;
  logic [15:0]  c_win_base, c_win_beats, c_wt_base, c_wt_beats, c_run_n, c_run_k, c_ref_base, c_ref_beats, c_dump_beats;
  logic [5:0]   c_layer;  logic [4:0] c_tag;  logic [6:0] c_rel_lo, c_rel_n;
  logic [2:0]   ms_seen;  logic multi_wt;  logic [31:0] ms_wt_count;
  logic         chk_idle;  logic [15:0] chk_rx;
  logic         dr_we;  logic [9:0] dr_waddr;  logic [511:0] dr_wdata;
  logic [31:0]  ref_streamed;
  logic [3:0]   ctrl_state;

  // ---- the matrix engine, as it is ----
  me_top #(.USE_PRIM(1'b1)) u_me (
    .clk, .rst_n(core_rst_n),
    .csr_we, .csr_waddr, .csr_wdata, .csr_raddr, .csr_rdata,
    .run_valid, .run_ready, .run_layer, .run_tag, .run_kind, .run_n, .run_k, .run_beats,
    .w_valid, .w_ready, .w_data, .ld_valid, .ld_ready, .ld_data, .dp_valid, .dp_ready, .dp_data,
    .rq_valid, .rq_ready, .rq_addr, .rq_beats, .wq_valid, .wq_ready, .wq_addr, .wq_beats, .wq_done,
    .wt_valid, .wt_ready, .wt_layer, .wt_tag,
    .fq_start(1'b0), .fq_cls('0), .fq_wr(1'b0), .fq_row('0), .fq_chunk('0), .fq_data('0),
    .fq_rc_wr(1'b0), .fq_rc_row('0), .fq_absmax('0), .fq_shift('0), .fq_done(1'b0),
    .busy(me_busy), .err(me_err));

  // ---- the host control ----
  bd_ctrl #(.CASE_INIT(CASE_INIT), .SCRIPT_INIT(SCRIPT_INIT), .TIMEOUT(TIMEOUT)) u_ctrl (
    .clk, .rst_n, .start, .sel, .rep, .gm,
    .core_rst_n, .csr_we, .csr_waddr, .csr_wdata, .csr_raddr, .csr_rdata, .me_err,
    .run_clr, .c_win_base, .c_win_beats, .c_wt_base, .c_wt_beats, .c_run_n, .c_run_k, .c_layer, .c_tag,
    .c_ref_base, .c_ref_beats, .c_dump_beats, .c_rel_lo, .c_rel_n,
    .multi_wt, .ms_wt_count, .ms_seen, .ms_err, .chk_idle, .chk_rx, .chk_checked, .chk_mism,
    .running, .done_any, .last_pass, .fail_any, .cur_sel, .cur_rep, .cur_gm, .gemm_count, .loop_cycles,
    .fail_code, .fail_pc,
    .run_count, .pass_count, .fail_count, .gemm_cycles, .run_cycles, .sat_count, .arr_beats,
    .state_o(ctrl_state));

  // ---- the memory side and the check ----
  bd_mem_side #(.WT_INIT(WT_INIT), .WIN_INIT(WIN_INIT), .REF_INIT(REF_INIT),
                .WT_DEPTH(WT_DEPTH), .WIN_DEPTH(WIN_DEPTH), .REF_DEPTH(REF_DEPTH)) u_mem (
    .clk, .rst_n, .clr(run_clr), .multi_wt,
    .c_win_base, .c_win_beats, .c_wt_base, .c_wt_beats, .c_run_n, .c_run_k, .c_layer, .c_tag,
    .c_ref_base, .c_ref_beats, .c_dump_beats, .c_rel_lo, .c_rel_n,
    .rq_valid, .rq_ready, .rq_beats, .wt_valid, .wt_ready, .wt_layer, .wt_tag,
    .wq_valid, .wq_ready, .wq_beats, .wq_done,
    .ld_valid, .ld_ready, .ld_data,
    .run_valid, .run_ready, .run_layer, .run_tag, .run_kind, .run_n, .run_k, .run_beats,
    .w_valid, .w_ready, .w_data, .dp_valid, .dp_ready, .dp_data,
    .dr_we, .dr_waddr, .dr_wdata,
    .chk_rx, .chk_idle, .chk_checked, .chk_mism, .chk_first_k, .chk_first_lane, .chk_first_got, .chk_first_exp,
    .seen(ms_seen), .err(ms_err), .wt_count(ms_wt_count), .ref_beats_out(ref_streamed));

  // ---- the dump RAM ----
  if (DUMP_RAM) begin : g_dump
    (* ram_style = "block" *) logic [511:0] dmem [512];
    logic [511:0] q0;
    always_ff @(posedge clk) begin
      if (dr_we) dmem[dr_waddr[8:0]] <= dr_wdata;
      q0 <= dmem[dump_raddr[8:0]];
      dump_rdata <= q0;
    end
  end else begin : g_nodump
    assign dump_rdata = '0;
  end

  // ---- for the ILA ----
  assign dbg = {ctrl_state, ms_seen, ms_err, me_err, me_busy, core_rst_n,                  // 127:107
                rq_valid, rq_ready, wt_valid, wt_ready, wq_valid, wq_ready, wq_done,         // 106:100
                run_valid, run_ready, w_valid, w_ready, ld_valid, ld_ready, dp_valid, dp_ready, // 99:92
                csr_we, csr_waddr, csr_wdata[15:0],                                          // 91:63
                chk_rx, chk_mism[15:0],                                                      // 62:31
                dp_data[30:0]};                                                              // 30:0

endmodule

`default_nettype wire
