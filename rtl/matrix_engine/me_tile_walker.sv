// ============================================================================
// File: me_tile_walker.sv
// Block: Matrix engine - tile walker (runs OP_GEMM commands on me_array)
//
// Takes one GEMM command word (me_cmd_pkg::cmd_t) and walks
// its weight tensor out of the weight ring into the array, in the order the
// weight stream holds it:
//
//   for g  in 0 .. N/32 - 1          32-channel group
//     scale slot                      the group's 64-B beat of 32 FP16 column
//                                     scales: one cycle, sent to the drain
//     for ct in 0 .. 3                8-channel tile (array columns = channels
//                                     32g + 8ct .. +7; tag grp = 4g + ct)
//       for rg in 0 .. nrg-1          row group of 4 window rows (replay)
//         for c in 0 .. K/16 - 1      input chunk: one beat
//           ring slot   base + c      the same K/16 slots for every row group
//           window      rg*96 + c     one address for all 4 lanes
//       free K/16 slots               after the tile's last row group
//
// Rows: the command's row set picks the rows (ROWSET_ALL: the step's rows;
// ROWSET_LOGIT: the rows that want logits). The window holds a command's rows
// compactly, used row i in lane i mod 4 at entry (i div 4)*96 (the window
// fill writes them so); a tile's tag says which (rgrp, rvalid), and the
// drain maps window rows back to step rows.
//
// Checks, in the reference model's order, each
// ending the command with a fault code (me_cmd_pkg::FAULT_*) instead of
// running it:
//   1. OP is not GEMM                                   FAULT_CMD_UNKNOWN
//   2. a reserved bit (cmd_t.rsv) is set                FAULT_CMD_FIELD
//   3. LAYER >= the model's layer count                 FAULT_CMD_FIELD
//   4. ROWSET is not ALL or LOGIT                       FAULT_CMD_FIELD
//   5. the row set is empty                             done, no fault, nothing read
//   6. the tensor the ring delivers next (entry port) is not this command's:
//      another tag, another layer (tags 0-15), not a GEMM entry, or a shape
//      the array cannot run (N not a multiple of 32, K not a multiple of 16,
//      K > 1,536, N > 49,152)                           FAULT_TAG
//   7. A is not RES/STG16/STG32/WIN, or A = WIN while the window does not hold
//      a fill at ICLASS                                 FAULT_CMD_FIELD
//   8. D is not RES/STG16/STG32 (unless KV or TOPK)     FAULT_CMD_FIELD
// A faulted command reads and frees nothing; the step's soft reset clears the
// ring and the walker.
//
// One beat per cycle when the ring holds the slot and, for a tile's first
// beat, a credit is free (contract rule P4: at most 2 tiles in flight). A
// scale slot takes one cycle and no credit. Beats leave in the contract's
// widths (me_if_pkg): INT8 weights and activations sign-extended, D = 0.
//
// Scale guarantee to the drain: group g's scales leave before the first beat
// of group g; when group g+2's scales leave, every tile of group g has
// completed its tile handshake (4 tiles per group, 2 in flight), so a drain
// holding two groups' scales, indexed by grp[0], never loses one in use.
//
// Interfaces to the neighbouring blocks:
//   - entry port (weight ring writer): one entry per tensor run, in stream
//     order, offered while that run is (being) written into the ring;
//   - ring port: write pointer out, read address in (from a register), data
//     W_LAT cycles later,
//     slots freed by count; a scale slot carries its beat in bits 511:0;
//   - window port and its state (row store): one address for the 4 lanes,
//     data X_LAT cycles later; win_valid / win_cls say what the window holds.
//     For A other than WIN the command path posts the command once the window
//     fill is complete.
//
// Reset: synchronous, active low, control state only; the data delay lines
// have no reset.
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module me_tile_walker
  import me_if_pkg::*;
#(
  parameter int W_LAT      = 2,      // ring read latency, address -> data (cycles, >= 1)
  parameter int X_LAT      = 2,      // window read latency, address -> data (cycles, >= 1)
  parameter int WIN_STRIDE = 96,     // window entries per window row (1,536 B / 16)
  parameter int TAG_DEPTH  = 4       // tag FIFO depth (>= MEI_TILES_IN_FLIGHT)
) (
  input  wire logic              clk,
  input  wire logic              rst_n,

  // ---- step context (descriptor, model shape), stable while a step runs ----
  input  wire logic [5:0]        n_layers,       // SHAPE.n_layers
  input  wire logic [4:0]        step_rows,      // the step's rows: 1 .. 24
  input  wire logic [3:0]        step_logit_rows,// rows that want logits: 0 .. 8

  // ---- command from the command path ----
  input  wire logic              cmd_valid,
  output logic                   cmd_ready,
  input  wire logic [63:0]       cmd_word,       // me_cmd_pkg::cmd_t, LAYER with the loop count added
  input  wire logic [5:0]        cmd_id,         // command slot, copied into every tag
  output logic                   cmd_done,       // pulse: last beat issued, or the command ended early
  output logic                   cmd_fault,      // with cmd_done: the command faulted
  output logic [7:0]             cmd_fault_code, // with cmd_done: me_cmd_pkg::fault_e (0 = none)
  output logic                   busy,

  // ---- tensor entries from the ring writer ----
  input  wire logic              ent_valid,
  output logic                   ent_ready,      // pop: the command starts on this entry
  input  wire logic [5:0]        ent_layer,
  input  wire logic [4:0]        ent_tag,
  input  wire logic [1:0]        ent_kind,       // wlist_entry_t.kind: 0 GEMM
  input  wire logic [15:0]       ent_n,
  input  wire logic [15:0]       ent_k,

  // ---- weight ring ----
  input  wire logic [10:0]       ring_wptr,      // slots written so far, modulo 2,048
  output logic                   ring_rd,
  output logic [9:0]             ring_raddr,
  input  wire logic [1023:0]     ring_rdata,     // valid W_LAT cycles after ring_rd
  output logic                   ring_free,      // pulse: ring_free_n slots are done with
  output logic [6:0]             ring_free_n,

  // ---- row store input window ----
  input  wire logic              win_valid,      // the window holds a fill ...
  input  wire logic [3:0]        win_cls,        // ... written at this class
  output logic                   win_rd,
  output logic [9:0]             win_raddr,
  input  wire logic [511:0]      win_rdata,      // lane r (array row r) at [r*128 +: 128], X_LAT later

  // ---- column scales to the drain ----
  output logic                   scale_valid,    // one per 32-channel group, in order
  output logic [10:0]            scale_grp,      // the group g (channels 32g .. 32g+31)
  output logic [511:0]           scale_data,     // channel 32g + j at [16j +: 16], FP16

  // ---- beat port to the array (contract) ----
  output logic                   op_valid,
  output logic                   op_first,
  output logic                   op_last,
  output logic                   op_bank,
  output mei_w_bus_t             op_w,
  output mei_dp_bus_t            op_dp,
  output mei_x_bus_t             op_x,

  // ---- tile port, observed, and the tags (contract rule P6) ----
  input  wire logic              tile_valid,
  input  wire logic              tile_ready,
  output logic                   tile_tag_valid,
  output mei_tag_t               tile_tag,       // the tag of the tile now on the tile port

  // ---- counters ----
  input  wire logic              cnt_clr,        // the step starts: clear the counters
  output logic [31:0]            perf_beats,     // array beats issued
  output logic [31:0]            perf_scales,    // scale slots sent to the drain
  output logic [31:0]            bubbles,        // result_hdr_t.bubbles: cycles a GEMM waited for the ring
  output logic [31:0]            perf_stall_credit // slot ready, waiting for a tile credit
);

  localparam int LAT  = (W_LAT > X_LAT) ? W_LAT : X_LAT;
  // the read requests leave from registers (one cycle after the issue decision), so the decision,
  // the walker's longest path, has the cycle to itself and the wire to the ring's and the window's
  // block RAM has the next one: the read data arrives 1 + W_LAT / 1 + X_LAT cycles after issue
  localparam int LATF = LAT + 1;

  // the interface's codes
  localparam logic [5:0] OPC_GEMM   = 6'(me_cmd_pkg::OP_GEMM);
  localparam logic [3:0] B_RES      = 4'(me_cmd_pkg::BUF_RES);
  localparam logic [3:0] B_STG16    = 4'(me_cmd_pkg::BUF_STG16);
  localparam logic [3:0] B_STG32    = 4'(me_cmd_pkg::BUF_STG32);
  localparam logic [3:0] B_WIN      = 4'(me_cmd_pkg::BUF_WIN);
  localparam logic [1:0] RS_ALL     = 2'(me_cmd_pkg::ROWSET_ALL);
  localparam logic [1:0] RS_LOGIT   = 2'(me_cmd_pkg::ROWSET_LOGIT);
  localparam logic [7:0] F_NONE     = 8'(me_cmd_pkg::FAULT_NONE);
  localparam logic [7:0] F_UNKNOWN  = 8'(me_cmd_pkg::FAULT_CMD_UNKNOWN);
  localparam logic [7:0] F_FIELD    = 8'(me_cmd_pkg::FAULT_CMD_FIELD);
  localparam logic [7:0] F_TAG      = 8'(me_cmd_pkg::FAULT_TAG);

  // --------------------------------------------------------------------------
  // command decode and checks
  // --------------------------------------------------------------------------
  typedef enum logic [1:0] { S_IDLE, S_CHK, S_ENT, S_RUN } state_e;
  state_e state;

  me_cmd_pkg::cmd_t cw;                      // the command being run
  logic [5:0]  id;

  wire logic       is_kv   = cw.flags[me_cmd_pkg::CF_KV];
  wire logic       is_topk = cw.flags[me_cmd_pkg::CF_TOPK];
  wire logic [4:0] nused   = (cw.rowset == RS_LOGIT) ? 5'(step_logit_rows) : step_rows;

  // checks 1-5 on the registered word (S_CHK)
  logic [7:0] chk_code;
  always_comb begin
    if      (cw.op != OPC_GEMM)                              chk_code = F_UNKNOWN;
    else if (cw.rsv != '0)                                   chk_code = F_FIELD;
    else if (cw.layer >= n_layers)                           chk_code = F_FIELD;
    else if (cw.rowset != RS_ALL && cw.rowset != RS_LOGIT)   chk_code = F_FIELD;
    else                                                     chk_code = F_NONE;
  end

  // checks 6-8 against the entry the ring delivers (S_ENT)
  wire logic ent_match = (ent_tag == cw.tag) && (cw.tag[4] || ent_layer == cw.layer) && (ent_kind == 2'd0);
  wire logic ent_shape = (ent_n[4:0] == '0) && (ent_n != '0) && (ent_n <= 16'd49152) &&
                         (ent_k[3:0] == '0) && (ent_k != '0) && (ent_k <= 16'(16 * WIN_STRIDE));
  wire logic a_ok = (cw.abuf == B_RES) || (cw.abuf == B_STG16) || (cw.abuf == B_STG32) ||
                    ((cw.abuf == B_WIN) && win_valid && (win_cls == cw.iclass));
  wire logic d_ok = is_kv || is_topk || (cw.dbuf == B_RES) || (cw.dbuf == B_STG16) || (cw.dbuf == B_STG32);
  wire logic [7:0] ent_code = !(ent_match && ent_shape) ? F_TAG :
                              !a_ok                     ? F_FIELD :
                              !d_ok                     ? F_FIELD : F_NONE;

  assign cmd_ready = (state == S_IDLE);
  assign busy      = (state != S_IDLE);
  assign ent_ready = (state == S_ENT) && ent_valid && (ent_code == F_NONE);

  // --------------------------------------------------------------------------
  // loop counters
  // --------------------------------------------------------------------------
  logic [6:0]  kch;            // K/16
  logic [10:0] ngrp;           // N/32
  logic [2:0]  nrg;            // ceil(rows / 4)
  logic [4:0]  nrows;

  logic [10:0] g;
  logic [1:0]  ct;
  logic [2:0]  rg;
  logic [6:0]  c;
  logic        sc;             // the group's scale slot is next
  logic [4:0]  rows_left;      // live rows from 4*rg on (for the tag's rvalid)
  logic [9:0]  xbase;          // rg * WIN_STRIDE
  logic [10:0] base;           // ring slot number of the next scale slot, or of the tile's chunk 0

  // --------------------------------------------------------------------------
  // issue decision
  // --------------------------------------------------------------------------
  logic [1:0] in_flight;                       // contract rule P4
  logic       bank_next;                       // bank of the next tile (rule P3)
  logic       bank_cur;                        // bank of the open tile

  wire logic        run       = (state == S_RUN);
  wire logic [10:0] slot      = sc ? base : base + 11'(c);
  wire logic [10:0] ahead     = ring_wptr - slot;          // slots written past this one
  wire logic        slot_ok   = (ahead != 11'd0) && (ahead <= 11'd1024);
  wire logic        first     = (c == 7'd0);
  wire logic        credit_ok = (in_flight < 2'(MEI_TILES_IN_FLIGHT));
  wire logic        issue_sc  = run && sc && slot_ok;
  wire logic        issue_bt  = run && !sc && slot_ok && (!first || credit_ok);
  wire logic        issue     = issue_sc || issue_bt;

  wire logic last_c  = (c == kch - 7'd1);
  wire logic last_rg = (rg == nrg - 3'd1);
  wire logic last_ct = (ct == 2'd3);
  wire logic last_g  = (g == ngrp - 11'd1);
  wire logic tile_done_hs = tile_valid && tile_ready;
  wire logic beat_bank    = first ? bank_next : bank_cur;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      ring_rd <= 1'b0;
      win_rd  <= 1'b0;
    end else begin
      ring_rd <= issue;
      win_rd  <= issue_bt;
    end
    ring_raddr <= slot[9:0];
    win_raddr  <= xbase + 10'(c);
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      state          <= S_IDLE;
      cmd_done       <= 1'b0;
      cmd_fault      <= 1'b0;
      cmd_fault_code <= F_NONE;
      base      <= '0;
      in_flight <= '0;
      bank_next <= 1'b0;
      bank_cur  <= 1'b0;
      cw <= '0; id <= '0;
      g <= '0; ct <= '0; rg <= '0; c <= '0; sc <= 1'b0; rows_left <= '0; xbase <= '0;
      kch <= '0; ngrp <= '0; nrg <= '0; nrows <= '0;
    end else begin
      cmd_done <= 1'b0;

      case (state)
        S_IDLE: if (cmd_valid) begin
          cw    <= me_cmd_pkg::cmd_t'(cmd_word);
          id    <= cmd_id;
          state <= S_CHK;
        end

        S_CHK: begin
          if (chk_code != F_NONE) begin
            state <= S_IDLE; cmd_done <= 1'b1; cmd_fault <= 1'b1; cmd_fault_code <= chk_code;
          end else if (nused == '0) begin
            state <= S_IDLE; cmd_done <= 1'b1; cmd_fault <= 1'b0; cmd_fault_code <= F_NONE;
          end else begin
            nrows <= nused;
            nrg   <= 3'((nused + 5'd3) >> 2);
            state <= S_ENT;
          end
        end

        S_ENT: if (ent_valid) begin
          if (ent_code != F_NONE) begin
            state <= S_IDLE; cmd_done <= 1'b1; cmd_fault <= 1'b1; cmd_fault_code <= ent_code;
          end else begin
            kch  <= 7'(ent_k >> 4);
            ngrp <= 11'(ent_n >> 5);
            g <= '0; ct <= '0; rg <= '0; c <= '0; sc <= 1'b1; xbase <= '0;
            rows_left <= nrows;
            state <= S_RUN;
          end
        end

        S_RUN: begin
          if (issue_sc) begin
            sc   <= 1'b0;
            base <= base + 11'd1;
          end
          if (issue_bt) begin
            if (!last_c) begin
              c <= c + 7'd1;
            end else begin
              c <= '0;
              if (!last_rg) begin
                rg        <= rg + 3'd1;
                xbase     <= xbase + 10'(WIN_STRIDE);
                rows_left <= rows_left - 5'd4;
              end else begin
                rg        <= '0;
                xbase     <= '0;
                rows_left <= nrows;
                base      <= base + 11'(kch);
                ct        <= ct + 2'd1;
                if (last_ct) begin
                  sc <= 1'b1;
                  g  <= g + 11'd1;
                  if (last_g) begin
                    state <= S_IDLE; cmd_done <= 1'b1; cmd_fault <= 1'b0; cmd_fault_code <= F_NONE;
                  end
                end
              end
            end
            if (first) begin
              bank_cur  <= bank_next;
              bank_next <= ~bank_next;
            end
          end
        end
      endcase

      // credits: spent when a tile's first beat issues, returned at its handshake
      in_flight <= in_flight + 2'(issue_bt && first) - 2'(tile_done_hs);
    end
  end

  // --------------------------------------------------------------------------
  // tags: pushed at a tile's first beat, popped at its handshake (in order)
  // --------------------------------------------------------------------------
  localparam int TAG_AW = $clog2(TAG_DEPTH);
  mei_tag_t            tag_mem [TAG_DEPTH];
  logic [TAG_AW:0]     tag_wp, tag_rp;

  mei_tag_t tag_new;
  always_comb begin
    tag_new        = '0;
    tag_new.dest   = MEI_DEST_DRAIN;
    tag_new.cmd    = id;
    tag_new.grp    = {g, ct};                  // 8-channel tile 4g + ct
    tag_new.rgrp   = rg;
    tag_new.rvalid = (rows_left >= 5'd4) ? 4'b1111 :
                     (rows_left == 5'd3) ? 4'b0111 :
                     (rows_left == 5'd2) ? 4'b0011 : 4'b0001;
    tag_new.sub    = '0;
    tag_new.last   = last_rg && last_ct && last_g;
  end

  always_ff @(posedge clk) begin
    if (issue_bt && first) tag_mem[tag_wp[TAG_AW-1:0]] <= tag_new;
  end
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      tag_wp <= '0;
      tag_rp <= '0;
    end else begin
      if (issue_bt && first) tag_wp <= tag_wp + 1'b1;
      if (tile_done_hs && tile_tag_valid) tag_rp <= tag_rp + 1'b1;
    end
  end
  assign tile_tag_valid = (tag_wp != tag_rp);
  assign tile_tag       = tag_mem[tag_rp[TAG_AW-1:0]];

  // --------------------------------------------------------------------------
  // align the beat's flags with the read data (LATF cycles after issue)
  // --------------------------------------------------------------------------
  logic [LATF-1:0] v_d, s_d;                    // reset: a stale valid is fatal
  logic [LATF-1:0] f_d, l_d, b_d, fr_d;         // qualified by v_d, no reset
  logic [6:0]      n_d [LATF];
  logic [10:0]     g_d [LATF];

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      v_d <= '0;
      s_d <= '0;
    end else begin
      v_d <= {v_d[LATF-2:0], issue_bt};
      s_d <= {s_d[LATF-2:0], issue_sc};
    end
  end
  always_ff @(posedge clk) begin
    f_d  <= {f_d[LATF-2:0],  first};
    l_d  <= {l_d[LATF-2:0],  last_c};
    b_d  <= {b_d[LATF-2:0],  beat_bank};
    fr_d <= {fr_d[LATF-2:0], last_c && last_rg};      // free the tile's slots after its last row group
    n_d[0] <= kch;
    g_d[0] <= g;
    for (int i = 1; i < LATF; i++) begin
      n_d[i] <= n_d[i-1];
      g_d[i] <= g_d[i-1];
    end
  end

  // the read data arrives W_LAT / X_LAT after its request; delay the earlier one to LAT
  logic [1023:0] w_al;
  logic [511:0]  x_al;
  generate
    if (LAT == W_LAT) begin : g_w_direct
      assign w_al = ring_rdata;
    end else begin : g_w_delay
      logic [1023:0] w_sr [LAT - W_LAT];
      always_ff @(posedge clk) begin
        w_sr[0] <= ring_rdata;
        for (int i = 1; i < LAT - W_LAT; i++) w_sr[i] <= w_sr[i-1];
      end
      assign w_al = w_sr[LAT - W_LAT - 1];
    end
    if (LAT == X_LAT) begin : g_x_direct
      assign x_al = win_rdata;
    end else begin : g_x_delay
      logic [511:0] x_sr [LAT - X_LAT];
      always_ff @(posedge clk) begin
        x_sr[0] <= win_rdata;
        for (int i = 1; i < LAT - X_LAT; i++) x_sr[i] <= x_sr[i-1];
      end
      assign x_al = x_sr[LAT - X_LAT - 1];
    end
  endgenerate

  assign op_valid = v_d[LATF-1];
  assign op_first = f_d[LATF-1];
  assign op_last  = l_d[LATF-1];
  assign op_bank  = b_d[LATF-1];

  // contract widths: INT8 sign-extended, pre-adder 0 in GEMM mode
  always_comb begin
    for (int e = 0; e < MEI_C * MEI_D; e++)
      op_w[e*MEI_A_W +: MEI_A_W] = MEI_A_W'($signed(w_al[e*8 +: 8]));
    for (int e = 0; e < MEI_R * MEI_D; e++)
      op_x[e*MEI_X_W +: MEI_X_W] = MEI_X_W'($signed(x_al[e*8 +: 8]));
    op_dp = '0;
  end

  assign scale_valid = s_d[LATF-1];
  assign scale_grp   = g_d[LATF-1];
  assign scale_data  = w_al[511:0];

  // a scale slot frees itself; a tile's slots go after its last row group
  // (never in the same cycle: they come from different issue cycles)
  assign ring_free   = s_d[LATF-1] || (v_d[LATF-1] && fr_d[LATF-1]);
  assign ring_free_n = s_d[LATF-1] ? 7'd1 : n_d[LATF-1];

  // --------------------------------------------------------------------------
  // counters
  // --------------------------------------------------------------------------
  // a bubble: a cycle a command is in hand, nothing issues, and the slot it
  // needs next (in S_CHK / S_ENT the tensor's first, at base) is not in the
  // ring yet; a cycle waiting only for a credit is not one
  wire logic [10:0] nxt_ahead = ring_wptr - base;
  wire logic        bubble    = run ? !slot_ok
                                    : ((state == S_CHK || state == S_ENT) &&
                                       !((nxt_ahead != 11'd0) && (nxt_ahead <= 11'd1024)));

  always_ff @(posedge clk) begin
    if (!rst_n || cnt_clr) begin
      perf_beats        <= '0;
      perf_scales       <= '0;
      bubbles           <= '0;
      perf_stall_credit <= '0;
    end else begin
      if (issue_bt)                                      perf_beats        <= perf_beats + 1;
      if (issue_sc)                                      perf_scales       <= perf_scales + 1;
      if (bubble)                                        bubbles           <= bubbles + 1;
      if (run && !sc && slot_ok && first && !credit_ok)  perf_stall_credit <= perf_stall_credit + 1;
    end
  end

endmodule

`default_nettype wire
