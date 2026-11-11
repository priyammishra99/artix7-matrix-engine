// ============================================================================
// File: me_row_store.sv
// Block: Matrix engine - row store: storage only
//
// Holds the step's rows on chip, up to 24 (interface MAX_ROWS):
//   window    the INT8 input window the array reads: 4 lanes of 128 bits, one
//             per array row; window row i (the command's i-th used row) in
//             lane i mod 4, its 16-B chunk c at entry (i div 4) x 96 + c.
//             576 entries a lane, 16 BRAM36 (BUF_WIN)
//   staging   24 rows x 6 KiB as 32-bit words, word row x 1,536 + j: INT32
//             element j (BUF_STG32), or INT16 elements 2j and 2j+1 in its low
//             and high halves (BUF_STG16). 36,864 words, 36 BRAM36
//   residual  24 rows x 576 x INT32 as 64-bit entries, entry row x 288 + j/2,
//             element j in half j mod 2 (BUF_RES). 6,912 entries, 14 BRAM36
//   own K/V   24 rows x (K, V) x 3 KV heads x 64 INT8 as 64-bit entries, entry
//             row x 48 + kv x 24 + head x 8 + d/8, byte d mod 8 (BUF_KVO).
//             1,152 entries, 3 BRAM36
// 69 BRAM36 in all. Every read port returns its data 2 cycles after the read.
//
// The window's per-row constants (the drain's absmax and wide-row shift, one
// per window row) are kept in two copies, so that a window fill for the next
// GEMM never overwrites the constants the drain still needs for the last
// tiles of the GEMM before it:
//   - a fill (the window loader now, the drain's quantiser from stage 2) writes
//     the spare copy; at fill done it becomes the latest copy, and the window
//     is valid at the fill's class (win_valid / win_cls, the walker's A = WIN
//     check);
//   - a GEMM uses the copy that is latest when the walker starts running it
//     (its entry pop, ent_pop); a GEMM that reuses the window (A = WIN) uses
//     the same copy, as there is no fill in between;
//   - the copies of the GEMMs started are queued in order, and the head of the
//     queue is popped at each command's last tile handshake (tile_last), so the
//     drain-side read (drn_row -> drn_absmax, drn_shift) always answers from
//     the copy of the tile now on the tile port. The drain takes the constants
//     at the tile handshake (the drain's rule);
//   - a fill may start only while the walker is idle (the old window's last
//     read is behind it) and never into a copy a queued GEMM still uses
//     (err_fill, sticky, if one is attempted).
// None of this needs a signal the walker does not already drive.
//
// Debug paths, so that stage 1 runs on the board before the quantiser exists:
//   - window loader: a beat stream from DDR (the reference's INT8 window and
//     its constants): beat 0 holds the 24 rows' constants, row r's absmax in
//     bits 20r +: 15 and its shift in 20r + 15 +: 5; then, per row 0 .. nrows-1,
//     ceil(kch / 4) beats of 4 chunks (chunk c in bits 128 (c mod 4) +: 128);
//   - dump: staging words or residual entries to a beat stream (to the memory
//     write side): 16 staging words or 8 residual entries a beat,
//     word or entry 0 in the low bits.
// The loader and the dump are started from the debug command path (me_s1_cmd).
//
// The ports are the store's side of the neighbours' interfaces: the walker (window read, window state,
// the entry pop and tile handshake observed), the drain (staging, residual and
// own K/V writes; the per-row constants), the quantiser (window fill, staging
// read), the norm and EMBED (residual), attention (staging, own K/V), the
// memory side (loader and dump streams). Each port is used by one client
// at a time, as the command path orders the operations; the store does not
// arbitrate between external clients, but the dump owns the staging and
// residual read ports while it runs (err_port, sticky, on a collision).
//
// USE_PRIM = 1 instantiates the staging (36 RAMB36E1 of 1K x 36), the residual
// (14 of 512 x 72) and the own K/V (3 of 512 x 72) through me_bram_banks, as the
// BRAM recount asks (inferred, Vivado took 64, 16 and 4); the window lanes are
// inferred (me_sdp_ram), which maps them as planned (4 BRAM36 each).
//
// Reset: synchronous, active low (the core's soft reset): the window state,
// the constants' copies and queue, the loader and the dump; the memories keep
// stale data.
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module me_row_store #(
  parameter bit USE_PRIM = 1'b1
) (
  input  wire logic              clk,
  input  wire logic              rst_n,

  // ---- window: read by the walker ----
  input  wire logic              win_rd,
  input  wire logic [9:0]        win_raddr,
  output logic [511:0]           win_rdata,      // lane r at [128 r +: 128], 2 cycles later
  output logic                   win_valid,      // the window holds a fill ...
  output logic [3:0]             win_cls,        // ... at this class

  // ---- window fill: the quantiser (stage 2); the loader drives the same port inside ----
  input  wire logic              fq_start,       // a fill begins (accepted when fill_ready)
  input  wire logic [3:0]        fq_cls,
  output logic                   fill_ready,     // a fill may start: no fill running, walker idle
  input  wire logic              fq_wr,          // one 16-B chunk of one window row
  input  wire logic [4:0]        fq_row,
  input  wire logic [6:0]        fq_chunk,
  input  wire logic [127:0]      fq_data,
  input  wire logic              fq_rc_wr,       // the row's constants
  input  wire logic [4:0]        fq_rc_row,
  input  wire logic [14:0]       fq_absmax,
  input  wire logic [4:0]        fq_shift,
  input  wire logic              fq_done,        // the fill is complete

  // ---- observed from the walker (signals it already drives) ----
  input  wire logic              walker_busy,
  input  wire logic              ent_pop,        // ent_valid && ent_ready: a GEMM starts on the window
  input  wire logic              tile_hs,        // tile_valid && tile_ready
  input  wire logic              tile_last,      // tile_tag.last

  // ---- the drain: the per-row constants of the tile now on the tile port ----
  input  wire logic [4:0]        drn_row,        // window row (4 tag.rgrp + r)
  output logic [14:0]            drn_absmax,
  output logic [4:0]             drn_shift,

  // ---- staging (BUF_STG16 / BUF_STG32) ----
  input  wire logic              stg_we,
  input  wire logic [15:0]       stg_waddr,      // word: row x 1,536 + j
  input  wire logic [1:0]        stg_whe,        // half enables: 01 the low INT16, 10 the high, 11 the word
  input  wire logic [31:0]       stg_wdata,
  input  wire logic              stg_rd,
  input  wire logic [15:0]       stg_raddr,
  output logic [31:0]            stg_rdata,      // 2 cycles after stg_rd

  // ---- residual (BUF_RES) ----
  input  wire logic              res_we,
  input  wire logic [12:0]       res_waddr,      // entry: row x 288 + j/2
  input  wire logic [1:0]        res_wee,        // element enables: 01 element 2e, 10 element 2e + 1
  input  wire logic [63:0]       res_wdata,
  input  wire logic              res_rd,
  input  wire logic [12:0]       res_raddr,
  output logic [63:0]            res_rdata,      // 2 cycles after res_rd

  // ---- the step's own K and V (BUF_KVO) ----
  input  wire logic              kvo_we,
  input  wire logic [10:0]       kvo_waddr,      // entry: row x 48 + kv x 24 + head x 8 + d/8
  input  wire logic [7:0]        kvo_wbe,        // byte d mod 8
  input  wire logic [63:0]       kvo_wdata,
  input  wire logic              kvo_rd,
  input  wire logic [10:0]       kvo_raddr,
  output logic [63:0]            kvo_rdata,      // 2 cycles after kvo_rd

  // ---- debug: window loader ----
  input  wire logic              ld_start,       // load a window (accepted when fill_ready)
  input  wire logic [3:0]        ld_cls,
  input  wire logic [4:0]        ld_nrows,       // 1 .. 24
  input  wire logic [6:0]        ld_kch,         // K/16: 1 .. 96
  input  wire logic              ld_valid,
  output logic                   ld_ready,
  input  wire logic [511:0]      ld_data,
  output logic                   ld_busy,

  // ---- debug: dump (the interface's DUMP) ----
  input  wire logic              dp_start,
  input  wire logic              dp_res,         // 0 staging, 1 residual
  input  wire logic [15:0]       dp_first,       // first word / entry
  input  wire logic [15:0]       dp_beats,       // beats to send: 1 ..
  output logic                   dp_valid,
  input  wire logic              dp_ready,
  output logic [511:0]           dp_data,
  output logic                   dp_busy,

  // ---- status ----
  output logic                   err_fill,       // sticky: a fill into a copy still in use, or while busy
  output logic                   err_port        // sticky: a client used a read port the dump owned
);

  // ==========================================================================
  // window fill: the loader or the quantiser, through one write port
  // ==========================================================================
  logic         ld_act;                          // the loader is filling
  logic         l_wr, l_rc_wr, l_done;
  logic [4:0]   l_row, l_rc_row;
  logic [6:0]   l_chunk;
  logic [127:0] l_data;
  logic [14:0]  l_absmax;
  logic [4:0]   l_shift;

  // the loader's writes leave a register after its state: it stays selected until they are out
  wire logic         l_sel    = ld_act || l_wr || l_rc_wr || l_done;
  wire logic         f_wr     = l_sel ? l_wr     : fq_wr;
  wire logic [4:0]   f_row    = l_sel ? l_row    : fq_row;
  wire logic [6:0]   f_chunk  = l_sel ? l_chunk  : fq_chunk;
  wire logic [127:0] f_data   = l_sel ? l_data   : fq_data;
  wire logic         f_rc_wr  = l_sel ? l_rc_wr  : fq_rc_wr;
  wire logic [4:0]   f_rc_row = l_sel ? l_rc_row : fq_rc_row;
  wire logic [14:0]  f_absmax = l_sel ? l_absmax : fq_absmax;
  wire logic [4:0]   f_shift  = l_sel ? l_shift  : fq_shift;
  wire logic         f_done   = l_sel ? l_done   : fq_done;

  // ---- window memory: 4 lanes of 576 x 128 ----
  wire logic [9:0] f_entry = 10'(f_row[4:2]) * 10'd96 + 10'(f_chunk);
  generate
    for (genvar r = 0; r < 4; r++) begin : g_lane
      me_sdp_ram #(.W(128), .DEPTH(576)) u_lane (
        .clk, .we(f_wr && (f_row[1:0] == 2'(r))), .wbe('1), .waddr(f_entry), .wdata(f_data),
        .rd(win_rd), .raddr(win_raddr), .rdata(win_rdata[128*r +: 128]));
    end
  endgenerate

  // ==========================================================================
  // the window state and the per-row constants' two copies
  // ==========================================================================
  logic        filling;                          // a fill is running
  logic [3:0]  fill_cls;
  logic        latest;                           // the copy of the last completed fill
  logic        have_latest;                      // a fill has completed since reset
  logic [14:0] rc_absmax [2][24];
  logic [4:0]  rc_shift  [2][24];

  // the copies of the GEMMs started, oldest first (at most 2 tiles are in flight, so
  // at most 2 GEMMs have tiles not yet handed over; 4 deep for margin)
  logic        q_copy [4];
  logic [2:0]  q_n;
  logic [1:0]  q_rp;
  // a copy is in use while a queued GEMM uses it
  function automatic logic in_use(logic c);
    in_use = 1'b0;
    for (int i = 0; i < 4; i++)
      if (3'(i) < q_n && q_copy[2'(q_rp + 2'(i))] == c) in_use = 1'b1;
  endfunction

  wire logic spare  = have_latest ? !latest : 1'b0;
  wire logic f_go   = (ld_start && fill_ready) || (!ld_start && fq_start && fill_ready);
  assign fill_ready = !filling && !walker_busy && !l_sel;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      filling <= 1'b0; fill_cls <= '0; latest <= 1'b0; have_latest <= 1'b0;
      win_valid <= 1'b0; win_cls <= '0; err_fill <= 1'b0;
      q_n <= '0; q_rp <= '0;
    end else begin
      if (f_go) begin
        filling  <= 1'b1;
        fill_cls <= ld_start ? ld_cls : fq_cls;
        if (have_latest && in_use(spare)) err_fill <= 1'b1;
        win_valid <= 1'b0;                       // the old window is being overwritten
      end
      if ((fq_start || ld_start) && !fill_ready) err_fill <= 1'b1;
      if (filling && f_done) begin
        filling     <= 1'b0;
        latest      <= spare;
        have_latest <= 1'b1;
        win_valid   <= 1'b1;
        win_cls     <= fill_cls;
      end
      // the queue: a GEMM starts on the latest copy; its last tile ends its use
      case ({ent_pop, tile_hs && tile_last && (q_n != 0)})
        2'b10: begin q_copy[2'(q_rp + 2'(q_n))] <= latest; q_n <= q_n + 3'd1; end
        2'b01: begin q_rp <= q_rp + 2'd1; q_n <= q_n - 3'd1; end
        2'b11: begin q_copy[2'(q_rp + 2'(q_n))] <= latest; q_rp <= q_rp + 2'd1; end
        default: ;
      endcase
    end
  end

  // the constants: the fill writes the spare copy
  always_ff @(posedge clk) begin
    if (filling && f_rc_wr && f_rc_row < 5'd24) begin
      rc_absmax[spare][f_rc_row] <= f_absmax;
      rc_shift[spare][f_rc_row]  <= f_shift;
    end
  end
  // the drain reads the copy of the tile now on the tile port (the oldest queued GEMM). The choice
  // is a register, computed from the next state, so it is right in the cycle after any change:
  // the drain's read is a register and a 24-to-1 select, not the queue's logic
  wire logic       q_push  = ent_pop;
  wire logic       q_pop   = tile_hs && tile_last && (q_n != 0);
  wire logic [2:0] q_n_nx  = q_n + 3'(q_push) - 3'(q_pop);
  wire logic [1:0] q_rp_nx = q_rp + 2'(q_pop);
  wire logic       lat_nx  = (filling && f_done) ? spare : latest;
  wire logic       head_nx = (q_push && (q_rp_nx == 2'(q_rp + 2'(q_n)))) ? latest : q_copy[q_rp_nx];
  logic d_copy;
  always_ff @(posedge clk) begin
    if (!rst_n) d_copy <= 1'b0;
    else        d_copy <= (q_n_nx != 0) ? head_nx : lat_nx;
  end
  assign drn_absmax = (drn_row < 5'd24) ? rc_absmax[d_copy][drn_row] : '0;
  assign drn_shift  = (drn_row < 5'd24) ? rc_shift[d_copy][drn_row]  : '0;

  // ==========================================================================
  // the window loader: beat 0 the constants, then each row's chunks, 4 a beat
  // ==========================================================================
  logic [4:0]   lw_nrows, lw_row;
  logic [6:0]   lw_kch, lw_chunk;                // the next chunk of the current row
  logic         lw_const;                        // the constants' beat is next
  logic [511:0] lw_beat;
  logic [1:0]   lw_sub;                          // the chunk within the held beat
  logic         lw_have;                         // a chunk beat is held
  logic [4:0]   lw_crow;                         // constants written so far

  assign ld_busy  = ld_act;
  assign ld_ready = ld_act && !lw_have && (lw_const ? (lw_crow == 5'd24) : 1'b1);

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      ld_act <= 1'b0; lw_have <= 1'b0; lw_const <= 1'b0; lw_crow <= '0;
      l_wr <= 1'b0; l_rc_wr <= 1'b0; l_done <= 1'b0;
    end else begin
      l_wr <= 1'b0; l_rc_wr <= 1'b0; l_done <= 1'b0;
      if (!ld_act) begin
        if (ld_start && fill_ready) begin
          ld_act   <= 1'b1;
          lw_nrows <= ld_nrows; lw_kch <= ld_kch;
          lw_row   <= '0; lw_chunk <= '0; lw_const <= 1'b1; lw_crow <= 5'd24; lw_have <= 1'b0;
        end
      end else begin
        // take a beat
        if (ld_valid && ld_ready) begin
          lw_beat <= ld_data;
          if (lw_const) lw_crow <= '0;           // the constants' beat: 24 writes follow
          else begin lw_have <= 1'b1; lw_sub <= 2'd0; end
        end
        // write the constants, one row a cycle
        if (lw_const && lw_crow != 5'd24 && !(ld_valid && ld_ready)) begin
          l_rc_wr  <= 1'b1;
          l_rc_row <= lw_crow;
          l_absmax <= lw_beat[20*lw_crow +: 15];
          l_shift  <= lw_beat[20*lw_crow + 15 +: 5];
          lw_crow  <= lw_crow + 5'd1;
          if (lw_crow == 5'd23) lw_const <= 1'b0;
        end
        // write the held beat's chunks, one a cycle
        if (lw_have) begin
          l_wr    <= 1'b1;
          l_row   <= lw_row;
          l_chunk <= lw_chunk;
          l_data  <= lw_beat[128*lw_sub +: 128];
          lw_sub  <= lw_sub + 2'd1;
          if (lw_chunk == lw_kch - 7'd1) begin   // the row's last chunk: the rest of the beat is padding
            lw_have  <= 1'b0;
            lw_chunk <= '0;
            if (lw_row == lw_nrows - 5'd1) begin
              ld_act <= 1'b0;
              l_done <= 1'b1;
            end
            lw_row <= lw_row + 5'd1;
          end else begin
            lw_chunk <= lw_chunk + 7'd1;
            if (lw_sub == 2'd3) lw_have <= 1'b0;
          end
        end
      end
    end
  end

  // ==========================================================================
  // staging, residual, own K/V
  // ==========================================================================
  // the dump owns the staging and residual read ports while it runs
  logic        dp_rd;
  logic [15:0] dp_raddr;
  logic        dp_res_q;
  wire  logic  dp_on_stg = dp_busy && !dp_res_q;
  wire  logic  dp_on_res = dp_busy &&  dp_res_q;

  wire logic        s_rd    = dp_on_stg ? dp_rd : stg_rd;
  wire logic [15:0] s_raddr = dp_on_stg ? dp_raddr : stg_raddr;
  wire logic        r_rd    = dp_on_res ? dp_rd : res_rd;
  wire logic [12:0] r_raddr = dp_on_res ? dp_raddr[12:0] : res_raddr;

  me_bram_banks #(.MODE(0), .NB(36), .AW(16), .USE_PRIM(USE_PRIM)) u_stg (
    .clk, .we(stg_we), .wbe({{2{stg_whe[1]}}, {2{stg_whe[0]}}}), .waddr(stg_waddr), .wdata(stg_wdata),
    .rd(s_rd), .raddr(s_raddr), .rdata(stg_rdata));

  me_bram_banks #(.MODE(1), .NB(14), .AW(13), .USE_PRIM(USE_PRIM)) u_res (
    .clk, .we(res_we), .wbe({{4{res_wee[1]}}, {4{res_wee[0]}}}), .waddr(res_waddr), .wdata(res_wdata),
    .rd(r_rd), .raddr(r_raddr), .rdata(res_rdata));

  me_bram_banks #(.MODE(1), .NB(3), .AW(11), .USE_PRIM(USE_PRIM)) u_kvo (
    .clk, .we(kvo_we), .wbe(kvo_wbe), .waddr(kvo_waddr), .wdata(kvo_wdata),
    .rd(kvo_rd), .raddr(kvo_raddr), .rdata(kvo_rdata));

  // ==========================================================================
  // the dump: staging words (16 a beat) or residual entries (8 a beat)
  // ==========================================================================
  logic [15:0] dp_left;                          // reads still to issue
  logic [15:0] dp_sent_left;                     // beats still to send
  logic [3:0]  dp_k;                             // the next read's slot in its beat
  logic [1:0]  dp_v;                             // reads in flight: valid pipeline
  logic [3:0]  dp_k1, dp_k2;
  logic [511:0] dp_acc;
  logic         dp_full;                         // dp_acc holds a whole beat, not yet sent
  wire  logic [3:0] per_beat_m1 = dp_res_q ? 4'd7 : 4'd15;

  assign dp_valid = dp_full;
  assign dp_data  = dp_acc;

  // a read is issued only when its beat can be assembled: not while a full beat waits
  // and not past the beats to send; the 2-cycle latency is covered by stopping 2 early
  wire logic dp_issue = dp_busy && (dp_left != 0) && !dp_full && !(dp_v[0] && dp_k1 == per_beat_m1) &&
                        !(dp_v[1] && dp_k2 == per_beat_m1);
  assign dp_rd = dp_issue;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      dp_busy <= 1'b0; dp_v <= '0; dp_full <= 1'b0; err_port <= 1'b0;
      dp_left <= '0; dp_sent_left <= '0; dp_k <= '0; dp_res_q <= 1'b0;
    end else begin
      if (!dp_busy && dp_start) begin
        dp_busy      <= 1'b1;
        dp_res_q     <= dp_res;
        dp_raddr     <= dp_first;
        dp_left      <= dp_beats * (dp_res ? 16'd8 : 16'd16);
        dp_sent_left <= dp_beats;
        dp_k         <= '0;
      end
      if (dp_issue) begin
        dp_raddr <= dp_raddr + 16'd1;
        dp_left  <= dp_left - 16'd1;
        dp_k     <= (dp_k == per_beat_m1) ? 4'd0 : dp_k + 4'd1;
      end
      dp_v  <= {dp_v[0], dp_issue};
      dp_k1 <= dp_k;
      dp_k2 <= dp_k1;
      if (dp_v[1]) begin
        if (dp_res_q) dp_acc[64*dp_k2[2:0] +: 64] <= res_rdata;
        else          dp_acc[32*dp_k2 +: 32]      <= stg_rdata;
        if (dp_k2 == per_beat_m1) dp_full <= 1'b1;
      end
      if (dp_full && dp_ready) begin
        dp_full      <= 1'b0;
        dp_sent_left <= dp_sent_left - 16'd1;
        if (dp_sent_left == 16'd1) dp_busy <= 1'b0;
      end
      if ((dp_on_stg && stg_rd) || (dp_on_res && res_rd)) err_port <= 1'b1;
    end
  end

endmodule

`default_nettype wire
