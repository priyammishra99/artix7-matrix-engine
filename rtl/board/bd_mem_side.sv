// ============================================================================
// File: bd_mem_side.sv
// Block: AC701 board shell (matrix engine only) - the memory side, from ROMs
//
// Answers me_top's three engine requests as tb_me_top_s1's models do, with the
// board's ROMs in place of DDR:
//   rq_*  (window read)   the window stream from the window ROM into the loader
//   wt_*  (weights)       the run descriptor, then every weight beat from the
//                         weight ROM into the ring
//   wq_*  (dump write)    takes the dump into the checker (bd_checker); wq_done
//                         the cycle after the last beat
// Each request is taken once a run (clr starts a run; multi_wt: a weight request
// each GEMM, each once the stream before it is out); the case table gives
// where each stream is and what each request must ask for. A request that asks
// for something else, or comes twice, sets its bit in err (sticky until clr):
//   0 window: beats differ    1 window: second request
//   2 weights: layer or tag   3 weights: second request
//   4 dump: beats differ      5 dump: second request
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module bd_mem_side #(
  parameter string WT_INIT   = "bd_wt.mem",
  parameter string WT_PRE    = "bd_wt",        // the weight ROM in banks of 1,024: <WT_PRE>_b<b>.mem
  parameter string WIN_INIT  = "bd_win.mem",
  parameter string REF_INIT  = "bd_ref.mem",
  parameter int    WT_DEPTH  = 10404,
  parameter int    WIN_DEPTH = 962,
  parameter int    REF_DEPTH = 1908
) (
  input  wire logic          clk,
  input  wire logic          rst_n,          // synchronous, active low
  input  wire logic          clr,            // a run starts
  input  wire logic          multi_wt,       // GEMM-only: a weight request each GEMM

  // the case (bd_ctrl's case table)
  input  wire logic [15:0]   c_win_base, c_win_beats, c_wt_base, c_wt_beats, c_run_n, c_run_k,
  input  wire logic [5:0]    c_layer,
  input  wire logic [4:0]    c_tag,
  input  wire logic [15:0]   c_ref_base, c_ref_beats, c_dump_beats,
  input  wire logic [6:0]    c_rel_lo, c_rel_n,

  // me_top's requests
  input  wire logic          rq_valid,
  output logic               rq_ready,
  input  wire logic [15:0]   rq_beats,
  input  wire logic          wt_valid,
  output logic               wt_ready,
  input  wire logic [5:0]    wt_layer,
  input  wire logic [4:0]    wt_tag,
  input  wire logic          wq_valid,
  output logic               wq_ready,
  input  wire logic [15:0]   wq_beats,
  output logic               wq_done,

  // me_top's streams
  output logic               ld_valid,
  input  wire logic          ld_ready,
  output logic [511:0]       ld_data,
  output logic               run_valid,
  input  wire logic          run_ready,
  output logic [5:0]         run_layer,
  output logic [4:0]         run_tag,
  output logic [1:0]         run_kind,
  output logic [15:0]        run_n,
  output logic [15:0]        run_k,
  output logic [23:0]        run_beats,
  output logic               w_valid,
  input  wire logic          w_ready,
  output logic [511:0]       w_data,
  input  wire logic          dp_valid,
  output logic               dp_ready,
  input  wire logic [511:0]  dp_data,

  // the dump RAM's write port, the checker's results
  output logic               dr_we,
  output logic [9:0]         dr_waddr,
  output logic [511:0]       dr_wdata,
  output logic [15:0]        chk_rx,
  output logic               chk_idle,
  output logic [31:0]        chk_checked, chk_mism,
  output logic [15:0]        chk_first_k,
  output logic [4:0]         chk_first_lane,
  output logic [15:0]        chk_first_got, chk_first_exp,

  // status
  output logic [2:0]         seen,           // window, weights, dump requested this run
  output logic [5:0]         err,
  output logic [31:0]        wt_count,       // weight requests taken this run
  output logic [31:0]        ref_beats_out   // reference beats streamed (= c_ref_beats at the end)
);

  localparam int WTA = $clog2(WT_DEPTH), WIA = $clog2(WIN_DEPTH), REA = $clog2(REF_DEPTH);

  // ---- the ROMs and their streamers ----
  logic           wi_en, wt_en, re_en;
  logic [WIA-1:0] wi_addr;  logic [WTA-1:0] wt_addr;  logic [REA-1:0] re_addr;
  logic [511:0]   wi_q, wt_q, re_q;
  logic           wi_go, wt_go, re_go, wi_busy, wt_busy, re_busy;
  logic           r_valid, r_ready;  logic [511:0] r_data;

  bd_rom #(.W(512), .DEPTH(WIN_DEPTH), .INIT(WIN_INIT)) u_win_rom (.clk, .en(wi_en), .addr(wi_addr), .q(wi_q));
  bd_rom #(.W(512), .DEPTH(WT_DEPTH),  .INIT(WT_INIT), .BANK(1024), .INIT_PRE(WT_PRE)) u_wt_rom  (.clk, .en(wt_en), .addr(wt_addr), .q(wt_q));
  bd_rom #(.W(512), .DEPTH(REF_DEPTH), .INIT(REF_INIT)) u_ref_rom (.clk, .en(re_en), .addr(re_addr), .q(re_q));

  bd_streamer #(.W(512), .AW(WIA)) u_win (
    .clk, .rst_n(rst_n && !clr), .start(wi_go), .base(WIA'(c_win_base)), .count(c_win_beats), .busy(wi_busy),
    .rom_en(wi_en), .rom_addr(wi_addr), .rom_q(wi_q), .o_valid(ld_valid), .o_ready(ld_ready), .o_data(ld_data));
  bd_streamer #(.W(512), .AW(WTA)) u_wt (
    .clk, .rst_n(rst_n && !clr), .start(wt_go), .base(WTA'(c_wt_base)), .count(c_wt_beats), .busy(wt_busy),
    .rom_en(wt_en), .rom_addr(wt_addr), .rom_q(wt_q), .o_valid(w_valid), .o_ready(w_ready), .o_data(w_data));
  bd_streamer #(.W(512), .AW(REA)) u_ref (
    .clk, .rst_n(rst_n && !clr), .start(re_go), .base(REA'(c_ref_base)), .count(c_ref_beats), .busy(re_busy),
    .rom_en(re_en), .rom_addr(re_addr), .rom_q(re_q), .o_valid(r_valid), .o_ready(r_ready), .o_data(r_data));

  // ---- the requests ----
  assign rq_ready = !seen[0];
  assign wt_ready = multi_wt ? (!run_valid && !wt_busy && !wt_go) : !seen[1];
  assign wq_ready = !seen[2];
  wire logic rq_hs = rq_valid && rq_ready, wt_hs = wt_valid && wt_ready, wq_hs = wq_valid && wq_ready;
  wire logic run_hs = run_valid && run_ready;
  logic [15:0] wq_n;
  logic        wq_armed;

  assign run_kind = 2'd0;
  assign run_n    = c_run_n;
  assign run_k    = c_run_k;
  assign run_beats = 24'(c_wt_beats);

  always_ff @(posedge clk) begin
    if (!rst_n || clr) begin
      seen <= '0; err <= '0; wi_go <= 1'b0; wt_go <= 1'b0; re_go <= 1'b0;
      run_valid <= 1'b0; run_layer <= '0; run_tag <= '0; wq_n <= '0; wq_armed <= 1'b0; wq_done <= 1'b0;
      ref_beats_out <= '0; wt_count <= '0;
    end else begin
      wi_go <= 1'b0; wt_go <= 1'b0; re_go <= 1'b0; wq_done <= 1'b0;
      // window: one request, for the case's beats
      if (rq_valid && seen[0]) err[1] <= 1'b1;
      if (rq_hs) begin
        seen[0] <= 1'b1; wi_go <= 1'b1;
        if (rq_beats != c_win_beats) err[0] <= 1'b1;
      end
      // weights: one request, for the case's tensor; the descriptor, then the beats
      if (wt_valid && seen[1] && !multi_wt) err[3] <= 1'b1;
      if (wt_hs) begin
        seen[1] <= 1'b1; run_valid <= 1'b1; run_layer <= wt_layer; run_tag <= wt_tag; wt_count <= wt_count + 32'd1;
        if (wt_layer != c_layer || wt_tag != c_tag) err[2] <= 1'b1;
      end
      if (run_hs) begin run_valid <= 1'b0; wt_go <= 1'b1; end
      // dump: one request, for the case's beats; the checker takes them
      if (wq_valid && seen[2]) err[5] <= 1'b1;
      if (wq_hs) begin
        seen[2] <= 1'b1; wq_n <= wq_beats; wq_armed <= 1'b1; re_go <= 1'b1;
        if (wq_beats != c_dump_beats) err[4] <= 1'b1;
      end
      if (wq_armed && chk_rx == wq_n) begin wq_done <= 1'b1; wq_armed <= 1'b0; end
      if (r_valid && r_ready) ref_beats_out <= ref_beats_out + 32'd1;
    end
  end

  // ---- the check ----
  bd_checker u_chk (
    .clk, .rst_n, .clr,
    .arm(wq_hs), .n(wq_beats), .rel_lo(c_rel_lo), .rel_n(c_rel_n),
    .dp_valid, .dp_ready, .dp_data,
    .r_valid, .r_ready, .r_data,
    .dr_we, .dr_waddr, .dr_wdata,
    .rx(chk_rx), .idle(chk_idle), .checked(chk_checked), .mism(chk_mism),
    .first_k(chk_first_k), .first_lane(chk_first_lane), .first_got(chk_first_got), .first_exp(chk_first_exp));

endmodule

`default_nettype wire
