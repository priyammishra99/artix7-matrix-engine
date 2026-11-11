// ============================================================================
// File: bd_checker.sv
// Block: AC701 board shell (matrix engine only) - the on-chip result check
//
// Takes me_top's dump stream (the staging rows, 96 beats a row, 16 words a
// beat) and compares, bit for bit, every beat that holds the GEMM's outputs
// (beats rel_lo .. rel_lo + rel_n - 1 of each row: 32 INT16 outputs a beat)
// with the next beat of the reference stream (gen_board_roms.py's bd_ref.mem,
// in the same order). Other staging words are not the GEMM's and are skipped,
// as tb_me_top_s1 skips them.
//
// Pipeline: the handshake registers the dump beat and its reference (stage 1;
// also the dump RAM's write), stage 2 compares the 32 lanes, stage 3 counts and
// keeps the first difference (its output beat, lane, and both values).
// dp_ready waits for the reference beat only on a beat that is checked.
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module bd_checker (
  input  wire logic          clk,
  input  wire logic          rst_n,          // synchronous, active low
  input  wire logic          clr,            // a run starts: all counts cleared

  input  wire logic          arm,            // the write engine's request was taken: n beats come
  input  wire logic [15:0]   n,
  input  wire logic [6:0]    rel_lo,
  input  wire logic [6:0]    rel_n,

  // the dump stream (from me_top)
  input  wire logic          dp_valid,
  output logic               dp_ready,
  input  wire logic [511:0]  dp_data,

  // the reference stream (bd_streamer on the reference ROM)
  input  wire logic          r_valid,
  output logic               r_ready,
  input  wire logic [511:0]  r_data,

  // the dump RAM (the checked beats, by output beat)
  output logic               dr_we,
  output logic [9:0]         dr_waddr,
  output logic [511:0]       dr_wdata,

  // results
  output logic [15:0]        rx,             // dump beats taken
  output logic               idle,           // armed and all n taken and compared, or never armed
  output logic [31:0]        checked,        // output beats compared
  output logic [31:0]        mism,           // output beats that differ
  output logic [15:0]        first_k,        // the first differing output beat
  output logic [4:0]         first_lane,     // its first differing lane (channel 32 (k mod rel_n) + lane)
  output logic [15:0]        first_got,
  output logic [15:0]        first_exp
);

  logic         armed;
  logic [15:0]  n_q;
  logic [6:0]   bi, lo_q, hi_q;            // beat within the staging row
  logic [15:0]  k;                         // output beats seen

  wire logic rel = (bi >= lo_q) && (bi < hi_q);
  assign dp_ready = armed && (rx != n_q) && (!rel || r_valid);
  wire logic hs   = dp_valid && dp_ready;
  assign r_ready  = hs && rel;

  // stage 1: the beat and its reference
  logic         s1_v;  logic [15:0] s1_k;  logic [511:0] s1_got, s1_exp;
  // stage 2: the lane compare
  logic         s2_v;  logic [15:0] s2_k;  logic [511:0] s2_got, s2_exp;  logic [31:0] s2_bad;

  always_ff @(posedge clk) begin
    if (!rst_n || clr) begin
      armed <= 1'b0; n_q <= '0; lo_q <= '0; hi_q <= '0; bi <= '0; k <= '0; rx <= '0;
      s1_v <= 1'b0; s2_v <= 1'b0;
      checked <= '0; mism <= '0; first_k <= '0; first_lane <= '0; first_got <= '0; first_exp <= '0;
    end else begin
      if (arm && !armed) begin
        armed <= 1'b1; n_q <= n; lo_q <= rel_lo; hi_q <= rel_lo + rel_n;
      end
      if (hs) begin
        rx <= rx + 16'd1;
        bi <= (bi == 7'd95) ? 7'd0 : bi + 7'd1;
        if (rel) k <= k + 16'd1;
      end
      s1_v <= hs && rel;
      s2_v <= s1_v;
      if (s2_v) begin
        checked <= checked + 32'd1;
        if (s2_bad != '0) begin
          mism <= mism + 32'd1;
          if (mism == '0) begin
            first_k <= s2_k;
            for (int j = 31; j >= 0; j--)
              if (s2_bad[j]) begin
                first_lane <= 5'(j); first_got <= s2_got[16*j +: 16]; first_exp <= s2_exp[16*j +: 16];
              end
          end
        end
      end
    end
  end

  // data registers (no reset: data only)
  always_ff @(posedge clk) begin
    if (hs) begin s1_got <= dp_data; s1_exp <= r_data; s1_k <= k; end
    s2_got <= s1_got; s2_exp <= s1_exp; s2_k <= s1_k;
    for (int j = 0; j < 32; j++) s2_bad[j] <= (s1_got[16*j +: 16] != s1_exp[16*j +: 16]);
  end

  assign dr_we    = s1_v;
  assign dr_waddr = s1_k[9:0];
  assign dr_wdata = s1_got;
  assign idle     = !armed || ((rx == n_q) && !s1_v && !s2_v);

endmodule

`default_nettype wire
