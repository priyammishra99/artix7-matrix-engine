// ============================================================================
// File: me_cell.sv
// Block: Matrix engine - one (row, column) cell = one D-deep DSP cascade chain
//
// DSP d multiplies (stationary element d + pre-adder element d) of its column
// by moving element d of its row; the products are added along the
// PCOUT -> PCIN cascade. The operands arrive already skewed (element d
// delayed d cycles) and distributed (this cell's own copies), so the chain
// output is the D-term dot product of ONE beat.
//
// Operand order: element d occupies [W*d +: W] of each bus. The pre-adder bus
// is fed one cycle after the stationary bus (me_dsp_mac: A has two registers,
// D one).
// Output `sum` is the last DSP's P, low SUM_W bits; the array registers it.
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module me_cell #(
  parameter int D     = 16,
  parameter int A_W   = 13,
  parameter int DP_W  = 12,
  parameter int X_W   = 16,
  parameter int SUM_W = 31
) (
  input  wire logic                    clk,
  input  wire logic [D*A_W-1:0]        w,     // stationary, skewed
  input  wire logic [D*DP_W-1:0]       dp,    // pre-adder, skewed, one cycle after w
  input  wire logic [D*X_W-1:0]        x,     // moving, skewed
  output wire logic signed [SUM_W-1:0] sum
);

  logic signed [47:0] pc [D];     // P of each stage
  logic        [47:0] pco [D];    // PCOUT of each stage

  generate
    for (genvar d = 0; d < D; d++) begin : g_dsp
      wire logic signed [A_W-1:0]  w_d  = w[A_W*d +: A_W];
      wire logic signed [DP_W-1:0] dp_d = dp[DP_W*d +: DP_W];
      wire logic signed [X_W-1:0]  x_d  = x[X_W*d +: X_W];

      if (d == 0) begin : g_first
        me_dsp_mac #(.FIRST(1'b1)) u_mac (
          .clk  (clk),
          .a    (25'(w_d)),
          .d    (25'(dp_d)),
          .b    (18'(x_d)),
          .pcin (48'd0),
          .p    (pc[d]),
          .pcout(pco[d])
        );
      end else begin : g_cascade
        me_dsp_mac #(.FIRST(1'b0)) u_mac (
          .clk  (clk),
          .a    (25'(w_d)),
          .d    (25'(dp_d)),
          .b    (18'(x_d)),
          .pcin (pco[d-1]),
          .p    (pc[d]),
          .pcout(pco[d])
        );
      end
    end
  endgenerate

  assign sum = pc[D-1][SUM_W-1:0];

endmodule

`default_nettype wire
