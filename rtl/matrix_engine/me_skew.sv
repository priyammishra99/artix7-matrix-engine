// ============================================================================
// File: me_skew.sv
// Block: Matrix engine - operand skew
//
// Delays an operand by N cycles. DSP number d of a cascade chain must see its
// operands exactly one cycle after DSP d-1 so that the partial sum arriving on
// PCIN belongs to the same beat (the cascade's timing). The element
// feeding DSP d therefore passes through a delay that grows by one per
// position.
//
// For N >= 2 the delay is an SRL followed by a flip-flop (srl_style
// "srl_reg"): the SRL's slow clock-to-out stays local, and the output that
// drives the array's distribution copies comes from a fast flip-flop.
//
// No reset on the datapath: lets Vivado map the delay into SRL16/SRL32 LUTs.
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module me_skew #(
  parameter int W = 8,   // operand width
  parameter int N = 0    // delay in cycles
) (
  input  wire logic         clk,
  input  wire logic [W-1:0] din,
  output wire logic [W-1:0] dout
);

  generate
    if (N == 0) begin : g_pass
      assign dout = din;
    end else if (N == 1) begin : g_reg
      logic [W-1:0] q;
      always_ff @(posedge clk) q <= din;
      assign dout = q;
    end else begin : g_delay
      (* shreg_extract = "yes", srl_style = "srl_reg" *) logic [W-1:0] sr [N];
      always_ff @(posedge clk) begin
        sr[0] <= din;
        for (int i = 1; i < N; i++) sr[i] <= sr[i-1];
      end
      assign dout = sr[N-1];
    end
  endgenerate

endmodule

`default_nettype wire
