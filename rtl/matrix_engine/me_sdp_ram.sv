// ============================================================================
// File: me_sdp_ram.sv
// Block: Matrix engine - simple dual-port block RAM (one write, one read port)
//
// Inferred block RAM with a byte-wide write enable and the block RAM's output
// register: read data 2 cycles after rd (address registered in the RAM, then
// the output register), the row store's and the walker's read latency.
// Write and read the same address in one cycle: the read returns the old data
// (READ_FIRST), but the row store never does it.
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module me_sdp_ram #(
  parameter int W     = 64,          // data width (a multiple of 8)
  parameter int DEPTH = 512,
  parameter int AW    = $clog2(DEPTH)
) (
  input  wire logic             clk,
  input  wire logic             we,
  input  wire logic [W/8-1:0]   wbe,             // byte enables
  input  wire logic [AW-1:0]    waddr,
  input  wire logic [W-1:0]     wdata,
  input  wire logic             rd,
  input  wire logic [AW-1:0]    raddr,
  output logic      [W-1:0]     rdata            // 2 cycles after rd
);
  (* ram_style = "block" *) logic [W-1:0] mem [DEPTH];
  logic [W-1:0] q1;

  always_ff @(posedge clk) begin
    if (we)
      for (int b = 0; b < W / 8; b++)
        if (wbe[b]) mem[waddr][8*b +: 8] <= wdata[8*b +: 8];
  end
  always_ff @(posedge clk) begin
    if (rd) q1 <= mem[raddr];
    rdata <= q1;                                 // the block RAM's output register
  end
endmodule

`default_nettype wire
