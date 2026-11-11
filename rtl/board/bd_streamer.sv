// ============================================================================
// File: bd_streamer.sv
// Block: AC701 board shell (matrix engine only) - ROM to beat stream
//
// On start, streams ROM entries base .. base + count - 1 out on a valid/ready
// port, one a cycle while the receiver takes them: the board's stand-in for a
// DDR read. Reads are issued only while the 4-entry FIFO has room for every
// read in flight (the ROM answers 2 cycles later), so nothing is ever dropped.
// A start while busy is ignored.
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module bd_streamer #(
  parameter int W  = 512,
  parameter int AW = 14
) (
  input  wire logic          clk,
  input  wire logic          rst_n,          // synchronous, active low
  input  wire logic          start,
  input  wire logic [AW-1:0] base,
  input  wire logic [15:0]   count,
  output logic               busy,

  // the ROM (bd_rom: latency 2)
  output logic               rom_en,
  output logic [AW-1:0]      rom_addr,
  input  wire logic [W-1:0]  rom_q,

  // the stream
  output logic               o_valid,
  input  wire logic          o_ready,
  output logic [W-1:0]       o_data
);

  logic [15:0]  left;                      // reads still to issue
  logic         v1, v2;                    // reads in flight, by age
  logic [W-1:0] fifo [4];
  logic [1:0]   wp, rp;
  logic [2:0]   cnt;

  wire logic issue = (left != 16'd0) && (3'(cnt) + 3'(v1) + 3'(v2) < 3'd4);
  wire logic pop   = o_valid && o_ready;

  assign rom_en  = issue;
  assign o_valid = (cnt != 3'd0);
  assign o_data  = fifo[rp];
  assign busy    = (left != 16'd0) || v1 || v2 || (cnt != 3'd0);

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      left <= '0; rom_addr <= '0; v1 <= 1'b0; v2 <= 1'b0; wp <= '0; rp <= '0; cnt <= '0;
    end else begin
      if (start && !busy) begin
        left <= count; rom_addr <= base;
      end else if (issue) begin
        left <= left - 16'd1; rom_addr <= rom_addr + AW'(1);
      end
      v1 <= issue;
      v2 <= v1;
      if (v2) wp <= wp + 2'd1;
      if (pop) rp <= rp + 2'd1;
      cnt <= cnt + 3'(v2) - 3'(pop);
    end
  end

  always_ff @(posedge clk) if (v2) fifo[wp] <= rom_q;

endmodule

`default_nettype wire
