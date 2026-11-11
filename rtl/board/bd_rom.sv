// ============================================================================
// File: bd_rom.sv
// Block: AC701 board shell (matrix engine only) - a ROM in block RAM
//
// Holds one of the board run's data sets (weights, windows, references), loaded
// from gen_board_roms.py's files at configuration. Read latency 2: the address
// with en at edge t, q valid after edge t + 2.
//
// BANK = 0: one memory, from INIT (q0 the BRAM's output, q its output register).
// BANK > 0: banks of BANK entries, bank b from <INIT_PRE>_b<b>.mem (b in decimal,
// no leading zero: Vivado ignores a field width in $sformatf),
// only the addressed bank enabled; q is the addressed bank's output. A deep
// memory in one piece is mapped by Vivado as 16K x 2 BRAMs (10,404 x 512 bits:
// 256 BRAM36); in banks of 1,024 it is mapped as 1K x 36 (15 a bank: 165).
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module bd_rom #(
  parameter int    W        = 512,
  parameter int    DEPTH    = 1024,
  parameter string INIT     = "",
  parameter int    BANK     = 0,
  parameter string INIT_PRE = ""
) (
  input  wire logic                     clk,
  input  wire logic                     en,
  input  wire logic [$clog2(DEPTH)-1:0] addr,
  output logic      [W-1:0]             q
);

  localparam int AW = $clog2(DEPTH);

  if (BANK == 0) begin : g_one
    (* rom_style = "block" *) logic [W-1:0] mem [DEPTH];
    initial if (INIT != "") $readmemh(INIT, mem);
    logic [W-1:0] q0;
    always_ff @(posedge clk) begin
      if (en) q0 <= mem[addr];
      q <= q0;
    end
  end else begin : g_banks
    localparam int NB = (DEPTH + BANK - 1) / BANK;
    localparam int BW = $clog2(BANK);
    logic [W-1:0] qb [NB];
    logic [AW-BW-1:0] sel_q;
    for (genvar b = 0; b < NB; b++) begin : g_b
      (* rom_style = "block" *) logic [W-1:0] mem [BANK];
      initial $readmemh($sformatf("%s_b%0d.mem", INIT_PRE, b), mem);
      always_ff @(posedge clk) if (en && addr[AW-1:BW] == (AW-BW)'(b)) qb[b] <= mem[addr[BW-1:0]];
    end
    always_ff @(posedge clk) begin
      if (en) sel_q <= addr[AW-1:BW];
      q <= qb[sel_q];
    end
  end

endmodule

`default_nettype wire
