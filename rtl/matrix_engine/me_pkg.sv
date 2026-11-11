// ============================================================================
// File: me_pkg.sv
// Project: artix7-matrix-engine (Artix-7 XC7A200T-2FBG676C)
// Block:  Matrix engine - shared constants (the array, the weight ring and the
//         row store); widths from the interface contract me_if_pkg (v2) -
//         compile me_if_pkg.sv first.
//
// The array is R4 x C8 x D16 = 512 DSP48E1:
//   R  rows     : tokens processed together; they share every weight slot
//   C  columns  : output channels computed each cycle
//   D  depth    : length of one DSP cascade chain = inputs summed per cycle
// ============================================================================

`timescale 1ns / 1ps

package me_pkg;
  import me_if_pkg::*;

  localparam int ME_R      = MEI_R;      // 4 rows per pass
  localparam int ME_C      = MEI_C;      // 8 output channels per cycle
  localparam int ME_D      = MEI_D;      // 16: cascade depth (inputs per chain)

  // Operand widths (me_if_pkg section 2):
  //   stationary A  13 bits: INT8 weight | c*r <= 3,810 | 15*k8 | 8h+3
  //   pre-adder D   12 bits: 15*mn, |.| <= 1,920 (0 in GEMM and the scan)
  //   moving B      16 bits: INT8 activation | q16 | p half | folded query
  localparam int ME_A_W    = MEI_A_W;    // 13
  localparam int ME_DP_W   = MEI_DP_W;   // 12
  localparam int ME_X_W    = MEI_X_W;    // 16

  // One cell's 16-term sum: worst case is the attention score pass,
  //   16 x 32,767 x 1,905 = 998,738,160 < 2^30  -> 31 bits signed.
  // (GEMM: 16 x 127 x 127 < 2^18; P.V half: 16 x 4,095 x 1,905 < 2^27.)
  localparam int ME_SUM_W  = MEI_SUM_W;  // 31

  // Channel accumulator: raw score over 64 dims, 4 x 998,738,160 < 2^32
  // -> 33 bits signed + 1 spare = 34. GEMM dot products over
  // K = 1,536 stay below 2^25.
  localparam int ME_ACC_W  = MEI_ACC_W;  // 34

  localparam int ME_BTAG_W = MEI_BTAG_W; // 8: tag carried with a bypass beat

  // Cycles from an operand beat entering the array to its chain sum sitting
  // in the capture register:
  //   1 input register + (D-1) skew + 2 distribution levels in fabric,
  //   4 DSP stages (A1/B1, A2/B2, M, P), 1 capture register   = D + 7 = 23
  // The tile is offered one edge later (MEI_TILE_LAT = 24), a bypass result at
  // ME_LAT (MEI_BYP_LAT = 23): the numbers with the build switches off.
  // ME_EAST_REG adds 1 to ME_LAT and so to both; ME_TILE_REG adds 1 to the
  // tile's only (26 / 24 with both on). me_if_pkg computes them.
  localparam int ME_LAT    = ME_D + 7 + int'(MEI_EAST_REG);   // + the east-half stage when switched on

endpackage
