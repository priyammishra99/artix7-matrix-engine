// ============================================================================
// File: me_if_pkg.sv
// Block: Matrix-engine interface contract, version 2
// Any change to it bumps MEI_VERSION.
//
// This package fixes every signal that crosses the boundary of the matrix
// array (me_array), so that the blocks around it can be built in parallel:
//
//   weight ring + row store --+
//   attention issuer ---------+--> tile walker --> me_array --> drain
//                                                         \----> attention consumer
//
// It contains types and constants only: no logic, nothing to synthesise. The
// protocol rules below are checked in simulation by me_if_checker.sv; every
// testbench on either side of the array should instantiate it.
//
// It covers the array, the GEMM loop order, the weight ring, the attention
// mode (operands, passes, widths), the drain, flow control at issue, the slot
// order of KV pages, the page scan's operands, and the row order in a step.
//
// ----------------------------------------------------------------------------
// 1. The array, in one paragraph
// ----------------------------------------------------------------------------
// 4 rows (R, tokens) x 8 columns (C, output channels) x 16 deep (D, one
// 16-term cascade per cell). Each accepted beat carries ONE stationary slot
// (8 columns x 16 elements, shared by the 4 rows), the 16 pre-adder values
// (one per cascade position, shared by all 32 cells) and 4 rows x 16 moving
// elements. DSP d of cell (r, j) computes (W[j][d] + Dp[d]) * X[r][d], and the
// cell adds its 16 products. A TILE is the run of beats from op_first to
// op_last; the 32 channel accumulators add the cell sums of every beat of the
// tile, and the finished tile (32 accumulators) leaves on the tile port.
//
// ----------------------------------------------------------------------------
// 2. Operand formats per mode (the array itself has no mode input)
// ----------------------------------------------------------------------------
//                    stationary A (W)        pre-adder D (Dp)    moving B (X)        result
//   weight GEMM      INT8 weight             0                   INT8 activation     tiles of K/16 beats -> drain (handshake)
//   attention score  c*r (sealed page) or    15*mn per dimension q16, INT16 query    tiles of 4 beats (64 dims) -> softmax
//                    15*k8 (INT8 page)       (0 for INT8 page)                       lanes (push mode, P10), 34-bit raw scores
//   attention P.V    c*r (sealed) or 15*v8   15*mn per token     p_lo (<= 4,095) or  BYPASS beats (P9), one 16-key sum per
//                                            (0 for INT8 page)   p_hi (<= 2,048)     cell -> C's 51-bit output accumulators
//   page scan        8h + 3 (h: 2-bit high   0                   folded query q8*r,  tiles of 4 beats (push mode)
//                    key code)                                   |.| <= 32,258
// All values are two's complement and sign-extended to the field width.
// Ranges: 0 <= c*r <= 3,810, |15*k8| <= 1,905, |Dp| <= 1,920 (12 bit),
// |A + Dp| <= 1,905 for sealed pages, |X| <= 32,767.  So A is 13 bits, Dp 12,
// X 16; A + Dp is formed by the DSP pre-adder exactly. The stationary
// dequantiser that forms A and Dp from codes and ranges is C's block.
//
// ----------------------------------------------------------------------------
// 3. Element layout inside a beat
// ----------------------------------------------------------------------------
//   w : element (j, d) at index j*16 + d   = W[8g + j][16c + d] in a GEMM
//   dp: element d      at index d
//   x : element (r, d) at index r*16 + d   = X[step row 4*rg + r][16c + d]
// Loop order of a GEMM: output group g outer, row group rg
// next (only when the step has more than 4 rows), input chunk c inner. A tile
// is the K/16 chunks of one (g, rg).
//
// ----------------------------------------------------------------------------
// 4. The tile and its tag
// ----------------------------------------------------------------------------
//   tile: accumulator (r, j) at index r*8 + j, MEI_ACC_W bits, signed.
//         In a GEMM it is the dot product for step row 4*rg + r and output
//         channel 8*g + j. Rows marked idle in the tag hold garbage.
//   tag:  issued by the tile walker with the tile's first beat and delivered
//         with the finished tile, in order (see rule P6).
//
// ----------------------------------------------------------------------------
// 5. Protocol rules (checked by me_if_checker)
// ----------------------------------------------------------------------------
//   P1  op_first, op_last, op_bypass and the payload are meaningful only with
//       op_valid. The array has no ready on its input: a beat offered is a
//       beat taken.
//   P2  Accumulated beats come in tiles: op_first opens a tile, op_last closes
//       it; a tile of one beat has both. No op_first inside an open tile, and
//       no accumulated beat outside one.
//   P3  op_bank is constant over a tile and alternates from tile to tile
//       (0, 1, 0, ...), starting at 0 after reset.
//   P4  CREDITS (handshake mode: weight GEMMs, tile walker -> drain): at most
//       MEI_TILES_IN_FLIGHT (2) tiles in flight. A tile is in flight from its
//       op_first beat until its tile handshake (tile_valid && tile_ready). The
//       walker must not start a third. Not applied in push mode (P10).
//   P5  tile_valid and tile_acc hold steady while tile_valid && !tile_ready.
//   P6  Tiles leave in the order they started; the k-th tile handshake
//       belongs to the k-th op_first. A consumer that needs the tag takes
//       the k-th tag (from the array, or from an in-order FIFO beside it).
//   P7  Latency: the finished tile is offered MEI_TILE_LAT clock edges after
//       the edge that took the tile's last beat (when its bank is free).
//   P8  SHORT TILES IN HANDSHAKE MODE. A credit returns only at the tile
//       handshake, about MEI_TILE_LAT + read latency + consumer delay cycles
//       after the tile started, so with 2 credits the producer runs at full
//       rate only for tiles of about 13 beats or more. Every GEMM of this
//       model has 36 or more (K >= 576), so this costs the GEMMs nothing.
//   P9  BYPASS BEATS (attention P.V, agreed with C, v2). A beat with
//       op_bypass = 1 does not touch the channel accumulators: its 32 cell
//       sums (MEI_SUM_W bits each) leave on the bypass port, byp_valid for
//       one cycle, MEI_BYP_LAT clock edges after the edge that took the beat,
//       with the beat's op_btag. op_first and op_last must be 0 on a bypass
//       beat, and a bypass beat must not fall inside an open tile (passes
//       are interleaved whole: a score or scan tile is never split by P.V
//       beats). The consumer (the attention core's output accumulators) takes
//       every bypass beat in the cycle it appears: there is no ready.
//   P10 PUSH MODE (attention score passes and the page scan, agreed with C,
//       v2). The consumer (the softmax lanes / score staging) holds
//       tile_ready = 1 throughout, so every tile is taken in the cycle it is
//       offered and frees its bank at once. The attention issuer is not bound
//       by P4; it applies flow control AT ISSUE, starting a pass only when its
//       own score / P staging has room for the results (the rule: data
//       never waits). Tiles of any length, one per cycle, run back
//       to back. err_overrun stays as the array's safety net.
//
// ----------------------------------------------------------------------------
// 6. Version history
// ----------------------------------------------------------------------------
// v1 (2026-10-13)  GEMM-verified me_array v0 (8-bit operands, 32-bit
//                  accumulators, no pre-adder input) behind me_v0_adapter.
// v2 (2026-10-14)  me_array widened to these widths (13-bit stationary,
//                  12-bit pre-adder, 16-bit moving, 31-bit chain sums,
//                  34-bit accumulators); bypass beats (P9) and push mode
//                  (P10) for attention; operand distribution
//                  pipelined for timing (two copy levels for the moving
//                  operand: post-route WNS +0.170 ns with one level), so
//                  MEI_TILE_LAT 20 -> 24 and MEI_BYP_LAT 23 (the fill is paid
//                  a few times per layer, about 0.005% of a step; the
//                  score-to-p budget of 56 cycles keeps ~12 cycles of slack). me_v0_adapter removed.
// v3 (2026-10-23)  two build switches for the full-chip placement, both OFF by
//                  default (then everything is exactly v2: latencies 24 / 23):
//                    ME_EAST_REG  one more register stage on every operand
//                                 copy, split west (rows 0-1) / east (rows
//                                 2-3), for the long wires into the east DSP
//                                 columns; +1 to
//                                 MEI_TILE_LAT and MEI_BYP_LAT
//                    ME_TILE_REG  tile_acc leaves a register;
//                                 one tile a cycle still; +1 to MEI_TILE_LAT
//                  Set them on the compiler / synthesis command line (xvlog -d
//                  ME_EAST_REG, synth_design -verilog_define ME_EAST_REG). Every
//                  consumer uses valid/ready and reads the latencies here.
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

package me_if_pkg;

  localparam int MEI_VERSION = 3;

  // ---- build switches (section 6, v3): OFF unless defined on the command line ----
`ifdef ME_EAST_REG
  localparam bit MEI_EAST_REG = 1'b1;
`else
  localparam bit MEI_EAST_REG = 1'b0;
`endif
`ifdef ME_TILE_REG
  localparam bit MEI_TILE_REG = 1'b1;
`else
  localparam bit MEI_TILE_REG = 1'b0;
`endif

  // ---- array shape ----
  localparam int MEI_R = 4;              // rows (tokens)
  localparam int MEI_C = 8;              // columns (output channels)
  localparam int MEI_D = 16;             // cascade depth
  localparam int MEI_NACC = MEI_R * MEI_C;   // 32 accumulators per tile

  // ---- element widths (section 2) ----
  localparam int MEI_A_W   = 13;         // stationary element, DSP A side
  localparam int MEI_DP_W  = 12;         // pre-adder element, DSP D port
  localparam int MEI_X_W   = 16;         // moving element, DSP B port
  localparam int MEI_SUM_W = 31;         // one cell's 16-term sum: 32,767 x 1,905 x 16 < 2^30
  localparam int MEI_ACC_W = 34;         // raw attention score: 32,767 x 1,905 x 64 < 2^32 (+1 spare)

  // GEMM operands are INT8, sign-extended into the fields above
  localparam int MEI_GEMM_W = 8;

  // ---- protocol ----
  localparam int MEI_TILES_IN_FLIGHT = 2;    // rule P4 (handshake mode)
  localparam int MEI_TILE_LAT        = 24 + int'(MEI_EAST_REG) + int'(MEI_TILE_REG);
                                             // rule P7: clock edges, last beat -> tile offered (24: switches off)
  localparam int MEI_BYP_LAT         = 23 + int'(MEI_EAST_REG);
                                             // rule P9: clock edges, bypass beat -> byp_valid (23: switches off)
  localparam int MEI_BTAG_W          = 8;    // rule P9: tag carried with a bypass beat
                                             // (C's use: P.V half, column pass, block slot)

  // ---- beat payload ----
  typedef logic signed [MEI_A_W-1:0]  mei_a_t;
  typedef logic signed [MEI_DP_W-1:0] mei_dp_t;
  typedef logic signed [MEI_X_W-1:0]  mei_x_t;

  typedef logic [MEI_C*MEI_D*MEI_A_W-1:0]  mei_w_bus_t;    // 128 x 13 = 1,664 bits
  typedef logic [MEI_D*MEI_DP_W-1:0]       mei_dp_bus_t;   //  16 x 12 =   192 bits
  typedef logic [MEI_R*MEI_D*MEI_X_W-1:0]  mei_x_bus_t;    //  64 x 16 = 1,024 bits

  // ---- tile tag (section 4) ----
  typedef enum logic [1:0] {
    MEI_DEST_DRAIN    = 2'd0,    // weight GEMM -> drain
    MEI_DEST_SCORE    = 2'd1,    // attention score pass -> attention core
    MEI_DEST_PV       = 2'd2,    // attention P.V pass -> attention core
    MEI_DEST_SCAN     = 2'd3     // page scan -> page selector
  } mei_dest_e;

  typedef struct packed {
    mei_dest_e   dest;           // who consumes the tile
    logic [5:0]  cmd;            // command slot: the consumer looks up
                                 // destination, exponent and flags by it
    logic [12:0] grp;            // output group g (channel 8g .. 8g+7); the head needs 6,144
    logic [2:0]  rgrp;           // row group: array row r = step row 4*rgrp + r
    logic [3:0]  rvalid;         // live rows of this tile (idle rows hold garbage)
    logic [2:0]  sub;            // mode detail: P.V half (0 = lo, 1 = hi), column pass, ...
    logic        last;           // last tile of this command
  } mei_tag_t;                   // 32 bits

  typedef logic [MEI_NACC*MEI_ACC_W-1:0] mei_tile_bus_t;  // 32 x 34 = 1,088 bits
  typedef logic [MEI_NACC*MEI_SUM_W-1:0] mei_byp_bus_t;   // 32 x 31 =   992 bits: cell (r, j) at index r*8 + j
  typedef logic [MEI_BTAG_W-1:0]         mei_btag_t;

  // ---- index helpers: use these instead of open-coded bit positions ----
  function automatic int mei_w_idx(int j, int d);   return j * MEI_D + d;  endfunction
  function automatic int mei_x_idx(int r, int d);   return r * MEI_D + d;  endfunction
  function automatic int mei_acc_idx(int r, int j); return r * MEI_C + j;  endfunction

  function automatic mei_a_t mei_w_get(mei_w_bus_t w, int j, int d);
    return w[mei_w_idx(j, d) * MEI_A_W +: MEI_A_W];
  endfunction
  function automatic mei_x_t mei_x_get(mei_x_bus_t x, int r, int d);
    return x[mei_x_idx(r, d) * MEI_X_W +: MEI_X_W];
  endfunction
  function automatic logic signed [MEI_ACC_W-1:0] mei_acc_get(mei_tile_bus_t t, int r, int j);
    return t[mei_acc_idx(r, j) * MEI_ACC_W +: MEI_ACC_W];
  endfunction
  function automatic logic signed [MEI_SUM_W-1:0] mei_byp_get(mei_byp_bus_t s, int r, int j);
    return s[mei_acc_idx(r, j) * MEI_SUM_W +: MEI_SUM_W];
  endfunction

endpackage

`default_nettype wire
