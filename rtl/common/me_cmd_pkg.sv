// ============================================================================
// File: me_cmd_pkg.sv
// Block: Matrix engine - the command interface
//
// The 64-bit GEMM command word the engine takes (me_s1_cmd -> me_tile_walker
// -> me_drain), and the codes in it: the opcode, the command flags the engine
// reads, the on-chip buffers a command names, the row sets, and the fault codes
// the engine reports.
// ============================================================================

`timescale 1ns / 1ps

package me_cmd_pkg;

  // the command word (most significant field first)
  typedef struct packed {
    logic [5:0] op;      // OP_*
    logic [9:0] flags;   // CF_* bit positions
    logic [3:0] abuf;    // BUF_*: the source of A
    logic [3:0] dbuf;    // BUF_*: the destination D
    logic [4:0] tag;     // the weight tensor: 0-15 per layer, 16-31 global
    logic [3:0] iclass;  // the activation class A was written at
    logic [3:0] oclass;  // the activation class D is written at
    logic [5:0] layer;   // the layer
    logic [1:0] rowset;  // ROWSET_*
    logic [5:0] doff;    // D's element offset within a staging row, in units of 64 elements
    logic [5:0] aoff;    // A's element offset within a staging row, in units of 64 elements
    logic [6:0] rsv;     // reserved (0)
  } cmd_t;

  // opcode
  localparam logic [4:0] OP_GEMM = 5'h3;

  // command flags: bit positions in cmd_t.flags
  localparam int CF_ACC  = 0;   // add into D (the residual)
  localparam int CF_KV   = 2;   // the output is K or V
  localparam int CF_TOPK = 5;   // the output head (logits)

  // which rows a command runs on
  localparam logic [1:0] ROWSET_ALL   = 2'h0;
  localparam logic [1:0] ROWSET_LOGIT = 2'h1;

  // on-chip buffers a command names
  localparam logic [3:0] BUF_RES   = 4'h1;   // the residual (INT32)
  localparam logic [3:0] BUF_STG16 = 4'h2;   // staging, INT16
  localparam logic [3:0] BUF_STG32 = 4'h3;   // staging, INT32
  localparam logic [3:0] BUF_WIN   = 4'h4;   // the activation window

  // fault codes the engine reports
  localparam logic [6:0] FAULT_NONE        = 7'h00;
  localparam logic [6:0] FAULT_CMD_UNKNOWN = 7'h20;   // not a GEMM
  localparam logic [6:0] FAULT_CMD_FIELD   = 7'h21;   // a field the engine does not support
  localparam logic [6:0] FAULT_TAG         = 7'h23;   // the weight run does not match the command
  localparam logic [6:0] FAULT_SCALE       = 7'h42;   // a malformed scale

endpackage
