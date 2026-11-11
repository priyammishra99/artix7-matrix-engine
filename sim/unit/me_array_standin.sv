// ============================================================================
// File: me_array_standin.sv
// Block: Matrix engine - SIMULATION ONLY: a behavioural stand-in for me_array
//
// The same module name and ports as rtl/matrix_engine/me_array.sv, compiled in
// its place (never with it) for fast top-level simulations: the same dot
// products, tiles offered in order MEI_TILE_LAT edges after their last beat
// and held until taken (contract rules P5-P7), two banks (at most 2 tiles
// held: P4's credits keep it so). No bypass beats (GEMM only). The real
// array, 512 DSP48E1 models, runs at about 730 cycles a minute; this one at
// thousands of times that. Taken from tb_me_tile_walker's g_model.
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module me_array
  import me_pkg::*;
  import me_if_pkg::*;
#(
  parameter int R      = ME_R,
  parameter int C      = ME_C,
  parameter int D      = ME_D,
  parameter int A_W    = ME_A_W,
  parameter int DP_W   = ME_DP_W,
  parameter int X_W    = ME_X_W,
  parameter int SUM_W  = ME_SUM_W,
  parameter int ACC_W  = ME_ACC_W,
  parameter int BTAG_W = ME_BTAG_W
) (
  input  wire logic                   clk,
  input  wire logic                   rst_n,
  input  wire logic                   op_valid,
  input  wire logic                   op_first,
  input  wire logic                   op_last,
  input  wire logic                   op_bank,
  input  wire logic                   op_bypass,
  input  wire logic [BTAG_W-1:0]      op_btag,
  input  wire logic [C*D*A_W-1:0]     op_w,
  input  wire logic [D*DP_W-1:0]      op_dp,
  input  wire logic [R*D*X_W-1:0]     op_x,
  output wire logic                   tile_valid,
  input  wire logic                   tile_ready,
  output wire logic [R*C*ACC_W-1:0]   tile_acc,
  output wire logic                   byp_valid,
  output wire logic [R*C*SUM_W-1:0]   byp_sum,
  output wire logic [BTAG_W-1:0]      byp_tag,
  output logic                        err_overrun,
  output logic                        err_bypass,
  output logic [31:0]                 perf_beats,
  output logic [31:0]                 perf_tiles,
  output logic [31:0]                 perf_byp
);

  typedef struct { mei_tile_bus_t acc; int ready_at; } mtile_t;
  mtile_t        q[$];
  logic signed [MEI_ACC_W-1:0] acc [32];
  int            m_cyc = 0;
  logic          m_valid = 1'b0;
  mei_tile_bus_t m_acc = '0;

  assign tile_valid = m_valid;
  assign tile_acc   = m_acc;
  assign byp_valid  = 1'b0;
  assign byp_sum    = '0;
  assign byp_tag    = '0;
  assign err_bypass = op_valid && op_bypass;
  assign perf_byp   = '0;

  always @(posedge clk) begin
    m_cyc++;
    if (rst_n) begin
      if (m_valid && tile_ready) begin void'(q.pop_front()); perf_tiles <= perf_tiles + 1; end
      if (op_valid) begin
        perf_beats <= perf_beats + 1;
        for (int i = 0; i < 32; i++) begin
          logic signed [MEI_ACC_W-1:0] s;
          s = 0;
          for (int d = 0; d < 16; d++)
            s += (mei_w_get(op_w, i % 8, d) + $signed(op_dp[d*MEI_DP_W +: MEI_DP_W])) * mei_x_get(op_x, i / 8, d);
          acc[i] = (op_first ? 0 : acc[i]) + s;
        end
        if (op_last) begin
          mtile_t t;
          for (int i = 0; i < 32; i++) t.acc[i*MEI_ACC_W +: MEI_ACC_W] = acc[i];
          t.ready_at = m_cyc + MEI_TILE_LAT;
          if (q.size() >= 2) err_overrun <= 1'b1;            // a third tile: both banks were full
          q.push_back(t);
        end
      end
      m_valid <= (q.size() > 0) && (q[0].ready_at <= m_cyc + 1);
      if (q.size() > 0) m_acc <= q[0].acc;
    end else begin                                         // a reset drops the tiles in flight
      q.delete();
      m_valid     <= 1'b0;
      err_overrun <= 1'b0;
      perf_beats  <= '0;
      perf_tiles  <= '0;
    end
  end

endmodule

`default_nettype wire
