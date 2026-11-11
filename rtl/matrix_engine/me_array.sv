// ============================================================================
// File: me_array.sv
// Project: artix7-matrix-engine (Artix-7 XC7A200T-2FBG676C)
// Block:  Matrix engine - array core, interface contract v2 (me_if_pkg):
//         tiles, double-banked channel accumulators, attention mode (operands,
//         passes, widths), page scan
//
// R x C x D DSP48E1 (4 x 8 x 16 = 512). Each beat ONE stationary slot
// (C columns x D elements, shared by all R rows) and the D pre-adder values
// (one per cascade position, shared by all R*C cells) meet R rows of D moving
// elements: DSP d of cell (r, j) computes (W[j][d] + Dp[d]) * X[r][d], and
// each cell sums its D products along the cascade.
//
// ---- Beat port (no stalls inside the array) --------------------------------
//   op_valid   a beat is presented this cycle
//   op_first   first beat of a tile (accumulators start from this beat)
//   op_last    last beat of a tile  (the tile completes with this beat)
//   op_bank    accumulator bank of this tile (0/1), alternated per tile
//   op_bypass  1: a bypass beat (attention P.V, rule P9): its R*C cell sums
//              leave on the bypass port and the accumulators are not touched
//   op_btag    tag returned with a bypass beat's sums
//   op_w       stationary: element (j*D + d), A_W bits signed
//   op_dp      pre-adder:  element d, DP_W bits signed (0 in GEMM / scan)
//   op_x       moving:     element (r*D + d), X_W bits signed
//
// ---- Tile port (valid/ready) -----------------------------------------------
//   tile_valid  a finished tile is held; tile_acc is stable while it is high
//   tile_ready  taken (one cycle); frees its bank. Handshake mode (GEMM) uses
//               2 credits (P4); push mode (attention) holds it at 1 (P10).
//   tile_acc    accumulator (r, j) at [(r*C + j)*ACC_W +: ACC_W], signed
//
// ---- Bypass port (no ready) ------------------------------------------------
//   byp_valid   one cycle per bypass beat, in beat order
//   byp_sum     cell (r, j) sum at [(r*C + j)*SUM_W +: SUM_W], signed
//   byp_tag     the beat's op_btag
//
// ---- Timing ----------------------------------------------------------------
//   Beat taken at edge P -> chain sum in the capture register after P+LAT-1
//   (LAT = D + 7 = 23) -> byp_valid seen at P+LAT (MEI_BYP_LAT = 23) or the
//   accumulators updated at P+LAT and the tile seen at P+LAT+1 (MEI_TILE_LAT
//   = 24). These are the numbers with the build switches off; ME_EAST_REG adds
//   1 to both (LAT = D + 8), ME_TILE_REG adds 1 to the tile's (the output
//   register): 26 / 24 with both on. me_if_pkg computes them.
//
// ---- Operand distribution (timing) -----------------------------------------
//   Every element passes: input register + skew (shared, ending in a
//   flip-flop) -> fabric copies per group of cells -> each DSP's own A1/B1
//   register. The moving operand (each row's value goes to all 8 columns) has
//   two copy levels: 2, then one per column. The pre-adder value (all 32
//   cells) has one copy per column, then one per cell, in place of A1/B1.
//   The weights have one level (2 copies). So no hop crosses the whole array
//   with a large fan-out. The copies carry dont_touch so that synthesis keeps
//   them apart. (v2 first cut, one x level: post-route WNS +0.170 ns.)
//
// ---- Build switches (me_if_pkg v3; both OFF by default = the v2 array) -----
//   ME_EAST_REG  one more register stage on every operand copy after its last
//                distribution level, split west (rows 0-1) / east (rows 2-3)
//                for the moving operand and the pre-adder, so each half's copy
//                sits by its own DSP columns (the full-chip placement puts rows
//                2-3 on the east columns). All four rows get the cycle, so they
//                stay aligned: latency +1 (tile and bypass).
//   ME_TILE_REG  tile_acc leaves a register: a full bank moves
//                into it when it is empty or being taken, so push mode still
//                takes one tile a cycle; the bank is free once moved. Tile
//                latency +1.
//
// Datapath registers have no reset (they pack into DSP/SRL structures); only
// the control state resets (synchronous, active low).
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module me_array
  import me_pkg::*, me_if_pkg::*;
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

  // operand beat
  input  wire logic                   op_valid,
  input  wire logic                   op_first,
  input  wire logic                   op_last,
  input  wire logic                   op_bank,
  input  wire logic                   op_bypass,
  input  wire logic [BTAG_W-1:0]      op_btag,
  input  wire logic [C*D*A_W-1:0]     op_w,
  input  wire logic [D*DP_W-1:0]      op_dp,
  input  wire logic [R*D*X_W-1:0]     op_x,

  // finished tile
  output wire logic                   tile_valid,
  input  wire logic                   tile_ready,
  output wire logic [R*C*ACC_W-1:0]   tile_acc,

  // bypass results
  output wire logic                   byp_valid,
  output wire logic [R*C*SUM_W-1:0]   byp_sum,
  output wire logic [BTAG_W-1:0]      byp_tag,

  // status
  output logic                        err_overrun,   // a tile started/finished in a full bank
  output logic                        err_bypass,    // a bypass beat inside a tile (P9)
  output logic [31:0]                 perf_beats,    // beats accepted
  output logic [31:0]                 perf_tiles,    // tiles completed
  output logic [31:0]                 perf_byp       // bypass beats completed
);

  localparam int LAT = D + 7 + int'(MEI_EAST_REG);
  localparam int XG  = 2;               // x first-level copies: column groups of C/XG cells
  localparam int WG  = 2;               // w copies: row groups of R/WG cells

  // --------------------------------------------------------------------------
  // 1. Input register + skew (shared). Element d is delayed d + 2 cycles for
  //    w and dp, and d + 1 for x, whose extra cycle is its second copy level
  //    (section 2); so all three reach their DSP registers aligned.
  // --------------------------------------------------------------------------
  logic [C*D*A_W-1:0]  w_sh;
  logic [D*DP_W-1:0]   dp_sh;
  logic [R*D*X_W-1:0]  x_sh;

  generate
    for (genvar j = 0; j < C; j++) begin : g_wsk_col
      for (genvar d = 0; d < D; d++) begin : g_wsk_d
        me_skew #(.W(A_W), .N(d + 2)) u_skew (
          .clk (clk),
          .din (op_w[(j*D + d)*A_W +: A_W]),
          .dout(w_sh[(j*D + d)*A_W +: A_W])
        );
      end
    end
    for (genvar d = 0; d < D; d++) begin : g_dpsk_d
      me_skew #(.W(DP_W), .N(d + 2)) u_skew (
        .clk (clk),
        .din (op_dp[d*DP_W +: DP_W]),
        .dout(dp_sh[d*DP_W +: DP_W])
      );
    end
    for (genvar r = 0; r < R; r++) begin : g_xsk_row
      for (genvar d = 0; d < D; d++) begin : g_xsk_d
        me_skew #(.W(X_W), .N(d + 1)) u_skew (
          .clk (clk),
          .din (op_x[(r*D + d)*X_W +: X_W]),
          .dout(x_sh[(r*D + d)*X_W +: X_W])
        );
      end
    end
  endgenerate

  // --------------------------------------------------------------------------
  // 2. Distribution copies (L1 for all; L2 for the pre-adder)
  // --------------------------------------------------------------------------
  //   w : WG copies, copy h feeds rows h*R/WG .. (h+1)*R/WG-1 (A1 is the last hop)
  //   x : XG copies (L1), then one copy per column (L2), copy j feeding the
  //       R cells of column j (B1 is the last hop): the moving operand, which
  //       each row sends to all C columns, never crosses the array in one hop
  //   dp: one L1 copy per column, then one L2 copy per cell (D register last)
  logic [C*D*A_W-1:0]  w_l1  [WG];
  logic [R*D*X_W-1:0]  x_l1  [XG];
  logic [R*D*X_W-1:0]  x_l2  [C];
  logic [D*DP_W-1:0]   dp_l1 [C];
  logic [D*DP_W-1:0]   dp_l2 [R*C];
  // what the cells take: the copies above, or (ME_EAST_REG) one register later (section 2b)
  logic [C*D*A_W-1:0]  w_c   [WG];
  logic [R*D*X_W-1:0]  x_c   [C];
  logic [D*DP_W-1:0]   dp_s  [R*C];                   // the source of each cell's dp_l2 copy

  generate
    for (genvar h = 0; h < WG; h++) begin : g_wl1
      (* dont_touch = "true" *) logic [C*D*A_W-1:0] q;
      always_ff @(posedge clk) q <= w_sh;
      assign w_l1[h] = q;
    end
    for (genvar h = 0; h < XG; h++) begin : g_xl1
      (* dont_touch = "true" *) logic [R*D*X_W-1:0] q;
      always_ff @(posedge clk) q <= x_sh;
      assign x_l1[h] = q;
    end
    for (genvar j = 0; j < C; j++) begin : g_xl2
      (* dont_touch = "true" *) logic [R*D*X_W-1:0] q;
      always_ff @(posedge clk) q <= x_l1[j / (C/XG)];
      assign x_l2[j] = q;
    end
    for (genvar j = 0; j < C; j++) begin : g_dpl1
      (* dont_touch = "true" *) logic [D*DP_W-1:0] q;
      always_ff @(posedge clk) q <= dp_sh;
      assign dp_l1[j] = q;
    end
    for (genvar r = 0; r < R; r++) begin : g_dpl2_row
      for (genvar j = 0; j < C; j++) begin : g_dpl2_col
        (* dont_touch = "true" *) logic [D*DP_W-1:0] q;
        always_ff @(posedge clk) q <= dp_s[r*C + j];
        assign dp_l2[r*C + j] = q;
      end
    end
  endgenerate

  // --------------------------------------------------------------------------
  // 2b. The east-half stage (ME_EAST_REG): one more register on every copy, the
  //     moving operand and the pre-adder split west (rows 0-1) / east (rows 2-3)
  // --------------------------------------------------------------------------
  localparam int HX = (R/2)*D*X_W;                    // one half's rows of a moving-operand copy

  generate
    if (MEI_EAST_REG) begin : g_east
      for (genvar h = 0; h < WG; h++) begin : g_w
        (* dont_touch = "true" *) logic [C*D*A_W-1:0] q;
        always_ff @(posedge clk) q <= w_l1[h];
        assign w_c[h] = q;
      end
      for (genvar j = 0; j < C; j++) begin : g_x
        (* dont_touch = "true" *) logic [HX-1:0] qw, qe;
        always_ff @(posedge clk) begin
          qw <= x_l2[j][0  +: HX];
          qe <= x_l2[j][HX +: HX];
        end
        assign x_c[j] = {qe, qw};
      end
      for (genvar j = 0; j < C; j++) begin : g_dp
        (* dont_touch = "true" *) logic [D*DP_W-1:0] qw, qe;
        always_ff @(posedge clk) begin
          qw <= dp_l1[j];
          qe <= dp_l1[j];
        end
        for (genvar r = 0; r < R; r++) begin : g_row
          assign dp_s[r*C + j] = (r < R/2) ? qw : qe;
        end
      end
    end else begin : g_west_only
      for (genvar h = 0; h < WG; h++) begin : g_w
        assign w_c[h] = w_l1[h];
      end
      for (genvar j = 0; j < C; j++) begin : g_x
        assign x_c[j] = x_l2[j];
        for (genvar r = 0; r < R; r++) begin : g_row
          assign dp_s[r*C + j] = dp_l1[j];
        end
      end
    end
  endgenerate

  // --------------------------------------------------------------------------
  // 3. R*C cascade chains
  // --------------------------------------------------------------------------
  logic [R*C*SUM_W-1:0] sum_raw;

  generate
    for (genvar r = 0; r < R; r++) begin : g_row
      for (genvar j = 0; j < C; j++) begin : g_col
        me_cell #(.D(D), .A_W(A_W), .DP_W(DP_W), .X_W(X_W), .SUM_W(SUM_W)) u_cell (
          .clk(clk),
          .w  (w_c[r / (R/WG)][j*D*A_W +: D*A_W]),
          .dp (dp_l2[r*C + j]),
          .x  (x_c[j][r*D*X_W +: D*X_W]),
          .sum(sum_raw[(r*C + j)*SUM_W +: SUM_W])
        );
      end
    end
  endgenerate

  // --------------------------------------------------------------------------
  // 4. Capture register (first fabric register after the DSP P registers)
  // --------------------------------------------------------------------------
  logic signed [SUM_W-1:0] sum_cap [R*C];

  always_ff @(posedge clk) begin
    for (int i = 0; i < R*C; i++)
      sum_cap[i] <= sum_raw[i*SUM_W +: SUM_W];
  end

  // --------------------------------------------------------------------------
  // 5. Control tags travel with the beat: LAT registers
  // --------------------------------------------------------------------------
  logic [LAT-1:0]    v_pipe;                         // reset: a stale valid is fatal
  logic [LAT-1:0]    f_pipe, l_pipe, b_pipe, y_pipe; // qualified by v_pipe, no reset
  logic [BTAG_W-1:0] t_pipe [LAT];

  always_ff @(posedge clk) begin
    if (!rst_n) v_pipe <= '0;
    else        v_pipe <= {v_pipe[LAT-2:0], op_valid};
    f_pipe <= {f_pipe[LAT-2:0], op_first};
    l_pipe <= {l_pipe[LAT-2:0], op_last};
    b_pipe <= {b_pipe[LAT-2:0], op_bank};
    y_pipe <= {y_pipe[LAT-2:0], op_bypass};
    t_pipe[0] <= op_btag;
    for (int i = 1; i < LAT; i++) t_pipe[i] <= t_pipe[i-1];
  end

  wire logic v_al = v_pipe[LAT-1];
  wire logic f_al = f_pipe[LAT-1];
  wire logic l_al = l_pipe[LAT-1];
  wire logic b_al = b_pipe[LAT-1];
  wire logic y_al = y_pipe[LAT-1];
  wire logic a_al = v_al && !y_al;                  // an accumulated beat

  // --------------------------------------------------------------------------
  // 6. Bypass port: the captured cell sums, as they are (rule P9)
  // --------------------------------------------------------------------------
  assign byp_valid = v_al && y_al;
  assign byp_tag   = t_pipe[LAT-1];
  generate
    for (genvar i = 0; i < R*C; i++) begin : g_byp
      assign byp_sum[i*SUM_W +: SUM_W] = sum_cap[i];
    end
  endgenerate

  // --------------------------------------------------------------------------
  // 7. Channel accumulators, two banks. One shared adder per channel; the
  //    selected bank's value is the addend (zero on the first beat).
  //    use_dsp "no": keep the DSP count at exactly the array's R*C*D.
  // --------------------------------------------------------------------------
  (* use_dsp = "no" *) logic signed [ACC_W-1:0] acc0 [R*C];
  (* use_dsp = "no" *) logic signed [ACC_W-1:0] acc1 [R*C];
  (* use_dsp = "no" *) logic signed [ACC_W-1:0] acc_nxt [R*C];

  always_comb begin
    for (int i = 0; i < R*C; i++)
      acc_nxt[i] = (f_al ? '0 : (b_al ? acc1[i] : acc0[i])) + ACC_W'(sum_cap[i]);
  end

  always_ff @(posedge clk) begin
    if (a_al) begin
      for (int i = 0; i < R*C; i++) begin
        if (!b_al) acc0[i] <= acc_nxt[i];
        else       acc1[i] <= acc_nxt[i];
      end
    end
  end

  // --------------------------------------------------------------------------
  // 8. Tile bookkeeping: full flags, head pointer, error detection
  // --------------------------------------------------------------------------
  logic [1:0] full;
  logic       head;
  logic       open_al;                              // a tile is open at the accumulators
  logic       pop_bank;                             // the head bank is emptied this edge

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      full        <= 2'b00;
      head        <= 1'b0;
      open_al     <= 1'b0;
      err_overrun <= 1'b0;
      err_bypass  <= 1'b0;
      perf_beats  <= '0;
      perf_tiles  <= '0;
      perf_byp    <= '0;
    end else begin
      if (op_valid) perf_beats <= perf_beats + 1'b1;

      if (pop_bank) begin
        full[head] <= 1'b0;
        head       <= ~head;
      end
      if (a_al && l_al) begin
        full[b_al] <= 1'b1;                         // set wins over a same-cycle clear
        perf_tiles <= perf_tiles + 1'b1;
      end
      if (a_al) begin
        if (l_al)      open_al <= 1'b0;
        else if (f_al) open_al <= 1'b1;
      end
      if (byp_valid) perf_byp <= perf_byp + 1'b1;

      // starting, or finishing, a tile in a bank that still holds one
      if (a_al && (f_al || l_al) && full[b_al] && !(pop_bank && head == b_al))
        err_overrun <= 1'b1;
      // a bypass beat marked first/last, or inside an open tile
      if (v_al && y_al && (f_al || l_al || open_al))
        err_bypass <= 1'b1;
    end
  end

  // --------------------------------------------------------------------------
  // 9. Tile output: the oldest full bank, directly, or (ME_TILE_REG) through
  //    an output register it moves into when that is empty or being taken
  // --------------------------------------------------------------------------
  generate
    if (MEI_TILE_REG) begin : g_out_reg
      logic                    out_v;
      logic signed [ACC_W-1:0] out_acc [R*C];
      assign pop_bank   = full[head] && (!out_v || tile_ready);
      assign tile_valid = out_v;
      always_ff @(posedge clk) begin
        if (!rst_n)        out_v <= 1'b0;
        else if (pop_bank) out_v <= 1'b1;
        else if (tile_ready) out_v <= 1'b0;
        if (pop_bank)
          for (int i = 0; i < R*C; i++) out_acc[i] <= head ? acc1[i] : acc0[i];
      end
      for (genvar i = 0; i < R*C; i++) begin : g_out
        assign tile_acc[i*ACC_W +: ACC_W] = out_acc[i];
      end
    end else begin : g_out_direct
      assign pop_bank   = tile_valid && tile_ready;
      assign tile_valid = full[head];
      for (genvar i = 0; i < R*C; i++) begin : g_out
        assign tile_acc[i*ACC_W +: ACC_W] = head ? acc1[i] : acc0[i];
      end
    end
  endgenerate

endmodule

`default_nettype wire
