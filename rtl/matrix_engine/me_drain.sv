// ============================================================================
// File: me_drain.sv
// Block: Matrix engine - the drain: tile accumulators to their destination
//        (bit for bit with the Python reference model)
//
// Takes each finished GEMM tile (4 rows x 8 channels of 34-bit accumulators)
// from the array and writes its 32 outputs to the row store, at the command's
// destination and class:
//
//   per channel        FP16 column scale (the walker's scale beat):
//                      m_col, e_col;  kc17, nc' = normalise17(m_col x 132,104)
//                      (the golden's normalise17(m_col x 132,104 x 1,024) is the
//                      same mantissa with nc = nc' + 10: m_col x 132,104 has 18
//                      bits or more whenever m_col > 0)
//   per (row, channel) c17, n2 = normalise17(kc17 x absmax[row])
//   per output         y = saturate(rhe(acc x c17, s), W)
//                      s = 39 + e_out - e_col - e_in - shift[row] - nc' - n2
//                      (= the golden's 15 + e_out - e_cx - e_in - in_shift + 24 - nc - n
//                      with e_cx = e_col - 10); s <= 0 is a saturating left shift;
//                      W = 32 when the output class is 32 bits wide, else 16
//   ACC                y = saturate(y + D[row][element], W), read and written in place
//
// Two lanes, one channel pair (2j, 2j + 1) a cycle (a tile in 16
// cycles against the 36 the next one takes at K = 576); a pair is one staging word
// (STG16) or one residual entry (RES), and the unit of RoPE and SwiGLU (stage 2).
// STG32 takes one element a cycle (one 32-bit staging word each), 32 cycles a tile.
//
// Destinations (row store), element e = 64 DOFF + 8 grp + j:
//   RES     entry srow x 288 + e/2, element e in half e mod 2 (INT32)
//   STG16   word  srow x 1,536 + e/2, element e in half e mod 2 (INT16)
//   STG32   word  srow x 1,536 + e (INT32)
// srow is the step row: window row i = 4 rgrp + r for ROWSET_ALL, or the i-th row
// that wants logits (logit_map) for ROWSET_LOGIT.
//
// Per tile, in order:
//   1. while the tile is offered: read its 4 rows' constants from the row store, one
//      row a cycle (drn_row -> drn_absmax, drn_shift, the same cycle; the
//      store's copy cannot switch before this tile's handshake);
//   2. the handshake (tile_ready) once the constants are in and the tile buffer is
//      free (or frees this cycle): the buffer takes the accumulators, the tag, the
//      constants, the tile's 8 column scales (from the two group registers: group
//      g + 2's scales may overwrite them right after this handshake, walker
//      guarantee) and the command's settings (by tag.cmd);
//   3. 16 beats (32 for STG32) into the pipeline; rows not in tag.rvalid write
//      nothing. The pipeline never stalls: the store's ports take a write a cycle.
//
// Command settings: each GEMM word, with its command slot id, is handed to
// the drain as it is handed to the walker (dc_valid); a 64-entry table indexed by
// the slot id keeps its resolved settings (classes' exponents and widths from the
// class table, destination, offset, row set, flags), and each tile finds its own by
// tag.cmd. A slot id is not reused while its command has tiles in flight.
//
// Faults (the first one is kept, sticky, until cnt_clr or reset):
//   0x21 CMD_FIELD  a flag this stage does not implement (any but ACC: RoPE, K/V,
//                   SwiGLU, TOPK, BIAS, GSCALE, ...) or a destination not RES/STG16/
//                   STG32: the command's tiles are taken and nothing is written
//   0x42 SCALE      a column scale with an all-ones exponent (infinity or NaN,
//                   interface): it decodes to zero, the outputs are written
// Reserved for later: guard bits (8) with RoPE and K/V, the rotation between
// the scale stage and the final rounding, SwiGLU after it, the K/V path to INT8
// (kvo_* ports, driven 0 now).
//
// cmd_fin pulses with the write of a command's last output: its results may be
// read from then on. sat_cnt counts outputs that saturated (the scale stage's or the
// ACC sum's; one count per output).
//
// Reset: synchronous, active low, control state only.
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module me_drain
  import me_if_pkg::*;
#(
  parameter int RD_LAT = 2           // row store read latency, read -> data (staging, residual)
) (
  input  wire logic              clk,
  input  wire logic              rst_n,

  // ---- step context, stable while a step runs ----
  input  wire logic [39:0]       logit_map,      // step row of logit row i at [5i +: 5] (ROWSET_LOGIT)
  input  wire logic              cnt_clr,        // the step starts: clear the counters and the fault

  // ---- class table (the core's tables, LOAD fills it) ----
  input  wire logic [127:0]      cls_e,          // class c's exponent at [8c +: 8], signed
  input  wire logic [127:0]      cls_w,          // class c's width in bits at [8c +: 8]: 8, 16 or 32

  // ---- command settings (from the command path) ----
  input  wire logic              dc_valid,       // a GEMM word, as it is handed to the walker
  input  wire logic [5:0]        dc_id,          // its command slot (the tiles' tag.cmd)
  input  wire logic [63:0]       dc_word,        // me_cmd_pkg::cmd_t

  // ---- column scales from the walker ----
  input  wire logic              scale_valid,
  input  wire logic [10:0]       scale_grp,      // the 32-channel group g
  input  wire logic [511:0]      scale_data,     // channel 32g + j at [16j +: 16], FP16

  // ---- tiles from the array, and their tags from the walker ----
  input  wire logic              tile_valid,
  output logic                   tile_ready,
  input  wire mei_tile_bus_t     tile_acc,
  input  wire logic              tile_tag_valid,
  input  wire mei_tag_t          tile_tag,

  // ---- the row store: the tile's row constants ----
  output logic [4:0]             drn_row,        // window row 4 rgrp + r
  input  wire logic [14:0]       drn_absmax,     // the same cycle
  input  wire logic [4:0]        drn_shift,

  // ---- the row store: staging ----
  output logic                   stg_we,
  output logic [15:0]            stg_waddr,
  output logic [1:0]             stg_whe,
  output logic [31:0]            stg_wdata,
  output logic                   stg_rd,
  output logic [15:0]            stg_raddr,
  input  wire logic [31:0]       stg_rdata,      // RD_LAT cycles after stg_rd

  // ---- the row store: residual ----
  output logic                   res_we,
  output logic [12:0]            res_waddr,
  output logic [1:0]             res_wee,
  output logic [63:0]            res_wdata,
  output logic                   res_rd,
  output logic [12:0]            res_raddr,
  input  wire logic [63:0]       res_rdata,      // RD_LAT cycles after res_rd

  // ---- the row store: own K/V (stage 2; driven 0) ----
  output logic                   kvo_we,
  output logic [10:0]            kvo_waddr,
  output logic [7:0]             kvo_wbe,
  output logic [63:0]            kvo_wdata,

  // ---- to the command path ----
  output logic                   cmd_fin,        // pulse: a command's last output is written
  output logic [5:0]             cmd_fin_id,
  output logic                   fault,          // sticky: the first fault ...
  output logic [7:0]             fault_code,     // ... its code (me_cmd_pkg::fault_e)
  output logic [5:0]             fault_cmd,      // ... and its command slot
  output logic                   fault_pulse,    // one cycle, when the first fault is taken
  output logic                   busy,           // a tile is held or in the pipeline

  // ---- counters ----
  output logic [31:0]            sat_cnt,        // outputs saturated
  output logic [31:0]            perf_tiles,     // tiles taken
  output logic [31:0]            perf_outs       // outputs written
);

  localparam logic [3:0] B_RES   = 4'(me_cmd_pkg::BUF_RES);
  localparam logic [3:0] B_STG16 = 4'(me_cmd_pkg::BUF_STG16);
  localparam logic [3:0] B_STG32 = 4'(me_cmd_pkg::BUF_STG32);
  localparam logic [1:0] RS_LOGIT = 2'(me_cmd_pkg::ROWSET_LOGIT);
  localparam logic [7:0] F_FIELD = 8'(me_cmd_pkg::FAULT_CMD_FIELD);
  localparam logic [7:0] F_SCALE = 8'(me_cmd_pkg::FAULT_SCALE);
  localparam int         CF_ACC  = me_cmd_pkg::CF_ACC;

  // pipeline stages: stage k's registers hold a beat k + 1 cycles after its issue cycle. The ACC sum
  // is formed while the beat is in stage P_ACC, with the store's read data, so the read leaves while
  // the beat is in stage P_ACC - RD_LAT; the outputs leave stage P_WR's registers.
  localparam int P_ACC = 11;
  localparam int P_WR  = P_ACC + 1;
  localparam int P_RD  = P_ACC - RD_LAT;
  initial assert (RD_LAT >= 1 && RD_LAT <= 9) else $fatal(1, "me_drain: RD_LAT out of range");

  typedef enum logic [1:0] { D_RES = 2'd0, D_STG16 = 2'd1, D_STG32 = 2'd2 } dst_e;

  // ==========================================================================
  // command settings, by command slot
  // ==========================================================================
  typedef struct packed {
    logic              unsup;    // a flag or destination this stage does not implement
    logic [1:0]        dst;      // dst_e
    logic              acc;      // CF_ACC
    logic              w32;      // the output class is 32 bits wide
    logic              logit;    // ROWSET_LOGIT
    logic [5:0]        doff;     // D's offset, 64 elements a unit
    logic signed [7:0] e_in;     // the input class's exponent
    logic signed [7:0] e_out;    // the output class's exponent
  } cset_t;

  cset_t ctab [64];
  me_cmd_pkg::cmd_t dcw;
  assign dcw = me_cmd_pkg::cmd_t'(dc_word);

  function automatic cset_t resolve(me_cmd_pkg::cmd_t c);
    cset_t s;
    logic [9:0] other = c.flags & ~(10'(1) << CF_ACC);
    s.dst   = (c.dbuf == B_RES) ? D_RES : (c.dbuf == B_STG16) ? D_STG16 : D_STG32;
    s.unsup = (other != '0) || !(c.dbuf == B_RES || c.dbuf == B_STG16 || c.dbuf == B_STG32);
    s.acc   = c.flags[CF_ACC];
    s.w32   = (cls_w[8*c.oclass +: 8] == 8'd32);
    s.logit = (c.rowset == RS_LOGIT);
    s.doff  = c.doff;
    s.e_in  = cls_e[8*c.iclass +: 8];
    s.e_out = cls_e[8*c.oclass +: 8];
    return s;
  endfunction

  always_ff @(posedge clk)
    if (dc_valid) ctab[dc_id] <= resolve(dcw);

  // ==========================================================================
  // column scales: two group registers, by g[0] (= tag.grp[2])
  // ==========================================================================
  logic [511:0] sreg [2];
  always_ff @(posedge clk)
    if (scale_valid) sreg[scale_grp[0]] <= scale_data;

  // ==========================================================================
  // the tile on the port: its row constants, then the handshake
  // ==========================================================================
  logic [2:0]  cph;                  // 0 waiting, 1-4 reading row cph-1, 5 ready to take
  logic [2:0]  c_rg;
  logic [14:0] c_ab [4];
  logic [4:0]  c_sh [4];

  // the tile buffer, and its beats
  logic                 bv;          // the buffer holds a tile being issued
  logic [4:0]           bb;          // the next beat
  mei_tile_bus_t        b_acc;
  mei_tag_t             b_tag;
  logic [14:0]          b_ab [4];
  logic [4:0]           b_sh [4];
  logic [15:0]          b_sc [8];
  cset_t                b_cs;

  wire logic single    = (b_cs.dst == D_STG32);
  wire logic last_beat = bv && (bb == (single ? 5'd31 : 5'd15));
  wire logic take      = (cph == 3'd5) && (!bv || last_beat);
  wire logic hs        = tile_valid && take;
  assign tile_ready = take;
  assign drn_row    = {c_rg, 2'(cph - 3'd1)};

  cset_t t_cs;
  assign t_cs = ctab[tile_tag.cmd];

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      cph <= '0; bv <= 1'b0; bb <= '0;
    end else begin
      // constants
      if (hs)                              cph <= '0;
      else if (cph == 3'd0 && tile_valid)  cph <= 3'd1;
      else if (cph != 3'd0 && cph != 3'd5) cph <= cph + 3'd1;
      // beats
      if (hs) begin
        bv <= 1'b1; bb <= '0;
      end else if (last_beat) begin
        bv <= 1'b0;
      end else if (bv) begin
        bb <= bb + 5'd1;
      end
    end
  end

  always_ff @(posedge clk) begin
    if (cph == 3'd0 && tile_valid) c_rg <= tile_tag.rgrp;
    if (cph >= 3'd1 && cph <= 3'd4) begin
      c_ab[2'(cph - 3'd1)] <= drn_absmax;
      c_sh[2'(cph - 3'd1)] <= drn_shift;
    end
    if (hs) begin
      b_acc <= tile_acc;
      b_tag <= tile_tag;
      b_cs  <= t_cs;
      for (int r = 0; r < 4; r++) begin b_ab[r] <= c_ab[r]; b_sh[r] <= c_sh[r]; end
      for (int j = 0; j < 8; j++)
        b_sc[j] <= sreg[tile_tag.grp[2]][16 * (8 * tile_tag.grp[1:0] + j) +: 16];
    end
  end

  // ==========================================================================
  // issue: one beat a cycle (a channel pair, or one element for STG32)
  // ==========================================================================
  typedef struct packed {
    logic              v;        // a beat
    logic [1:0]        en;       // lanes that write
    logic [1:0]        lv;       // lanes that exist (the scale check)
    logic [1:0]        dst;
    logic              acc;
    logic              w32;
    logic [15:0]       addr;     // staging word or residual entry
    logic              fin;      // the last beat of a command's last tile
    logic [5:0]        cmd;
    logic signed [10:0] t;       // 39 + e_out - e_in - shift[row] (the per-channel terms follow)
  } info_t;

  info_t                 inf [P_WR + 1];

  // lane data through the stages
  logic signed [33:0]    l_acc  [2][8];          // the accumulator, to the product's stage
  logic [15:0]           i_sc   [2];
  logic [14:0]           i_ab;

  wire logic [1:0]  i_r  = single ? bb[4:3] : bb[3:2];
  wire logic [2:0]  i_j0 = single ? bb[2:0] : {bb[1:0], 1'b0};
  wire logic [4:0]  i_wr = {b_tag.rgrp, i_r};                        // window row
  wire logic [4:0]  i_sr = b_cs.logit ? logit_map[5 * i_wr[2:0] +: 5] : i_wr;   // step row
  wire logic [16:0] i_e  = {b_cs.doff, 6'b0} + {1'b0, b_tag.grp, 3'b0} + 17'(i_j0);
  wire logic [15:0] i_stg = 16'({i_sr, 10'b0}) + 16'({i_sr, 9'b0}) + (single ? i_e[15:0] : 16'(i_e[16:1]));
  wire logic [15:0] i_res = 16'({i_sr, 8'b0}) + 16'({i_sr, 5'b0}) + 16'(i_e[16:1]);
  wire logic        i_row_ok = b_tag.rvalid[i_r] && !b_cs.unsup;
  wire logic        i_fin = last_beat && b_tag.last;

  always_ff @(posedge clk) begin
    info_t n;
    n.v    = bv;
    n.lv   = single ? 2'b01 : 2'b11;
    n.en   = (bv && i_row_ok) ? n.lv : 2'b00;
    n.dst  = b_cs.dst;
    n.acc  = b_cs.acc;
    n.w32  = b_cs.w32;
    n.addr = (b_cs.dst == D_RES) ? i_res : i_stg;
    n.fin  = bv && i_fin;
    n.cmd  = b_tag.cmd;
    n.t    = 11'sd39 + 11'($signed(b_cs.e_out)) - 11'($signed(b_cs.e_in)) - $signed({6'd0, b_sh[i_r]});
    if (!rst_n) n = '0;
    inf[0] <= n;
    for (int k = 1; k <= P_WR; k++) inf[k] <= rst_n ? inf[k - 1] : '0;
    for (int l = 0; l < 2; l++) begin
      l_acc[l][0] <= b_acc[MEI_ACC_W * int'(5'(8 * i_r + i_j0 + l)) +: MEI_ACC_W];   // lane 1 of STG32: unused
      for (int k = 1; k < 8; k++) l_acc[l][k] <= l_acc[l][k - 1];
    end
    i_sc[0] <= b_sc[i_j0];
    i_sc[1] <= b_sc[i_j0 + 3'd1];
    i_ab    <= b_ab[i_r];
  end

  // ==========================================================================
  // the two lanes: the constant, the product, the rounding, the saturation
  // ==========================================================================
  function automatic logic [5:0] bitlen32(logic [31:0] v);
    bitlen32 = '0;
    for (int i = 0; i < 32; i++) if (v[i]) bitlen32 = 6'(i + 1);
  endfunction

  // v ~= m x 2^n, m < 2^17, round half up (n from the stage before: max(0, bitlen(v) - 17))
  function automatic logic [21:0] round17(logic [31:0] v, logic [3:0] n);   // {n (5 bits), m (17 bits)}
    logic [32:0] a, m;
    a = {1'b0, v} + ((n != 4'd0) ? (33'd1 << (n - 4'd1)) : 33'd0);
    m = a >> n;
    return m[17] ? {5'(n) + 5'd1, m[17:1]} : {5'(n), m[16:0]};
  endfunction

  logic [1:0]  bad_scale;                        // stage 1: a malformed scale, per lane
  logic [1:0]  o_sat;                            // stage P_WR: saturated, per lane
  logic [31:0] o_val [2];                        // stage P_WR: the output, sign-extended

  generate
    for (genvar l = 0; l < 2; l++) begin : g_lane
      // stage 1: the scale's fields; kc = m_col x 132,104 = m (2^17 + 2^10 + 2^3)
      logic [28:0]       s1_kc;
      logic signed [5:0] s1_ec;
      // stage 2, 3: normalise17(kc)
      logic [28:0]       s2_kc;
      logic [3:0]        s2_n;
      logic signed [5:0] s2_ec, s3_ec;
      logic [16:0]       s3_kc17;
      logic [4:0]        s3_nc;
      logic [14:0]       s1_ab, s2_ab, s3_ab;
      // stage 4, 5: kc17 x absmax; stage 6, 7: normalise17
      logic [31:0]       s4_m1, s5_m1, s6_m1;
      logic [3:0]        s6_n;
      logic [16:0]       s7_c17;
      logic signed [10:0] s4_t, s5_t, s6_t, s7_t;   // the shift less n2
      logic [4:0]        s7_n2;
      // stage 8, 9: acc x c17; the shift
      logic signed [51:0] s8_p, s9_p;
      logic signed [10:0] s8_s;
      logic              s9_pos;
      logic [5:0]        s9_sr;                  // right shift, 1 .. 51
      logic [5:0]        s9_k;                   // left shift, 0 .. 32
      // stage 10: the rounding's parts, or the left shift
      logic signed [51:0] s10_q;
      logic              s10_inc, s10_pos, s10_neg, s10_lsat;
      logic [31:0]       s10_lv;
      // stage 11, 12: the value at the output's width, then ACC
      logic signed [31:0] s11_v, s12_v;
      logic              s11_sat, s12_sat;

      wire logic [15:0] h  = i_sc[l];
      wire logic [4:0]  he = h[14:10];
      wire logic        w32_10 = inf[9].w32;

      always_ff @(posedge clk) begin
        // ---- stage 1 ----
        begin
          logic [10:0] m;
          m = (he == 5'd0) ? {1'b0, h[9:0]} : (he == 5'h1F) ? 11'd0 : {1'b1, h[9:0]};
          s1_kc <= {m, 17'b0} + {8'b0, m, 10'b0} + {15'b0, m, 3'b0};
          s1_ec <= (he == 5'd0) ? -6'sd24 : 6'(he) - 6'sd25;
          bad_scale[l] <= inf[0].v && inf[0].lv[l] && (he == 5'h1F);
          s1_ab <= i_ab;
        end
        // ---- stage 2, 3 ----
        s2_kc <= s1_kc;
        s2_n  <= (bitlen32(32'(s1_kc)) > 6'd17) ? 4'(bitlen32(32'(s1_kc)) - 6'd17) : 4'd0;
        s2_ec <= s1_ec;
        s2_ab <= s1_ab;
        {s3_nc, s3_kc17} <= round17(32'(s2_kc), s2_n);
        s3_ec <= s2_ec;
        s3_ab <= s2_ab;
        // ---- stage 4, 5: kc17 x absmax (one DSP) ----
        s4_m1 <= s3_kc17 * s3_ab;
        s4_t  <= inf[3].t - 11'(s3_ec) - 11'(s3_nc);
        s5_m1 <= s4_m1;
        s5_t  <= s4_t;
        // ---- stage 6, 7: normalise17 ----
        s6_m1 <= s5_m1;
        s6_n  <= (bitlen32(s5_m1) > 6'd17) ? 4'(bitlen32(s5_m1) - 6'd17) : 4'd0;
        s6_t  <= s5_t;
        {s7_n2, s7_c17} <= round17(s6_m1, s6_n);
        s7_t  <= s6_t;
        // ---- stage 8, 9: acc x c17 (two DSPs), the shift ----
        s8_p  <= l_acc[l][7] * $signed({1'b0, s7_c17});
        s8_s  <= s7_t - 11'(s7_n2);
        s9_p  <= s8_p;
        s9_pos <= (s8_s > 11'sd0);
        s9_sr <= (s8_s > 11'sd51) ? 6'd51 : (s8_s > 11'sd0) ? 6'(s8_s) : 6'd1;   // 51 and more: 0
        s9_k  <= (s8_s < -11'sd32) ? 6'd32 : (s8_s > 11'sd0) ? 6'd0 : 6'(-s8_s);
        // ---- stage 10: round half even (right), or saturating left shift ----
        begin
          logic [51:0] low;
          logic        g, st;
          logic [31:0] hi_k, lo_k;
          low  = (52'd1 << (s9_sr - 6'd1)) - 52'd1;
          g    = s9_p[s9_sr - 6'd1];
          st   = |(s9_p & low);
          s10_q   <= s9_p >>> s9_sr;
          s10_inc <= g && (st || s9_p[s9_sr]);
          s10_pos <= s9_pos;
          s10_neg <= s9_p[51];
          hi_k = (w32_10 ? 32'h7FFF_FFFF : 32'h0000_7FFF) >> s9_k;
          lo_k = (w32_10 ? 32'h8000_0000 : 32'h0000_8000) >> s9_k;
          s10_lsat <= (s9_p > $signed({20'd0, hi_k})) || (s9_p < -$signed({20'd0, lo_k}));
          s10_lv   <= s9_p[31:0] << s9_k;
        end
        // ---- stage 11: the value at width W ----
        begin
          logic signed [52:0] v;
          logic               w32;
          w32 = inf[10].w32;
          v = $signed({s10_q[51], s10_q}) + $signed({52'd0, s10_inc});
          if (s10_pos) begin
            if (w32) begin
              s11_sat <= (v > 53'sd2147483647) || (v < -53'sd2147483648);
              s11_v   <= (v > 53'sd2147483647) ? 32'sh7FFF_FFFF : (v < -53'sd2147483648) ? 32'sh8000_0000 : 32'(v);
            end else begin
              s11_sat <= (v > 53'sd32767) || (v < -53'sd32768);
              s11_v   <= (v > 53'sd32767) ? 32'sd32767 : (v < -53'sd32768) ? -32'sd32768 : 32'(v);
            end
          end else begin
            s11_sat <= s10_lsat;
            if (w32) s11_v <= s10_lsat ? (s10_neg ? 32'sh8000_0000 : 32'sh7FFF_FFFF) : $signed(s10_lv);
            else     s11_v <= s10_lsat ? (s10_neg ? -32'sd32768 : 32'sd32767)
                                       : 32'($signed(s10_lv[15:0]));
          end
        end
        // ---- stage 12: ACC, with the store's read data (the beat in stage P_ACC = 11) ----
        begin
          logic signed [32:0] d, sum;
          logic               w32;
          w32 = inf[11].w32;
          case (inf[11].dst)
            D_RES:   d = 33'($signed(res_rdata[32 * l +: 32]));
            D_STG16: d = 33'($signed(stg_rdata[16 * l +: 16]));
            default: d = 33'($signed(stg_rdata));
          endcase
          sum = 33'(s11_v) + d;
          if (!inf[11].acc) begin
            s12_v <= s11_v; s12_sat <= s11_sat;
          end else if (w32) begin
            s12_v   <= (sum > 33'sd2147483647) ? 32'sh7FFF_FFFF : (sum < -33'sd2147483648) ? 32'sh8000_0000 : 32'(sum);
            s12_sat <= s11_sat || (sum > 33'sd2147483647) || (sum < -33'sd2147483648);
          end else begin
            s12_v   <= (sum > 33'sd32767) ? 32'sd32767 : (sum < -33'sd32768) ? -32'sd32768 : 32'(sum);
            s12_sat <= s11_sat || (sum > 33'sd32767) || (sum < -33'sd32768);
          end
        end
      end
      assign o_val[l] = s12_v;
      assign o_sat[l] = s12_sat;
    end
  endgenerate

  // ==========================================================================
  // the store's ports: ACC reads (stage P_RD), writes (stage P_WR)
  // ==========================================================================
  info_t ir, iw;
  assign ir = inf[P_RD];
  assign iw = inf[P_WR];
  assign res_rd    = ir.v && ir.acc && (ir.en != 2'b00) && (ir.dst == D_RES);
  assign res_raddr = ir.addr[12:0];
  assign stg_rd    = ir.v && ir.acc && (ir.en != 2'b00) && (ir.dst != D_RES);
  assign stg_raddr = ir.addr;

  assign res_we    = iw.v && (iw.en != 2'b00) && (iw.dst == D_RES);
  assign res_waddr = iw.addr[12:0];
  assign res_wee   = iw.en;
  assign res_wdata = {o_val[1], o_val[0]};
  assign stg_we    = iw.v && (iw.en != 2'b00) && (iw.dst != D_RES);
  assign stg_waddr = iw.addr;
  assign stg_whe   = (iw.dst == D_STG32) ? 2'b11 : iw.en;
  assign stg_wdata = (iw.dst == D_STG32) ? o_val[0] : {o_val[1][15:0], o_val[0][15:0]};

  assign kvo_we = 1'b0; assign kvo_waddr = '0; assign kvo_wbe = '0; assign kvo_wdata = '0;

  assign cmd_fin    = iw.v && iw.fin;
  assign cmd_fin_id = iw.cmd;

  // a tile is held, read or in the pipeline
  logic any_inf;
  always_comb begin
    any_inf = 1'b0;
    for (int k = 0; k <= P_WR; k++) any_inf |= inf[k].v;
  end
  assign busy = bv || (cph != 3'd0) || any_inf;

  // ==========================================================================
  // faults and counters
  // ==========================================================================
  wire logic f42 = |bad_scale;
  wire logic f21 = hs && t_cs.unsup;
  always_ff @(posedge clk) begin
    fault_pulse <= 1'b0;
    if (!rst_n || cnt_clr) begin
      fault <= 1'b0; fault_code <= '0; fault_cmd <= '0;
      sat_cnt <= '0; perf_tiles <= '0; perf_outs <= '0;
    end else begin
      if (!fault && (f42 || f21)) begin           // the older first: stage 1's beat left before this tile
        fault       <= 1'b1;
        fault_pulse <= 1'b1;
        fault_code  <= f42 ? F_SCALE : F_FIELD;
        fault_cmd   <= f42 ? inf[1].cmd : tile_tag.cmd;
      end
      if (hs) perf_tiles <= perf_tiles + 32'd1;
      if (iw.v) begin
        perf_outs <= perf_outs + 32'(iw.en[0]) + 32'(iw.en[1]);
        sat_cnt   <= sat_cnt + 32'(iw.en[0] && o_sat[0]) + 32'(iw.en[1] && o_sat[1]);
      end
    end
  end

endmodule

`default_nettype wire
