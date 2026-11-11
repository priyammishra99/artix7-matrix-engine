// ============================================================================
// File: tb_me_tile_walker.sv
// Block: Matrix engine - testbench for me_tile_walker (OP_GEMM)
//        driving the real me_array, on real SmolLM2 GEMMs
//
//   command model   --cmd words--> me_tile_walker --beats (contract v2)--> me_array
//   weight engine + ring model  <--entries, reads, frees--|        |  tiles + tags v
//   row store window model      <--reads------------------|        |       drain model
//                                       scales --> drain model     +-- me_if_checker
//
// Vectors (vectors/): the bit-exact Python reference model's numbers for
// each GEMM of one prompt step. The ring is fed
// the tensors' runs as DDR holds them, back to back (the weight engine does
// not stop at tensor boundaries): per 32-channel group one scale slot (the
// beat in bits 511:0), then two beats per weight slot. Checks:
//   - every ring and window read in order (the scale slot, the tiles, the
//     replay, the ring base across commands), every read in the ring's live
//     window (written, not yet freed);
//   - every scale slot sent to the drain: its data and group, in order;
//   - every beat at the array: weights = the ring slot, activations = the
//     window entry, D = 0, first/last;
//   - every tile: its tag (command, 8-channel tile, row group, live rows,
//     last) and its live rows' 34-bit accumulators against the reference model's;
//   - faults (FAULTS 1): each command of gemm_faults.txt ends with the
//     reference model's fault code, plus the hardware-only cases (another layer's
//     entry, an op that is not GEMM, a window at another class, the tag checked
//     before the buffers, an empty row set); none reads the ring or pops an
//     entry;
//   - at the end: every slot freed, every entry popped, counters, no array
//     error, 0 protocol errors from me_if_checker, no bypass results.
//
// Configuration (tw_run.cfg in the run folder, "KEY value" lines):
//   DIR    the vector folder         CASES  names, comma separated, run in order
//   FAULTS 1: the fault cases first (CASES must start with l0_q)
//   RATE   0 one slot per cycle | 1 bus-like, 5 slots per 13 cycles | 2 random
//   DRAIN  0 always ready | 1 random | 2 stalling (long holds: the credit rule)      SEED n
//   RESET_AT n: a soft reset once n beats have reached the array (part-way through a command, with
//          tiles in flight), then the whole run again: it must match exactly
//   COLD   1: the ring starts each tensor's run (but the first) 20 cycles after its command is
//          posted, so the walker waits for the first slot (bubbles before the first beat)
// Parameters: W_LAT, X_LAT (read latencies), USE_ARRAY (0: behavioural array
// model). Tops: tb_me_tile_walker (the real me_array: the joint run), and on the
// behavioural array tb_me_tile_walker_model, _lat (ring 1, window 3), _lat2 (ring 3,
// window 1).
// Ends with "TW RESULTS: N checks, F failures" and ">> VERDICT: PASS/FAIL <<".
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module tb_me_tile_walker #(
  parameter int W_LAT     = 2,
  parameter int X_LAT     = 2,
  parameter bit USE_ARRAY = 1'b1,
  parameter bit USE_RING  = 1'b0          // 1: the real me_weight_ring (block RAM; W_LAT must be 2)
);
  import me_if_pkg::*;

  localparam int MAXC = 16;               // commands per run

  // --------------------------------------------------------------------------
  // clock, reset
  // --------------------------------------------------------------------------
  logic clk = 1'b0;
  always #4 clk = ~clk;                   // 125 MHz
  logic rst_n = 1'b0;

  // --------------------------------------------------------------------------
  // the run's commands and golden data
  // --------------------------------------------------------------------------
  typedef struct {
    logic [63:0] word;
    int layer, tag, n, k, beats, nstep, nused, abuf, iclass, rowset;
  } cmd_s;
  cmd_s          cmds [MAXC];
  string         names [MAXC];
  int            ncmd = 0;
  logic [511:0]  win_mem [MAXC][576];
  logic signed [33:0] acc_all [$];       // every command's accumulators, one flat queue
  int            acc_base [MAXC];        // each command's first entry in acc_all
  logic signed [33:0] atmp [65536];       // one command's accumulators ($readmemh)
  logic [1023:0] slots [$];               // the ring's slot sequence, all commands
  int            ent_first [MAXC];        // first slot of each command's run
  logic [511:0]  beat_q [$];              // the stream's beats as DDR holds them (the real ring's input)
  int            ent_first_beat [MAXC];   // first beat of each command's run
  int            di = 0, bq = 0;          // the real ring's feed: descriptors and beats given
  logic [10:0]   wptr_q = '0;

  typedef struct { bit sc; int slot; int waddr; } iss_s;
  typedef struct { int slot; int waddr; int cmd; bit first, last; } beat_s;
  typedef struct { int cmd, grp, rg, rv, last; } tag_s;
  iss_s          exp_iss [$], exp_iss0 [$];      // ...0: a copy, to restart after a soft reset
  beat_s         exp_beat [$], exp_beat0 [$];
  tag_s          exp_tag [$], exp_tag0 [$];
  logic [511:0]  exp_scale [$], exp_scale0 [$];
  int            exp_scale_g [$], exp_scale_g0 [$];
  int            tot_beats = 0, tot_groups = 0;

  string dir = "", cases = "";
  bit    loaded = 1'b0;
  int    faults = 0, rate_mode = 0, drain_mode = 0, seed = 1;
  int    reset_at = 0;                     // > 0: a soft reset once this many beats have reached the array
  int    cold = 0;                         // 1: each tensor's run (but the first) starts 20 cycles after its command
  int    cyc = 0;
  int    post_cyc [MAXC];

  // --------------------------------------------------------------------------
  // DUT and its neighbours
  // --------------------------------------------------------------------------
  logic [5:0]  n_layers = 6'd30;
  logic [4:0]  step_rows = 5'd1;
  logic [3:0]  step_logit_rows = 4'd1;
  logic        cmd_valid = 1'b0, cmd_ready, cmd_done, cmd_fault, busy;
  logic [63:0] cmd_word = '0;
  logic [5:0]  cmd_id = '0;
  logic [7:0]  cmd_fault_code;

  logic        ent_valid, ent_ready;
  logic [5:0]  ent_layer;
  logic [4:0]  ent_tag;
  logic [1:0]  ent_kind;
  logic [15:0] ent_n, ent_k;

  logic [10:0]   ring_wptr;
  logic          ring_rd, ring_free;
  logic [9:0]    ring_raddr;
  logic [6:0]    ring_free_n;
  logic [1023:0] ring_rdata;
  logic          win_valid = 1'b0;
  logic [3:0]    win_cls = '0;
  logic          win_rd;
  logic [9:0]    win_raddr;
  logic [511:0]  win_rdata;
  logic          scale_valid;
  logic [10:0]   scale_grp;
  logic [511:0]  scale_data;

  logic          op_valid, op_first, op_last, op_bank;
  mei_w_bus_t    op_w;
  mei_dp_bus_t   op_dp;
  mei_x_bus_t    op_x;

  logic          tile_valid, tile_ready = 1'b1, tile_tag_valid;
  mei_tag_t      tile_tag;
  mei_tile_bus_t tile_acc;
  logic          byp_valid;
  logic          err_overrun, err_bypass;
  logic          cnt_clr = 1'b0;
  logic [31:0]   arr_beats, arr_tiles, perf_beats, perf_scales, bubbles, perf_stall_credit;

  function automatic mei_w_bus_t sext_w(logic [1023:0] slot);
    for (int e = 0; e < MEI_C * MEI_D; e++) sext_w[e*MEI_A_W +: MEI_A_W] = MEI_A_W'($signed(slot[e*8 +: 8]));
  endfunction
  function automatic mei_x_bus_t sext_x(logic [511:0] win);
    for (int e = 0; e < MEI_R * MEI_D; e++) sext_x[e*MEI_X_W +: MEI_X_W] = MEI_X_W'($signed(win[e*8 +: 8]));
  endfunction

  me_tile_walker #(.W_LAT(W_LAT), .X_LAT(X_LAT)) dut (
    .clk, .rst_n,
    .n_layers, .step_rows, .step_logit_rows,
    .cmd_valid, .cmd_ready, .cmd_word, .cmd_id, .cmd_done, .cmd_fault, .cmd_fault_code, .busy,
    .ent_valid, .ent_ready, .ent_layer, .ent_tag, .ent_kind, .ent_n, .ent_k,
    .ring_wptr, .ring_rd, .ring_raddr, .ring_rdata, .ring_free, .ring_free_n,
    .win_valid, .win_cls, .win_rd, .win_raddr, .win_rdata,
    .scale_valid, .scale_grp, .scale_data,
    .op_valid, .op_first, .op_last, .op_bank, .op_w, .op_dp, .op_x,
    .tile_valid, .tile_ready, .tile_tag_valid, .tile_tag,
    .cnt_clr, .perf_beats, .perf_scales, .bubbles, .perf_stall_credit);

  generate
    if (USE_ARRAY) begin : g_array
      logic [MEI_NACC*MEI_SUM_W-1:0] byp_sum;
      logic [MEI_BTAG_W-1:0]         byp_tag;
      logic [31:0]                   arr_byp;
      me_array u_array (
        .clk, .rst_n,
        .op_valid, .op_first, .op_last, .op_bank, .op_bypass(1'b0), .op_btag('0),
        .op_w, .op_dp, .op_x,
        .tile_valid, .tile_ready, .tile_acc,
        .byp_valid, .byp_sum, .byp_tag,
        .err_overrun, .err_bypass, .perf_beats(arr_beats), .perf_tiles(arr_tiles), .perf_byp(arr_byp));
    end else begin : g_model
      // behavioural stand-in: the same dot products, tiles offered in order
      // MEI_TILE_LAT edges after their last beat, held until taken
      typedef struct { mei_tile_bus_t acc; int ready_at; } mtile_t;
      mtile_t      q[$];
      logic signed [MEI_ACC_W-1:0] acc [32];
      int          m_cyc = 0, m_beats = 0, m_tiles = 0;
      logic        m_valid = 1'b0;
      mei_tile_bus_t m_acc = '0;
      assign tile_valid  = m_valid;
      assign tile_acc    = m_acc;
      assign byp_valid   = 1'b0;
      assign err_overrun = 1'b0;
      assign err_bypass  = 1'b0;
      assign arr_beats   = 32'(m_beats);
      assign arr_tiles   = 32'(m_tiles);
      always @(posedge clk) begin
        m_cyc++;
        if (rst_n) begin
          if (m_valid && tile_ready) begin void'(q.pop_front()); m_tiles++; end
          if (op_valid) begin
            m_beats++;
            for (int i = 0; i < 32; i++) begin
              logic signed [MEI_ACC_W-1:0] s;
              s = 0;
              for (int d = 0; d < 16; d++)
                s += (mei_w_get(op_w, i % 8, d) + $signed(op_dp[d*MEI_DP_W +: MEI_DP_W])) *
                     mei_x_get(op_x, i / 8, d);
              acc[i] = (op_first ? 0 : acc[i]) + s;
            end
            if (op_last) begin
              mtile_t t;
              for (int i = 0; i < 32; i++) t.acc[i*MEI_ACC_W +: MEI_ACC_W] = acc[i];
              t.ready_at = m_cyc + MEI_TILE_LAT;
              q.push_back(t);
            end
          end
          m_valid <= (q.size() > 0) && (q[0].ready_at <= m_cyc + 1);
          if (q.size() > 0) m_acc <= q[0].acc;
        end else begin                          // a reset drops the tiles in flight, as me_array's does
          q.delete();
          m_valid <= 1'b0;
          m_beats = 0;
          m_tiles = 0;
        end
      end
    end
  endgenerate

  me_if_checker u_chk (
    .clk, .rst_n, .push(1'b0), .op_valid, .op_first, .op_last, .op_bank, .op_bypass(1'b0),
    .tile_valid, .tile_ready, .tile_acc, .byp_valid);

  // --------------------------------------------------------------------------
  // scoreboard
  // --------------------------------------------------------------------------
  int checks = 0, fails = 0;
  task automatic fail(string msg);
    fails++;
    if (fails <= 15) $display("[FAIL] t=%0t %s", $time, msg);
  endtask

  logic [31:0] rng_state = 32'h1;
  function automatic logic [31:0] rnd32();
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return rng_state;
  endfunction

  // --------------------------------------------------------------------------
  // weight engine + ring model: writes slots in order while the ring has room;
  // offers each run's entry once its first slot is written
  // --------------------------------------------------------------------------
  int            wr_cnt = 0, fr_cnt = 0, rate_acc = 0, ei = 0;
  always @(posedge clk) cyc <= cyc + 1;
  logic [1:0]  kind_force = 2'd0;          // fault cases: the entry the ring offers, altered
  int          n_force    = 0;
  logic        r_ent_valid;                // the entry as the ring (model or real) offers it
  logic [5:0]  r_ent_layer;
  logic [4:0]  r_ent_tag;
  logic [1:0]  r_ent_kind;
  logic [15:0] r_ent_n, r_ent_k;
  assign ent_valid  = r_ent_valid;
  assign ent_layer  = r_ent_layer;
  assign ent_tag    = r_ent_tag;
  assign ent_kind   = (kind_force != 0) ? kind_force : r_ent_kind;
  assign ent_n      = (n_force != 0) ? 16'(n_force) : r_ent_n;
  assign ent_k      = r_ent_k;

  // the weight engine's pace: RATE 0 one slot (model) or beat (real ring) a cycle; 1 the bus,
  // 5 slots / 10 beats per 13 cycles; 2 random. COLD holds back a tensor's run until its command.
  function automatic bit pace();
    case (rate_mode)
      0: return 1'b1;
      1: begin rate_acc += USE_RING ? 10 : 5; if (rate_acc >= 13) begin rate_acc -= 13; return 1'b1; end return 1'b0; end
      default: return ((rnd32() % 3) == 0);
    endcase
  endfunction

  // checks common to both rings: every read in the live window, frees, entries popped
  always @(posedge clk) begin
    if (rst_n) begin
      if (ring_rd) begin
        int offset;
        offset = (int'(ring_raddr) - (fr_cnt % 1024) + 1024) % 1024;
        checks++;
        if (offset >= wr_cnt - fr_cnt)
          fail($sformatf("ring read of slot position %0d: not in the live window (freed %0d, written %0d)",
                         ring_raddr, fr_cnt, wr_cnt));
      end
      if (ring_free) begin
        fr_cnt <= fr_cnt + ring_free_n;
        checks++;
        if (fr_cnt + ring_free_n > wr_cnt) fail($sformatf("freed %0d slots, only %0d written", fr_cnt + ring_free_n, wr_cnt));
      end
      if (ent_valid && ent_ready) ei <= ei + 1;
    end
  end

  generate
    if (!USE_RING) begin : g_ring_model
      // weight engine + ring model: writes slots in order while the ring has room; offers each
      // run's entry once its first slot is written
      logic [1023:0] ring_mem [1024];
      logic [1023:0] ring_q [W_LAT];
      assign ring_wptr   = 11'(wr_cnt);
      assign ring_rdata  = ring_q[W_LAT-1];
      assign r_ent_valid = (ei < ncmd) && (wr_cnt > ent_first[ei]);
      assign r_ent_layer = (ei < ncmd) ? 6'(cmds[ei].layer) : '0;
      assign r_ent_tag   = (ei < ncmd) ? 5'(cmds[ei].tag)   : '0;
      assign r_ent_kind  = 2'd0;
      assign r_ent_n     = (ei < ncmd) ? 16'(cmds[ei].n)    : '0;
      assign r_ent_k     = (ei < ncmd) ? 16'(cmds[ei].k)    : '0;
      always @(posedge clk) begin
        if (rst_n) begin
          bit go;
          go = pace();
          if (cold != 0)
            for (int i = 1; i < ncmd; i++)
              if (ent_first[i] == wr_cnt && cyc < post_cyc[i] + 20) go = 1'b0;     // the weight engine is late
          if (go && wr_cnt < slots.size() && (wr_cnt - fr_cnt) < 1024) begin
            ring_mem[wr_cnt % 1024] <= slots[wr_cnt];
            wr_cnt <= wr_cnt + 1;
          end
        end
        ring_q[0] <= ring_rd ? ring_mem[ring_raddr] : 'x;
        for (int i = 1; i < W_LAT; i++) ring_q[i] <= ring_q[i-1];
      end
    end else begin : g_ring_real
      // the real me_weight_ring (block RAM primitives), fed as the weight engine would: every
      // tensor's descriptor, then its beats as DDR holds them, at the chosen pace
      logic        run_valid = 1'b0, run_ready, in_valid = 1'b0, in_ready, err_free;
      logic [5:0]  run_layer = '0;
      logic [4:0]  run_tag = '0;
      logic [15:0] run_n = '0, run_k = '0;
      logic [23:0] run_beats = '0;
      logic [511:0] in_data = '0;
      logic [31:0] perf_slots, perf_full;
      me_weight_ring u_ring (
        .clk, .rst_n,
        .run_valid, .run_ready, .run_layer, .run_tag, .run_kind(2'd0), .run_n, .run_k, .run_beats,
        .in_valid, .in_ready, .in_data,
        .ent_valid(r_ent_valid), .ent_ready, .ent_layer(r_ent_layer), .ent_tag(r_ent_tag), .ent_kind(r_ent_kind),
        .ent_n(r_ent_n), .ent_k(r_ent_k),
        .ring_wptr, .ring_rd, .ring_raddr, .ring_rdata, .ring_free, .ring_free_n,
        .err_free, .perf_slots, .perf_full);
      always @(negedge clk) begin
        if (rst_n) begin
          run_valid = (di < ncmd);
          if (di < ncmd) begin
            run_layer = 6'(cmds[di].layer); run_tag = 5'(cmds[di].tag);
            run_n = 16'(cmds[di].n); run_k = 16'(cmds[di].k); run_beats = 24'(cmds[di].beats);
          end
          in_valid = 1'b0;
          if (bq < beat_q.size()) begin
            bit go;
            go = pace();
            if (cold != 0)
              for (int i = 1; i < ncmd; i++)
                if (ent_first_beat[i] == bq && cyc < post_cyc[i] + 20) go = 1'b0;    // the weight engine is late
            in_valid = go;
            in_data  = beat_q[bq];
          end
        end else begin
          run_valid = 1'b0; in_valid = 1'b0;
        end
      end
      always @(posedge clk) begin
        if (rst_n) begin
          if (run_valid && run_ready) di++;
          if (in_valid && in_ready)   bq++;
          if (err_free) fail("me_weight_ring err_free set");
        end
      end
      // the ring's slots written, unwrapped: read 1 ns after the edge, once ring_wptr has settled
      // (and before the mid-cycle monitors read wr_cnt)
      always @(posedge clk) begin
        #1;
        if (rst_n) begin
          wr_cnt += 11'(ring_wptr - wptr_q);
          wptr_q  = ring_wptr;
        end else begin
          wptr_q = '0;
        end
      end
    end
  endgenerate

  // --------------------------------------------------------------------------
  // row store window model: the current command's window
  // --------------------------------------------------------------------------
  int cur = 0;
  logic [511:0] win_q [X_LAT];
  assign win_rdata = win_q[X_LAT-1];
  always @(posedge clk) begin
    win_q[0] <= win_rd ? win_mem[cur][win_raddr] : 'x;
    for (int i = 1; i < X_LAT; i++) win_q[i] <= win_q[i-1];
  end

  // drain model
  // drain model: 0 always ready; 1 random (1 in 3); 2 stalling: ready, then now and then held low for up
  // to 300 cycles, longer than a tile, so the credit rule (P4) is exercised
  int stall = 0;
  always @(negedge clk)
    case (drain_mode)
      0: tile_ready = 1'b1;
      1: tile_ready = ((rnd32() % 3) == 0);
      default: begin
        if (stall > 0) begin tile_ready = 1'b0; stall--; end
        else begin tile_ready = 1'b1; if ((rnd32() % 16) == 0) stall = rnd32() % 300; end
      end
    endcase

  // --------------------------------------------------------------------------
  // monitors
  // --------------------------------------------------------------------------
  bit fault_phase = 1'b0;
  // the testbench's own bubble count: a cycle the walker is busy, the slot it needs next is not
  // written yet, and it issues nothing. The read request leaves one cycle after the decision, so the
  // condition is sampled mid-cycle (the slot it needs: the expected read not yet on ring_rd) and judged
  // at the next mid-cycle, against the request that cycle carries
  int bub_ref = 0;
  bit bub_wait = 1'b0;
  always @(negedge clk) begin
    if (bub_wait && !ring_rd) bub_ref++;
    bub_wait = 1'b0;
    if (rst_n && !fault_phase && !cnt_clr && busy) begin
      int k;
      k = ring_rd ? 1 : 0;                     // the read on ring_rd now was decided last cycle
      if (exp_iss.size() > k) bub_wait = (exp_iss[k].slot >= wr_cnt);
    end
  end
  int n_iss = 0, n_beat = 0, n_tile = 0, n_scale = 0, done_cnt = 0;

  always @(posedge clk) begin
    if (rst_n) begin
      if (fault_phase && (ring_rd || op_valid || (ent_valid && ent_ready)))
        fail("a faulted command read the ring or popped an entry");
      if (ring_rd && !fault_phase) begin
        checks++;
        if (exp_iss.size() == 0) fail("more ring reads than expected");
        else begin
          automatic iss_s x = exp_iss.pop_front();
          if (x.sc != !win_rd || ring_raddr != 10'(x.slot % 1024) || (!x.sc && win_raddr != 10'(x.waddr)))
            fail($sformatf("read %0d: ring %0d window %0d (rd %0b), expected %s slot %0d window %0d",
                           n_iss, ring_raddr, win_raddr, win_rd, x.sc ? "scale" : "weight", x.slot % 1024, x.waddr));
        end
        n_iss++;
      end
      if (scale_valid) begin
        checks++;
        if (exp_scale.size() == 0) fail("more scale slots than expected");
        else begin
          automatic logic [511:0] s = exp_scale.pop_front();
          automatic int sg = exp_scale_g.pop_front();
          if (scale_data !== s || scale_grp != 11'(sg))
            fail($sformatf("scale %0d: group %0d, data %s, expected group %0d", n_scale, scale_grp,
                           (scale_data === s) ? "ok" : "WRONG", sg));
        end
        n_scale++;
      end
      if (op_valid) begin
        checks++;
        if (exp_beat.size() == 0) fail("more beats than expected");
        else begin
          automatic beat_s b = exp_beat.pop_front();
          if (op_w !== sext_w(slots[b.slot]))           fail($sformatf("beat %0d: weights are not ring slot %0d", n_beat, b.slot));
          if (op_x !== sext_x(win_mem[b.cmd][b.waddr]))  fail($sformatf("beat %0d: activations are not window entry %0d", n_beat, b.waddr));
          if (op_dp !== '0)                             fail($sformatf("beat %0d: pre-adder input not 0", n_beat));
          if (op_first !== b.first || op_last !== b.last)
            fail($sformatf("beat %0d: first/last %0b%0b, expected %0b%0b", n_beat, op_first, op_last, b.first, b.last));
        end
        n_beat++;
      end
      if (tile_valid && tile_ready) begin
        checks++;
        if (exp_tag.size() == 0) fail("unexpected extra tile");
        else begin
          automatic tag_s t = exp_tag.pop_front();
          if (!tile_tag_valid) fail($sformatf("tile %0d: no tag", n_tile));
          if (tile_tag.grp != 13'(t.grp) || tile_tag.rgrp != 3'(t.rg) || tile_tag.rvalid != 4'(t.rv) ||
              tile_tag.last != 1'(t.last) || tile_tag.cmd != 6'(t.cmd) || tile_tag.dest != MEI_DEST_DRAIN)
            fail($sformatf("tile %0d: tag cmd=%0d g=%0d rg=%0d rv=%b last=%0d, expected cmd=%0d g=%0d rg=%0d rv=%b last=%0d",
                           n_tile, tile_tag.cmd, tile_tag.grp, tile_tag.rgrp, tile_tag.rvalid, tile_tag.last,
                           t.cmd, t.grp, t.rg, 4'(t.rv), t.last));
          for (int r = 0; r < 4; r++) begin
            if (!t.rv[r]) continue;                       // idle rows hold garbage by contract
            for (int j = 0; j < 8; j++) begin
              logic signed [MEI_ACC_W-1:0] got, want;
              got  = mei_acc_get(tile_acc, r, j);
              want = acc_all[acc_base[t.cmd] + (4 * t.rg + r) * cmds[t.cmd].n + 8 * t.grp + j];
              checks++;
              if (got !== want)
                fail($sformatf("tile %0d (%s) row %0d channel %0d: got %0d, expected %0d", n_tile,
                               names[t.cmd], 4 * t.rg + r, 8 * t.grp + j, got, want));
            end
          end
        end
        n_tile++;
      end
      if (cmd_done && !fault_phase) begin
        done_cnt++;
        checks++;
        if (cmd_fault) fail($sformatf("command %0d faulted with %02x", done_cnt - 1, cmd_fault_code));
      end
    end
  end

  // --------------------------------------------------------------------------
  // configuration and golden files
  // --------------------------------------------------------------------------
  task automatic load_config();
    int fd, n, v; string k, s;
    fd = $fopen("tw_run.cfg", "r");
    if (fd == 0) return;
    while (!$feof(fd)) begin
      n = $fscanf(fd, "%s %s\n", k, s);
      if (n != 2) continue;
      void'($sscanf(s, "%d", v));
      case (k)
        "DIR":    dir = s;         "CASES":  cases = s;
        "FAULTS": faults = v;      "RATE":   rate_mode = v;
        "DRAIN":  drain_mode = v;  "SEED":   seed = v;
        "COLD":   cold = v;        "RESET_AT": reset_at = v;
        default: $display("[WARN] unknown config key %s", k);
      endcase
    end
    $fclose(fd);
  endtask

  // one command's files; appends its slots and the expected reads, beats, scales, tiles
  task automatic load_case(int i, string name);
    int fd, base0, kch, nrg, groups;
    logic [63:0] w;
    int f[10];
    logic [511:0] beat, beat2, wtmp [576];
    fd = $fopen({dir, "/gemm_", name, "_cmd.txt"}, "r");
    if (fd == 0) begin $display("[FATAL] no vectors for case %s", name); $finish; end
    void'($fscanf(fd, "%h %h %h %h %h %h %h %h %h %h %h", w, f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7], f[8], f[9]));
    $fclose(fd);
    names[i] = name;
    cmds[i].word = w;  cmds[i].layer = f[0]; cmds[i].tag = f[1]; cmds[i].n = f[2]; cmds[i].k = f[3];
    cmds[i].beats = f[4]; cmds[i].nstep = f[5]; cmds[i].nused = f[6];
    cmds[i].abuf = w[47:44]; cmds[i].iclass = w[34:31]; cmds[i].rowset = w[20:19];
    $readmemh({dir, "/gemm_", name, "_win.mem"}, wtmp);
    for (int e = 0; e < 576; e++) win_mem[i][e] = wtmp[e];
    acc_base[i] = acc_all.size();
    $readmemh({dir, "/gemm_", name, "_acc.mem"}, atmp, 0, cmds[i].nused * cmds[i].n - 1);
    for (int e = 0; e < cmds[i].nused * cmds[i].n; e++)
      acc_all.push_back(atmp[e]);
    // the run: per group the scale slot, then the weight slots, two beats each
    ent_first[i] = slots.size();
    ent_first_beat[i] = beat_q.size();
    kch = cmds[i].k / 16; groups = cmds[i].n / 32; nrg = (cmds[i].nused + 3) / 4;
    fd = $fopen({dir, "/gemm_", name, "_stream.mem"}, "r");
    for (int g = 0; g < groups; g++) begin
      void'($fscanf(fd, "%h\n", beat));
      slots.push_back({512'b0, beat});
      beat_q.push_back(beat);
      for (int s = 0; s < 4 * kch; s++) begin
        void'($fscanf(fd, "%h\n", beat));
        void'($fscanf(fd, "%h\n", beat2));
        slots.push_back({beat2, beat});
        beat_q.push_back(beat); beat_q.push_back(beat2);
      end
    end
    $fclose(fd);
    // the walker's reference order
    base0 = ent_first[i];
    for (int g = 0; g < groups; g++) begin
      exp_iss.push_back('{1'b1, base0, 0});
      exp_scale.push_back(slots[base0][511:0]);
      exp_scale_g.push_back(g);
      base0++;
      for (int ct = 0; ct < 4; ct++) begin
        for (int rg = 0; rg < nrg; rg++) begin
          int left = cmds[i].nused - 4 * rg;
          exp_tag.push_back('{i, 4 * g + ct, rg, (left >= 4) ? 15 : (1 << left) - 1,
                              (g == groups - 1 && ct == 3 && rg == nrg - 1)});
          for (int c = 0; c < kch; c++) begin
            exp_iss.push_back('{1'b0, base0 + c, rg * 96 + c});
            exp_beat.push_back('{base0 + c, rg * 96 + c, i, c == 0, c == kch - 1});
          end
        end
        base0 += kch;
      end
    end
    tot_beats  += groups * 4 * nrg * kch;
    tot_groups += groups;
    $display("case %0d %-12s N %0d K %0d rows %0d (of %0d) beats %0d slots %0d", i, name, cmds[i].n, cmds[i].k,
             cmds[i].nused, cmds[i].nstep, cmds[i].beats, slots.size() - ent_first[i]);
  endtask

  // post one command; returns at its cmd_done with the result
  task automatic post(logic [63:0] w, int id, output bit flt, output logic [7:0] code);
    cmd_word = w; cmd_id = 6'(id); cmd_valid = 1'b1;
    @(posedge clk);                         // taken at this edge (the walker is idle)
    @(negedge clk);
    cmd_valid = 1'b0;
    while (!cmd_done) @(negedge clk);
    flt = cmd_fault; code = cmd_fault_code;
    @(negedge clk);                         // past the done pulse (the monitors count it once)
  endtask

  // the data phase: every command in order, each once its window is filled
  task automatic run_cmds();
    for (int i = 0; i < ncmd; i++) begin
      bit flt; logic [7:0] code;
      cur = i;
      if (cmds[i].abuf != 4) begin win_valid = 1'b1; win_cls = 4'(cmds[i].iclass); end   // the fill is done
      step_rows = 5'(cmds[i].nstep);
      step_logit_rows = (cmds[i].rowset == 1) ? 4'(cmds[i].nused) : 4'd1;
      post_cyc[i] = cyc;
      post(cmds[i].word, i, flt, code);
      if (drain_mode != 0) repeat (rnd32() % 5) @(negedge clk);
    end
  endtask

  // a fault case: no ring read, no pop, the expected code
  task automatic fault_case(logic [63:0] w, int want, string note);
    bit flt; logic [7:0] code;
    post(w, 63, flt, code);
    checks++;
    if (flt !== (want != 0) || code != 8'(want))
      fail($sformatf("fault case %s: fault %0b code %02x, expected code %02x", note, flt, code, want));
    else $display("  ok %02x  %s", want, note);
  endtask

  // --------------------------------------------------------------------------
  // command model and the end-of-run checks
  // --------------------------------------------------------------------------
  initial begin
    int t0, t1;
    load_config();
    for (int i = 0; i < MAXC; i++) post_cyc[i] = 32'h3FFFFFFF;
    if (dir == "" || cases == "") begin
      $display("[FATAL] bad or missing tw_run.cfg");
      $display(">> VERDICT: FAIL <<");
      $finish;
    end
    rng_state = 32'h2545F491 ^ seed;
    if (rng_state == 0) rng_state = 32'h1;
    begin
      automatic string nm = "";
      for (int p = 0; p <= cases.len(); p++) begin
        if (p == cases.len() || cases[p] == ",") begin
          if (nm != "") begin load_case(ncmd, nm); ncmd++; end
          nm = "";
        end else nm = {nm, cases[p]};
      end
    end
    exp_iss0 = exp_iss; exp_beat0 = exp_beat; exp_tag0 = exp_tag; exp_scale0 = exp_scale; exp_scale_g0 = exp_scale_g;
    loaded = 1'b1;
    $display("run: %0d command(s), %0d ring slots, %0d beats, %0d tiles, rate=%0d drain=%0d faults=%0d",
             ncmd, slots.size(), tot_beats, exp_tag.size(), rate_mode, drain_mode, faults);

    // the DSP48E1 models are held in global set/reset for the first 100 ns
    repeat (6) @(negedge clk);
    while ($time < 120) @(negedge clk);
    rst_n = 1'b1;
    repeat (3) @(negedge clk);

    if (faults != 0) begin
      int fd, want, n; logic [63:0] w; string note;
      logic [63:0] q;
      if (cmds[0].tag != 1 || cmds[0].layer != 0) begin
        $display("[FATAL] FAULTS 1 needs CASES to start with l0_q"); $display(">> VERDICT: FAIL <<"); $finish;
      end
      while (wr_cnt == 0) @(negedge clk);   // the ring offers W_Q's entry
      fault_phase = 1'b1;
      step_rows = 5'd5; step_logit_rows = 4'd1;
      $display("fault cases (the reference model's codes):");
      fd = $fopen({dir, "/gemm_faults.txt"}, "r");
      while (!$feof(fd)) begin
        n = $fscanf(fd, "%h %h", w, want);
        if (n != 2) break;
        void'($fgets(note, fd));
        if (want == 0) continue;            // the accepted command: run below as l0_q
        fault_case(w, want, note.substr(1, note.len() - 2));
      end
      $fclose(fd);
      $display("fault cases (hardware only):");
      q = cmds[0].word;
      fault_case((q & ~(64'h3F << 21)) | (64'd1 << 21), 8'h23, "W_Q of layer 1, the ring delivers layer 0's");
      fault_case((q & ~(64'h3F << 58)) | (64'd2 << 58), 8'h20, "OP = NORM sent to the walker");
      fault_case((q & ~(64'h1F << 35)) | (64'd7 << 35) | (64'hF << 44), 8'h23, "tag 7 and A = 15: the tag first");
      win_valid = 1'b1; win_cls = 4'd3;
      fault_case((q & ~(64'hF << 44)) | (64'(4) << 44), 8'h21, "A = WIN holding class O, ICLASS XN");
      win_valid = 1'b0;
      kind_force = 2'd1;
      fault_case(q, 8'h23, "the entry the ring delivers is a VECTOR with tag 1");
      kind_force = 2'd0;
      n_force = 100;
      fault_case(q, 8'h23, "the entry's N = 100: not a multiple of 32");
      n_force = 0;
      win_valid = 1'b0;
      step_logit_rows = 4'd0;
      fault_case((q & ~(64'h3 << 19)) | (64'd1 << 19), 8'h00, "ROWSET_LOGIT with no logit rows: done, nothing read");
      checks++; if (ei != 0) fail("an entry was popped by a fault case");
      fault_phase = 1'b0;
    end

    cnt_clr = 1'b1; @(negedge clk); cnt_clr = 1'b0;
    bub_ref = 0;
    t0 = $time;
    if (reset_at > 0) begin
      // the step faults part-way: the core's soft reset (rst_n here) clears the walker, the array and
      // the ring, and the step is posted again; it must then run exactly as if nothing had happened
      fork
        run_cmds();
        wait (n_beat >= reset_at);
      join_any
      disable fork;
      @(negedge clk);
      cmd_valid = 1'b0;
      rst_n = 1'b0;
      $display("soft reset after %0d beats, %0d tiles, %0d scale slots (walker %s): the step is posted again",
               n_beat, n_tile, n_scale, busy ? "busy" : "idle");
      repeat (8) @(negedge clk);
      wr_cnt = 0; fr_cnt = 0; rate_acc = 0; ei = 0; stall = 0; di = 0; bq = 0;                       // the ring's reset
      n_iss = 0; n_beat = 0; n_tile = 0; n_scale = 0; done_cnt = 0;
      exp_iss = exp_iss0; exp_beat = exp_beat0; exp_tag = exp_tag0; exp_scale = exp_scale0; exp_scale_g = exp_scale_g0;
      win_valid = 1'b0; cur = 0;
      for (int i = 0; i < MAXC; i++) post_cyc[i] = 32'h3FFFFFFF;
      rst_n = 1'b1;
      repeat (3) @(negedge clk);
      checks++; if (busy || tile_tag_valid || ring_rd || op_valid || tile_valid || scale_valid)
        fail("state left over after the soft reset");
      cnt_clr = 1'b1; @(negedge clk); cnt_clr = 1'b0;
      bub_ref = 0;
      t0 = $time;
    end
    run_cmds();
    // wait for the last tile (bounded), then make sure nothing more comes
    for (int guard = 0; guard < 4000 + 4 * slots.size() && exp_tag.size() > 0; guard++) @(posedge clk);
    t1 = $time;
    repeat (60) @(posedge clk);

    checks++; if (exp_iss.size() != 0)   fail($sformatf("%0d ring reads missing", exp_iss.size()));
    checks++; if (exp_beat.size() != 0)  fail($sformatf("%0d beats missing", exp_beat.size()));
    checks++; if (exp_tag.size() != 0)   fail($sformatf("%0d tiles missing", exp_tag.size()));
    checks++; if (exp_scale.size() != 0) fail($sformatf("%0d scale slots missing", exp_scale.size()));
    checks++; if (done_cnt != ncmd)      fail($sformatf("%0d commands done, expected %0d", done_cnt, ncmd));
    checks++; if (fr_cnt != slots.size()) fail($sformatf("%0d slots freed, expected %0d", fr_cnt, slots.size()));
    checks++; if (ei != ncmd)            fail($sformatf("%0d entries popped, expected %0d", ei, ncmd));
    checks++; if (busy)                  fail("walker still busy at the end");
    checks++; if (tile_tag_valid)        fail("tag FIFO not empty at the end");
    checks++; if (err_overrun)           fail("me_array err_overrun set");
    checks++; if (perf_beats != tot_beats || arr_beats != tot_beats || perf_scales != tot_groups)
      fail($sformatf("counters: walker beats %0d scales %0d, array beats %0d; expected %0d, %0d",
                     perf_beats, perf_scales, arr_beats, tot_beats, tot_groups));
    checks++; if (u_chk.err_count != 0)  fail($sformatf("%0d contract protocol errors", u_chk.err_count));
    checks++; if (err_bypass)            fail("me_array err_bypass set");
    checks++; if (byp_valid)             fail("bypass result from a GEMM");
    checks++; if (bubbles != bub_ref)    fail($sformatf("bubbles %0d, the testbench counts %0d", bubbles, bub_ref));

    $display("cycles %0d for %0d beats + %0d scale slots (array use %0.1f%%); bubbles %0d, credit stalls %0d",
             (t1 - t0) / 8, tot_beats, tot_groups, 100.0 * tot_beats / ((t1 - t0) / 8.0), bubbles, perf_stall_credit);
    $display("TW RESULTS: %0d checks, %0d failures", checks, fails);
    $display(">> VERDICT: %s <<", (fails == 0 && checks > 0) ? "PASS" : "FAIL");
    $finish;
  end

  // watchdog: scales with the run
  initial begin
    wait (loaded);
    #(8 * (20000 + 8 * slots.size() + ((drain_mode == 2) ? 400 * exp_tag.size() : 0)));   // a stalling drain: up to 300 cycles a tile
    $display("[FATAL] watchdog expired (reads left %0d, beats left %0d, tiles left %0d)",
             exp_iss.size(), exp_beat.size(), exp_tag.size());
    $display(">> VERDICT: FAIL <<");
    $finish;
  end

endmodule

// variants as their own tops (xelab's .bat wrapper splits -generic_top at '='), all on the
// behavioural array: the walker's logic does not depend on the DSP models, which the real-array
// top (tb_me_tile_walker) covers once as the joint run
module tb_me_tile_walker_model;                 // ring and window latency 2: both read paths direct
  tb_me_tile_walker #(.USE_ARRAY(1'b0)) u_tb ();
endmodule

module tb_me_tile_walker_lat;                   // ring 1, window 3: the ring data is delayed to match
  tb_me_tile_walker #(.USE_ARRAY(1'b0), .W_LAT(1), .X_LAT(3)) u_tb ();
endmodule

module tb_me_tile_walker_ring;                  // the real me_weight_ring between the stream and the walker
  tb_me_tile_walker #(.USE_ARRAY(1'b0), .USE_RING(1'b1)) u_tb ();
endmodule

module tb_me_tile_walker_lat2;                  // ring 3, window 1: the window data is delayed to match
  tb_me_tile_walker #(.USE_ARRAY(1'b0), .W_LAT(3), .X_LAT(1)) u_tb ();
endmodule

`default_nettype wire
