// ============================================================================
// File: tb_me_array.sv
// Block: Matrix engine - unit testbench for me_array (the "L0" test), contract v2
//
// Feeds golden beats (gen_me_vectors.py) into the real array (512 DSP48E1
// models) and checks every result:
//   - every accumulator of every tile (34 bits) against the golden tile;
//   - every bypass beat's 32 cell sums (31 bits) and its tag, in order;
//   - exact latencies: first tile 23 edges after its last beat (MEI_TILE_LAT),
//     every bypass result 22 edges after its beat (MEI_BYP_LAT);
//   - tile_valid / tile_acc stable until taken; no extra tiles or results;
//     perf counters; err_overrun and err_bypass never set;
//   - the contract checker (me_if_checker) on the same ports: 0 errors.
//
// Configuration: lines "KEY value" in me_run.cfg (current directory):
//   BEATS TILES BYP            golden files
//   NBEAT NTILE NBYP           their sizes (manifest.txt)
//   MODE  0 handshake (GEMM: 2 credits, drain may stall) | 1 push (attention:
//         tile_ready held at 1, no credits, rule P10)
//   GAP   1: random bubbles between beats, garbage on every idle operand
//   DRAIN 1: (handshake mode) the drain takes tiles at random times
//   SEED  n
// (xsim.bat splits plusargs at '=', so the run script uses the file.)
// Ends with "ME_ARRAY RESULTS: N checks, F failures" and a PASS/FAIL verdict.
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module tb_me_array;
  import me_pkg::*;
  import me_if_pkg::*;

  localparam int R       = ME_R;
  localparam int C       = ME_C;
  localparam int D       = ME_D;
  localparam int NACC    = R * C;
  localparam int W_BITS  = C * D * ME_A_W;              // 1,664
  localparam int DP_BITS = D * ME_DP_W;                 //   192
  localparam int X_BITS  = R * D * ME_X_W;              // 1,024
  localparam int BEAT_W  = 16 + X_BITS + DP_BITS + W_BITS;
  localparam int TILE_W  = NACC * ME_ACC_W;             // 1,088
  localparam int BYPS_W  = NACC * ME_SUM_W;             //   992
  localparam int MAXB    = 4096;
  localparam int MAXT    = 256;
  localparam int MAXY    = 1024;
  localparam real PERIOD = 8.0;                         // 125 MHz

  // --------------------------------------------------------------------------
  // clock, reset, DUT
  // --------------------------------------------------------------------------
  logic clk   = 1'b0;
  logic rst_n = 1'b0;
  always #(PERIOD / 2.0) clk = ~clk;

  logic                 op_valid  = 1'b0;
  logic                 op_first  = 1'b0;
  logic                 op_last   = 1'b0;
  logic                 op_bank   = 1'b0;
  logic                 op_bypass = 1'b0;
  logic [7:0]           op_btag   = '0;
  logic [W_BITS-1:0]    op_w      = '0;
  logic [DP_BITS-1:0]   op_dp     = '0;
  logic [X_BITS-1:0]    op_x      = '0;
  logic                 tile_valid;
  logic                 tile_ready = 1'b1;
  logic [TILE_W-1:0]    tile_acc;
  logic                 byp_valid;
  logic [BYPS_W-1:0]    byp_sum;
  logic [7:0]           byp_tag;
  logic                 err_overrun, err_bypass;
  logic [31:0]          perf_beats, perf_tiles, perf_byp;
  logic                 push = 1'b0;

  me_array dut (
    .clk, .rst_n,
    .op_valid, .op_first, .op_last, .op_bank, .op_bypass, .op_btag, .op_w, .op_dp, .op_x,
    .tile_valid, .tile_ready, .tile_acc,
    .byp_valid, .byp_sum, .byp_tag,
    .err_overrun, .err_bypass, .perf_beats, .perf_tiles, .perf_byp);

  me_if_checker u_chk (
    .clk, .rst_n, .push,
    .op_valid, .op_first, .op_last, .op_bank, .op_bypass,
    .tile_valid, .tile_ready, .tile_acc, .byp_valid);

  // --------------------------------------------------------------------------
  // golden data and configuration
  // --------------------------------------------------------------------------
  logic [BEAT_W-1:0]  beats  [MAXB];
  logic [TILE_W-1:0]  exp_t  [MAXT];
  logic [8+BYPS_W-1:0] exp_y [MAXY];
  string beats_file, tiles_file, byp_file;
  int    nbeat, ntile, nbyp, mode, gap_mode, drain_mode, seed;

  // --------------------------------------------------------------------------
  // random numbers (xorshift32; xsim lacks a seeded $urandom)
  // --------------------------------------------------------------------------
  logic [31:0] rng_state = 32'h1;
  function automatic logic [31:0] rnd32();
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return rng_state;
  endfunction

  // --------------------------------------------------------------------------
  // monitor and checker (samples on every rising edge)
  // --------------------------------------------------------------------------
  int  cyc = 0, checks = 0, fails = 0;
  int  tiles_checked = 0, byp_checked = 0, ret_count = 0;
  int  last_cyc [MAXT];
  int  last_idx = 0;
  int  byp_cyc [MAXY];
  int  byp_idx = 0;
  int  valid_cyc0 = -1;
  logic              prev_valid = 1'b0, prev_ready = 1'b0;
  logic [TILE_W-1:0] prev_acc   = '0;

  task automatic fail(string msg);
    fails++;
    if (fails <= 12) $display("[FAIL] %s", msg);
  endtask

  always @(posedge clk) cyc <= cyc + 1;

  always @(posedge clk) begin
    if (rst_n) begin
      if (op_valid) begin
        if (op_bypass) begin
          if (byp_idx < MAXY) byp_cyc[byp_idx] = cyc;
          byp_idx++;
        end else if (op_last) begin
          if (last_idx < MAXT) last_cyc[last_idx] = cyc;
          last_idx++;
        end
      end

      // a held tile must not change until it is taken
      if (prev_valid && !prev_ready) begin
        checks++;
        if (!tile_valid || tile_acc !== prev_acc)
          fail($sformatf("tile_valid/tile_acc changed while waiting for tile_ready at edge %0d", cyc));
      end
      prev_valid = tile_valid;
      prev_ready = tile_ready;
      prev_acc   = tile_acc;

      if (tile_valid && valid_cyc0 < 0) valid_cyc0 = cyc;

      // tiles
      if (tile_valid && tile_ready) begin
        checks++;
        if (tiles_checked >= ntile) begin
          fail($sformatf("unexpected extra tile %0d", tiles_checked));
        end else begin
          for (int i = 0; i < NACC; i++) begin
            logic signed [ME_ACC_W-1:0] got, want;
            got  = tile_acc[i*ME_ACC_W +: ME_ACC_W];
            want = exp_t[tiles_checked][i*ME_ACC_W +: ME_ACC_W];
            checks++;
            if (got !== want)
              fail($sformatf("tile %0d acc(r=%0d,j=%0d): got %0d, expected %0d",
                             tiles_checked, i / C, i % C, got, want));
          end
        end
        tiles_checked++;
        ret_count++;
      end

      // bypass results: exact latency, sums and tag
      if (byp_valid) begin
        checks++;
        if (byp_checked >= nbyp) begin
          fail($sformatf("unexpected extra bypass result %0d", byp_checked));
        end else begin
          checks++;
          if (cyc - byp_cyc[byp_checked] != MEI_BYP_LAT)
            fail($sformatf("bypass %0d: result %0d edges after its beat, expected %0d",
                           byp_checked, cyc - byp_cyc[byp_checked], MEI_BYP_LAT));
          checks++;
          if (byp_tag !== exp_y[byp_checked][BYPS_W +: 8])
            fail($sformatf("bypass %0d: tag %0d, expected %0d", byp_checked, byp_tag,
                           exp_y[byp_checked][BYPS_W +: 8]));
          for (int i = 0; i < NACC; i++) begin
            logic signed [ME_SUM_W-1:0] got, want;
            got  = byp_sum[i*ME_SUM_W +: ME_SUM_W];
            want = exp_y[byp_checked][i*ME_SUM_W +: ME_SUM_W];
            checks++;
            if (got !== want)
              fail($sformatf("bypass %0d sum(r=%0d,j=%0d): got %0d, expected %0d",
                             byp_checked, i / C, i % C, got, want));
          end
        end
        byp_checked++;
      end
    end
  end

  // drain: always ready (push mode, or handshake without DRAIN), or ready at random
  always @(negedge clk) begin
    if (mode == 0 && drain_mode != 0) tile_ready = ((rnd32() % 3) == 0);
    else                              tile_ready = 1'b1;
  end

  // --------------------------------------------------------------------------
  // producer
  // --------------------------------------------------------------------------
  function automatic logic [W_BITS-1:0] rnd_w();
    for (int i = 0; i < W_BITS / 32; i++) rnd_w[i*32 +: 32] = rnd32();
  endfunction
  function automatic logic [X_BITS-1:0] rnd_x();
    for (int i = 0; i < X_BITS / 32; i++) rnd_x[i*32 +: 32] = rnd32();
  endfunction
  function automatic logic [DP_BITS-1:0] rnd_dp();
    for (int i = 0; i < DP_BITS / 32; i++) rnd_dp[i*32 +: 32] = rnd32();
  endfunction

  int   bi, issued_tiles, credits, guard, lat0;
  logic bank, issue, in_tile;
  logic [15:0] ctrl;

  task automatic load_config();
    int fd, n, v; string k, s;
    beats_file = ""; tiles_file = ""; byp_file = "";
    nbeat = 0; ntile = 0; nbyp = 0; mode = 0; gap_mode = 0; drain_mode = 0; seed = 1;
    fd = $fopen("me_run.cfg", "r");
    if (fd == 0) return;
    while (!$feof(fd)) begin
      n = $fscanf(fd, "%s %s\n", k, s);
      if (n != 2) continue;
      void'($sscanf(s, "%d", v));
      case (k)
        "BEATS": beats_file = s;  "TILES": tiles_file = s;  "BYP": byp_file = s;
        "NBEAT": nbeat = v;       "NTILE": ntile = v;       "NBYP": nbyp = v;
        "MODE":  mode = v;        "GAP":   gap_mode = v;    "DRAIN": drain_mode = v;
        "SEED":  seed = v;
        default: $display("[WARN] unknown config key %s", k);
      endcase
    end
    $fclose(fd);
  endtask

  initial begin
    load_config();
    if (beats_file == "" || nbeat <= 0 || nbeat > MAXB || ntile > MAXT || nbyp > MAXY) begin
      $display("[FATAL] bad or missing me_run.cfg");
      $display(">> VERDICT: FAIL <<");
      $finish;
    end
    rng_state = 32'h2545F491 ^ seed;
    if (rng_state == 0) rng_state = 32'h1;
    push = (mode == 1);

    $display("case: %0d beats, %0d tiles, %0d bypass beats, mode=%s gap=%0d drain=%0d",
             nbeat, ntile, nbyp, (mode == 1) ? "push" : "handshake", gap_mode, drain_mode);
    $readmemh(beats_file, beats);
    $readmemh(tiles_file, exp_t);
    $readmemh(byp_file,   exp_y);

    // the DSP48E1 models are held in global set/reset (glbl.GSR) for the
    // first 100 ns, as the FPGA is during configuration: no operand may
    // enter before that, or it is lost
    repeat (6) @(negedge clk);
    while ($time < 120) @(negedge clk);
    rst_n = 1'b1;
    repeat (3) @(negedge clk);

    bi = 0; issued_tiles = 0; bank = 1'b0; in_tile = 1'b0;
    while (bi < nbeat) begin
      @(negedge clk);
      ctrl    = beats[bi][BEAT_W-16 +: 16];
      credits = 2 - (issued_tiles - ret_count);
      issue   = 1'b1;
      if (gap_mode != 0 && ((rnd32() % 4) == 0) && !in_tile) issue = 1'b0;   // bubbles between tiles/passes
      if (gap_mode != 0 && ((rnd32() % 6) == 0))             issue = 1'b0;   // and inside them
      if (mode == 0 && ctrl[0] && credits <= 0)              issue = 1'b0;   // P4 (handshake only)

      if (issue) begin
        {op_x, op_dp, op_w} = beats[bi][0 +: X_BITS + DP_BITS + W_BITS];
        op_valid  = 1'b1;
        op_first  = ctrl[0];
        op_last   = ctrl[1];
        op_bypass = ctrl[2];
        op_btag   = ctrl[15:8];
        op_bank   = bank;
        if (ctrl[0]) begin issued_tiles++; in_tile = 1'b1; end
        if (ctrl[1]) begin bank = ~bank; in_tile = 1'b0; end
        bi++;
      end else begin
        op_valid  = 1'b0;                    // garbage on every idle operand
        op_first  = rnd32();
        op_last   = rnd32();
        op_bypass = rnd32();
        op_btag   = rnd32();
        op_bank   = rnd32();
        op_w      = rnd_w();
        op_dp     = rnd_dp();
        op_x      = rnd_x();
      end
    end
    @(negedge clk);
    op_valid = 1'b0; op_first = 1'b0; op_last = 1'b0; op_bypass = 1'b0;

    guard = 0;
    while ((tiles_checked < ntile || byp_checked < nbyp) && guard < 2000) begin
      @(posedge clk);
      guard++;
    end
    repeat (60) @(posedge clk);          // nothing more may appear

    // ---- end-of-run checks ----
    checks++; if (tiles_checked != ntile) fail($sformatf("%0d of %0d tiles came out", tiles_checked, ntile));
    checks++; if (byp_checked != nbyp)    fail($sformatf("%0d of %0d bypass results came out", byp_checked, nbyp));
    checks++; if (tile_valid)             fail("tile_valid still high at the end");
    checks++; if (err_overrun)            fail("err_overrun is set");
    checks++; if (err_bypass)             fail("err_bypass is set");
    checks++; if (perf_beats !== nbeat)   fail($sformatf("perf_beats %0d, expected %0d", perf_beats, nbeat));
    checks++; if (perf_tiles !== ntile)   fail($sformatf("perf_tiles %0d, expected %0d", perf_tiles, ntile));
    checks++; if (perf_byp !== nbyp)      fail($sformatf("perf_byp %0d, expected %0d", perf_byp, nbyp));
    checks++; if (u_chk.err_count != 0)   fail($sformatf("%0d contract protocol errors", u_chk.err_count));
    if (ntile > 0) begin
      checks++;
      lat0 = valid_cyc0 - last_cyc[0];
      if (lat0 !== MEI_TILE_LAT)
        fail($sformatf("latency last beat -> tile_valid is %0d edges, expected %0d", lat0, MEI_TILE_LAT));
      else
        $display("latency last beat -> tile_valid: %0d edges (as specified)", lat0);
    end
    if (nbyp > 0) $display("latency bypass beat -> byp_valid: checked on every beat (%0d edges)", MEI_BYP_LAT);

    $display("ME_ARRAY RESULTS: %0d checks, %0d failures", checks, fails);
    $display(">> VERDICT: %s <<", (fails == 0 && checks > 0) ? "PASS" : "FAIL");
    $finish;
  end

  // watchdog: scales with the test (beats take 1-2 cycles each even with
  // bubbles and a slow drain), so a hung DUT fails in seconds, not hours
  initial begin
    wait (nbeat > 0);
    #(PERIOD * (2000 + 16 * nbeat));
    $display("[FATAL] watchdog expired");
    $display(">> VERDICT: FAIL <<");
    $finish;
  end

endmodule

`default_nettype wire
