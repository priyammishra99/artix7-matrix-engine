// ============================================================================
// File: tb_me_row_store.sv
// Block: Matrix engine - unit testbench for me_row_store (storage only)
//
// A reference model of every memory, and these cases, in order:
//   T1 window loader on real data: the reference model's INT8 window and
//      row maxima for one GEMM (vectors/: gemm_<case>_win.mem,
//      _absmax.mem), loaded as the stage-1 board run will; every window
//      entry read back as the walker reads it, and every row's constants
//   T2 window fill through the quantiser's port: 24 rows x 96 chunks of
//      random data, every entry read back; then loader shapes K/16 = 1, 5, 37
//      (partial last beats); win_valid / win_cls through each fill
//   T3 the constants' two copies: a GEMM's tiles still on their way to the
//      drain while the next window is filled: the drain-side read answers
//      from the old copy until that GEMM's last tile handshake, then from the
//      new; a fill into a copy still in use, and a fill while the walker is
//      busy, set err_fill (and the second is refused)
//   T4 staging: random writes with half enables (INT16 / INT32 views), reads
//      at 2 cycles, the ends of the address range; a dump compared beat by beat
//   T5 residual: the same with element enables; a dump (both memories: the
//      RAMB36E1 build and the behavioural one, by top)
//   T6 own K/V: byte-enable writes, reads
//   T7 a staging read during a dump sets err_port
//   T8 a soft reset during a load: the state clears; a fresh load then matches
// Configuration (rs_run.cfg, "KEY value" lines): DIR <vectors>, CASE <name>,
// SEED n. Tops: tb_me_row_store (RAMB36E1 residual), tb_me_row_store_model.
// Ends with "RS RESULTS: N checks, F failures" and ">> VERDICT: PASS/FAIL <<".
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module tb_me_row_store #(parameter bit USE_PRIM = 1'b1);

  logic clk = 1'b0;
  always #4 clk = ~clk;
  logic rst_n = 1'b0;

  // ---- DUT ----
  logic         win_rd = 0;            logic [9:0] win_raddr = '0;
  logic [511:0] win_rdata;             logic win_valid; logic [3:0] win_cls;
  logic         fq_start = 0;          logic [3:0] fq_cls = '0;   logic fill_ready;
  logic         fq_wr = 0;             logic [4:0] fq_row = '0;   logic [6:0] fq_chunk = '0;
  logic [127:0] fq_data = '0;
  logic         fq_rc_wr = 0;          logic [4:0] fq_rc_row = '0;
  logic [14:0]  fq_absmax = '0;        logic [4:0] fq_shift = '0;  logic fq_done = 0;
  logic         walker_busy = 0, ent_pop = 0, tile_hs = 0, tile_last = 0;
  logic [4:0]   drn_row = '0;          logic [14:0] drn_absmax;    logic [4:0] drn_shift;
  logic         stg_we = 0;            logic [15:0] stg_waddr = '0; logic [1:0] stg_whe = '0;
  logic [31:0]  stg_wdata = '0;        logic stg_rd = 0;           logic [15:0] stg_raddr = '0;
  logic [31:0]  stg_rdata;
  logic         res_we = 0;            logic [12:0] res_waddr = '0; logic [1:0] res_wee = '0;
  logic [63:0]  res_wdata = '0;        logic res_rd = 0;           logic [12:0] res_raddr = '0;
  logic [63:0]  res_rdata;
  logic         kvo_we = 0;            logic [10:0] kvo_waddr = '0; logic [7:0] kvo_wbe = '0;
  logic [63:0]  kvo_wdata = '0;        logic kvo_rd = 0;           logic [10:0] kvo_raddr = '0;
  logic [63:0]  kvo_rdata;
  logic         ld_start = 0;          logic [3:0] ld_cls = '0;    logic [4:0] ld_nrows = '0;
  logic [6:0]   ld_kch = '0;           logic ld_valid = 0, ld_ready, ld_busy;
  logic [511:0] ld_data = '0;
  logic         dp_start = 0, dp_res = 0; logic [15:0] dp_first = '0, dp_beats = '0;
  logic         dp_valid, dp_ready = 0, dp_busy;  logic [511:0] dp_data;
  logic         err_fill, err_port;

  me_row_store #(.USE_PRIM(USE_PRIM)) dut (.*);

  int checks = 0, fails = 0;
  task automatic fail(string m);
    fails++;
    if (fails <= 20) $display("[FAIL] t=%0t %s", $time, m);
  endtask
  task automatic check(bit ok, string m);
    checks++;
    if (!ok) fail(m);
  endtask

  logic [31:0] rng = 32'h1;
  function automatic logic [31:0] rnd();
    rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
    return rng;
  endfunction
  function automatic logic [127:0] rnd128();
    return {rnd(), rnd(), rnd(), rnd()};
  endfunction

  // ---- reference models ----
  logic [511:0] m_win [576];
  logic [14:0]  m_absmax [24];
  logic [4:0]   m_shift [24];
  logic [31:0]  m_stg [36864];
  logic [63:0]  m_res [6912];
  logic [63:0]  m_kvo [1152];

  // ---- helpers ----
  task automatic tick(int n = 1); repeat (n) @(negedge clk); endtask

  // read every window entry as the walker does (2-cycle latency), compare with m_win[0 .. n-1]
  task automatic check_window(int n_entries, string what);
    int bad = 0;
    // a read set up before edge e is in the RAM's register at edge e and on rdata after edge e + 1
    for (int e = 0; e < n_entries + 1; e++) begin
      win_rd = (e < n_entries); win_raddr = 10'(e);
      @(posedge clk);
      if (e >= 1) begin
        #1;
        checks++;
        if (win_rdata !== m_win[e - 1]) begin
          bad++;
          if (bad <= 3) fail($sformatf("%s: window entry %0d differs", what, e - 1));
        end
      end
      @(negedge clk);
    end
    win_rd = 0;
    if (bad > 3) fail($sformatf("%s: %0d window entries differ in all", what, bad));
  endtask

  task automatic check_consts(int n_rows, string what);
    for (int r = 0; r < n_rows; r++) begin
      drn_row = 5'(r); #1;
      check(drn_absmax === m_absmax[r] && drn_shift === m_shift[r],
            $sformatf("%s: row %0d constants %0d/%0d, expected %0d/%0d", what, r, drn_absmax, drn_shift,
                      m_absmax[r], m_shift[r]));
    end
  endtask

  // the loader: beat 0 the constants, then each row's chunks 4 a beat; the stream at a random pace
  task automatic load_window(int nrows, int kch, int cls);
    logic [511:0] b;
    logic [511:0] beats [$];
    b = '0;
    for (int r = 0; r < 24; r++) b[20*r +: 20] = {m_shift[r], m_absmax[r]};
    beats.push_back(b);
    for (int r = 0; r < nrows; r++)
      for (int c0 = 0; c0 < kch; c0 += 4) begin
        b = '0;
        for (int s = 0; s < 4 && c0 + s < kch; s++)
          b[128*s +: 128] = m_win[(r / 4) * 96 + c0 + s][128 * (r % 4) +: 128];
        beats.push_back(b);
      end
    while (!fill_ready) tick();
    ld_start = 1; ld_cls = 4'(cls); ld_nrows = 5'(nrows); ld_kch = 7'(kch);
    tick();
    ld_start = 0;
    for (int i = 0; i < beats.size(); ) begin
      ld_valid = ((rnd() % 4) != 0); ld_data = beats[i];
      @(posedge clk);
      if (ld_valid && ld_ready) i++;
      @(negedge clk);
    end
    ld_valid = 0;
    while (ld_busy) tick();
    tick(2);
  endtask

  // a fill through the quantiser's port: random rows, every chunk, then the constants
  task automatic quantiser_fill(int nrows, int kch, int cls);
    while (!fill_ready) tick();
    fq_start = 1; fq_cls = 4'(cls);
    tick();
    fq_start = 0;
    for (int r = 0; r < nrows; r++) begin
      for (int c = 0; c < kch; c++) begin
        logic [127:0] d = rnd128();
        fq_wr = 1; fq_row = 5'(r); fq_chunk = 7'(c); fq_data = d;
        m_win[(r / 4) * 96 + c][128 * (r % 4) +: 128] = d;
        tick();
      end
      fq_wr = 0;
      fq_rc_wr = 1; fq_rc_row = 5'(r); fq_absmax = 15'(rnd()); fq_shift = 5'(rnd() % 17);
      m_absmax[r] = fq_absmax; m_shift[r] = fq_shift;
      tick();
      fq_rc_wr = 0;
    end
    fq_done = 1; tick(); fq_done = 0;
    tick(2);
  endtask

  // a dump: compare its beats with the model
  task automatic dump_check(bit res, int first, int nbeats, string what);
    int got = 0;
    dp_start = 1; dp_res = res; dp_first = 16'(first); dp_beats = 16'(nbeats);
    tick(); dp_start = 0;
    while (got < nbeats) begin
      dp_ready = ((rnd() % 3) != 0);
      @(posedge clk);
      if (dp_valid && dp_ready) begin
        logic [511:0] want;
        for (int k = 0; k < (res ? 8 : 16); k++)
          if (res) want[64*k +: 64] = m_res[first + 8 * got + k];
          else     want[32*k +: 32] = m_stg[first + 16 * got + k];
        check(dp_data === want, $sformatf("%s: dump beat %0d differs", what, got));
        got++;
      end
      @(negedge clk);
    end
    dp_ready = 0;
    tick(3);
    check(!dp_busy, $sformatf("%s: dump still busy", what));
  endtask

  string dir = "", cname = "l0_q";
  int seed = 7;

  initial begin
    begin
      int fd, n, v; string k, s;
      fd = $fopen("rs_run.cfg", "r");
      if (fd != 0) begin
        while (!$feof(fd)) begin
          n = $fscanf(fd, "%s %s\n", k, s);
          if (n != 2) continue;
          void'($sscanf(s, "%d", v));
          case (k) "DIR": dir = s; "CASE": cname = s; "SEED": seed = v;
            default: $display("[WARN] unknown key %s", k); endcase
        end
        $fclose(fd);
      end
    end
    rng = 32'hC0FFEE ^ seed;
    $display("row store test: USE_PRIM=%0d, real case %s from %s", USE_PRIM, cname, dir);
    tick(6);
    while ($time < 120) tick();
    rst_n = 1;
    tick(3);
    check(!win_valid && fill_ready && !err_fill, "state after reset");

    // ---------------- T1: the loader on the reference's own window ----------------
    begin
      int fd, f[10], nused, kch;
      logic [63:0] w;
      logic [15:0] am [24];
      fd = $fopen({dir, "/gemm_", cname, "_cmd.txt"}, "r");
      if (fd == 0) begin $display("[FATAL] no vectors for %s", cname); $display(">> VERDICT: FAIL <<"); $finish; end
      void'($fscanf(fd, "%h %h %h %h %h %h %h %h %h %h %h", w, f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7], f[8], f[9]));
      $fclose(fd);
      nused = f[6]; kch = f[3] / 16;
      $readmemh({dir, "/gemm_", cname, "_win.mem"}, m_win);
      for (int r = 0; r < 24; r++) begin am[r] = 0; m_shift[r] = 0; end
      $readmemh({dir, "/gemm_", cname, "_absmax.mem"}, am, 0, nused - 1);
      for (int r = 0; r < 24; r++) m_absmax[r] = am[r][14:0];
      load_window(nused, kch, w[34:31]);
      // the file pads the window with zeros; the GEMM loads only its rows' chunks, and the rest of the
      // (never written) window stays as it was: expect exactly the loaded part, the rest untouched
      for (int e = 0; e < 576; e++)
        for (int r = 0; r < 4; r++)
          if (4 * (e / 96) + r >= nused || e % 96 >= kch) m_win[e][128*r +: 128] = 'x;
      check(win_valid && win_cls == w[34:31], $sformatf("T1: win_valid %0b cls %0d", win_valid, win_cls));
      check_window(576, "T1 real window");
      check_consts(24, "T1 real constants");
      $display("T1 the reference's window for %s (%0d rows, K/16 = %0d) loaded and read back", cname, nused, kch);
    end

    // ---------------- T2: the quantiser's port, then loader shapes ----------------
    quantiser_fill(24, 96, 2);
    check(win_valid && win_cls == 2, "T2: window state after the quantiser fill");
    check_window(576, "T2 quantiser fill");
    check_consts(24, "T2 constants");
    begin
      automatic int shapes [3][2] = '{'{3, 1}, '{9, 5}, '{24, 37}};
      for (int i = 0; i < 3; i++) begin
        for (int r = 0; r < shapes[i][0]; r++)
          for (int c = 0; c < shapes[i][1]; c++) m_win[(r / 4) * 96 + c][128 * (r % 4) +: 128] = rnd128();
        for (int r = 0; r < 24; r++) begin m_absmax[r] = 15'(rnd()); m_shift[r] = 5'(rnd() % 17); end
        load_window(shapes[i][0], shapes[i][1], 3 + i);
        check(win_valid && win_cls == 4'(3 + i), $sformatf("T2: window state, shape %0d", i));
        check_window(576, $sformatf("T2 loader %0d rows x %0d chunks", shapes[i][0], shapes[i][1]));
        check_consts(24, $sformatf("T2 loader constants %0d", i));
      end
      $display("T2 quantiser-port fill and loader shapes K/16 = 1, 5, 37 read back");
    end

    // ---------------- T3: the constants' two copies ----------------
    begin
      logic [14:0] a_old [24];
      logic [4:0]  s_old [24];
      bit ok;
      for (int r = 0; r < 24; r++) begin a_old[r] = m_absmax[r]; s_old[r] = m_shift[r]; end
      check(!err_fill, "T3: err_fill clear before");
      // GEMM 1 starts on the current window (its entry pop) and goes idle with its tiles in flight
      walker_busy = 1; ent_pop = 1; tick(); ent_pop = 0;
      tick(5);
      walker_busy = 0; tick();
      // the next window fills while GEMM 1's tiles are still on their way to the drain
      quantiser_fill(24, 8, 1);
      check(!err_fill, "T3: a fill into the free copy is not an error");
      ok = 1;
      for (int r = 0; r < 24; r++) begin
        drn_row = 5'(r); #1;
        if (drn_absmax !== a_old[r] || drn_shift !== s_old[r]) ok = 0;
      end
      check(ok, "T3: before GEMM 1's last tile, the drain reads GEMM 1's copy");
      // GEMM 2 starts on the new window; GEMM 1's tiles are handed over, the last one last
      walker_busy = 1; ent_pop = 1; tick(); ent_pop = 0;
      tile_hs = 1; tile_last = 0; tick(); tile_hs = 0;
      ok = 1;
      for (int r = 0; r < 24; r++) begin
        drn_row = 5'(r); #1;
        if (drn_absmax !== a_old[r]) ok = 0;
      end
      check(ok, "T3: a tile that is not the last keeps GEMM 1's copy");
      tile_hs = 1; tile_last = 1; tick(); tile_hs = 0; tile_last = 0;
      check_consts(24, "T3 after GEMM 1's last tile the drain reads GEMM 2's constants");
      // GEMM 3 reuses the window (A = WIN): the same copy as GEMM 2
      ent_pop = 1; tick(); ent_pop = 0;
      walker_busy = 0; tick();
      quantiser_fill(4, 2, 5);                       // writes the spare copy (GEMM 1's): free
      check(!err_fill, "T3: a fill into the copy no GEMM uses is not an error");
      // GEMM 4 starts on that newest copy; GEMM 2 and 3 (the other copy) are still queued
      walker_busy = 1; ent_pop = 1; tick(); ent_pop = 0; walker_busy = 0; tick();
      // a fill now would write the copy GEMM 2 and 3 still use
      fq_start = 1; tick(); fq_start = 0;
      fq_done = 1; tick(); fq_done = 0; tick(2);
      check(err_fill, "T3: a fill into a copy still in use sets err_fill");
      repeat (3) begin tile_hs = 1; tile_last = 1; tick(); tile_hs = 0; tile_last = 0; end
      // a fill while the walker is busy is refused (and flagged)
      walker_busy = 1; tick();
      check(!fill_ready, "T3: no fill while the walker is busy");
      begin
        automatic logic v0 = win_valid;
        fq_start = 1; tick(); fq_start = 0; tick();
        check(win_valid == v0 && fill_ready == 0, "T3: the refused fill left the window as it was");
      end
      walker_busy = 0; tick();
      $display("T3 the constants' two copies: the old copy until the last tile; err_fill on misuse");
    end

    // ---------------- T4: staging ----------------
    for (int i = 0; i < 36864; i++) m_stg[i] = 'x;
    begin
      int addrs [$];
      for (int i = 0; i < 3000; i++) addrs.push_back(rnd() % 36864);
      addrs.push_back(0); addrs.push_back(36863); addrs.push_back(1535); addrs.push_back(1536);
      for (int i = 0; i < 2048; i++) addrs.push_back(1536 * 23 + i % 1536);   // the last row, whole
      foreach (addrs[i]) begin
        automatic logic [31:0] d = rnd();
        automatic logic [1:0]  he = (m_stg[addrs[i]] === 'x) ? 2'b11 : 2'(1 + rnd() % 3);
        stg_we = 1; stg_waddr = 16'(addrs[i]); stg_whe = he; stg_wdata = d;
        if (he[0]) m_stg[addrs[i]][15:0]  = d[15:0];
        if (he[1]) m_stg[addrs[i]][31:16] = d[31:16];
        tick();
      end
      stg_we = 0;
      foreach (addrs[i]) begin
        stg_rd = 1; stg_raddr = 16'(addrs[i]);
        @(posedge clk); @(negedge clk); stg_rd = 0; @(posedge clk); #1;
        check(stg_rdata === m_stg[addrs[i]], $sformatf("T4: staging word %0d", addrs[i]));
        @(negedge clk);
      end
      dump_check(1'b0, 1536 * 23, 96, "T4 staging row 23");
      $display("T4 staging: %0d writes with half enables, reads, a dump of row 23", addrs.size());
    end

    // ---------------- T5: residual ----------------
    for (int i = 0; i < 6912; i++) m_res[i] = 'x;
    begin
      int addrs [$];
      for (int i = 0; i < 2000; i++) addrs.push_back(rnd() % 6912);
      addrs.push_back(0); addrs.push_back(6911); addrs.push_back(511); addrs.push_back(512);
      for (int i = 0; i < 288; i++) addrs.push_back(288 * 5 + i);              // row 5, whole
      foreach (addrs[i]) begin
        automatic logic [63:0] d = {rnd(), rnd()};
        automatic logic [1:0]  ee = (m_res[addrs[i]] === 'x) ? 2'b11 : 2'(1 + rnd() % 3);
        res_we = 1; res_waddr = 13'(addrs[i]); res_wee = ee; res_wdata = d;
        if (ee[0]) m_res[addrs[i]][31:0]  = d[31:0];
        if (ee[1]) m_res[addrs[i]][63:32] = d[63:32];
        tick();
      end
      res_we = 0;
      foreach (addrs[i]) begin
        res_rd = 1; res_raddr = 13'(addrs[i]);
        @(posedge clk); @(negedge clk); res_rd = 0; @(posedge clk); #1;
        check(res_rdata === m_res[addrs[i]], $sformatf("T5: residual entry %0d", addrs[i]));
        @(negedge clk);
      end
      dump_check(1'b1, 288 * 5, 36, "T5 residual row 5");
      $display("T5 residual: %0d writes with element enables, reads, a dump of row 5", addrs.size());
    end

    // ---------------- T6: own K/V ----------------
    for (int i = 0; i < 1152; i++) m_kvo[i] = 'x;
    begin
      int addrs [$];
      for (int i = 0; i < 1500; i++) addrs.push_back(rnd() % 1152);
      addrs.push_back(0); addrs.push_back(1151);
      foreach (addrs[i]) begin
        automatic logic [63:0] d = {rnd(), rnd()};
        automatic logic [7:0]  be = (m_kvo[addrs[i]] === 'x) ? 8'hFF : 8'(1 + rnd() % 255);
        kvo_we = 1; kvo_waddr = 11'(addrs[i]); kvo_wbe = be; kvo_wdata = d;
        for (int b = 0; b < 8; b++) if (be[b]) m_kvo[addrs[i]][8*b +: 8] = d[8*b +: 8];
        tick();
      end
      kvo_we = 0;
      foreach (addrs[i]) begin
        kvo_rd = 1; kvo_raddr = 11'(addrs[i]);
        @(posedge clk); @(negedge clk); kvo_rd = 0; @(posedge clk); #1;
        check(kvo_rdata === m_kvo[addrs[i]], $sformatf("T6: own K/V entry %0d", addrs[i]));
        @(negedge clk);
      end
      $display("T6 own K/V: %0d byte-enable writes, reads", addrs.size());
    end

    // ---------------- T7: a read during a dump ----------------
    check(!err_port, "T7: err_port clear before");
    dp_start = 1; dp_res = 0; dp_first = 0; dp_beats = 4; tick(); dp_start = 0;
    stg_rd = 1; stg_raddr = 16'd7; tick(); stg_rd = 0;
    check(err_port, "T7: a staging read during the dump sets err_port");
    dp_ready = 1; while (dp_busy) tick(); dp_ready = 0;
    $display("T7 a client read during a dump flagged");

    // ---------------- T8: a soft reset during a load ----------------
    begin
      while (!fill_ready) tick();
      ld_start = 1; ld_cls = 4'd1; ld_nrows = 5'd8; ld_kch = 7'd36; tick(); ld_start = 0;
      for (int i = 0; i < 6; i++) begin ld_valid = 1; ld_data = {16{rnd()}}; tick(); end
      ld_valid = 0;
      rst_n = 0; tick(3); rst_n = 1; tick(2);
      check(!ld_busy && !win_valid && fill_ready && !err_fill && !err_port && !dp_busy,
            "T8: state after a reset during a load");
      // a fresh real load matches again
      load_window(5, 36, 1);
      check(win_valid && win_cls == 1, "T8: window state after the reload");
      check_window(576, "T8 reload");
      $display("T8 a soft reset during a load: clean; a fresh load works");
    end

    $display("RS RESULTS: %0d checks, %0d failures", checks, fails);
    $display(">> VERDICT: %s <<", (fails == 0 && checks > 0) ? "PASS" : "FAIL");
    $finish;
  end

  initial begin
    #(8 * 3_000_000);
    $display("[FATAL] watchdog expired");
    $display(">> VERDICT: FAIL <<");
    $finish;
  end

endmodule

module tb_me_row_store_model;
  tb_me_row_store #(.USE_PRIM(1'b0)) u_tb ();
endmodule

`default_nettype wire
