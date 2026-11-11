// ============================================================================
// File: tb_me_top_s1.sv
// Block: Matrix engine - the Stage-1 simulation: one real SmolLM2 GEMM through
//        me_top, end to end, as the board runs it
//
// The testbench plays the host (the debug registers) and the memory side
// (window reads, the weight stream and dump writes, with a DDR model), from
// the vector files for one case (default l0_q: layer 0's W_Q):
//   1. step context (rows, the logit row and its map, 30 layers) and the class
//      table (drain_classes.txt) through S1_STEP / S1_LOGIT / S1_CLASS; CLR
//   2. WIN: the read engine is asked for the window stream; it sends the
//      reference's constants beat (absmax, row shift) and each row's INT8
//      chunks (gemm_<c>_win.mem), 4 to a beat, into the loader
//   3. GEMM: the command word (gemm_<c>_cmd.txt) with FLAGS = 0 and A = WIN
//      (the window's class is checked); the weight stream is asked for the tensor
//      (layer, tag) and streams its run descriptor and every beat
//      (gemm_<c>_stream.mem) into the ring; DONE only after the drain's
//      cmd_fin
//   4. DUMP: the staging rows, 16 words a beat, to the DDR model; DONE after
//      the write engine's done
//   5. compare: every output against the reference's flag-free drain
//      (gemm_<c>_ref.mem), byte for byte; no fault, no error flag; the
//      array's beat count and the saturation count
// The array's port is checked by me_if_checker throughout. The real array (512
// DSP48E1 models) or the behavioural stand-in (sim/unit/me_array_standin.sv)
// is chosen by which file the runner compiles.
// Configuration (s1top.cfg): DIR <vectors>, CASE <name>, SEED n, RATE 0/1
// (0: the engines move a beat every cycle; 1: random gaps and back-pressure).
// Ends with "S1TOP RESULTS: N checks, F failures" and ">> VERDICT: PASS/FAIL <<".
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module tb_me_top_s1;
  import me_if_pkg::*;

  logic clk = 1'b0;
  always #4 clk = ~clk;
  logic rst_n = 1'b0;
  longint cyc = 0;
  always @(posedge clk) cyc <= cyc + 1;

  // ---- DUT ----
  logic         csr_we = 1'b0;  logic [11:0] csr_waddr = '0, csr_raddr = '0;  logic [31:0] csr_wdata = '0, csr_rdata;
  logic         run_valid = 1'b0, run_ready;  logic [5:0] run_layer = '0;  logic [4:0] run_tag = '0;
  logic [1:0]   run_kind = '0;  logic [15:0] run_n = '0, run_k = '0;  logic [23:0] run_beats = '0;
  logic         w_valid = 1'b0, w_ready;  logic [511:0] w_data = '0;
  logic         ld_valid = 1'b0, ld_ready;  logic [511:0] ld_data = '0;
  logic         dp_valid, dp_ready = 1'b0;  logic [511:0] dp_data;
  logic         rq_valid, rq_ready = 1'b0;  logic [31:0] rq_addr;  logic [15:0] rq_beats;
  logic         wq_valid, wq_ready = 1'b0;  logic [31:0] wq_addr;  logic [15:0] wq_beats;  logic wq_done = 1'b0;
  logic         wt_valid, wt_ready = 1'b0;  logic [5:0] wt_layer;  logic [4:0] wt_tag;
  logic         busy;  logic [5:0] err;

  me_top #(.USE_PRIM(1'b1)) dut (
    .clk, .rst_n, .csr_we, .csr_waddr, .csr_wdata, .csr_raddr, .csr_rdata,
    .run_valid, .run_ready, .run_layer, .run_tag, .run_kind, .run_n, .run_k, .run_beats,
    .w_valid, .w_ready, .w_data, .ld_valid, .ld_ready, .ld_data, .dp_valid, .dp_ready, .dp_data,
    .rq_valid, .rq_ready, .rq_addr, .rq_beats, .wq_valid, .wq_ready, .wq_addr, .wq_beats, .wq_done,
    .wt_valid, .wt_ready, .wt_layer, .wt_tag,
    .fq_start(1'b0), .fq_cls('0), .fq_wr(1'b0), .fq_row('0), .fq_chunk('0), .fq_data('0),
    .fq_rc_wr(1'b0), .fq_rc_row('0), .fq_absmax('0), .fq_shift('0), .fq_done(1'b0),
    .busy, .err);

  // the array's port, checked by the contract's checker
  me_if_checker u_chk (
    .clk, .rst_n, .push(1'b0), .op_valid(dut.op_valid), .op_first(dut.op_first), .op_last(dut.op_last),
    .op_bank(dut.op_bank), .op_bypass(1'b0), .tile_valid(dut.tile_valid), .tile_ready(dut.tile_ready),
    .tile_acc(dut.tile_acc), .byp_valid(dut.byp_valid));

  int checks = 0, fails = 0;
  task automatic check(bit ok, string what);
    checks++;
    if (!ok) begin fails++; if (fails < 30) $display("[FAIL] t=%0t %s", $time, what); end
  endtask

  // ==========================================================================
  // the vectors
  // ==========================================================================
  string dir = "", cname = "l0_q";
  int    seed = 1, rate = 0;
  logic [63:0]  cmd64;  int c_layer, c_tag, c_n, c_k, c_beats, c_nstep, c_nused;
  logic [511:0] f_stream [32768];
  logic [511:0] f_win [576];
  logic [15:0]  f_ab [24];
  logic [7:0]   f_sh [24];
  logic [31:0]  f_ref [65536];
  int           ref_mode, ref_sat;

  localparam logic [11:0] A_CMD_LO = 12'h100, A_CMD_HI = 12'h104, A_GO = 12'h108, A_STATUS = 12'h10C,
                          A_STEP = 12'h110, A_LOGIT_LO = 12'h114, A_LOGIT_HI = 12'h118, A_CLASS = 12'h11C,
                          A_WIN = 12'h120, A_WIN_ADDR = 12'h124, A_DUMP = 12'h128, A_DUMP_BEATS = 12'h12C,
                          A_DUMP_ADDR = 12'h130, A_SAT = 12'h134, A_CYCLES = 12'h138, A_BEATS = 12'h13C;
  localparam logic [31:0] WIN_DDR = 32'h1000_0000, DUMP_DDR = 32'h2000_0000;

  // ==========================================================================
  // the host's register accesses
  // ==========================================================================
  task automatic wr(logic [11:0] a, logic [31:0] d);
    @(negedge clk) begin csr_we <= 1'b1; csr_waddr <= a; csr_wdata <= d; end
    @(negedge clk) csr_we <= 1'b0;
  endtask
  task automatic rd(logic [11:0] a, output logic [31:0] d);
    @(negedge clk) csr_raddr <= a;
    @(negedge clk) d = csr_rdata;
  endtask
  task automatic run_op(logic [31:0] go, string what, int limit);
    logic [31:0] st, cy;
    longint t0 = cyc;
    wr(A_GO, go);
    repeat (2) @(negedge clk);
    do rd(A_STATUS, st); while (st[0] && cyc - t0 < limit);
    rd(A_CYCLES, cy);
    check(!st[0], $sformatf("%s: still busy after %0d cycles", what, limit));
    check(st[1] && !st[2] && !st[3], $sformatf("%s: status %h (code %h)", what, st, st[15:8]));
    $display("  %-6s done in %0d cycles, status %h", what, cy, st);
  endtask

  // ==========================================================================
  // the memory side: window reads, the weight stream, dump writes
  // ==========================================================================
  function automatic bit go_now();                         // RATE 1: a beat about 3 cycles in 4
    return (rate == 0) || ($urandom_range(3, 0) != 0);
  endfunction

  // read engine: on a request, the window stream into the loader
  int n_rq = 0, ld_sent = 0;  logic [31:0] got_rq_addr;  logic [15:0] got_rq_beats;
  logic [511:0] ld_q [$];
  always @(negedge clk) rq_ready <= go_now();
  always @(posedge clk) if (rq_valid && rq_ready) begin
    n_rq <= n_rq + 1; got_rq_addr <= rq_addr; got_rq_beats <= rq_beats;
  end

  // weight stream: on a request, the run descriptor, then every beat of the tensor
  int n_wt = 0, w_sent = 0;  logic [5:0] got_wt_layer;  logic [4:0] got_wt_tag;
  bit w_go = 0;
  always @(negedge clk) wt_ready <= go_now();
  always @(posedge clk) if (wt_valid && wt_ready) begin
    n_wt <= n_wt + 1; got_wt_layer <= wt_layer; got_wt_tag <= wt_tag; w_go <= 1'b1;
  end

  // write engine + DDR: takes the dump's beats, then says done
  int n_wq = 0, dump_n = 0;  logic [31:0] got_wq_addr;  logic [15:0] got_wq_beats;
  logic [511:0] ddr [$];
  always @(negedge clk) begin wq_ready <= go_now(); dp_ready <= go_now(); end
  always @(posedge clk) begin
    wq_done <= 1'b0;
    if (wq_valid && wq_ready) begin n_wq <= n_wq + 1; got_wq_addr <= wq_addr; got_wq_beats <= wq_beats; end
    if (dp_valid && dp_ready) begin
      ddr.push_back(dp_data);
      if (ddr.size() == dump_n) wq_done <= 1'b1;            // the last beat is in DDR
    end
  end

  // the handshakes as sampled at the edge (the streams are driven at negedges)
  logic ld_ready_seen = 1'b0, w_ready_seen = 1'b0;
  always @(posedge clk) begin ld_ready_seen <= ld_ready; w_ready_seen <= w_ready; end

  // ==========================================================================
  // the run
  // ==========================================================================
  initial begin
    int fd, nr;  string k, v;  logic [31:0] st, x;
    me_cmd_pkg::cmd_t c;
    int iclass, doff, nst, kch, ngrp;
    fd = $fopen("s1top.cfg", "r");
    if (fd) begin
      while (!$feof(fd)) begin
        nr = $fscanf(fd, "%s %s\n", k, v);
        if (nr == 2) begin
          if (k == "DIR") dir = v; else if (k == "CASE") cname = v;
          else if (k == "SEED") seed = v.atoi(); else if (k == "RATE") rate = v.atoi();
        end
      end
      $fclose(fd);
    end
    begin process pr; pr = process::self(); pr.srandom(seed); end
    $display("tb_me_top_s1: DIR %s CASE %s SEED %0d RATE %0d", dir, cname, seed, rate);

    // ---- the case ----
    begin
      int f [10];
      fd = $fopen({dir, "/gemm_", cname, "_cmd.txt"}, "r");
      if (!fd) $fatal(1, "no gemm_%s_cmd.txt in %s", cname, dir);
      void'($fscanf(fd, "%h %h %h %h %h %h %h %h %h %h %h", cmd64, f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7], f[8], f[9]));
      $fclose(fd);
      c_layer = f[0]; c_tag = f[1]; c_n = f[2]; c_k = f[3]; c_beats = f[4]; c_nstep = f[5]; c_nused = f[6];
    end
    fd = $fopen({dir, "/gemm_", cname, "_drain.txt"}, "r");
    begin int a, b, e1, e2, o1, o2; void'($fscanf(fd, "%d %d %d %d %d %d %d %d", a, b, e1, e2, ref_mode, ref_sat, o1, o2)); end
    $fclose(fd);
    if (ref_mode != 0) $fatal(1, "case %s: the Stage-1 test wants an INT16 flag-free output (ref_mode %0d)", cname, ref_mode);
    if (c_beats > 32768 || c_nused * c_n > 65536) $fatal(1, "case %s is too large for this testbench", cname);
    $readmemh({dir, "/gemm_", cname, "_stream.mem"}, f_stream, 0, c_beats - 1);
    $readmemh({dir, "/gemm_", cname, "_win.mem"}, f_win);
    $readmemh({dir, "/gemm_", cname, "_absmax.mem"}, f_ab, 0, c_nused - 1);
    $readmemh({dir, "/gemm_", cname, "_shift.mem"}, f_sh, 0, c_nused - 1);
    $readmemh({dir, "/gemm_", cname, "_ref.mem"}, f_ref, 0, c_nused * c_n - 1);
    // the Stage-1 command: the case's word, flag-free, reading the window
    c = me_cmd_pkg::cmd_t'(cmd64);
    c.flags = '0;
    c.abuf  = 4'(me_cmd_pkg::BUF_WIN);
    iclass = c.iclass; doff = c.doff; kch = c_k / 16; ngrp = c_n / 32;
    check(c.dbuf == 4'(me_cmd_pkg::BUF_STG16), $sformatf("case %s: D is %0d, not STG16", cname, c.dbuf));
    $display("case %s: layer %0d tag %0d, N %0d K %0d, %0d rows of %0d, %0d weight beats, D offset %0d",
             cname, c_layer, c_tag, c_n, c_k, c_nused, c_nstep, c_beats, doff);

    // the window stream: the constants beat (row r: absmax at 20r, shift at 20r + 15), then each row's
    // K/16 chunks, 4 a beat (chunk c in bits 128 (c mod 4)); row i is lane i mod 4 of entry (i div 4) x 96 + c
    begin
      logic [511:0] b;
      b = '0;
      for (int r = 0; r < c_nused; r++) begin b[20*r +: 15] = f_ab[r][14:0]; b[20*r + 15 +: 5] = f_sh[r][4:0]; end
      ld_q.push_back(b);
      for (int r = 0; r < c_nused; r++)
        for (int ch = 0; ch < kch; ch += 4) begin
          b = '0;
          for (int j = 0; j < 4 && ch + j < kch; j++) b[128*j +: 128] = f_win[(r / 4) * 96 + ch + j][128 * (r % 4) +: 128];
          ld_q.push_back(b);
        end
    end

    // ---- reset (the DSP48E1 models are held in global set/reset for the first 100 ns) ----
    repeat (6) @(negedge clk);
    while ($time < 120) @(negedge clk);
    rst_n = 1'b1;
    repeat (3) @(negedge clk);

    // ---- 1. step context and the class table ----
    wr(A_STEP, {10'd0, 6'd30, 4'd0, 4'd1, 3'd0, 5'(c_nstep)});   // 30 layers, 1 logit row, the step's rows
    wr(A_LOGIT_LO, 32'(c_nstep - 1));                              // logit row 0 = the last step row
    wr(A_LOGIT_HI, 32'd0);
    fd = $fopen({dir, "/drain_classes.txt"}, "r");
    while (!$feof(fd)) begin
      int id, e, w; string nm;
      nr = $fscanf(fd, "%d %s %d %d\n", id, nm, e, w);
      if (nr == 4) wr(A_CLASS, {8'd0, 8'(w), 8'(e), 4'd0, 4'(id)});
    end
    $fclose(fd);
    run_op(32'h8, "CLR", 100);

    // ---- 2. the window ----
    wr(A_WIN, {9'd0, 7'(kch), 3'd0, 5'(c_nused), 4'd0, 4'(iclass)});
    wr(A_WIN_ADDR, WIN_DDR);
    fork
      run_op(32'h2, "WIN", 20000);
      begin                                                   // the read engine's stream
        wait (n_rq == 1);
        while (ld_q.size()) begin
          @(negedge clk);
          if (ld_valid && ld_ready_seen) begin void'(ld_q.pop_front()); ld_sent++; end
          ld_valid <= (ld_q.size() > 0) && go_now();
          if (ld_q.size()) ld_data <= ld_q[0];
        end
        @(negedge clk) ld_valid <= 1'b0;
      end
    join
    check(n_rq == 1 && got_rq_addr == WIN_DDR && got_rq_beats == 16'(1 + c_nused * ((kch + 3) / 4)) &&
          ld_sent == int'(got_rq_beats),
          $sformatf("window: %0d read requests at %h for %0d beats; %0d sent", n_rq, got_rq_addr, got_rq_beats, ld_sent));

    // ---- 3. the GEMM ----
    wr(A_CMD_LO, 32'(c)); wr(A_CMD_HI, 32'(64'(c) >> 32));
    fork
      run_op(32'h1, "GEMM", 400000);
      begin                                                   // the weight stream: descriptor, then the beats
        wait (w_go);
        @(negedge clk) begin
          run_valid <= 1'b1; run_layer <= got_wt_layer; run_tag <= got_wt_tag; run_kind <= 2'd0;
          run_n <= 16'(c_n); run_k <= 16'(c_k); run_beats <= 24'(c_beats);
        end
        @(posedge clk); while (!run_ready) @(posedge clk);
        @(negedge clk) run_valid <= 1'b0;
        while (w_sent < c_beats) begin
          @(negedge clk);
          if (w_valid && w_ready_seen) w_sent++;
          w_valid <= (w_sent < c_beats) && go_now();
          if (w_sent < c_beats) w_data <= f_stream[w_sent];
        end
        @(negedge clk) w_valid <= 1'b0;
      end
    join
    check(n_wt == 1 && got_wt_layer == 6'(c_layer) && got_wt_tag == 5'(c_tag),
          $sformatf("weight request: %0d, layer %0d tag %0d", n_wt, got_wt_layer, got_wt_tag));
    check(w_sent == c_beats, $sformatf("weights: %0d of %0d beats taken", w_sent, c_beats));

    // ---- 4. the dump: the used rows of staging, 16 words a beat ----
    wr(A_DUMP, 32'd0);                                        // staging from word 0
    dump_n = c_nused * 1536 / 16;
    wr(A_DUMP_BEATS, 32'(dump_n));
    wr(A_DUMP_ADDR, DUMP_DDR);
    run_op(32'h4, "DUMP", 100000);
    check(n_wq == 1 && got_wq_addr == DUMP_DDR && ddr.size() == c_nused * 96,
          $sformatf("dump: %0d write requests at %h, %0d beats in DDR", n_wq, got_wq_addr, ddr.size()));

    // ---- 5. compare: output (row i, channel n) is staging word i x 1,536 + e/2, half e mod 2, e = 64 DOFF + n ----
    begin
      int bad = 0;
      int fo;
      fo = $fopen("s1top_out.txt", "w");                      // every output: the hardware's, the reference's
      $fdisplay(fo, "row channel hardware reference");
      for (int i = 0; i < c_nused; i++)
        for (int n = 0; n < c_n; n++) begin
          int e, wd;  logic [15:0] got;  logic [31:0] exp;
          e = 64 * doff + n;  wd = i * 1536 + e / 2;
          got = ddr[wd / 16][32 * (wd % 16) + 16 * (e % 2) +: 16];
          exp = f_ref[i * c_n + n];
          $fdisplay(fo, "%0d %0d %0d %0d", i, n, $signed(got), $signed(exp));
          if (got !== exp[15:0] || (exp[31:16] !== {16{exp[15]}})) begin
            if (bad < 8) $display("[FAIL] row %0d channel %0d: got %h, reference %h", i, n, got, exp);
            bad++;
          end
        end
      $fclose(fo);
      check(bad == 0, $sformatf("outputs: %0d of %0d differ from the reference", bad, c_nused * c_n));
      $display("  compare: %0d outputs, %0d differ from the reference model's drain", c_nused * c_n, bad);
    end
    rd(A_BEATS, x);
    check(x == 32'(ngrp * 4 * ((c_nused + 3) / 4) * kch), $sformatf("array beats %0d, expected %0d", x, ngrp * 4 * ((c_nused + 3) / 4) * kch));
    rd(A_SAT, x);
    check(x == 32'(ref_sat), $sformatf("saturation count %0d, expected %0d", x, ref_sat));
    check(err == 6'd0, $sformatf("error flags %b (drain fault, store port, store fill, ring free, array bypass, array overrun)", err));

    $display("S1TOP RESULTS: %0d checks, %0d failures", checks, fails);
    $display(">> VERDICT: %s <<", fails == 0 ? "PASS" : "FAIL");
    $finish;
  end

  initial begin
    #200ms;
    $display("[FAIL] watchdog");
    $display("S1TOP RESULTS: %0d checks, %0d failures", checks, fails + 1);
    $display(">> VERDICT: FAIL <<");
    $finish;
  end

endmodule

`default_nettype wire
