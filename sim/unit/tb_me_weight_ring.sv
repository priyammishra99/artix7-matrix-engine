// ============================================================================
// File: tb_me_weight_ring.sv
// Block: Matrix engine - unit testbench for me_weight_ring
//
//   producer (the weight engine) --run descriptors, beats--> me_weight_ring
//   consumer (a walker stand-in) <--entries, slots, frees--
//
// The producer streams runs: GEMM tensors of several shapes (K = 16 .. 1,536,
// N = 32 .. 128; one scale beat, then two beats a weight slot, per group) and
// VECTOR entries (one beat a slot), with random data, descriptors queued ahead
// of their beats, and beats at a random rate. The consumer pops the entries,
// reads every slot in order at random moments, frees them in random chunks and
// sometimes lags far enough behind that the ring fills. A reference built from
// the beat list checks:
//   - every slot read: its 1,024 bits (a scale or VECTOR beat in 511:0 with
//     1,023:512 = 0; a weight slot's two beats), 2 cycles after ring_rd;
//   - every entry: its fields, in stream order;
//   - the ring never holds more than 1,024 live slots, and fills (perf_full);
//   - slot and beat counts, the pointers' wrap (more than 2,048 slots);
//   - the descriptor FIFO (4 deep here) fills: descriptors wait for both readers (the consumer
//     pops an entry only when the run before it is read, as the walker does);
//   - err_free: clear throughout, set by one excess free at the end;
//   - a soft reset part-way (RESET 1): everything restarts from 0 and the
//     whole stream runs again, matching.
// Configuration (ring_run.cfg, "KEY value" lines): SEED n; RATE 0 (a beat every cycle) | 1 (random);
//   LAG 0 (the consumer keeps up) | 1 (it lags: the ring fills); RESET 0 | 1
// Tops: tb_me_weight_ring (the block RAM primitives), tb_me_weight_ring_model
// (the behavioural memory).
// Ends with "RING RESULTS: N checks, F failures" and ">> VERDICT: PASS/FAIL <<".
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module tb_me_weight_ring #(parameter bit USE_PRIM = 1'b1);

  logic clk = 1'b0;
  always #4 clk = ~clk;                   // 125 MHz
  logic rst_n = 1'b0;

  // ---- the stream ----
  typedef struct { int layer, tag, kind, n, k, beats; } run_s;
  run_s          runs [$];
  logic [511:0]  beats [$];               // every beat, in stream order
  logic [1023:0] exp_slot [$];            // every slot, in order
  int            run_first [$];           // each run's first slot
  int            seed = 1, rate = 1, lag = 0, do_reset = 0;

  logic [31:0] rng = 32'h1;
  function automatic logic [31:0] rnd();
    rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
    return rng;
  endfunction
  function automatic logic [511:0] rnd_beat();
    for (int i = 0; i < 16; i++) rnd_beat[32*i +: 32] = rnd();
  endfunction

  task automatic add_run(int kind, int n, int k, int beats_n);
    run_s r;
    r.layer = rnd() % 30; r.tag = rnd() % 32; r.kind = kind; r.n = n; r.k = k; r.beats = beats_n;
    runs.push_back(r);
    run_first.push_back(exp_slot.size());
    if (kind == 0) begin
      for (int g = 0; g < n / 32; g++) begin
        logic [511:0] s = rnd_beat();
        beats.push_back(s);
        exp_slot.push_back({512'b0, s});
        for (int i = 0; i < 4 * (k / 16); i++) begin
          logic [511:0] lo = rnd_beat(), hi = rnd_beat();
          beats.push_back(lo); beats.push_back(hi);
          exp_slot.push_back({hi, lo});
        end
      end
    end else begin
      for (int b = 0; b < beats_n; b++) begin
        logic [511:0] v = rnd_beat();
        beats.push_back(v);
        exp_slot.push_back({512'b0, v});
      end
    end
  endtask

  // ---- DUT ----
  logic        run_valid = 1'b0, run_ready;
  logic [5:0]  run_layer = '0;
  logic [4:0]  run_tag = '0;
  logic [1:0]  run_kind = '0;
  logic [15:0] run_n = '0, run_k = '0;
  logic [23:0] run_beats = '0;
  logic        in_valid = 1'b0, in_ready;
  logic [511:0] in_data = '0;
  logic        ent_valid, ent_ready;
  logic [5:0]  ent_layer;
  logic [4:0]  ent_tag;
  logic [1:0]  ent_kind;
  logic [15:0] ent_n, ent_k;
  logic [10:0] ring_wptr;
  logic        ring_rd = 1'b0, ring_free = 1'b0;
  logic [9:0]  ring_raddr = '0;
  logic [1023:0] ring_rdata;
  logic [6:0]  ring_free_n = '0;
  logic        err_free;
  logic [31:0] perf_slots, perf_full;

  // a 4-deep descriptor FIFO (the design's default is 16), so the producer's descriptors fill it
  me_weight_ring #(.USE_PRIM(USE_PRIM), .ENT_DEPTH(4)) dut (
    .clk, .rst_n,
    .run_valid, .run_ready, .run_layer, .run_tag, .run_kind, .run_n, .run_k, .run_beats,
    .in_valid, .in_ready, .in_data,
    .ent_valid, .ent_ready, .ent_layer, .ent_tag, .ent_kind, .ent_n, .ent_k,
    .ring_wptr, .ring_rd, .ring_raddr, .ring_rdata, .ring_free, .ring_free_n,
    .err_free, .perf_slots, .perf_full);

  int checks = 0, fails = 0;
  task automatic fail(string m);
    fails++;
    if (fails <= 15) $display("[FAIL] t=%0t %s", $time, m);
  endtask

  // ---- producer: descriptors (up to the FIFO's room), beats at the chosen rate ----
  int ri = 0, bi = 0;
  bit go = 1'b0;
  always @(negedge clk) begin
    if (go && rst_n) begin
      run_valid = (ri < runs.size()) && ((rnd() % 4) != 0);
      if (ri < runs.size()) begin
        run_layer = 6'(runs[ri].layer); run_tag = 5'(runs[ri].tag); run_kind = 2'(runs[ri].kind);
        run_n = 16'(runs[ri].n); run_k = 16'(runs[ri].k); run_beats = 24'(runs[ri].beats);
      end
      in_valid = (bi < beats.size()) && (rate == 0 || (rnd() % 3) != 0);
      if (bi < beats.size()) in_data = beats[bi];
    end else begin
      run_valid = 1'b0; in_valid = 1'b0;
    end
  end
  always @(posedge clk) if (rst_n && go) begin
    if (run_valid && run_ready) ri++;
    if (in_valid && in_ready)   bi++;
  end

  // ---- consumer: entries, reads in order, frees in chunks ----
  bit fifo_full_seen = 1'b0;
  always @(posedge clk) if (rst_n && run_valid && !run_ready) fifo_full_seen = 1'b1;
  int ki = 0, rd_next = 0, rd_done = 0, freed = 0, max_live = 0;
  int pend_slot [$];
  bit cons_go = 1'b0;
  int wr_seen;                               // ring_wptr unwrapped
  logic [10:0] wptr_q = '0;
  always @(posedge clk) begin
    if (!rst_n) begin wr_seen = 0; wptr_q = '0; end
    else begin wr_seen += 11'(ring_wptr - wptr_q); wptr_q = ring_wptr; end
  end

  logic [1:0] rd_pipe = '0;
  int         rd_pipe_slot [2];
  always @(negedge clk) begin
    if (!(cons_go && rst_n)) begin
      ring_rd = 1'b0; ring_free = 1'b0; ent_ready = 1'b0;
    end else begin
      // entries
      // entries: popped as the walker does, when the run's command starts, i.e. once every slot
      // of the run before it has been read (so with a lagging consumer the writer runs ahead)
      ent_ready = ent_valid && (ki < runs.size()) && (rd_done >= run_first[ki]) && ((rnd() % 2) == 0);
      // a read when the next slot is written, at random moments (and rarely when lagging)
      ring_rd = (rd_next < wr_seen) && ((lag != 0) ? ((rnd() % 7) == 0) : ((rnd() % 4) != 0));
      ring_raddr = 10'(rd_next % 1024);
      // a free of a random chunk of the slots read
      ring_free = 1'b0;
      if (rd_done > freed && ((rnd() % 3) == 0)) begin
        automatic int c = 1 + rnd() % 64;
        if (c > rd_done - freed) c = rd_done - freed;
        ring_free = 1'b1; ring_free_n = 7'(c);
      end
    end
  end
  always @(posedge clk) if (rst_n && cons_go) begin
    // check the read 2 cycles back
    if (rd_pipe[1]) begin
      checks++;
      if (ring_rdata !== exp_slot[rd_pipe_slot[1]])
        fail($sformatf("slot %0d: data differs", rd_pipe_slot[1]));
      rd_done++;
    end
    rd_pipe[1] <= rd_pipe[0];  rd_pipe_slot[1] <= rd_pipe_slot[0];
    rd_pipe[0] <= ring_rd;     rd_pipe_slot[0] <= rd_next;
    if (ring_rd) rd_next++;
    if (ring_free) freed += ring_free_n;
    if (ent_valid && ent_ready) begin
      checks++;
      if (ki >= runs.size()) fail("an extra entry");
      else if (ent_layer != 6'(runs[ki].layer) || ent_tag != 5'(runs[ki].tag) || ent_kind != 2'(runs[ki].kind) ||
               ent_n != 16'(runs[ki].n) || ent_k != 16'(runs[ki].k))
        fail($sformatf("entry %0d: layer %0d tag %0d kind %0d n %0d k %0d", ki, ent_layer, ent_tag, ent_kind, ent_n, ent_k));
      ki++;
    end
    if (wr_seen - freed > max_live) max_live = wr_seen - freed;
    checks++;
    if (wr_seen - freed > 1024) fail($sformatf("%0d slots live", wr_seen - freed));
  end

  task automatic restart_state();
    ri = 0; bi = 0; ki = 0; rd_next = 0; rd_done = 0; freed = 0; max_live = 0;
    rd_pipe = '0;
  endtask

  task automatic run_stream(input int stop_after);
    // returns when everything is consumed, or once stop_after slots are read (> 0)
    for (int guard = 0; guard < 400000; guard++) begin
      @(posedge clk);
      if (stop_after > 0 && rd_done >= stop_after) return;
      if (rd_done == exp_slot.size() && ki == runs.size() && freed == rd_done) return;
    end
    fail($sformatf("timeout: %0d of %0d slots read, %0d freed, %0d of %0d entries", rd_done, exp_slot.size(),
                   freed, ki, runs.size()));
  endtask

  initial begin
    begin                                    // ring_run.cfg: "KEY value" lines (xsim.bat splits +K=V at '=')
      int fd, n, v; string k;
      fd = $fopen("ring_run.cfg", "r");
      if (fd != 0) begin
        while (!$feof(fd)) begin
          n = $fscanf(fd, "%s %d\n", k, v);
          if (n != 2) continue;
          case (k) "SEED": seed = v; "RATE": rate = v; "LAG": lag = v; "RESET": do_reset = v;
            default: $display("[WARN] unknown key %s", k); endcase
        end
        $fclose(fd);
      end
    end
    rng = 32'h9E3779B9 ^ seed;
    if (rng == 0) rng = 1;
    // the stream: real shapes (K 576 / 1,536), the smallest (K 16), VECTOR entries; > 2,048 slots
    add_run(0, 64, 576, 0);
    add_run(1, 0, 576, 18);                 // a gamma: 576 x INT16 = 18 beats
    add_run(0, 32, 16, 0);                  // K = 16: one chunk a tile
    add_run(0, 96, 1536, 0);                // K = 1,536 (W_down's)
    add_run(0, 128, 32, 0);
    add_run(1, 0, 16, 1);
    add_run(0, 32, 576, 0);
    add_run(0, 64, 576, 0);
    add_run(0, 64, 1536, 0);                // past 2,048 slots: the pointers wrap
    $display("stream: %0d runs, %0d beats, %0d slots; USE_PRIM=%0d RATE=%0d LAG=%0d RESET=%0d",
             runs.size(), beats.size(), exp_slot.size(), USE_PRIM, rate, lag, do_reset);

    repeat (6) @(negedge clk);
    while ($time < 120) @(negedge clk);      // the block RAM models' global reset
    rst_n = 1'b1;
    repeat (3) @(negedge clk);
    go = 1'b1; cons_go = 1'b1;

    if (do_reset) begin
      run_stream(exp_slot.size() / 3);
      @(negedge clk);
      rst_n = 1'b0;
      $display("soft reset after %0d slots read, %0d written: the stream restarts", rd_done, wr_seen);
      repeat (5) @(negedge clk);
      restart_state();
      rst_n = 1'b1;
      @(negedge clk);
      checks++;
      if (ring_wptr != 0 || ent_valid || in_ready === 1'bx || perf_slots != 0) fail("state left after the reset");
    end
    run_stream(0);

    checks++; if (rd_done != exp_slot.size()) fail($sformatf("%0d slots read, %0d expected", rd_done, exp_slot.size()));
    checks++; if (ki != runs.size())          fail($sformatf("%0d entries, %0d expected", ki, runs.size()));
    checks++; if (bi != beats.size())         fail($sformatf("%0d beats taken, %0d expected", bi, beats.size()));
    checks++; if (perf_slots != exp_slot.size()) fail($sformatf("perf_slots %0d", perf_slots));
    checks++; if (err_free)                   fail("err_free set by a legal stream");
    checks++; if (!fifo_full_seen)            fail("the descriptor FIFO never filled");
    checks++; if (lag != 0 && perf_full == 0) fail("the consumer lagged but the ring never filled");
    // one free too many: err_free must set
    cons_go = 1'b0;
    @(negedge clk); ring_free = 1'b1; ring_free_n = 7'd1;
    @(negedge clk); ring_free = 1'b0;
    @(negedge clk);
    checks++; if (!err_free) fail("err_free not set by an excess free");

    $display("max live %0d slots, ring full for %0d cycles, %0d slots written (pointer wrapped %0d times)",
             max_live, perf_full, perf_slots, perf_slots / 2048);
    $display("RING RESULTS: %0d checks, %0d failures", checks, fails);
    $display(">> VERDICT: %s <<", (fails == 0 && checks > 0) ? "PASS" : "FAIL");
    $finish;
  end

endmodule

module tb_me_weight_ring_model;
  tb_me_weight_ring #(.USE_PRIM(1'b0)) u_tb ();
endmodule

`default_nettype wire
