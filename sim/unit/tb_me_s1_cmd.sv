// ============================================================================
// File: tb_me_s1_cmd.sv
// Block: Matrix engine - unit testbench for me_s1_cmd (the Stage-1 command path)
//
// The host's side through the debug registers (csr_*), and models of what
// the command path drives, each with random delays:
//   walker   takes the GEMM word when idle; then issues beats (perf_beats) and
//            cmd_done, or ends at once with a fault, or with no beats (an empty
//            row set)
//   drain    records each dc_* word; after the walker's last beat and a delay
//            pulses cmd_fin for the slot it was given (and now and then a
//            cmd_fin for another slot first, to be ignored); may raise its fault
//            for that slot, or for another
//   store    fill_ready; the loader busy for a while after ld_start, the window
//            valid a little later; the dump busy for a while after dp_start
//   engines  read, write and weight requests taken after random waits; the
//            write engine's done some cycles after the dump's last beat
// Checks:
//   T1 every register written and read back; the step context and the class
//      table reach their outputs
//   T2 random operations (GEMM normal, walker fault, empty, drain fault for its
//      slot or another, window loads, dumps, CLR): each command's word, slot id
//      and tensor reach the walker, the drain (after the walker's accept, the
//      same word and id) and the weight engine; the window's and the dump's
//      fields and request lengths; the operation ends only after its last event
//      (a GEMM after the drain's cmd_fin for its slot, never at the walker's
//      cmd_done; a window after it is valid; a dump after the write engine's
//      done); S1_STATUS's code, slot and op; S1_CYCLES the cycles it took
//   T3 refusals: GO while an operation runs, no or several bits, a window the
//      store cannot take or out of range, a dump of no beats, the step context
//      or the class table written while an operation runs: REFUSED set,
//      nothing started, the context unchanged; CLR clears it, with one cnt_clr
//   T4 a soft reset while a GEMM runs: idle, then a fresh GEMM completes
// Configuration (s1_run.cfg): SEED n, OPS n.
// Ends with "S1 RESULTS: N checks, F failures" and ">> VERDICT: PASS/FAIL <<".
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module tb_me_s1_cmd;

  logic clk = 1'b0;
  always #4 clk = ~clk;
  logic rst_n = 1'b0;
  longint cyc = 0;
  always @(posedge clk) cyc <= cyc + 1;

  // ---- DUT ----
  logic        csr_we = 1'b0;  logic [11:0] csr_waddr = '0, csr_raddr = '0;  logic [31:0] csr_wdata = '0, csr_rdata;
  logic        busy;
  logic [5:0]  n_layers;  logic [4:0] step_rows;  logic [3:0] step_logit_rows;  logic [39:0] logit_map;
  logic [127:0] cls_e, cls_w;
  logic        cnt_clr;
  logic        cmd_valid, cmd_ready, cmd_done = 1'b0, cmd_fault = 1'b0;
  logic [63:0] cmd_word;  logic [5:0] cmd_id;  logic [7:0] cmd_fault_code = '0;  logic [31:0] perf_beats = '0;
  logic        dc_valid;  logic [5:0] dc_id;  logic [63:0] dc_word;
  logic        cmd_fin = 1'b0;  logic [5:0] cmd_fin_id = '0;
  logic        drn_fault = 1'b0;  logic [7:0] drn_fault_code = '0;  logic [5:0] drn_fault_cmd = '0;
  logic [31:0] drn_sat_cnt = '0;
  logic        ld_start;  logic [3:0] ld_cls;  logic [4:0] ld_nrows;  logic [6:0] ld_kch;
  logic        fill_ready = 1'b1, ld_busy = 1'b0, win_valid = 1'b0;
  logic        dp_start, dp_res;  logic [15:0] dp_first, dp_beats;  logic dp_busy = 1'b0;
  logic        rq_valid, rq_ready = 1'b0;  logic [31:0] rq_addr;  logic [15:0] rq_beats;
  logic        wq_valid, wq_ready = 1'b0;  logic [31:0] wq_addr;  logic [15:0] wq_beats;  logic wq_done = 1'b0;
  logic        wt_valid, wt_ready = 1'b0;  logic [5:0] wt_layer;  logic [4:0] wt_tag;

  me_s1_cmd dut (.*);

  int checks = 0, fails = 0;
  task automatic check(bit ok, string what);
    checks++;
    if (!ok) begin fails++; if (fails < 40) $display("[FAIL] t=%0t %s", $time, what); end
  endtask

  // ==========================================================================
  // models
  // ==========================================================================
  // the plan for the next GEMM, set by the main process
  int  p_mode = 0;            // 0 beats, 1 walker fault, 2 empty, 3 beats + drain fault (its slot), 4 + another's
  int  p_beats = 10;
  logic [7:0] p_code = 8'h23;
  bit  p_stray = 0;           // a cmd_fin for another slot before the real one

  // what the models saw, with times (cycles)
  logic [63:0] m_wword, m_dword;  logic [5:0] m_wid, m_did;
  longint      t_accept, t_dc, t_wdone, t_fin, t_winv, t_dpdone, t_wqdone, t_busy0;
  int          n_dc, n_accept, n_clr;
  logic [5:0]  m_wt_layer;  logic [4:0] m_wt_tag;  int n_wt;
  logic [31:0] m_rq_addr, m_wq_addr;  logic [15:0] m_rq_beats, m_wq_beats;  int n_rq, n_wq;

  // walker
  logic w_busy = 1'b0;
  assign cmd_ready = !w_busy;
  int   w_left, w_wait;
  int   w_mode;
  always @(posedge clk) begin
    cmd_done <= 1'b0; cmd_fault <= 1'b0;
    if (!rst_n) begin w_busy <= 1'b0; end
    else if (!w_busy && cmd_valid) begin
      w_busy <= 1'b1; m_wword <= cmd_word; m_wid <= cmd_id; t_accept <= cyc; n_accept <= n_accept + 1;
      w_mode = p_mode; w_wait = $urandom_range(12, 1); w_left = (w_mode == 1 || w_mode == 2) ? 0 : p_beats;
    end else if (w_busy) begin
      if (w_wait > 0) w_wait--;
      else if (w_left > 0) begin
        perf_beats <= perf_beats + 1;
        w_left--;
        if (w_left == 0) begin cmd_done <= 1'b1; w_busy <= 1'b0; t_wdone <= cyc; end
      end else begin
        cmd_done <= 1'b1; cmd_fault <= (w_mode == 1); cmd_fault_code <= (w_mode == 1) ? p_code : 8'h00;
        w_busy <= 1'b0; t_wdone <= cyc;
      end
    end
  end

  // drain: the dc words; cmd_fin after the walker's last beat
  int  d_wait = -1;  logic [5:0] d_id;  bit d_stray;
  always @(posedge clk) begin
    cmd_fin <= 1'b0;
    if (dc_valid) begin m_dword <= dc_word; m_did <= dc_id; t_dc <= cyc; n_dc <= n_dc + 1; d_id <= dc_id; end
    if (cnt_clr) begin drn_fault <= 1'b0; drn_fault_code <= '0; drn_fault_cmd <= '0; n_clr <= n_clr + 1; end
    if (!rst_n) d_wait <= -1;
    else if (cmd_done && !cmd_fault && (w_mode == 0 || w_mode >= 3)) begin
      d_wait <= $urandom_range(60, 20); d_stray <= p_stray;
    end else if (d_wait > 0) begin
      d_wait <= d_wait - 1;
      if (d_wait == 8 && d_stray) begin cmd_fin <= 1'b1; cmd_fin_id <= d_id + 6'd5; end   // another slot's
      if (d_wait == 10 && w_mode >= 3 && !drn_fault) begin
        drn_fault <= 1'b1; drn_fault_code <= (w_mode == 3) ? 8'h42 : 8'h21;
        drn_fault_cmd <= (w_mode == 3) ? d_id : d_id + 6'd9;
      end
    end else if (d_wait == 0) begin
      cmd_fin <= 1'b1; cmd_fin_id <= d_id; t_fin <= cyc; d_wait <= -1;
    end
  end

  // row store: loader, window, dump
  int l_left = 0, l_win = -1, p_left = 0, q_wait = -1;
  int p_ld_len = 10, p_dp_len = 10;
  always @(posedge clk) begin
    if (!rst_n) begin ld_busy <= 1'b0; dp_busy <= 1'b0; l_left = 0; p_left = 0; l_win = -1; q_wait = -1; end
    else begin
      if (ld_start && fill_ready && !ld_busy) begin ld_busy <= 1'b1; l_left = p_ld_len; win_valid <= 1'b0; end
      else if (ld_busy) begin
        if (l_left > 1) l_left--; else begin ld_busy <= 1'b0; l_win = $urandom_range(5, 1); end
      end
      if (l_win > 0) l_win--; else if (l_win == 0) begin win_valid <= 1'b1; t_winv <= cyc; l_win = -1; end
      if (dp_start && !dp_busy) begin dp_busy <= 1'b1; p_left = p_dp_len; end
      else if (dp_busy) begin
        if (p_left > 1) p_left--; else begin dp_busy <= 1'b0; t_dpdone <= cyc; q_wait = $urandom_range(10, 0); end
      end
      wq_done <= 1'b0;
      if (q_wait > 0) q_wait--; else if (q_wait == 0) begin wq_done <= 1'b1; t_wqdone <= cyc; q_wait = -1; end
    end
  end

  // engines: requests taken after random waits
  always @(negedge clk) begin
    rq_ready <= ($urandom_range(3, 0) == 0);
    wq_ready <= ($urandom_range(3, 0) == 0);
    wt_ready <= ($urandom_range(3, 0) == 0);
  end
  always @(posedge clk) begin
    if (rq_valid && rq_ready) begin m_rq_addr <= rq_addr; m_rq_beats <= rq_beats; n_rq <= n_rq + 1; end
    if (wq_valid && wq_ready) begin m_wq_addr <= wq_addr; m_wq_beats <= wq_beats; n_wq <= n_wq + 1; end
    if (wt_valid && wt_ready) begin m_wt_layer <= wt_layer; m_wt_tag <= wt_tag; n_wt <= n_wt + 1; end
  end

  // the end of each operation, and the cycles it was busy
  logic   busy_q = 1'b0;
  longint busy_cycles = 0;
  always @(posedge clk) begin
    busy_q <= busy;
    if (busy && !busy_q) busy_cycles <= 1;
    else if (busy) busy_cycles <= busy_cycles + 1;
    if (busy_q && !busy) t_busy0 <= cyc;
  end

  // ==========================================================================
  // the host's side
  // ==========================================================================
  task automatic wr(logic [11:0] a, logic [31:0] d);
    @(negedge clk) begin csr_we <= 1'b1; csr_waddr <= a; csr_wdata <= d; end
    @(negedge clk) csr_we <= 1'b0;
  endtask
  task automatic rd(logic [11:0] a, output logic [31:0] d);
    @(negedge clk) csr_raddr <= a;
    @(negedge clk) d = csr_rdata;
  endtask
  task automatic wait_idle(int limit = 20000);
    logic [31:0] st;
    int n = 0;
    do begin rd(12'h10C, st); n++; end while (st[0] && n < limit);
    check(!st[0], "operation never ended");
  endtask

  localparam logic [11:0] A_CMD_LO = 12'h100, A_CMD_HI = 12'h104, A_GO = 12'h108, A_STATUS = 12'h10C,
                          A_STEP = 12'h110, A_LOGIT_LO = 12'h114, A_LOGIT_HI = 12'h118, A_CLASS = 12'h11C,
                          A_WIN = 12'h120, A_WIN_ADDR = 12'h124, A_DUMP = 12'h128, A_DUMP_BEATS = 12'h12C,
                          A_DUMP_ADDR = 12'h130, A_SAT = 12'h134, A_CYCLES = 12'h138, A_BEATS = 12'h13C;

  int seed = 1, nops = 300;
  logic [5:0] exp_slot = 6'd0;

  task automatic do_gemm(int mode);
    logic [63:0] w;  logic [31:0] st, cy;  int nacc0, ndc0, nwt0;
    me_cmd_pkg::cmd_t c;
    w = {$urandom, $urandom};
    c = me_cmd_pkg::cmd_t'(w);
    p_mode = mode; p_beats = $urandom_range(40, 1); p_code = 8'($urandom_range(255, 1)); p_stray = $urandom_range(1, 0);
    nacc0 = n_accept; ndc0 = n_dc; nwt0 = n_wt;
    wr(A_CMD_LO, w[31:0]); wr(A_CMD_HI, w[63:32]); wr(A_GO, 32'h1);
    wait_idle();
    repeat (3) @(negedge clk);
    rd(A_STATUS, st); rd(A_CYCLES, cy);
    check(n_accept == nacc0 + 1 && m_wword == w && m_wid == exp_slot,
          $sformatf("GEMM mode %0d: walker got %h slot %0d, expected %h slot %0d", mode, m_wword, m_wid, w, exp_slot));
    check(n_dc == ndc0 + 1 && m_dword == w && m_did == exp_slot && t_dc > t_accept,
          $sformatf("GEMM mode %0d: drain got %0d words, %h slot %0d at %0d (accept %0d)", mode, n_dc - ndc0, m_dword, m_did, t_dc, t_accept));
    check(n_wt == nwt0 + 1 && m_wt_layer == c.layer && m_wt_tag == c.tag,
          $sformatf("GEMM mode %0d: weight request %0d: layer %0d tag %0d", mode, n_wt - nwt0, m_wt_layer, m_wt_tag));
    case (mode)
      0, 3, 4: check(t_busy0 > t_fin && t_fin > t_wdone, $sformatf("GEMM mode %0d ended at %0d, cmd_done %0d, cmd_fin %0d", mode, t_busy0, t_wdone, t_fin));
      default: check(t_busy0 > t_wdone, $sformatf("GEMM mode %0d ended at %0d before cmd_done %0d", mode, t_busy0, t_wdone));
    endcase
    case (mode)
      1:       check(st[2] && st[15:8] == p_code, $sformatf("walker fault: status %h, code %h", st, p_code));
      3:       check(st[2] && st[15:8] == 8'h42, $sformatf("drain fault for its slot: status %h", st));
      default: check(!st[2] && st[15:8] == 8'h00, $sformatf("GEMM mode %0d: fault reported, status %h", mode, st));
    endcase
    check(st[1] && !st[0] && st[6:4] == 3'd1 && st[21:16] == exp_slot, $sformatf("GEMM status %h", st));
    check(cy == 32'(busy_cycles), $sformatf("S1_CYCLES %0d, busy %0d cycles", cy, busy_cycles));
    exp_slot++;
    if (mode >= 3) begin wr(A_GO, 32'h8); repeat (3) @(negedge clk); end   // the drain's fault is sticky until CLR
  endtask

  task automatic do_win();
    logic [31:0] st, a;  int rows, kch, nrq0;
    rows = $urandom_range(24, 1); kch = $urandom_range(96, 1); a = $urandom & 32'hFFFF_FFC0;
    p_ld_len = $urandom_range(40, 2); nrq0 = n_rq;
    wr(A_WIN, {9'd0, 7'(kch), 3'd0, 5'(rows), 4'd0, 4'($urandom)}); wr(A_WIN_ADDR, a); wr(A_GO, 32'h2);
    wait_idle();
    rd(A_STATUS, st);
    check(ld_nrows == 5'(rows) && ld_kch == 7'(kch), $sformatf("window: loader rows %0d K/16 %0d", ld_nrows, ld_kch));
    check(n_rq == nrq0 + 1 && m_rq_addr == a && m_rq_beats == 16'(1 + rows * ((kch + 3) / 4)),
          $sformatf("window: read request %0d at %h, %0d beats (rows %0d, K/16 %0d)", n_rq - nrq0, m_rq_addr, m_rq_beats, rows, kch));
    check(t_busy0 > t_winv && win_valid, $sformatf("window: ended at %0d, valid at %0d", t_busy0, t_winv));
    check(st[1] && !st[2] && st[6:4] == 3'd2, $sformatf("window status %h", st));
  endtask

  task automatic do_dump();
    logic [31:0] st, a;  int first, beats, res, nwq0;
    first = $urandom_range(65535, 0); beats = $urandom_range(300, 1); res = $urandom_range(1, 0);
    a = $urandom & 32'hFFFF_FFC0; p_dp_len = beats + $urandom_range(5, 0); nwq0 = n_wq;
    wr(A_DUMP, {15'd0, 1'(res), 16'(first)}); wr(A_DUMP_BEATS, beats); wr(A_DUMP_ADDR, a); wr(A_GO, 32'h4);
    wait_idle();
    rd(A_STATUS, st);
    check(dp_first == 16'(first) && dp_beats == 16'(beats) && dp_res == 1'(res), $sformatf("dump fields %0d %0d %0d", dp_first, dp_beats, dp_res));
    check(n_wq == nwq0 + 1 && m_wq_addr == a && m_wq_beats == 16'(beats), $sformatf("dump: write request %0d at %h, %0d beats", n_wq - nwq0, m_wq_addr, m_wq_beats));
    check(t_busy0 > t_dpdone && t_busy0 > t_wqdone, $sformatf("dump: ended at %0d, last beat %0d, written %0d", t_busy0, t_dpdone, t_wqdone));
    check(st[1] && !st[2] && st[6:4] == 3'd3, $sformatf("dump status %h", st));
  endtask

  task automatic refused(string what);
    logic [31:0] st;
    rd(A_STATUS, st);
    check(st[3], {what, ": not refused"});
    wr(A_GO, 32'h8);                                           // CLR
    repeat (3) @(negedge clk);
    rd(A_STATUS, st);
    check(!st[3], {what, ": CLR did not clear REFUSED"});
  endtask

  initial begin
    int fd, nr;  string k, v;  logic [31:0] x;
    fd = $fopen("s1_run.cfg", "r");
    if (fd) begin
      while (!$feof(fd)) begin
        nr = $fscanf(fd, "%s %s\n", k, v);
        if (nr == 2) begin if (k == "SEED") seed = v.atoi(); else if (k == "OPS") nops = v.atoi(); end
      end
      $fclose(fd);
    end
    begin process pr; pr = process::self(); pr.srandom(seed); end
    $display("tb_me_s1_cmd: SEED %0d OPS %0d", seed, nops);
    n_dc = 0; n_accept = 0; n_clr = 0; n_wt = 0; n_rq = 0; n_wq = 0;
    repeat (4) @(negedge clk);
    rst_n = 1'b1;

    // ---- T1: registers ----
    begin
      static logic [11:0] rw [9] = '{A_CMD_LO, A_CMD_HI, A_LOGIT_LO, A_WIN, A_WIN_ADDR, A_DUMP, A_DUMP_BEATS, A_DUMP_ADDR, A_STEP};
      static logic [31:0] mask [9] = '{32'hFFFF_FFFF, 32'hFFFF_FFFF, 32'hFFFF_FFFF, 32'hFFFF_FFFF, 32'hFFFF_FFFF, 32'hFFFF_FFFF,
                                32'hFFFF_FFFF, 32'hFFFF_FFFF, 32'h003F_0F1F};
      for (int i = 0; i < 9; i++) begin
        logic [31:0] d;
        d = $urandom;
        wr(rw[i], d); rd(rw[i], x);
        check(x == (d & mask[i]), $sformatf("register %h: wrote %h, read %h", rw[i], d, x));
      end
      wr(A_STEP, 32'h0012_0510);   // 18 layers, 5 logit rows, 16 rows
      check(n_layers == 6'd18 && step_logit_rows == 4'd5 && step_rows == 5'd16, "step context outputs");
      wr(A_LOGIT_LO, 32'hDEAD_BEEF); wr(A_LOGIT_HI, 32'h0000_00A5); rd(A_LOGIT_HI, x);
      check(logit_map == 40'hA5_DEAD_BEEF && x == 32'hA5, $sformatf("logit map %h, read %h", logit_map, x));
      for (int c = 0; c < 16; c++) wr(A_CLASS, {8'd0, 8'(c * 3 + 8), 8'(-c), 4'd0, 4'(c)});
      for (int c = 0; c < 16; c++) begin
        check(cls_e[8 * c +: 8] == 8'(-c) && cls_w[8 * c +: 8] == 8'(c * 3 + 8), $sformatf("class %0d outputs", c));
      end
      wr(A_CLASS, {8'd0, 8'd32, 8'hF1, 4'd0, 4'd7}); rd(A_CLASS, x);
      check(x == {8'd0, 8'd32, 8'hF1, 4'd0, 4'd7}, $sformatf("class read %h", x));
      drn_sat_cnt = 32'h1234_5678; rd(A_SAT, x); check(x == 32'h1234_5678, "S1_SAT");
      rd(A_BEATS, x); check(x == perf_beats, "S1_BEATS");
      rd(A_STATUS, x); check(x == 32'h0, $sformatf("status after reset %h", x));
    end

    // ---- T2: random operations ----
    for (int i = 0; i < nops; i++) begin
      int r;
      r = $urandom_range(9, 0);
      if (r < 5)      do_gemm(r == 0 ? 1 : r == 1 ? 2 : r == 2 ? 3 : r == 3 ? 4 : 0);
      else if (r < 7) do_win();
      else if (r < 9) do_dump();
      else begin
        int c0;  logic [31:0] st;
        c0 = n_clr;
        wr(A_GO, 32'h8);
        repeat (2) @(negedge clk);
        rd(A_STATUS, st);
        check(n_clr == c0 + 1 && st[1] && st[6:4] == 3'd0, $sformatf("CLR: %0d pulses, status %h", n_clr - c0, st));
      end
    end
    $display("T2 done: %0d operations", nops);

    // ---- T3: refusals ----
    begin
      logic [31:0] st, s0;  int a0, l0, c0;
      // GO while each kind of operation runs
      p_mode = 0; p_beats = 200;
      wr(A_GO, 32'h1); repeat (3) @(negedge clk); a0 = n_accept;
      wr(A_GO, 32'h1); wr(A_GO, 32'h2); wr(A_GO, 32'h4);
      rd(A_STEP, s0); wr(A_STEP, ~s0); wr(A_CLASS, 32'h00FF_FF03);
      rd(A_STEP, x); check(x == s0, "step context changed while busy");
      check(cls_e[8 * 3 +: 8] != 8'hFF, "class table changed while busy");
      wait_idle(); exp_slot++;
      check(n_accept == a0, "a GO while busy started an operation");
      refused("GO and context writes while a GEMM runs");
      // no bits, several bits
      wr(A_GO, 32'h0); refused("GO with no operation");
      wr(A_GO, 32'h3); refused("GO with two operations");
      // windows: the store cannot take a fill; rows and K/16 out of range
      fill_ready = 1'b0; l0 = n_rq;
      wr(A_WIN, {9'd0, 7'd36, 3'd0, 5'd5, 8'd1}); wr(A_GO, 32'h2); refused("window while the store cannot fill");
      fill_ready = 1'b1;
      wr(A_WIN, {9'd0, 7'd36, 3'd0, 5'd0, 8'd1});  wr(A_GO, 32'h2); refused("window of 0 rows");
      wr(A_WIN, {9'd0, 7'd36, 3'd0, 5'd25, 8'd1}); wr(A_GO, 32'h2); refused("window of 25 rows");
      wr(A_WIN, {9'd0, 7'd0, 3'd0, 5'd5, 8'd1});   wr(A_GO, 32'h2); refused("window of K/16 0");
      wr(A_WIN, {9'd0, 7'd97, 3'd0, 5'd5, 8'd1});  wr(A_GO, 32'h2); refused("window of K/16 97");
      repeat (20) @(negedge clk);
      check(n_rq == l0, "a refused window asked the read engine");
      wr(A_DUMP_BEATS, 0); wr(A_GO, 32'h4); refused("dump of 0 beats");
      // CLR: one cnt_clr
      c0 = n_clr; wr(A_GO, 32'h8); repeat (3) @(negedge clk);
      check(n_clr == c0 + 1, $sformatf("CLR gave %0d cnt_clr", n_clr - c0));
    end
    $display("T3 done");

    // ---- T4: a soft reset while a GEMM runs ----
    begin
      logic [31:0] st;
      p_mode = 0; p_beats = 400;
      wr(A_CMD_LO, 32'h1111_2222); wr(A_CMD_HI, 32'h0C00_3333); wr(A_GO, 32'h1);
      repeat (50) @(negedge clk);
      rst_n = 1'b0; repeat (3) @(negedge clk); rst_n = 1'b1;
      rd(A_STATUS, st);
      check(!busy && st == 32'h0, $sformatf("after a soft reset: busy %b status %h", busy, st));
      exp_slot = 6'd0;
      repeat (500) @(negedge clk);                                // the old walker run ends (the model is reset too)
      do_gemm(0);
    end
    $display("T4 done");

    $display("S1 RESULTS: %0d checks, %0d failures", checks, fails);
    $display(">> VERDICT: %s <<", fails == 0 ? "PASS" : "FAIL");
    $finish;
  end

  initial begin
    #50ms;
    $display("[FAIL] watchdog");
    $display("S1 RESULTS: %0d checks, %0d failures", checks, fails + 1);
    $display(">> VERDICT: FAIL <<");
    $finish;
  end

endmodule

`default_nettype wire
