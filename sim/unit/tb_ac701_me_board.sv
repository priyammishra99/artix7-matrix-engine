// ============================================================================
// File: tb_ac701_me_board.sv
// Block: AC701 board top for the matrix-engine-only run - the simulation of
//        the whole board design, from the board's pins
//
// The testbench is the board and the person at it: the 200 MHz SYSCLK pair,
// CPU_RESET, the DIP switches and the centre button; it watches the LEDs and
// reads the shell's result registers (what VIO shows on the board).
//   1. every case (sel 0-7: layer 0 / layer 29 W_Q at 5, 8, 16, 24 rows): set
//      the DIP switches, press the button, wait for the run; it must PASS with
//      every output beat compared and none different, and the LEDs must show
//      done + PASS; layer 0 and layer 29 must take the same cycles
//   2. repeat mode (DIP 3) on one case: at least 3 runs, all passing, all with
//      the same GEMM cycle count; switching DIP 3 off ends the loop
//   3. the check itself: one bit of one output beat is flipped on its way into
//      the checker; the run must FAIL (outputs differ, exactly 1 beat, the right
//      beat and lane) and the FAIL LED must light; the next run passes again
//   4. GEMM-only mode (button N, DIP 3 on; the power measurement's loop) at 5 and
//      24 rows: the GEMM repeated on one window; DIP 3 off ends it with the dump;
//      it must PASS with every GEMM as long as the first, the array beats N times
//      one GEMM's, and the last GEMM's outputs bit-exact; the GEMMs' share of the
//      loop is printed
// Configuration (bd.cfg): CASES <list, e.g. 01234567>, REPEAT 0/1, INJECT 0/1, GEMMONLY 0/1.
// Ends with "BOARD RESULTS: N checks, F failures" and ">> VERDICT: PASS/FAIL <<".
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module tb_ac701_me_board;

  logic sys_p = 1'b0;
  always #2.5 sys_p = ~sys_p;                                   // 200 MHz
  logic cpu_reset = 1'b1, btn = 1'b0, btn_n = 1'b0;
  logic [3:0] dip = '0;
  logic [3:0] led;

  ac701_me_board #(.CORE_DIV(8.0), .DB_BITS(3), .HB_BIT(6)) dut (
    .SYSCLK_P(sys_p), .SYSCLK_N(~sys_p), .CPU_RESET(cpu_reset), .GPIO_SW_C(btn), .GPIO_SW_N(btn_n), .GPIO_DIP_SW(dip),
    .GPIO_LED(led));

  wire logic clk = dut.clk;
  `define SH dut.u_shell

  int checks = 0, fails = 0;
  task automatic check(bit ok, string what);
    checks++;
    if (!ok) begin fails++; if (fails < 30) $display("[FAIL] t=%0t %s", $time, what); end
  endtask

  string names [8] = '{"l0_q  5 rows", "l0_q  8 rows", "l0_q 16 rows", "l0_q 24 rows",
                       "l29_q 5 rows", "l29_q 8 rows", "l29_q 16 rows", "l29_q 24 rows"};
  int exp_ref [8] = '{90, 144, 288, 432, 90, 144, 288, 432};   // output beats (bd_cases.txt)
  int exp_arr [8] = '{5184, 5184, 10368, 15552, 5184, 5184, 10368, 15552};
  int gcyc [8];

  task automatic press(bit north = 1'b0);
    repeat (3) @(posedge clk);
    if (north) btn_n = 1'b1; else btn = 1'b1;
    repeat (40) @(posedge clk);
    btn = 1'b0; btn_n = 1'b0;
    repeat (40) @(posedge clk);
  endtask

  task automatic wait_idle(int limit_us, string what);
    int n;
    n = 0;
    while (!`SH.running && n < 200) begin @(posedge clk); n++; end
    n = 0;
    while (`SH.running && n < limit_us * 125) begin @(posedge clk); n++; end
    check(!`SH.running, $sformatf("%s: still running after %0d us", what, limit_us));
    repeat (4) @(posedge clk);
  endtask

  task automatic report(string what);
    $display("  %-14s %s  code %02h  GEMM %0d cycles  run %0d cycles  compared %0d  differ %0d  sat %0d  array beats %0d  LEDs %b",
             what, `SH.last_pass ? "PASS" : "FAIL", `SH.fail_code, `SH.gemm_cycles, `SH.run_cycles,
             `SH.chk_checked, `SH.chk_mism, `SH.sat_count, `SH.arr_beats, led);
  endtask

  task automatic run_case(int s, int limit_us);
    dip = 4'(s);
    press();
    wait_idle(limit_us, names[s]);
    report(names[s]);
    check(`SH.last_pass && `SH.fail_code == 8'd0, $sformatf("%s: failed, code %h (pc %0d)", names[s], `SH.fail_code, `SH.fail_pc));
    check(`SH.run_count == 1 && `SH.pass_count == 1, $sformatf("%s: runs %0d passes %0d", names[s], `SH.run_count, `SH.pass_count));
    check(`SH.chk_checked == exp_ref[s] && `SH.chk_mism == 0, $sformatf("%s: %0d compared, %0d differ", names[s], `SH.chk_checked, `SH.chk_mism));
    check(led[3:1] == 3'b011, $sformatf("%s: LEDs %b, want done + PASS", names[s], led));
    gcyc[s] = `SH.gemm_cycles;
  endtask

  initial begin
    int fd, nr;  string k, v;  string cases = "01234567";  int do_rep = 1, do_inj = 1, do_gm = 1;
    fd = $fopen("bd.cfg", "r");
    if (fd) begin
      while (!$feof(fd)) begin
        nr = $fscanf(fd, "%s %s\n", k, v);
        if (nr == 2) begin
          if (k == "CASES") cases = v; else if (k == "REPEAT") do_rep = v.atoi(); else if (k == "INJECT") do_inj = v.atoi();
          else if (k == "GEMMONLY") do_gm = v.atoi();
        end
      end
      $fclose(fd);
    end
    $display("tb_ac701_me_board: CASES %s REPEAT %0d INJECT %0d GEMMONLY %0d", cases, do_rep, do_inj, do_gm);
    foreach (gcyc[i]) gcyc[i] = -1;

    // ---- power-up: CPU_RESET, the MMCM locks, the reset releases ----
    #300 cpu_reset = 1'b0;
    wait (dut.locked);
    wait (dut.rst_n);
    repeat (40) @(posedge clk);
    $display("  MMCM locked at %0t, core clock period %0.3f ns", $time, 8.0);
    check(led[3:1] == 3'b000, $sformatf("after reset: LEDs %b", led));

    // ---- 1. every case ----
    for (int i = 0; i < cases.len(); i++) run_case(cases[i] - "0", 3000);
    for (int s = 0; s < 4; s++)
      if (gcyc[s] >= 0 && gcyc[s + 4] >= 0)
        check(gcyc[s] == gcyc[s + 4], $sformatf("rows case %0d: layer 0 %0d cycles, layer 29 %0d", s, gcyc[s], gcyc[s + 4]));

    // ---- 2. repeat mode ----
    if (do_rep) begin
      int g0, n;
      dip = 4'b1000 | 4'(cases[0] - "0");
      press();
      wait (`SH.run_count >= 1);
      g0 = `SH.gemm_cycles;
      n = 1;
      while (`SH.run_count < 3) begin
        @(posedge clk);
        if (`SH.run_count != n) begin
          n = `SH.run_count;
          check(`SH.gemm_cycles == g0, $sformatf("repeat run %0d: %0d GEMM cycles, first %0d", n, `SH.gemm_cycles, g0));
        end
      end
      check(`SH.running && led[0], "repeat: not running after 3 runs");
      dip[3] = 1'b0;
      wait_idle(3000, "repeat");
      $display("  repeat         %0d runs, %0d passed, %0d failed, GEMM %0d cycles each, LEDs %b",
               `SH.run_count, `SH.pass_count, `SH.fail_count, g0, led);
      check(`SH.run_count >= 3 && `SH.fail_count == 0 && `SH.pass_count == `SH.run_count,
            $sformatf("repeat: %0d runs, %0d failed", `SH.run_count, `SH.fail_count));
      check(led[3:1] == 3'b011, $sformatf("repeat: LEDs %b", led));
    end

    // ---- 3. the check catches a wrong output ----
    if (do_inj) begin
      int s = cases[0] - "0";
      logic b;
      dip = 4'(s);
      press();
      // the 4th output beat (k = 3), bit 5 (lane 0): flipped for the one cycle it is taken
      forever begin
        @(negedge clk);
        if (`SH.u_mem.u_chk.dp_valid && `SH.u_mem.u_chk.dp_ready && `SH.u_mem.u_chk.rel && `SH.u_mem.u_chk.k == 16'd3) break;
      end
      b = `SH.u_mem.u_chk.dp_data[5];
      force `SH.u_mem.u_chk.dp_data[5] = ~b;
      @(posedge clk); #1;
      release `SH.u_mem.u_chk.dp_data[5];
      wait_idle(3000, "inject");
      report("inject");
      check(!`SH.last_pass && `SH.fail_code == 8'h04, $sformatf("inject: code %h, want 04 (outputs differ only)", `SH.fail_code));
      check(`SH.chk_mism == 1 && `SH.chk_first_k == 3 && `SH.chk_first_lane == 0 &&
            (`SH.chk_first_got ^ `SH.chk_first_exp) == 16'h0020,
            $sformatf("inject: %0d differ, first beat %0d lane %0d got %h expected %h", `SH.chk_mism, `SH.chk_first_k,
                      `SH.chk_first_lane, `SH.chk_first_got, `SH.chk_first_exp));
      check(led[3] == 1'b1 && led[2] == 1'b0, $sformatf("inject: LEDs %b, want FAIL", led));
      run_case(s, 3000);                                         // and the next run passes again
    end

    // ---- 4. GEMM-only mode ----
    if (do_gm) begin
      int gsel [2] = '{0, 3};
      int want [2] = '{6, 3};
      foreach (gsel[g]) begin
        int s = gsel[g];
        real duty;
        dip = 4'b1000 | 4'(s);
        press(1'b1);
        while (`SH.gemm_count < want[g]) @(posedge clk);
        dip[3] = 1'b0;
        wait_idle(5000, "GEMM-only");
        duty = 100.0 * `SH.gemm_count * `SH.gemm_cycles / `SH.loop_cycles;
        $display("  GEMM-only %-13s %s  code %02h  %0d GEMMs of %0d cycles in %0d loop cycles (GEMMs %0.1f%% of the loop)  compared %0d  differ %0d  array beats %0d  LEDs %b",
                 names[s], `SH.last_pass ? "PASS" : "FAIL", `SH.fail_code, `SH.gemm_count, `SH.gemm_cycles, `SH.loop_cycles, duty,
                 `SH.chk_checked, `SH.chk_mism, `SH.arr_beats, led);
        check(`SH.last_pass && `SH.fail_code == 8'd0 && `SH.cur_gm, $sformatf("GEMM-only %s: code %h (pc %0d)", names[s], `SH.fail_code, `SH.fail_pc));
        check(`SH.gemm_count >= want[g] && `SH.run_count == 1, $sformatf("GEMM-only %s: %0d GEMMs, %0d runs", names[s], `SH.gemm_count, `SH.run_count));
        check(`SH.gemm_cycles == gcyc[s] || gcyc[s] < 0, $sformatf("GEMM-only %s: %0d cycles, a single run %0d", names[s], `SH.gemm_cycles, gcyc[s]));
        check(`SH.arr_beats == `SH.gemm_count * exp_arr[s], $sformatf("GEMM-only %s: array beats %0d", names[s], `SH.arr_beats));
        check(`SH.chk_checked == exp_ref[s] && `SH.chk_mism == 0, $sformatf("GEMM-only %s: %0d compared, %0d differ", names[s], `SH.chk_checked, `SH.chk_mism));
        check(duty > 95.0, $sformatf("GEMM-only %s: GEMMs only %0.1f%% of the loop", names[s], duty));
        check(led[3:1] == 3'b011, $sformatf("GEMM-only %s: LEDs %b", names[s], led));
      end
    end

    $display("BOARD RESULTS: %0d checks, %0d failures", checks, fails);
    $display(">> VERDICT: %s <<", fails == 0 ? "PASS" : "FAIL");
    $finish;
  end

  initial begin
    #500ms;
    $display("[FAIL] watchdog");
    $display("BOARD RESULTS: %0d checks, %0d failures", checks, fails + 1);
    $display(">> VERDICT: FAIL <<");
    $finish;
  end

endmodule

`default_nettype wire
