// ============================================================================
// File: tb_me_if_checker.sv
// Block: Matrix-engine interface contract - self-test of me_if_checker
//
// No array here: the testbench plays both sides of the beat and tile ports.
//   T0       correct traffic (multi-beat, one-beat, held tiles) -> 0 errors
//   T1..T7   one deliberate break of one rule each               -> exactly 1
// A reset between tests clears the checker's protocol state (not its count).
// Ends with "ME_IF_CHECKER SELFTEST: n tests, f failures" and a verdict.
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module tb_me_if_checker;
  import me_if_pkg::*;

  localparam int TILE_W = 32 * 32;          // any width: the checker is width-agnostic

  logic clk = 1'b0;
  always #4 clk = ~clk;                     // 125 MHz

  logic rst_n = 1'b0;
  logic push = 0;
  logic op_valid = 0, op_first = 0, op_last = 0, op_bank = 0, op_bypass = 0;
  logic tile_valid = 0, tile_ready = 1;
  logic byp_valid = 0;
  logic [TILE_W-1:0] tile_acc = '0;

  me_if_checker #(.TILE_W(TILE_W), .MAX_MSG(50)) u_chk (
    .clk, .rst_n, .push, .op_valid, .op_first, .op_last, .op_bank, .op_bypass,
    .tile_valid, .tile_ready, .tile_acc, .byp_valid);

  // a bypass beat now, its result `lat` edges later (forked, so beats can overlap)
  task automatic byp_beat(int lat, bit first = 0);      // lat < 0: the result never comes
    @(negedge clk);
    op_valid = 1; op_first = first; op_last = 0; op_bypass = 1;
    if (lat >= 0) fork
      begin
        repeat (lat) @(negedge clk);
        byp_valid = 1;
        @(negedge clk);
        byp_valid = 0;
      end
    join_none
    @(negedge clk); op_valid = 0; op_bypass = 0; op_first = 0;
  endtask

  int tests = 0, failures = 0;

  // ---- drivers: everything changes on the falling edge ----
  task automatic beat(bit first, bit last, bit bank);
    @(negedge clk);
    op_valid = 1; op_first = first; op_last = last; op_bank = bank;
  endtask

  task automatic idle(int n);
    repeat (n) begin @(negedge clk); op_valid = 0; op_first = 0; op_last = 0; end
  endtask

  // offer one tile LAT edges after the last beat; hold it for `hold` cycles
  // with tile_ready low first. Call right after the tile's last beat().
  task automatic offer(int lat, int hold, bit change_while_held = 0, bit drop_while_held = 0);
    // the last beat is sampled at the next rising edge E; the tile must be
    // seen at edge E + lat, so drive it after lat - 1 more falling edges
    @(negedge clk); op_valid = 0; op_first = 0; op_last = 0;
    repeat (lat - 1) @(negedge clk);
    tile_valid = 1; tile_acc = {32{$urandom()}}; tile_ready = (hold == 0);
    for (int k = 1; k <= hold; k++) begin
      @(negedge clk);
      if (k == 1 && change_while_held) tile_acc = ~tile_acc;
      if (k == 1 && drop_while_held) begin tile_valid = 0; tile_ready = 1; return; end
      tile_ready = (k == hold);
    end
    @(negedge clk); tile_valid = 0; tile_ready = 1;
  endtask

  task automatic do_reset();
    @(negedge clk); rst_n = 0; op_valid = 0; tile_valid = 0; tile_ready = 1;
    repeat (2) @(negedge clk); rst_n = 1;
  endtask

  task automatic expect_errors(string name, int unsigned base, int unsigned want);
    int unsigned got;
    idle(3);
    got = u_chk.err_count - base;
    tests++;
    if (got != want) begin
      failures++;
      $display("[FAIL] %s: checker reported %0d error(s), expected %0d", name, got, want);
    end else begin
      $display("[ OK ] %s: %0d error(s) reported, as expected", name, got);
    end
  endtask

  int unsigned e0;

  initial begin
    do_reset();

    // T0: correct traffic
    e0 = u_chk.err_count;
    beat(1, 0, 0); beat(0, 0, 0); beat(0, 1, 0); offer(MEI_TILE_LAT, 0);   // 3-beat tile, bank 0
    beat(1, 1, 1);                               offer(MEI_TILE_LAT, 3);   // 1-beat tile, bank 1, held 3 cycles
    beat(1, 0, 0); beat(0, 1, 0);                offer(MEI_TILE_LAT + 5, 0); // late offer is legal (bank busy)
    expect_errors("T0 correct traffic", e0, 0);

    do_reset(); e0 = u_chk.err_count;
    beat(1, 0, 0); beat(1, 1, 1);                         // op_first again before op_last
    expect_errors("T1 P2 op_first inside an open tile", e0, 1);

    do_reset(); e0 = u_chk.err_count;
    beat(1, 1, 1);                                        // first tile after reset must use bank 0
    expect_errors("T2 P3 wrong bank", e0, 1);

    do_reset(); e0 = u_chk.err_count;
    beat(1, 1, 0); beat(1, 1, 1); beat(1, 1, 0);          // third tile with no tile taken
    expect_errors("T3 P4 third tile in flight", e0, 1);

    do_reset(); e0 = u_chk.err_count;
    beat(0, 0, 0);                                        // a beat with no tile open
    expect_errors("T4 P2 beat outside a tile", e0, 1);

    do_reset(); e0 = u_chk.err_count;
    beat(1, 1, 0); offer(MEI_TILE_LAT, 3, 1, 0);          // tile_acc changes while held
    expect_errors("T5 P5 tile_acc changed while held", e0, 1);

    do_reset(); e0 = u_chk.err_count;
    beat(1, 1, 0); offer(MEI_TILE_LAT - 5, 0);            // offered too early
    expect_errors("T6 P7 latency shorter than contract", e0, 1);

    do_reset(); e0 = u_chk.err_count;
    beat(1, 1, 0); offer(MEI_TILE_LAT, 3, 0, 1);          // tile_valid dropped before ready
    expect_errors("T7 P5 tile_valid dropped while held", e0, 1);

    // ---- contract v2: bypass beats (P9) and push mode (P10) ----
    do_reset(); e0 = u_chk.err_count;
    byp_beat(MEI_BYP_LAT); byp_beat(MEI_BYP_LAT); byp_beat(MEI_BYP_LAT);
    beat(1, 1, 0); offer(MEI_TILE_LAT, 0);                // bypass beats, then a tile
    idle(30);
    expect_errors("T8 correct bypass traffic", e0, 0);

    do_reset(); e0 = u_chk.err_count;
    beat(1, 0, 0); byp_beat(MEI_BYP_LAT); beat(0, 1, 0);  // bypass beat inside an open tile
    idle(30);
    expect_errors("T9 P9 bypass beat inside an open tile", e0, 1);

    do_reset(); e0 = u_chk.err_count;
    byp_beat(MEI_BYP_LAT, 1);                             // bypass beat with op_first
    idle(30);
    expect_errors("T10 P9 bypass beat with op_first", e0, 1);

    do_reset(); e0 = u_chk.err_count;
    byp_beat(MEI_BYP_LAT - 1);                            // result one edge early
    idle(30);
    expect_errors("T11 P9 bypass latency wrong", e0, 1);

    do_reset(); e0 = u_chk.err_count;
    byp_beat(-1);                                         // result never comes
    idle(30);
    expect_errors("T12 P9 bypass result lost", e0, 1);

    do_reset(); e0 = u_chk.err_count; push = 1;
    beat(1, 1, 0); offer(MEI_TILE_LAT, 1);                // push mode, consumer not ready one cycle
    expect_errors("T13 P10 consumer not ready in push mode", e0, 1);

    do_reset(); e0 = u_chk.err_count; push = 1;
    beat(1, 1, 0); beat(1, 1, 1); beat(1, 1, 0);          // three tiles in flight: legal in push mode
    expect_errors("T14 P4 not applied in push mode", e0, 0);
    push = 0;

    $display("ME_IF_CHECKER SELFTEST: %0d tests, %0d failures", tests, failures);
    $display(">> VERDICT: %s <<", (failures == 0) ? "PASS" : "FAIL");
    $finish;
  end

  initial begin
    #100000;
    $display("[FATAL] watchdog expired");
    $display(">> VERDICT: FAIL <<");
    $finish;
  end

endmodule

`default_nettype wire
