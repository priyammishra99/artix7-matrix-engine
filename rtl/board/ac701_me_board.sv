// ============================================================================
// File: ac701_me_board.sv
// Block: AC701 board top for the matrix-engine-only run (Stage 1, no DDR)
//
// Pins (UG952 v1.4, checked against the board files of 2026-10-29):
//   SYSCLK_P/N R3/P3 (200 MHz, LVDS_25, external 100 ohm), CPU_RESET U4 (active
//   high), GPIO_SW_C U6 (start a run), GPIO_SW_N P6 (start GEMM-only), GPIO_DIP_SW[3:0]
//   R8 P8 R7 R6 (0-2 the case, 3 repeat), GPIO_LED[3:0] M26 T24 T25 R26.
//   UART T19/U19/V19/W19 reserved.
//
//   SYSCLK ─IBUFDS─ MMCM (VCO 1,000 MHz; /8 = 125 MHz, /10 = 100 MHz) ─BUFG─ clk
//   CPU_RESET or MMCM unlocked ─> 16-cycle reset release ─> bd_shell
//   button C or N (debounced) or VIO start ─> start; DIP or VIO override ─> case, repeat
//
// Button C: a run (window, GEMM, dump, check); with DIP 3, runs again and again.
// Button N: GEMM-only (for the power measurement): the window once, then the GEMM
// again and again while DIP 3 is on; DIP 3 off ends it with the dump and check.
//
// LEDs: 0 running (on), idle (slow blink: the clock runs); 1 a run done;
// 2 PASS (done, no run failed); 3 FAIL.
// Cases (sel = DIP 2:0): 0-3 layer 0 W_Q at 5 / 8 / 16 / 24 rows, 4-7 layer 29.
//
// With BD_DEBUG_IP defined (the board build), the Vivado IPs bd_vio and bd_ila
// (made by the project script) are added:
//   bd_vio  in0 [15:0]  {fail_code, sel, rep, running, done, last_pass, fail}
//           in1-3 [31:0] runs, passes, fails      in4 [31:0] GEMM cycles
//           in5 [31:0] run cycles                 in6 [31:0] output beats that differ
//           in7 [31:0] output beats compared      in8 [31:0] {first bad beat, 11'b0, lane}
//           in9 [31:0] {first got, first expected} in10 [31:0] saturations
//           in11 [31:0] array beats               in12, in13 [255:0] the dump RAM at out4
//           in14 [31:0] {fail_pc, 2'b0, me_top err, 2'b0, memory-side err, 7'b0, GEMM-only}
//           in15 [31:0] GEMMs in the run          in16 [31:0] GEMM-only loop cycles
//           out0 [0] start   out1 [0] override   out2 [2:0] case   out3 [0] repeat
//           out4 [9:0] dump RAM address          out5 [0] GEMM-only (with out0)
//   bd_ila  probe0 [127:0] bd_shell's dbg, 1,024 samples
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module ac701_me_board #(
  parameter real   CORE_DIV    = 8.0,        // 8.0: 125 MHz; 10.0: 100 MHz (the fallback)
  parameter int    DB_BITS     = 20,         // button debounce: stable for 2^DB_BITS cycles (8 ms)
  parameter int    HB_BIT      = 26,         // idle blink: counter bit (0.9 Hz at 125 MHz)
  parameter string WT_INIT     = "bd_wt.mem",
  parameter string WIN_INIT    = "bd_win.mem",
  parameter string REF_INIT    = "bd_ref.mem",
  parameter string CASE_INIT   = "bd_case.mem",
  parameter string SCRIPT_INIT = "bd_script.mem",
  parameter bit    DUMP_RAM    = 1'b1
) (
  input  wire logic        SYSCLK_P,
  input  wire logic        SYSCLK_N,
  input  wire logic        CPU_RESET,
  input  wire logic        GPIO_SW_C,
  input  wire logic        GPIO_SW_N,
  input  wire logic [3:0]  GPIO_DIP_SW,
  output logic      [3:0]  GPIO_LED
);

  // ---- clock ----
  logic sysclk, clk_mmcm, clk, fb, locked;
  IBUFDS u_sysclk (.I(SYSCLK_P), .IB(SYSCLK_N), .O(sysclk));
  MMCME2_BASE #(
    .CLKIN1_PERIOD(5.000), .DIVCLK_DIVIDE(1), .CLKFBOUT_MULT_F(5.000), .CLKOUT0_DIVIDE_F(CORE_DIV)
  ) u_mmcm (
    .CLKIN1(sysclk), .CLKFBIN(fb), .CLKFBOUT(fb), .CLKOUT0(clk_mmcm), .LOCKED(locked),
    .RST(CPU_RESET), .PWRDWN(1'b0),
    .CLKFBOUTB(), .CLKOUT0B(), .CLKOUT1(), .CLKOUT1B(), .CLKOUT2(), .CLKOUT2B(), .CLKOUT3(), .CLKOUT3B(),
    .CLKOUT4(), .CLKOUT5(), .CLKOUT6());
  BUFG u_bufg (.I(clk_mmcm), .O(clk));

  // ---- reset: held while CPU_RESET or the MMCM is not locked, released 16 cycles later ----
  wire logic arst = CPU_RESET || !locked;
  (* ASYNC_REG = "TRUE" *) logic [15:0] rst_sr;
  always_ff @(posedge clk or posedge arst)
    if (arst) rst_sr <= '1;
    else      rst_sr <= {rst_sr[14:0], 1'b0};
  logic rst_n;
  always_ff @(posedge clk) rst_n <= !rst_sr[15];

  // ---- switches: synchronised; the buttons debounced ----
  (* ASYNC_REG = "TRUE" *) logic [5:0] sw_m, sw_s;
  always_ff @(posedge clk) begin sw_m <= {GPIO_SW_N, GPIO_SW_C, GPIO_DIP_SW}; sw_s <= sw_m; end
  logic [DB_BITS-1:0] db_cnt [2];  logic [1:0] btn, btn_q;       // 0: C, 1: N
  always_ff @(posedge clk) begin
    for (int b = 0; b < 2; b++) begin
      if (!rst_n) begin db_cnt[b] <= '0; btn[b] <= 1'b0; btn_q[b] <= 1'b0; end
      else begin
        if (sw_s[4 + b] == btn[b]) db_cnt[b] <= '0;
        else begin
          db_cnt[b] <= db_cnt[b] + 1'b1;
          if (&db_cnt[b]) btn[b] <= sw_s[4 + b];
        end
        btn_q[b] <= btn[b];
      end
    end
  end

  // ---- VIO (board build) ----
  logic        vio_start, vio_ovr, vio_rep, vio_gm;  logic [2:0] vio_sel;  logic [9:0] vio_daddr;
  logic        vio_start_q;

  // ---- the shell ----
  logic         running, done_any, last_pass, fail_any, cur_rep, cur_gm;  logic [2:0] cur_sel;  logic [31:0] gemm_count, loop_cycles;
  logic [7:0]   fail_code, fail_pc;
  logic [31:0]  run_count, pass_count, fail_count, gemm_cycles, run_cycles, sat_count, arr_beats, chk_checked, chk_mism;
  logic [15:0]  chk_first_k, chk_first_got, chk_first_exp;  logic [4:0] chk_first_lane;
  logic [5:0]   me_err, ms_err;
  logic [511:0] dump_rdata;
  logic [127:0] dbg;

  always_ff @(posedge clk) vio_start_q <= vio_start;
  wire logic c_go = btn[0] && !btn_q[0], n_go = btn[1] && !btn_q[1], v_go = vio_start && !vio_start_q;
  wire logic start = c_go || n_go || v_go;
  wire logic gm    = n_go || (v_go && vio_gm);
  wire logic [2:0] sel = vio_ovr ? vio_sel : sw_s[2:0];
  wire logic       rep = vio_ovr ? vio_rep : sw_s[3];

  bd_shell #(.WT_INIT(WT_INIT), .WIN_INIT(WIN_INIT), .REF_INIT(REF_INIT), .CASE_INIT(CASE_INIT),
             .SCRIPT_INIT(SCRIPT_INIT), .DUMP_RAM(DUMP_RAM)) u_shell (
    .clk, .rst_n, .start, .sel, .rep, .gm, .dump_raddr(vio_daddr), .dump_rdata,
    .running, .done_any, .last_pass, .fail_any, .cur_sel, .cur_rep, .cur_gm, .gemm_count, .loop_cycles,
    .fail_code, .fail_pc,
    .run_count, .pass_count, .fail_count, .gemm_cycles, .run_cycles, .sat_count, .arr_beats,
    .chk_checked, .chk_mism, .chk_first_k, .chk_first_lane, .chk_first_got, .chk_first_exp,
    .me_err, .ms_err, .dbg);

`ifdef BD_DEBUG_IP
  // each VIO value on a named net, so the Hardware Manager shows (and the board script finds) vio_i<n> / vio_o<n>
  (* keep = "true" *) logic [15:0]  vio_i0;
  (* keep = "true" *) logic [31:0]  vio_i1, vio_i2, vio_i3, vio_i4, vio_i5, vio_i6, vio_i7, vio_i8, vio_i9, vio_i10,
                                    vio_i11, vio_i14, vio_i15, vio_i16;
  (* keep = "true" *) logic [255:0] vio_i12, vio_i13;
  (* keep = "true" *) logic         vio_o0, vio_o1, vio_o3, vio_o5;
  (* keep = "true" *) logic [2:0]   vio_o2;
  (* keep = "true" *) logic [9:0]   vio_o4;
  assign vio_i0  = {fail_code, cur_sel, cur_rep, running, done_any, last_pass, fail_any};
  assign vio_i1  = run_count;    assign vio_i2  = pass_count;   assign vio_i3  = fail_count;
  assign vio_i4  = gemm_cycles;  assign vio_i5  = run_cycles;   assign vio_i6  = chk_mism;     assign vio_i7 = chk_checked;
  assign vio_i8  = {chk_first_k, 11'd0, chk_first_lane};        assign vio_i9  = {chk_first_got, chk_first_exp};
  assign vio_i10 = sat_count;    assign vio_i11 = arr_beats;
  assign vio_i12 = dump_rdata[255:0];                           assign vio_i13 = dump_rdata[511:256];
  assign vio_i14 = {fail_pc, 2'd0, me_err, 2'd0, ms_err, 7'd0, cur_gm};
  assign vio_i15 = gemm_count;   assign vio_i16 = loop_cycles;
  assign vio_start = vio_o0;  assign vio_ovr = vio_o1;  assign vio_sel = vio_o2;  assign vio_rep = vio_o3;
  assign vio_daddr = vio_o4;  assign vio_gm  = vio_o5;
  bd_vio u_vio (
    .clk,
    .probe_in0(vio_i0), .probe_in1(vio_i1), .probe_in2(vio_i2), .probe_in3(vio_i3), .probe_in4(vio_i4),
    .probe_in5(vio_i5), .probe_in6(vio_i6), .probe_in7(vio_i7), .probe_in8(vio_i8), .probe_in9(vio_i9),
    .probe_in10(vio_i10), .probe_in11(vio_i11), .probe_in12(vio_i12), .probe_in13(vio_i13), .probe_in14(vio_i14),
    .probe_in15(vio_i15), .probe_in16(vio_i16),
    .probe_out0(vio_o0), .probe_out1(vio_o1), .probe_out2(vio_o2), .probe_out3(vio_o3), .probe_out4(vio_o4),
    .probe_out5(vio_o5));
  bd_ila u_ila (.clk, .probe0(dbg));
`else
  assign vio_start = 1'b0; assign vio_ovr = 1'b0; assign vio_sel = '0; assign vio_rep = 1'b0; assign vio_daddr = '0;
  assign vio_gm = 1'b0;
`endif

  // ---- LEDs ----
  logic [HB_BIT:0] hb = '0;
  always_ff @(posedge clk) begin
    hb <= hb + 1'b1;
    GPIO_LED <= {fail_any, done_any && !fail_any, done_any, running || (!running && hb[HB_BIT])};
  end

endmodule

`default_nettype wire
