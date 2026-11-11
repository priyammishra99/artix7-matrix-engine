// ============================================================================
// File: bd_ctrl.sv
// Block: AC701 board shell (matrix engine only) - the run controller
//
// Plays the host for one board run, as tb_me_top_s1 does in simulation:
//   1. start: the case (sel) and repeat mode are taken; the case's 16 fields
//      come from the case table (bd_case.mem, 32 lines a case)
//   2. me_top is held in its soft reset for 16 cycles (every run starts from
//      the same state), the memory side and the checker are cleared
//   3. the case's register script (bd_script.mem) runs: writes, and for each
//      GO (CLR, WIN, GEMM, DUMP) the status is polled until not busy and must
//      read DONE without FAULT or REFUSED; after the GEMM the cycle, saturation
//      and array-beat counts are read
//   4. the run passes when every operation did, the checker compared every
//      output beat with none different, all dump beats came, the counts match
//      the reference's, me_top raised no error and the memory side saw each
//      request once as expected
// Repeat mode: while rep is high and the run passed, the next run starts at 2
// (a run that fails stops the loop with its results kept). The results of the
// last run, and the run / pass / fail counts since start from idle, are held.
//
// GEMM-only mode (gm at start; for the power measurement): the window is
// loaded once, then the GEMM's GO is repeated, each after the one before is
// done (its status checked, its cycle count read), while rep is high; then the
// dump and the check of the last GEMM's outputs (every GEMM writes the same
// outputs: the same window and weights, no accumulate). gemm_count and
// loop_cycles (the first GEMM's GO to the last one's done) give the sustained
// rate; every GEMM must take the first one's cycle count.
//
// fail_code bits: 0 an operation's status, 1 a time-out, 2 outputs differ,
// 3 beats missing, 4 saturation, array-beat or GEMM cycle count, 5 me_top
// error flag, 6 a request asked for the wrong thing or twice, 7 a request never
// came (or, GEMM-only, not one weight request a GEMM).
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module bd_ctrl #(
  parameter string CASE_INIT   = "bd_case.mem",
  parameter string SCRIPT_INIT = "bd_script.mem",
  parameter int    TIMEOUT     = 1 << 20     // cycles one operation may take (8 ms at 125 MHz)
) (
  input  wire logic          clk,
  input  wire logic          rst_n,          // synchronous, active low
  input  wire logic          start,          // pulse
  input  wire logic [2:0]    sel,
  input  wire logic          rep,
  input  wire logic          gm,             // with start: GEMM-only mode

  // me_top
  output logic               core_rst_n,
  output logic               csr_we,
  output logic [11:0]        csr_waddr,
  output logic [31:0]        csr_wdata,
  output logic [11:0]        csr_raddr,
  input  wire logic [31:0]   csr_rdata,
  input  wire logic [5:0]    me_err,

  // the case, to the memory side
  output logic               run_clr,
  output logic [15:0]        c_win_base, c_win_beats, c_wt_base, c_wt_beats, c_run_n, c_run_k,
  output logic [5:0]         c_layer,
  output logic [4:0]         c_tag,
  output logic [15:0]        c_ref_base, c_ref_beats, c_dump_beats,
  output logic [6:0]         c_rel_lo, c_rel_n,

  // from the memory side and the checker
  output logic               multi_wt,       // GEMM-only: a weight request each GEMM
  input  wire logic [31:0]   ms_wt_count,
  input  wire logic [2:0]    ms_seen,
  input  wire logic [5:0]    ms_err,
  input  wire logic          chk_idle,
  input  wire logic [15:0]   chk_rx,
  input  wire logic [31:0]   chk_checked, chk_mism,

  // results
  output logic               running,
  output logic               done_any,       // a run finished since start from idle
  output logic               last_pass,
  output logic               fail_any,       // a run failed since start from idle
  output logic [2:0]         cur_sel,
  output logic               cur_rep,
  output logic               cur_gm,
  output logic [31:0]        gemm_count,     // GEMMs in the last run (1, or the GEMM-only loop's)
  output logic [31:0]        loop_cycles,    // GEMM-only: the first GEMM's GO to the last one's done
  output logic [7:0]         fail_code,
  output logic [7:0]         fail_pc,        // the script line that failed (an operation's status or time-out)
  output logic [31:0]        run_count, pass_count, fail_count,
  output logic [31:0]        gemm_cycles,    // S1_CYCLES after the GEMM (me_s1_cmd's own count)
  output logic [31:0]        run_cycles,     // the whole run: reset release to the end of the dump
  output logic [31:0]        sat_count, arr_beats,
  output logic [3:0]         state_o
);

  localparam logic [11:0] A_GO = 12'h108, A_STATUS = 12'h10C;
  localparam int NF = 16;                    // fields a case (of 32 lines)

  logic [31:0] case_mem [256];
  logic [63:0] scr_mem  [256];
  initial begin
    $readmemh(CASE_INIT, case_mem);
    $readmemh(SCRIPT_INIT, scr_mem);
  end

  typedef enum logic [3:0] {S_IDLE, S_LOAD, S_RST, S_FETCH, S_DECODE, S_GOWAIT, S_POLL, S_RDWAIT, S_FINISH, S_EVAL}
    state_t;
  state_t st;
  assign state_o = st;

  logic [4:0]  fi;                           // field being loaded
  logic [31:0] cf [NF];                      // the case's fields
  logic [7:0]  pc;
  logic [63:0] ins;
  logic [4:0]  wcnt;
  logic [31:0] tmo;
  logic [1:0]  slot;
  logic [7:0]  fc;                           // this run's fail bits
  logic [31:0] cyc;
  logic [7:0]  gemm_pc;                      // the GEMM's GO line (GEMM-only: the loop's start)
  logic [31:0] g_first, loop_t0;
  logic        go_gemm;                      // the GO being waited for is the GEMM's
  logic [31:0] e_sat, e_arr;                 // the counts expected after the GEMMs so far

  assign multi_wt = cur_gm;

  assign c_win_base   = cf[1][15:0];   assign c_win_beats = cf[2][15:0];
  assign c_wt_base    = cf[3][15:0];   assign c_wt_beats  = cf[4][15:0];
  assign c_run_n      = cf[5][15:0];   assign c_run_k     = cf[6][15:0];
  assign c_layer      = cf[7][5:0];    assign c_tag       = cf[8][4:0];
  assign c_ref_base   = cf[9][15:0];   assign c_ref_beats = cf[10][15:0];
  assign c_dump_beats = cf[11][15:0];  assign c_rel_lo    = cf[12][6:0];
  assign c_rel_n      = cf[13][6:0];
  wire logic [31:0] c_exp_sat = cf[14], c_exp_arr = cf[15];

  wire logic [1:0] op = ins[63:62];
  wire logic [31:0] st_word = csr_rdata;

  always_ff @(posedge clk) ins <= scr_mem[pc];

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      st <= S_IDLE; fi <= '0; pc <= '0; wcnt <= '0; tmo <= '0; slot <= '0; fc <= '0; cyc <= '0;
      core_rst_n <= 1'b0; run_clr <= 1'b1;
      csr_we <= 1'b0; csr_waddr <= '0; csr_wdata <= '0; csr_raddr <= A_STATUS;
      running <= 1'b0; done_any <= 1'b0; last_pass <= 1'b0; fail_any <= 1'b0; cur_sel <= '0; cur_rep <= 1'b0;
      cur_gm <= 1'b0; gemm_count <= '0; loop_cycles <= '0; gemm_pc <= '0; g_first <= '0; loop_t0 <= '0;
      go_gemm <= 1'b0; e_sat <= '0; e_arr <= '0;
      fail_code <= '0; fail_pc <= '0; run_count <= '0; pass_count <= '0; fail_count <= '0;
      gemm_cycles <= '0; run_cycles <= '0; sat_count <= '0; arr_beats <= '0;
      for (int i = 0; i < NF; i++) cf[i] <= '0;
    end else begin
      csr_we  <= 1'b0;
      run_clr <= 1'b0;
      if (core_rst_n) cyc <= cyc + 32'd1;
      case (st)
        S_IDLE: begin
          core_rst_n <= 1'b1;                // idle: me_top runs (idle) so its state can be looked at
          if (start) begin
            cur_sel <= sel; cur_rep <= rep; cur_gm <= gm; running <= 1'b1;
            done_any <= 1'b0; last_pass <= 1'b0; fail_any <= 1'b0;
            run_count <= '0; pass_count <= '0; fail_count <= '0;
            fi <= '0; st <= S_LOAD;
          end
        end
        S_LOAD: begin
          cf[fi[3:0]] <= case_mem[{cur_sel, fi}];
          fi <= fi + 5'd1;
          if (fi == 5'(NF - 1)) begin wcnt <= '0; st <= S_RST; end
        end
        S_RST: begin                         // 16 cycles of soft reset; the run's state cleared
          core_rst_n <= 1'b0; run_clr <= 1'b1; fc <= '0; cyc <= '0;
          slot <= '0; tmo <= '0; gemm_count <= '0; loop_cycles <= '0; e_sat <= '0; e_arr <= '0;
          pc <= cf[0][7:0];
          wcnt <= wcnt + 5'd1;
          if (wcnt == 5'd15) begin core_rst_n <= 1'b1; run_clr <= 1'b0; st <= S_FETCH; end
        end
        S_FETCH: st <= S_DECODE;             // ins <= scr_mem[pc] this cycle
        S_DECODE: begin
          wcnt <= '0; tmo <= '0;
          case (op)
            2'd0: begin                      // write
              csr_we <= 1'b1; csr_waddr <= ins[43:32]; csr_wdata <= ins[31:0];
              pc <= pc + 8'd1; st <= S_FETCH;
            end
            2'd1: begin                      // GO, then wait
              csr_we <= 1'b1; csr_waddr <= A_GO; csr_wdata <= ins[31:0];
              csr_raddr <= A_STATUS; st <= S_GOWAIT;
              go_gemm <= ins[0];
              if (ins[0]) begin
                gemm_pc <= pc;
                if (gemm_count == '0) loop_t0 <= cyc;
              end
            end
            2'd2: begin                      // read into a capture slot
              csr_raddr <= ins[43:32]; slot <= ins[1:0]; st <= S_RDWAIT;
            end
            default: st <= S_FINISH;         // end
          endcase
        end
        S_GOWAIT: begin                      // the operation has started; STATUS is on csr_rdata
          wcnt <= wcnt + 5'd1;
          if (wcnt == 5'd3) st <= S_POLL;
        end
        S_POLL: begin
          tmo <= tmo + 32'd1;
          if (!st_word[0]) begin             // not busy
            if (!(st_word[1] && !st_word[2] && !st_word[3])) begin
              fc[0] <= 1'b1; fail_pc <= pc; st <= S_FINISH;
            end else begin
              pc <= pc + 8'd1; st <= S_FETCH;
              if (go_gemm) begin
                gemm_count  <= gemm_count + 32'd1;
                e_sat <= e_sat + c_exp_sat; e_arr <= e_arr + c_exp_arr;
                loop_cycles <= cyc - loop_t0;
              end
            end
          end else if (tmo == 32'(TIMEOUT)) begin
            fc[1] <= 1'b1; fail_pc <= pc; st <= S_FINISH;
          end
        end
        S_RDWAIT: begin                      // csr_raddr set last cycle; the data now
          wcnt <= wcnt + 5'd1;
          if (wcnt == 5'd1) begin
            case (slot)
              2'd0: begin
                gemm_cycles <= csr_rdata;
                if (gemm_count == 32'd1) g_first <= csr_rdata;
                else if (csr_rdata != g_first) fc[4] <= 1'b1;   // GEMM-only: every GEMM as long as the first
              end
              2'd1: sat_count   <= csr_rdata;
              default: arr_beats <= csr_rdata;
            endcase
            csr_raddr <= A_STATUS; st <= S_FETCH;
            // GEMM-only: after the last read of a GEMM, its GO again while rep is high
            pc <= (cur_gm && rep && slot == 2'd2) ? gemm_pc : pc + 8'd1;
          end
        end
        S_FINISH: begin                      // let the checker's pipeline empty
          wcnt <= wcnt + 5'd1;
          if (chk_idle && wcnt >= 5'd8) st <= S_EVAL;
        end
        S_EVAL: begin
          automatic logic [7:0] f = fc;
          if (chk_mism != 32'd0)                                                 f[2] = 1'b1;
          if (chk_checked != 32'(c_ref_beats) || chk_rx != c_dump_beats)         f[3] = 1'b1;
          // the drain's saturation and the walker's beat counts run since CLR: over every GEMM of the run
          if (sat_count != e_sat || arr_beats != e_arr)                          f[4] = 1'b1;
          if (me_err != 6'd0)                                                    f[5] = 1'b1;
          if (ms_err != 6'd0)                                                    f[6] = 1'b1;
          if (ms_seen != 3'b111 || ms_wt_count != gemm_count || gemm_count == 0) f[7] = 1'b1;
          fail_code  <= f;
          run_cycles <= cyc;
          run_count  <= run_count + 32'd1;
          done_any   <= 1'b1;
          last_pass  <= (f == 8'd0);
          if (f == 8'd0) pass_count <= pass_count + 32'd1;
          else begin fail_count <= fail_count + 32'd1; fail_any <= 1'b1; end
          if (f == 8'd0 && rep && !cur_gm) begin wcnt <= '0; st <= S_RST; end
          else begin running <= 1'b0; st <= S_IDLE; end
        end
        default: st <= S_IDLE;
      endcase
    end
  end

endmodule

`default_nettype wire
