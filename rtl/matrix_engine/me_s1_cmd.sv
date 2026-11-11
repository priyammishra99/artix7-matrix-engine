// ============================================================================
// File: me_s1_cmd.sv
// Block: Matrix engine - the Stage-1 command path
//
// Lets the host run the matrix engine: through debug registers it loads a window, runs one GEMM command and
// dumps its results, one operation at a time, as the Stage-1 board flow needs:
//
//   WIN   the row store's window loader takes a window (the reference's INT8
//         rows and their constants) from DDR: the read engine is asked for the
//         stream, the loader is started; done when the window is valid
//   GEMM  the 64-bit GEMM word goes to the walker (cmd_*) and, at the walker's
//         accept, to the drain (dc_*) with the same slot id; the weight
//         engine is asked for the command's tensor (layer, tag); done at the
//         drain's cmd_fin for that slot (the results are in the store),
//         or at the walker's cmd_done when it faulted or issued nothing
//   DUMP  the row store's dump sends staging words or residual entries to the
//         write engine for DDR; done when the write engine reports the beats
//         written
//   CLR   the step starts: cnt_clr to the walker and the drain (counters, the
//         drain's sticky fault), and the status cleared
// One operation runs at a time, so a dump never reads a GEMM's results before
// they are written, and a window is never loaded under a running GEMM. A GO
// while an operation runs, with no or several operations, or a window load the
// row store cannot take (fill_ready low, rows or K/16 out of range), a dump of
// no beats, or a write to the step context or the class table while an
// operation runs is refused: S1_STATUS.REFUSED, sticky until CLR. An operation
// ends only once its requests to the engines have been taken. After a faulted
// GEMM the host soft-resets the core (CSR_SRESET) before the next one: the
// weight engine may already have streamed the tensor into the ring, which the
// soft reset clears (as the walker's header says).
//
// It also holds the step's settings:
// the step context (rows, logit rows and their map, the layer count) and the
// class table (each class's exponent and width).
//
// Registers (byte offsets in the control window's CSR space; the CSR block
// that decodes the AXI window drives the csr_* port). All 32 bits wide.
//   0x100  S1_CMD_LO     RW  the GEMM word, bits 31:0
//   0x104  S1_CMD_HI     RW  the GEMM word, bits 63:32
//   0x108  S1_GO         W   bit 0 GEMM, bit 1 WIN, bit 2 DUMP, bit 3 CLR (one bit a write)
//   0x10C  S1_STATUS     R   0 BUSY, 1 DONE, 2 FAULT, 3 REFUSED, 6:4 the last operation
//                            (1 GEMM, 2 WIN, 3 DUMP), 15:8 FAULT_CODE, 21:16 the GEMM's slot
//   0x110  S1_STEP       RW  4:0 step rows (1-24), 11:8 logit rows (0-8), 21:16 layers
//   0x114  S1_LOGIT_LO   RW  logit_map 31:0 (step row of logit row i at bits 5i +: 5)
//   0x118  S1_LOGIT_HI   RW  logit_map 39:32 in bits 7:0
//   0x11C  S1_CLASS      RW  write: 3:0 class, 15:8 exponent, 23:16 width; read: that class's entry
//   0x120  S1_WIN        RW  3:0 class, 12:8 rows (1-24), 22:16 K/16 (1-96)
//   0x124  S1_WIN_ADDR   RW  DDR byte address of the window stream
//   0x128  S1_DUMP       RW  15:0 first word / entry, 16 RES (0 staging, 1 residual)
//   0x12C  S1_DUMP_BEATS RW  15:0 beats (16 words or 8 entries a beat)
//   0x130  S1_DUMP_ADDR  RW  DDR byte address of the dump
//   0x134  S1_SAT        R   the drain's saturation count
//   0x138  S1_CYCLES     R   cycles the last operation took
//   0x13C  S1_BEATS      R   array beats the walker issued (since CLR)
// Reads return the register addressed the cycle before (csr_rdata, one cycle).
//
// Requests to the memory side (valid/ready, one an operation):
//   rq_*  read engine: rq_beats 64-B beats from rq_addr into the window loader
//   wq_*  write engine: wq_beats 64-B beats from the dump to wq_addr; wq_done
//         pulses when they are in DDR
//   wt_*  weight engine: stream tensor (wt_layer, wt_tag) into the ring
// A hardware sequencer can take the walker's and the drain's ports in its
// place; the blocks behind them do not change.
//
// Reset: synchronous, active low (the core's soft reset), all state.
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module me_s1_cmd (
  input  wire logic              clk,
  input  wire logic              rst_n,

  // ---- debug registers (the CSR block) ----
  input  wire logic              csr_we,
  input  wire logic [11:0]       csr_waddr,
  input  wire logic [31:0]       csr_wdata,
  input  wire logic [11:0]       csr_raddr,
  output logic [31:0]            csr_rdata,      // the register at csr_raddr, a cycle later
  output logic                   busy,           // an operation runs

  // ---- stand-ins: the step context and the class table ----
  output logic [5:0]             n_layers,
  output logic [4:0]             step_rows,
  output logic [3:0]             step_logit_rows,
  output logic [39:0]            logit_map,
  output logic [127:0]           cls_e,          // class c's exponent at [8c +: 8]
  output logic [127:0]           cls_w,          // class c's width at [8c +: 8]
  output logic                   cnt_clr,        // pulse: the step starts

  // ---- the walker's command port ----
  output logic                   cmd_valid,
  input  wire logic              cmd_ready,
  output logic [63:0]            cmd_word,
  output logic [5:0]             cmd_id,
  input  wire logic              cmd_done,
  input  wire logic              cmd_fault,
  input  wire logic [7:0]        cmd_fault_code,
  input  wire logic [31:0]       perf_beats,     // the walker's beat count

  // ---- the drain's command settings, finish and fault ----
  output logic                   dc_valid,
  output logic [5:0]             dc_id,
  output logic [63:0]            dc_word,
  input  wire logic              cmd_fin,
  input  wire logic [5:0]        cmd_fin_id,
  input  wire logic              drn_fault,
  input  wire logic [7:0]        drn_fault_code,
  input  wire logic [5:0]        drn_fault_cmd,
  input  wire logic [31:0]       drn_sat_cnt,

  // ---- the row store: window loader and dump ----
  output logic                   ld_start,
  output logic [3:0]             ld_cls,
  output logic [4:0]             ld_nrows,
  output logic [6:0]             ld_kch,
  input  wire logic              fill_ready,
  input  wire logic              ld_busy,
  input  wire logic              win_valid,
  output logic                   dp_start,
  output logic                   dp_res,
  output logic [15:0]            dp_first,
  output logic [15:0]            dp_beats,
  input  wire logic              dp_busy,

  // ---- the memory side ----
  output logic                   rq_valid,
  input  wire logic              rq_ready,
  output logic [31:0]            rq_addr,
  output logic [15:0]            rq_beats,
  output logic                   wq_valid,
  input  wire logic              wq_ready,
  output logic [31:0]            wq_addr,
  output logic [15:0]            wq_beats,
  input  wire logic              wq_done,
  output logic                   wt_valid,
  input  wire logic              wt_ready,
  output logic [5:0]             wt_layer,
  output logic [4:0]             wt_tag
);

  localparam logic [11:0] A_CMD_LO = 12'h100, A_CMD_HI = 12'h104, A_GO = 12'h108, A_STATUS = 12'h10C,
                          A_STEP = 12'h110, A_LOGIT_LO = 12'h114, A_LOGIT_HI = 12'h118, A_CLASS = 12'h11C,
                          A_WIN = 12'h120, A_WIN_ADDR = 12'h124, A_DUMP = 12'h128, A_DUMP_BEATS = 12'h12C,
                          A_DUMP_ADDR = 12'h130, A_SAT = 12'h134, A_CYCLES = 12'h138, A_BEATS = 12'h13C;
  localparam logic [2:0]  OP_NONE = 3'd0, OP_GEMM = 3'd1, OP_WIN = 3'd2, OP_DUMP = 3'd3;

  // ==========================================================================
  // the registers
  // ==========================================================================
  logic [63:0] r_cmd;
  logic [31:0] r_win, r_win_addr, r_dump, r_dump_beats, r_dump_addr;
  logic [3:0]  r_cls_sel;
  logic        st_done, st_fault, st_refused;
  logic [2:0]  st_op;
  logic [7:0]  st_code;
  logic [5:0]  st_slot;
  logic [31:0] st_cycles;

  // ---- the operations ----
  typedef enum logic [3:0] {
    S_IDLE,
    S_G_ISSUE,       // the GEMM word offered to the walker (and the weight request)
    S_G_RUN,         // the walker runs it
    S_G_CHK,         // after cmd_done: did it issue beats?
    S_G_FIN,         // waiting for the drain's cmd_fin
    S_W_REQ,         // the loader started, the read request offered
    S_W_RUN,         // the loader takes the stream
    S_W_FIN,         // the loader done: waiting for the window to be valid
    S_D_REQ,         // the dump started, the write request offered
    S_D_RUN,         // waiting for the dump's last beat and the write engine's done
    S_END            // the operation is over: its requests to the engines taken, then idle
  } state_e;
  state_e state;

  logic        wt_pend, rq_pend, wq_pend;        // a request not yet taken
  logic        seen_busy;                        // the loader / dump has started
  logic        wq_seen;                          // the write engine's done has come
  logic [31:0] beats0;                           // perf_beats at the walker's accept
  logic [5:0]  next_id;

  wire logic       go      = csr_we && (csr_waddr == A_GO);
  wire logic [3:0] go_bits = csr_wdata[3:0];
  wire logic       one_op  = (go_bits == 4'b0001) || (go_bits == 4'b0010) || (go_bits == 4'b0100) ||
                             (go_bits == 4'b1000);
  // window stream length: the constants' beat, then per row ceil(K/16 / 4) beats of 4 chunks
  wire logic [4:0]  w_rows  = r_win[12:8];
  wire logic [6:0]  w_kch   = r_win[22:16];
  wire logic [15:0] w_beats = 16'd1 + 16'(w_rows) * 16'((w_kch + 7'd3) >> 2);
  // an operation the blocks can take: a window the loader can hold, while the store can take a fill; a
  // dump of at least one beat
  wire logic       win_ok  = fill_ready && w_rows >= 5'd1 && w_rows <= 5'd24 && w_kch >= 7'd1 && w_kch <= 7'd96;
  wire logic       dump_ok = (r_dump_beats[15:0] != 16'd0);
  wire logic       accept  = go && one_op && (state == S_IDLE) && (!go_bits[1] || win_ok) && (!go_bits[2] || dump_ok);
  // the step context and the class table are live: not changed while an operation runs
  wire logic       ctx_wr  = csr_we && (csr_waddr == A_STEP || csr_waddr == A_LOGIT_LO || csr_waddr == A_LOGIT_HI ||
                                        csr_waddr == A_CLASS);
  me_cmd_pkg::cmd_t rc;
  assign rc = me_cmd_pkg::cmd_t'(r_cmd);

  always_ff @(posedge clk) begin
    cnt_clr  <= 1'b0;
    dc_valid <= 1'b0;
    ld_start <= 1'b0;
    dp_start <= 1'b0;
    if (!rst_n) begin
      r_cmd <= '0; r_win <= '0; r_win_addr <= '0; r_dump <= '0; r_dump_beats <= '0; r_dump_addr <= '0;
      n_layers <= '0; step_rows <= '0; step_logit_rows <= '0; logit_map <= '0;
      cls_e <= '0; cls_w <= '0; r_cls_sel <= '0;
      st_done <= 1'b0; st_fault <= 1'b0; st_refused <= 1'b0; st_op <= OP_NONE; st_code <= '0; st_slot <= '0;
      st_cycles <= '0;
      state <= S_IDLE; cmd_valid <= 1'b0; wt_pend <= 1'b0; rq_pend <= 1'b0; wq_pend <= 1'b0;
      seen_busy <= 1'b0; wq_seen <= 1'b0; beats0 <= '0; next_id <= '0; cmd_id <= '0; dc_id <= '0;
    end else begin
      // ---- register writes (the settings are not changed under a running operation's feet:
      //      they are copied at its start) ----
      if (csr_we && !(ctx_wr && busy)) begin
        case (csr_waddr)
          A_CMD_LO:     r_cmd[31:0]  <= csr_wdata;
          A_CMD_HI:     r_cmd[63:32] <= csr_wdata;
          A_STEP:       begin step_rows <= csr_wdata[4:0]; step_logit_rows <= csr_wdata[11:8];
                              n_layers <= csr_wdata[21:16]; end
          A_LOGIT_LO:   logit_map[31:0]  <= csr_wdata;
          A_LOGIT_HI:   logit_map[39:32] <= csr_wdata[7:0];
          A_CLASS:      begin r_cls_sel <= csr_wdata[3:0];
                              cls_e[8 * csr_wdata[3:0] +: 8] <= csr_wdata[15:8];
                              cls_w[8 * csr_wdata[3:0] +: 8] <= csr_wdata[23:16]; end
          A_WIN:        r_win        <= csr_wdata;
          A_WIN_ADDR:   r_win_addr   <= csr_wdata;
          A_DUMP:       r_dump       <= csr_wdata;
          A_DUMP_BEATS: r_dump_beats <= csr_wdata;
          A_DUMP_ADDR:  r_dump_addr  <= csr_wdata;
          default: ;
        endcase
      end

      // ---- GO ----
      if ((go && !accept) || (ctx_wr && busy)) st_refused <= 1'b1;
      if (accept) begin
        st_done <= 1'b0; st_fault <= 1'b0; st_code <= '0; st_cycles <= '0;
        case (go_bits)
          4'b0001: begin                                       // GEMM
            st_op     <= OP_GEMM;
            state     <= S_G_ISSUE;
            cmd_valid <= 1'b1;
            cmd_word  <= r_cmd;
            cmd_id    <= next_id;
            dc_word   <= r_cmd;
            dc_id     <= next_id;
            st_slot   <= next_id;
            next_id   <= next_id + 6'd1;
            wt_pend   <= 1'b1;
            wt_layer  <= rc.layer;
            wt_tag    <= rc.tag;
          end
          4'b0010: begin                                       // WIN
            st_op    <= OP_WIN;
            state    <= S_W_REQ;
            ld_start <= 1'b1;
            ld_cls   <= r_win[3:0];
            ld_nrows <= w_rows;
            ld_kch   <= w_kch;
            rq_pend  <= 1'b1;
            rq_addr  <= r_win_addr;
            rq_beats <= w_beats;
            seen_busy <= 1'b0;
          end
          4'b0100: begin                                       // DUMP
            st_op    <= OP_DUMP;
            state    <= S_D_REQ;
            dp_start <= 1'b1;
            dp_res   <= r_dump[16];
            dp_first <= r_dump[15:0];
            dp_beats <= r_dump_beats[15:0];
            wq_pend  <= 1'b1;
            wq_addr  <= r_dump_addr;
            wq_beats <= r_dump_beats[15:0];
            seen_busy <= 1'b0;
            wq_seen  <= 1'b0;
          end
          default: begin                                       // CLR: done at once
            cnt_clr    <= 1'b1;
            st_refused <= 1'b0;
            st_op      <= OP_NONE;
            st_done    <= 1'b1;
          end
        endcase
      end

      // ---- the requests to the engines ----
      if (wt_pend && wt_ready) wt_pend <= 1'b0;
      if (rq_pend && rq_ready) rq_pend <= 1'b0;
      if (wq_pend && wq_ready) wq_pend <= 1'b0;

      // ---- the operation ----
      if (state != S_IDLE) st_cycles <= st_cycles + 32'd1;
      case (state)
        S_G_ISSUE: if (cmd_ready) begin                        // the walker takes it: the drain gets it too
          cmd_valid <= 1'b0;
          dc_valid  <= 1'b1;
          beats0    <= perf_beats;
          state     <= S_G_RUN;
        end
        S_G_RUN: if (cmd_done) begin
          if (cmd_fault) begin                                 // the walker refused it: no tiles
            st_fault <= 1'b1; st_code <= cmd_fault_code;
            state    <= S_END;
          end else state <= S_G_CHK;
        end
        S_G_CHK: state <= (perf_beats == beats0) ? S_END : S_G_FIN;   // an empty row set: no tiles to wait for
        S_G_FIN: if (cmd_fin && cmd_fin_id == st_slot) begin
          if (drn_fault && drn_fault_cmd == st_slot) begin st_fault <= 1'b1; st_code <= drn_fault_code; end
          state <= S_END;
        end
        S_W_REQ: begin
          if (ld_busy) seen_busy <= 1'b1;
          if (!rq_pend || rq_ready) state <= S_W_RUN;
        end
        S_W_RUN: begin
          if (ld_busy) seen_busy <= 1'b1;
          if (seen_busy && !ld_busy) state <= S_W_FIN;         // the loader has taken the whole stream
        end
        S_W_FIN: if (win_valid) state <= S_END;
        S_D_REQ: begin
          if (dp_busy) seen_busy <= 1'b1;
          if (wq_done) wq_seen <= 1'b1;
          if (!wq_pend || wq_ready) state <= S_D_RUN;
        end
        S_D_RUN: begin
          if (dp_busy) seen_busy <= 1'b1;
          if (wq_done) wq_seen <= 1'b1;
          if ((seen_busy && !dp_busy) && (wq_seen || wq_done)) state <= S_END;
        end
        S_END: if (!wt_pend && !rq_pend && !wq_pend) begin state <= S_IDLE; st_done <= 1'b1; end
        default: ;
      endcase
    end
  end

  assign wt_valid = wt_pend;
  assign rq_valid = rq_pend;
  assign wq_valid = wq_pend;
  assign busy     = (state != S_IDLE);

  // ==========================================================================
  // reads
  // ==========================================================================
  always_ff @(posedge clk) begin
    case (csr_raddr)
      A_CMD_LO:     csr_rdata <= r_cmd[31:0];
      A_CMD_HI:     csr_rdata <= r_cmd[63:32];
      A_STATUS:     csr_rdata <= {10'd0, st_slot, st_code, 1'b0, st_op, st_refused, st_fault, st_done, busy};
      A_STEP:       csr_rdata <= {10'd0, n_layers, 4'd0, step_logit_rows, 3'd0, step_rows};
      A_LOGIT_LO:   csr_rdata <= logit_map[31:0];
      A_LOGIT_HI:   csr_rdata <= {24'd0, logit_map[39:32]};
      A_CLASS:      csr_rdata <= {8'd0, cls_w[8 * r_cls_sel +: 8], cls_e[8 * r_cls_sel +: 8], 4'd0, r_cls_sel};
      A_WIN:        csr_rdata <= r_win;
      A_WIN_ADDR:   csr_rdata <= r_win_addr;
      A_DUMP:       csr_rdata <= r_dump;
      A_DUMP_BEATS: csr_rdata <= r_dump_beats;
      A_DUMP_ADDR:  csr_rdata <= r_dump_addr;
      A_SAT:        csr_rdata <= drn_sat_cnt;
      A_CYCLES:     csr_rdata <= st_cycles;
      A_BEATS:      csr_rdata <= perf_beats;
      default:      csr_rdata <= '0;
    endcase
  end

endmodule

`default_nettype wire
