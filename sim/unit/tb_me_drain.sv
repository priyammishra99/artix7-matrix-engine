// ============================================================================
// File: tb_me_drain.sv
// Block: Matrix engine - unit testbench for me_drain (the GEMM path)
//
// Drives the drain as the walker and the array do, from the vector
// files (vectors/): the command word (dc_*), each 32-channel group's scale beat, and the
// tiles (4 rows x 8 channels of the reference model's accumulators, in the
// walker's order g, ct, rg, with their tags), offered with random gaps or back
// to back. Group g + 2's scale beat is sent the cycle after group g's last tile
// handshake (the earliest the walker's guarantee allows). A model of the row
// store answers the row constants the same cycle and its staging and residual
// reads 2 cycles later, and keeps the written memory.
//
// Commands run back to back (the next one's tiles follow the last one's at
// once) unless they share a destination memory; at each cmd_fin the whole of
// staging and the residual is compared with the expected image (every output
// where it belongs, nothing written anywhere else), and cmd_fin_id, the
// saturation count and the fault are checked.
//
// Cases, in order:
//   real GEMMs (r5: 5 rows, a partial row group): each flag-free (FLAGS = 0) to
//     its real destination against gemm_<c>_ref.mem; the ACC ones (W_O, W_down)
//     with their real flags against the reference model's gemm_<c>_out.mem, the
//     old values preloaded from gemm_<c>_dest.mem; l0_q flag-free to STG32 (the
//     one-element-a-cycle mode); l0_q with its real flags (RoPE): fault 0x21,
//     nothing written
//   edge cases (drain_manifest.txt): rounding ties, the accumulator's extremes,
//     zero, subnormal and huge scales, negative shifts, saturation at INT16 and
//     INT32, ACC at the limits, at offsets DOFF > 0; edge_i16 again on
//     ROWSET_LOGIT (rows through logit_map); the SCALE fault (infinity, NaN):
//     0x42 with the command's slot
// Configuration (drain_run.cfg, "KEY value" lines): DIR <vectors>, SEED n,
// GAP n (0: tiles back to back; n: 0 .. n idle cycles before each tile),
// REAL <case,case,...> (default l0_q,l0_o,l0_down,l29_q,l29_down), EDGE 0/1.
// Ends with "DRAIN RESULTS: N checks, F failures" and ">> VERDICT: PASS/FAIL <<".
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module tb_me_drain #(parameter int RD_LAT = 2);
  import me_if_pkg::*;

  logic clk = 1'b0;
  always #4 clk = ~clk;
  logic rst_n = 1'b0;

  // ---- DUT ----
  logic [39:0]   logit_map = '0;
  logic          cnt_clr = 1'b0;
  logic [127:0]  cls_e = '0, cls_w = '0;
  logic          dc_valid = 1'b0;  logic [5:0] dc_id = '0;  logic [63:0] dc_word = '0;
  logic          scale_valid = 1'b0; logic [10:0] scale_grp = '0; logic [511:0] scale_data = '0;
  logic          tile_valid = 1'b0, tile_ready;
  mei_tile_bus_t tile_acc = '0;
  mei_tag_t      tile_tag = '0;
  logic [4:0]    drn_row;  logic [14:0] drn_absmax;  logic [4:0] drn_shift;
  logic          stg_we, stg_rd;  logic [15:0] stg_waddr, stg_raddr;  logic [1:0] stg_whe;
  logic [31:0]   stg_wdata, stg_rdata;
  logic          res_we, res_rd;  logic [12:0] res_waddr, res_raddr;  logic [1:0] res_wee;
  logic [63:0]   res_wdata, res_rdata;
  logic          kvo_we;  logic [10:0] kvo_waddr;  logic [7:0] kvo_wbe;  logic [63:0] kvo_wdata;
  logic          cmd_fin, fault, fault_pulse, busy;
  logic [5:0]    cmd_fin_id, fault_cmd;
  logic [7:0]    fault_code;
  logic [31:0]   sat_cnt, perf_tiles, perf_outs;

  me_drain #(.RD_LAT(RD_LAT)) dut (
    .clk, .rst_n, .logit_map, .cnt_clr, .cls_e, .cls_w,
    .dc_valid, .dc_id, .dc_word, .scale_valid, .scale_grp, .scale_data,
    .tile_valid, .tile_ready, .tile_acc, .tile_tag_valid(tile_valid), .tile_tag,
    .drn_row, .drn_absmax, .drn_shift,
    .stg_we, .stg_waddr, .stg_whe, .stg_wdata, .stg_rd, .stg_raddr, .stg_rdata,
    .res_we, .res_waddr, .res_wee, .res_wdata, .res_rd, .res_raddr, .res_rdata,
    .kvo_we, .kvo_waddr, .kvo_wbe, .kvo_wdata,
    .cmd_fin, .cmd_fin_id, .fault, .fault_code, .fault_cmd, .fault_pulse, .busy,
    .sat_cnt, .perf_tiles, .perf_outs);

  // ==========================================================================
  // the row store's model
  // ==========================================================================
  localparam int NSTG = 36864, NRES = 6912;
  logic [31:0] stg [NSTG];   logic [31:0] stg_x [NSTG];       // the memory, and the expected image
  logic [63:0] res [NRES];   logic [63:0] res_x [NRES];
  logic [14:0] rc_ab [24];   logic [4:0] rc_sh [24];
  assign drn_absmax = rc_ab[drn_row];
  assign drn_shift  = rc_sh[drn_row];

  logic [31:0] sr [RD_LAT];  logic [63:0] rr [RD_LAT];
  assign stg_rdata = sr[RD_LAT - 1];
  assign res_rdata = rr[RD_LAT - 1];
  int checks = 0, fails = 0;
  always @(posedge clk) begin
    sr[0] <= stg_rd ? stg[stg_raddr] : 32'hDEAD_BEEF;
    rr[0] <= res_rd ? res[res_raddr] : 64'hDEAD_BEEF_DEAD_BEEF;
    for (int k = 1; k < RD_LAT; k++) begin sr[k] <= sr[k - 1]; rr[k] <= rr[k - 1]; end
    if (stg_we) begin
      if (stg_waddr >= NSTG) begin fails++; $display("[FAIL] staging write out of range %0d", stg_waddr); end
      else begin
        if (stg_whe[0]) stg[stg_waddr][15:0]  <= stg_wdata[15:0];
        if (stg_whe[1]) stg[stg_waddr][31:16] <= stg_wdata[31:16];
      end
    end
    if (res_we) begin
      if (res_waddr >= NRES) begin fails++; $display("[FAIL] residual write out of range %0d", res_waddr); end
      else begin
        if (res_wee[0]) res[res_waddr][31:0]  <= res_wdata[31:0];
        if (res_wee[1]) res[res_waddr][63:32] <= res_wdata[63:32];
      end
    end
    if (kvo_we) begin fails++; $display("[FAIL] K/V write in stage 1"); end
  end

  // ==========================================================================
  // the commands and their expected writes
  // ==========================================================================
  localparam int B_RES = 0, B_STG16 = 1, B_STG32 = 2;
  localparam int K_NORMAL = 0, K_SCALE = 1, K_UNSUP = 2;
  typedef struct { int buf_; int addr; int half; logic [31:0] val; } wr_t;   // half: 0, 1; 2 = the word
  wr_t   ew [$];                                    // every command's writes, one after another
  localparam int NCMD = 64;
  string c_name [NCMD];
  int    c_id [NCMD], c_buf [NCMD], c_kind [NCMD], c_sat [NCMD], c_base [NCMD], c_cnt [NCMD];
  int    n_cmd = 0, n_fin = 0, cum_sat = 0;
  real   c_rate [NCMD];                             // cycles a tile, handshake to handshake
  longint cyc = 0, t_first, t_last;
  always @(posedge clk) cyc <= cyc + 1;
  bit    all_sent = 0;

  function automatic void put(int b, int srow, int e, logic [31:0] v, input bit to_mem);
    wr_t w;
    w.buf_ = b;
    case (b)
      B_RES:   begin w.addr = srow * 288 + e / 2;  w.half = e % 2; end
      B_STG16: begin w.addr = srow * 1536 + e / 2; w.half = e % 2; end
      default: begin w.addr = srow * 1536 + e;     w.half = 2;     end
    endcase
    w.val = v;
    if (to_mem) begin                                // a preload: the model and the image alike
      case (b)
        B_RES:   begin res[w.addr][32 * w.half +: 32] = v; res_x[w.addr][32 * w.half +: 32] = v; end
        B_STG16: begin stg[w.addr][16 * w.half +: 16] = v[15:0]; stg_x[w.addr][16 * w.half +: 16] = v[15:0]; end
        default: begin stg[w.addr] = v; stg_x[w.addr] = v; end
      endcase
    end else ew.push_back(w);
  endfunction

  // at each cmd_fin: the command's writes into the image, then the whole memory against it. cmd_fin comes
  // with the last write; the memory has it from the next edge on, so the check is half a cycle after it
  int fin_i;
  logic fin_seen = 1'b0;
  logic [5:0] fin_id;
  always @(posedge clk) begin fin_seen <= rst_n && cmd_fin; fin_id <= cmd_fin_id; end
  always @(negedge clk) begin
    if (fin_seen) begin
      fin_i = n_fin;
      checks++;
      if (fin_i >= n_cmd) begin fails++; $display("[FAIL] cmd_fin with no command running"); end
      else begin
        int bad;
        wr_t w;
        bad = 0;
        if (fin_id != 6'(c_id[fin_i])) begin
          fails++; $display("[FAIL] %s: cmd_fin_id %0d, expected %0d", c_name[fin_i], fin_id, c_id[fin_i]);
        end
        for (int i = c_base[fin_i]; i < c_base[fin_i] + c_cnt[fin_i]; i++) begin
          w = ew[i];
          case (w.buf_)
            B_RES:   res_x[w.addr][32 * w.half +: 32] = w.val;
            B_STG16: stg_x[w.addr][16 * w.half +: 16] = w.val[15:0];
            default: stg_x[w.addr] = w.val;
          endcase
        end
        if (c_kind[fin_i] == K_SCALE) begin            // outputs of a malformed scale: not compared
          for (int a = 0; a < NSTG; a++) stg_x[a] = stg[a];
          for (int a = 0; a < NRES; a++) res_x[a] = res[a];
        end
        for (int a = 0; a < NSTG; a++)
          if (stg[a] !== stg_x[a]) begin
            if (bad < 6) $display("[FAIL] %s: staging word %0d (row %0d, word %0d) = %h, expected %h",
                                  c_name[fin_i], a, a / 1536, a % 1536, stg[a], stg_x[a]);
            bad++;
          end
        for (int a = 0; a < NRES; a++)
          if (res[a] !== res_x[a]) begin
            if (bad < 6) $display("[FAIL] %s: residual entry %0d (row %0d, entry %0d) = %h, expected %h",
                                  c_name[fin_i], a, a / 288, a % 288, res[a], res_x[a]);
            bad++;
          end
        checks++;
        if (bad) begin fails++; $display("[FAIL] %s: %0d memory words wrong", c_name[fin_i], bad); end
        // saturation and fault
        cum_sat += c_sat[fin_i];
        checks++;
        if (c_kind[fin_i] != K_SCALE && sat_cnt != 32'(cum_sat)) begin
          fails++; $display("[FAIL] %s: sat_cnt %0d, expected %0d", c_name[fin_i], sat_cnt, cum_sat);
        end
        checks++;
        case (c_kind[fin_i])
          K_NORMAL: if (fault) begin fails++; $display("[FAIL] %s: fault %h", c_name[fin_i], fault_code); end
          K_SCALE:  if (!fault || fault_code != 8'h42 || fault_cmd != 6'(c_id[fin_i])) begin
                      fails++; $display("[FAIL] %s: fault %b code %h cmd %0d, expected 42 cmd %0d",
                                        c_name[fin_i], fault, fault_code, fault_cmd, c_id[fin_i]);
                    end
          K_UNSUP:  if (!fault || fault_code != 8'h21 || fault_cmd != 6'(c_id[fin_i])) begin
                      fails++; $display("[FAIL] %s: fault %b code %h cmd %0d, expected 21 cmd %0d",
                                        c_name[fin_i], fault, fault_code, fault_cmd, c_id[fin_i]);
                    end
        endcase
        $display("  done %-26s slot %2d  %6d outputs  sat %3d  %5.1f cycles/tile  %s%s", c_name[fin_i], c_id[fin_i],
                 c_cnt[fin_i], c_sat[fin_i], c_rate[fin_i], bad ? "MEMORY WRONG" : "memory ok",
                 c_kind[fin_i] == K_NORMAL ? "" : $sformatf(", fault %h", fault_code));
      end
      n_fin++;
    end
  end

  // ==========================================================================
  // drivers: dc words and scale beats, one a cycle, from queues
  // ==========================================================================
  logic [63:0]  dcq_w [$];  int dcq_id [$];
  logic [511:0] scq_d [$];  int scq_g [$];
  always @(negedge clk) begin
    dc_valid <= 1'b0;  scale_valid <= 1'b0;
    if (dcq_w.size()) begin dc_valid <= 1'b1; dc_word <= dcq_w.pop_front(); dc_id <= 6'(dcq_id.pop_front()); end
    if (scq_d.size()) begin scale_valid <= 1'b1; scale_data <= scq_d.pop_front(); scale_grp <= 11'(scq_g.pop_front()); end
  end

  // ==========================================================================
  // the run
  // ==========================================================================
  string dir = "", real_list = "l0_q,l0_o,l0_down,l29_q,l29_down";
  int    seed = 1, gap = 0, edge_on = 1;
  int    cls_ev [16], cls_wv [16];

  logic [33:0]  f_acc [65536];
  logic [511:0] f_sc [2048];
  logic [15:0]  f_ab [24];
  logic [7:0]   f_sh [24];
  logic [31:0]  f_exp [65536];
  logic [31:0]  f_dst [65536];

  function automatic logic [63:0] mk_cmd(int flags, int dbuf, int tag, int ic, int oc, int layer, int rowset, int doff);
    me_cmd_pkg::cmd_t c;
    c = '0;
    c.op = 6'(me_cmd_pkg::OP_GEMM); c.flags = 10'(flags); c.abuf = 4'(me_cmd_pkg::BUF_STG16);
    c.dbuf = 4'(dbuf); c.tag = 5'(tag); c.iclass = 4'(ic); c.oclass = 4'(oc); c.layer = 6'(layer);
    c.rowset = 2'(rowset); c.doff = 6'(doff);
    return 64'(c);
  endfunction

  function automatic int ifbuf(int b);                // interface buffer code -> ours
    return (b == me_cmd_pkg::BUF_RES) ? B_RES : (b == me_cmd_pkg::BUF_STG16) ? B_STG16 : B_STG32;
  endfunction

  // one command: preload, expected writes, then its dc word, scales and tiles
  task automatic run_cmd(string name, logic [63:0] word, int nrows, int n, int ngrp, int kind, int sat,
                         bit have_exp, bit have_dst);
    me_cmd_pkg::cmd_t c = me_cmd_pkg::cmd_t'(word);
    int b   = ifbuf(c.dbuf);
    int k   = n_cmd;
    int id  = (7 * k + 3) % 64;
    int nrg = (nrows + 3) / 4;
    bit lg  = (c.rowset == 2'(me_cmd_pkg::ROWSET_LOGIT));
    // a command sharing the last one's memory (STG16 and STG32 are one memory) waits for its end, and so
    // do the fault cases (they start from a clear count); any other waits for the one before the last
    if (k > 0 && (kind != K_NORMAL || c_kind[k - 1] != K_NORMAL || ((b == B_RES) == (c_buf[k - 1] == B_RES))))
      wait (n_fin >= k);
    else if (k > 1)
      wait (n_fin >= k - 1);
    if (kind != K_NORMAL || (k > 0 && c_kind[k - 1] != K_NORMAL)) begin
      @(negedge clk) cnt_clr <= 1'b1;
      @(negedge clk) cnt_clr <= 1'b0;
      cum_sat = 0;
    end
    c_name[k] = name; c_id[k] = id; c_buf[k] = b; c_kind[k] = kind; c_sat[k] = sat; c_base[k] = ew.size();
    for (int i = 0; i < nrows; i++) begin
      int srow = lg ? int'(logit_map[5 * i +: 5]) : i;
      for (int ch = 0; ch < n; ch++) begin
        int e = 64 * c.doff + ch;
        if (have_dst) put(b, srow, e, f_dst[i * n + ch], 1'b1);
        else if (kind != K_UNSUP) put(b, srow, e, $urandom(), 1'b1);       // garbage to be overwritten
        if (have_exp && kind == K_NORMAL) put(b, srow, e, f_exp[i * n + ch], 1'b0);
      end
    end
    c_cnt[k] = ew.size() - c_base[k];
    for (int r = 0; r < 24; r++) begin rc_ab[r] = (r < nrows) ? f_ab[r][14:0] : 15'($urandom); rc_sh[r] = (r < nrows) ? f_sh[r][4:0] : 5'($urandom); end
    n_cmd++;
    // the dc word, the first two groups' scales
    dcq_w.push_back(word); dcq_id.push_back(id);
    for (int g = 0; g < 2 && g < ngrp; g++) begin scq_d.push_back(f_sc[g]); scq_g.push_back(g); end
    wait (dcq_w.size() == 0 && scq_d.size() == 0);
    @(negedge clk);
    // the tiles, in the walker's order
    t_first = -1;
    for (int g = 0; g < ngrp; g++)
      for (int ct = 0; ct < 4; ct++)
        for (int rg = 0; rg < nrg; rg++) begin
          mei_tag_t t;
          int gp = (gap > 0) ? $urandom_range(gap, 0) : 0;
          repeat (gp) @(negedge clk);
          t = '0;
          t.dest = MEI_DEST_DRAIN; t.cmd = 6'(id); t.grp = 13'(4 * g + ct); t.rgrp = 3'(rg);
          for (int r = 0; r < 4; r++) t.rvalid[r] = (4 * rg + r < nrows);
          t.last = (g == ngrp - 1) && (ct == 3) && (rg == nrg - 1);
          for (int r = 0; r < 4; r++)
            for (int j = 0; j < 8; j++)
              tile_acc[MEI_ACC_W * (8 * r + j) +: MEI_ACC_W] <= (4 * rg + r < nrows)
                ? f_acc[(4 * rg + r) * n + 32 * g + 8 * ct + j] : {$urandom, $urandom};
          tile_tag <= t; tile_valid <= 1'b1;
          @(posedge clk);
          while (!tile_ready) @(posedge clk);
          if (t_first < 0) t_first = cyc;
          t_last = cyc;
          @(negedge clk) tile_valid <= 1'b0;
          tile_acc <= {34 {$urandom}};                 // stale data after the handshake
          if (ct == 3 && rg == nrg - 1 && g + 2 < ngrp) begin     // group g done: g + 2's scales at once
            scq_d.push_back(f_sc[g + 2]); scq_g.push_back(g + 2);
          end
        end
    c_rate[k] = (ngrp * 4 * nrg > 1) ? real'(t_last - t_first) / real'(ngrp * 4 * nrg - 1) : 0.0;
  endtask

  // watchdog: no command finished for 200,000 cycles (a tile takes at most ~80 here)
  longint last_fin = 0;
  always @(posedge clk) begin
    if (fin_seen) last_fin <= cyc;
    if (cyc - last_fin > 200000) begin
      $display("[FAIL] watchdog: %0d of %0d commands finished", n_fin, n_cmd);
      $display("DRAIN RESULTS: %0d checks, %0d failures (%0d commands, %0d tiles)", checks, fails + 1, n_cmd, perf_tiles);
      $display(">> VERDICT: FAIL <<");
      $finish;
    end
  end

  initial begin
    int fd, nr, cnt;
    string key, val;
    // ---- configuration ----
    fd = $fopen("drain_run.cfg", "r");
    if (fd == 0) $fatal(1, "no drain_run.cfg");
    while (!$feof(fd)) begin
      nr = $fscanf(fd, "%s %s\n", key, val);
      if (nr == 2) begin
        if (key == "DIR") dir = val;
        else if (key == "SEED") seed = val.atoi();
        else if (key == "GAP") gap = val.atoi();
        else if (key == "REAL") real_list = val;
        else if (key == "EDGE") edge_on = val.atoi();
      end
    end
    $fclose(fd);
    begin process pr; pr = process::self(); pr.srandom(seed); end
    $display("tb_me_drain: DIR %s SEED %0d GAP %0d REAL %s EDGE %0d RD_LAT %0d", dir, seed, gap, real_list, edge_on, RD_LAT);
    // ---- class table ----
    fd = $fopen({dir, "/drain_classes.txt"}, "r");
    while (!$feof(fd)) begin
      int id, e, w; string nm;
      nr = $fscanf(fd, "%d %s %d %d\n", id, nm, e, w);
      if (nr == 4) begin cls_e[8 * id +: 8] = 8'(e); cls_w[8 * id +: 8] = 8'(w); cls_ev[id] = e; cls_wv[id] = w; end
    end
    $fclose(fd);
    for (int i = 0; i < 8; i++) logit_map[5 * i +: 5] = 5'((7 * i + 19) % 24);    // logit row i -> a scattered step row
    for (int a = 0; a < NSTG; a++) begin stg[a] = $urandom; stg_x[a] = stg[a]; end
    for (int a = 0; a < NRES; a++) begin res[a] = {$urandom, $urandom}; res_x[a] = res[a]; end
    repeat (4) @(negedge clk);
    rst_n = 1'b1;

    // ---- real GEMMs ----
    begin
      string names [$];
      int p;
      p = 0;
      for (int i = 0; i <= real_list.len(); i++)
        if (i == real_list.len() || real_list[i] == ",") begin names.push_back(real_list.substr(p, i - 1)); p = i + 1; end
      foreach (names[q]) begin
        string nm;
        logic [63:0] w; int f [10];
        int ic, oc, e_in, e_out, ref_mode, ref_sat, out_check, out_sat;
        int n, kk, nused, beats, groups, nout;
        me_cmd_pkg::cmd_t c;
        nm = names[q];
        // manifest line
        fd = $fopen({dir, "/manifest.txt"}, "r");
        n = 0;
        while (!$feof(fd)) begin
          string mn;
          nr = $fscanf(fd, "%s %d %d %d %d %d %d\n", mn, n, kk, nused, beats, groups, nout);
          if (nr == 7 && mn == nm) break;
          n = 0;
        end
        $fclose(fd);
        if (n == 0) $fatal(1, "case %s not in the manifest", nm);
        fd = $fopen({dir, "/gemm_", nm, "_cmd.txt"}, "r");
        void'($fscanf(fd, "%h %h %h %h %h %h %h %h %h %h %h", w, f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7], f[8], f[9]));
        $fclose(fd);
        fd = $fopen({dir, "/gemm_", nm, "_drain.txt"}, "r");
        void'($fscanf(fd, "%d %d %d %d %d %d %d %d", ic, oc, e_in, e_out, ref_mode, ref_sat, out_check, out_sat));
        $fclose(fd);
        $readmemh({dir, "/gemm_", nm, "_acc.mem"}, f_acc, 0, nused * n - 1);
        $readmemh({dir, "/gemm_", nm, "_scales.mem"}, f_sc, 0, groups - 1);
        $readmemh({dir, "/gemm_", nm, "_absmax.mem"}, f_ab, 0, nused - 1);
        $readmemh({dir, "/gemm_", nm, "_shift.mem"}, f_sh, 0, nused - 1);
        c = me_cmd_pkg::cmd_t'(w);
        // flag-free to its real destination (or STG16 / STG32 by the output width), when it fits a row
        if (ref_mode >= 0) begin
          int db, cap;
          db  = (c.dbuf == 4'(me_cmd_pkg::BUF_RES) || c.dbuf == 4'(me_cmd_pkg::BUF_STG16) ||
                 c.dbuf == 4'(me_cmd_pkg::BUF_STG32)) ? int'(c.dbuf)
                : (ref_mode == 2 ? me_cmd_pkg::BUF_STG32 : me_cmd_pkg::BUF_STG16);
          cap = (db == me_cmd_pkg::BUF_RES) ? 576 : (db == me_cmd_pkg::BUF_STG16) ? 3072 : 1536;
          if (64 * c.doff + n <= cap) begin
            $readmemh({dir, "/gemm_", nm, "_ref.mem"}, f_exp, 0, nused * n - 1);
            run_cmd({nm, " flag-free"}, mk_cmd(0, db, c.tag, c.iclass, c.oclass, c.layer, c.rowset, c.doff),
                    nused, n, groups, K_NORMAL, ref_sat, 1, 0);
          end
          // the one-element-a-cycle mode on real data
          if (nm == "l0_q") begin
            run_cmd({nm, " flag-free STG32"}, mk_cmd(0, me_cmd_pkg::BUF_STG32, c.tag, c.iclass, c.oclass, c.layer,
                    c.rowset, 9), nused, n, groups, K_NORMAL, ref_sat, 1, 0);
          end
        end
        // with its real flags: ACC against the reference model's outputs; another flag: refused
        if (out_check && c.flags != 0) begin
          $readmemh({dir, "/gemm_", nm, "_out.mem"}, f_exp, 0, nused * n - 1);
          $readmemh({dir, "/gemm_", nm, "_dest.mem"}, f_dst, 0, nused * n - 1);
          run_cmd({nm, " ACC (reference)"}, w, nused, n, groups, K_NORMAL, out_sat, 1, 1);
        end else if (c.flags & ~10'(1 << me_cmd_pkg::CF_ACC)) begin
          run_cmd({nm, " real flags"}, w, nused, n, groups, K_UNSUP, 0, 0, 0);
        end
      end
    end

    // ---- edge cases ----
    if (edge_on) begin
      fd = $fopen({dir, "/drain_manifest.txt"}, "r");
      while (!$feof(fd)) begin
        string nm; int rows, n, ic, oc, mode, acc, flt, sat, db, doff, kind;
        nr = $fscanf(fd, "%s %d %d %d %d %d %d %h %d\n", nm, rows, n, ic, oc, mode, acc, flt, sat);
        if (nr != 9) continue;
        $readmemh({dir, "/drain_", nm, "_acc.mem"}, f_acc, 0, rows * n - 1);
        $readmemh({dir, "/drain_", nm, "_scales.mem"}, f_sc, 0, n / 32 - 1);
        $readmemh({dir, "/drain_", nm, "_absmax.mem"}, f_ab, 0, rows - 1);
        $readmemh({dir, "/drain_", nm, "_shift.mem"}, f_sh, 0, rows - 1);
        if (acc) $readmemh({dir, "/drain_", nm, "_dest.mem"}, f_dst, 0, rows * n - 1);
        if (!flt) $readmemh({dir, "/drain_", nm, "_ref.mem"}, f_exp, 0, rows * n - 1);
        kind = flt ? K_SCALE : K_NORMAL;
        // destinations: INT16 to STG16; INT32 with ACC to the residual, without to STG32
        db   = (mode == 0) ? me_cmd_pkg::BUF_STG16 : acc ? me_cmd_pkg::BUF_RES : me_cmd_pkg::BUF_STG32;
        doff = (db == me_cmd_pkg::BUF_RES) ? 3 : (db == me_cmd_pkg::BUF_STG16) ? 5 : 2;
        run_cmd({"edge ", nm}, mk_cmd(acc ? (1 << me_cmd_pkg::CF_ACC) : 0, db, 1, ic, oc, 0, 0, doff),
                rows, n, n / 32, kind, sat, 1, acc);
        if (nm == "edge_i16")
          run_cmd({"edge ", nm, " LOGIT"}, mk_cmd(0, db, 17, ic, oc, 0, me_cmd_pkg::ROWSET_LOGIT, 7),
                  rows, n, n / 32, K_NORMAL, sat, 1, 0);
      end
      $fclose(fd);
    end

    // ---- the end ----
    all_sent = 1;
    wait (n_fin >= n_cmd);
    repeat (20) @(posedge clk);
    checks++;
    if (busy) begin fails++; $display("[FAIL] busy at the end"); end
    $display("DRAIN RESULTS: %0d checks, %0d failures (%0d commands, %0d tiles)", checks, fails, n_cmd, perf_tiles);
    $display(">> VERDICT: %s <<", fails == 0 ? "PASS" : "FAIL");
    $finish;
  end

endmodule

// the store's reads one cycle slower
module tb_me_drain_lat3;
  tb_me_drain #(.RD_LAT(3)) u ();
endmodule

`default_nettype wire
