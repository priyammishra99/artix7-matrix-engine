// ============================================================================
// File: me_weight_ring.sv
// Block: Matrix engine - weight ring (1,024 slots x 1,024 bits, 28.5 BRAM36,
//        instantiated)
//
// The weight stream lands here in the order DDR holds it, one 64-B beat a cycle at most, and the tile walker reads
// it one slot a cycle:
//   - a GEMM tensor's run is N/32 groups; each group is one scale beat, then
//     4 x K/16 weight slots of two beats each. A scale beat takes a slot of its
//     own (bits 511:0; 1,023:512 are 0); a weight slot is beat 2i in bits 511:0
//     and beat 2i+1 in bits 1,023:512, so byte 16c + d = W[8ct + c][16kc + d];
//   - any other entry (a VECTOR's gamma) is stored one beat a slot, its
//     beats count taken from the weight list.
// Each run is announced by its descriptor (the weight list entry: layer, tag,
// kind, N, K, beats) before its beats. The ring keeps the descriptors in one
// FIFO with two read pointers: the writer's (to frame the run's beats into
// slots) and the walker's (the entry port: offered as soon as it is queued,
// so at the latest when the run's first slot is written).
//
// Slots are freed by count from the walker (oldest first). A beat is taken only
// while fewer than 1,024 slots are live, so the ring never holds more than its
// 1,024 slots and a live slot is never overwritten.
//
// Storage: 28 RAMB36E1 (1K x 36) and one RAMB18E1 (1K x 18, 16 bits used), side
// by side: 1,024 bits a slot, one write port (a whole slot at once: a weight
// slot's first beat waits in a 512-bit register for its second) and one read
// port with its output register: data W_LAT = 2 cycles after ring_rd.
// USE_PRIM = 0 replaces the primitives with an equivalent behavioural memory
// (fast simulation).
//
// Interfaces to the memory side:
//   - run descriptors: one per weight list entry streamed, in stream order;
//   - beats: from the read crossing FIFO (EPORT_RING), valid / ready.
//
// Reset: synchronous, active low (the core's soft reset): the counters, the
// descriptor FIFO and the writer clear; the memory keeps stale data that no
// pointer reaches.
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module me_weight_ring #(
  parameter int  ENT_DEPTH = 16,     // run descriptors queued (power of 2)
  parameter bit  USE_PRIM  = 1'b1    // 1: RAMB36E1 / RAMB18E1 primitives; 0: behavioural memory
) (
  input  wire logic              clk,
  input  wire logic              rst_n,

  // ---- run descriptors, from the memory side ----
  input  wire logic              run_valid,
  output logic                   run_ready,
  input  wire logic [5:0]        run_layer,
  input  wire logic [4:0]        run_tag,
  input  wire logic [1:0]        run_kind,       // wlist_entry_t.kind: 0 GEMM
  input  wire logic [15:0]       run_n,
  input  wire logic [15:0]       run_k,
  input  wire logic [23:0]       run_beats,      // wlist_entry_t.beats (used for kinds other than GEMM)

  // ---- beats, from the memory side ----
  input  wire logic              in_valid,
  output logic                   in_ready,
  input  wire logic [511:0]      in_data,        // byte 0 in bits 7:0

  // ---- to the walker: tensor entries ----
  output logic                   ent_valid,
  input  wire logic              ent_ready,
  output logic [5:0]             ent_layer,
  output logic [4:0]             ent_tag,
  output logic [1:0]             ent_kind,
  output logic [15:0]            ent_n,
  output logic [15:0]            ent_k,

  // ---- to the walker: slots ----
  output logic [10:0]            ring_wptr,      // slots written since reset, modulo 2,048
  input  wire logic              ring_rd,
  input  wire logic [9:0]        ring_raddr,
  output logic [1023:0]          ring_rdata,     // 2 cycles after ring_rd
  input  wire logic              ring_free,
  input  wire logic [6:0]        ring_free_n,

  // ---- status ----
  output logic                   err_free,       // sticky: more slots freed than written
  output logic [31:0]            perf_slots,     // slots written
  output logic [31:0]            perf_full       // cycles a beat waited because 1,024 slots were live
);

  // --------------------------------------------------------------------------
  // run descriptors: one FIFO, two readers (the writer, the walker's entry port)
  // --------------------------------------------------------------------------
  localparam int EA = $clog2(ENT_DEPTH);
  typedef struct packed {
    logic [5:0]  layer;
    logic [4:0]  tag;
    logic [1:0]  kind;
    logic [15:0] n;
    logic [15:0] k;
    logic [23:0] beats;
  } run_t;

  run_t        ent_mem [ENT_DEPTH];
  logic [EA:0] e_wp, e_rpw, e_rpk;               // write; read: the writer's, the walker's
  wire logic   e_full  = ((e_wp - e_rpw) == (EA+1)'(ENT_DEPTH)) || ((e_wp - e_rpk) == (EA+1)'(ENT_DEPTH));
  wire logic   e_push  = run_valid && run_ready;
  wire logic   e_popk  = ent_valid && ent_ready;

  assign run_ready = !e_full;

  always_ff @(posedge clk) begin
    if (e_push) ent_mem[e_wp[EA-1:0]] <= '{layer: run_layer, tag: run_tag, kind: run_kind,
                                           n: run_n, k: run_k, beats: run_beats};
  end

  run_t ek;
  assign ek        = ent_mem[e_rpk[EA-1:0]];
  assign ent_valid = (e_rpk != e_wp);
  assign ent_layer = ek.layer;
  assign ent_tag   = ek.tag;
  assign ent_kind  = ek.kind;
  assign ent_n     = ek.n;
  assign ent_k     = ek.k;

  // --------------------------------------------------------------------------
  // the writer: frames each run's beats into slots
  // --------------------------------------------------------------------------
  logic [10:0] wr_cnt, fr_cnt;                   // slots written / freed, modulo 2,048
  wire  logic [10:0] live = wr_cnt - fr_cnt;
  wire  logic  room = (live < 11'd1024);

  logic        w_run;                            // a run is being framed
  logic        w_gemm;
  logic [6:0]  w_kch;                            // K/16
  logic [10:0] w_groups_left;                    // groups after this one (GEMM)
  logic        w_sc;                             // the next beat is the group's scale beat (GEMM)
  logic [9:0]  w_beats_left;                     // weight beats left in this group, - 1 (GEMM: 8 K/16 - 1)
  logic [23:0] w_left;                           // beats left in the run, - 1 (other kinds)
  logic        w_half;                           // a weight slot's first beat is held
  logic [511:0] w_hold;

  run_t ew;
  assign ew = ent_mem[e_rpw[EA-1:0]];

  assign in_ready = w_run && room;
  wire logic take    = in_valid && in_ready;
  // the beat that closes a slot: a scale beat, a weight slot's second beat, or any beat of another kind
  wire logic closes  = !w_gemm || w_sc || w_half;
  wire logic wr_en   = take && closes;
  wire logic [1023:0] wr_data = (w_gemm && !w_sc) ? {in_data, w_hold} : {512'b0, in_data};
  // the beat that ends the run
  wire logic run_end = take && (w_gemm ? (!w_sc && (w_beats_left == '0) && (w_groups_left == '0))
                                       : (w_left == '0));

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      w_run <= 1'b0; w_gemm <= 1'b0; w_sc <= 1'b0; w_half <= 1'b0;
      w_kch <= '0; w_groups_left <= '0; w_beats_left <= '0; w_left <= '0;
      e_rpw <= '0;
    end else begin
      if (!w_run) begin
        if (e_rpw != e_wp) begin                 // start the next run
          w_run         <= 1'b1;
          w_gemm        <= (ew.kind == 2'd0);
          w_kch         <= 7'(ew.k >> 4);
          w_groups_left <= 11'((ew.n >> 5) - 16'd1);
          w_sc          <= 1'b1;
          w_beats_left  <= 10'((ew.k >> 1) - 16'd1);      // 8 x K/16 weight beats a group
          w_left        <= ew.beats - 24'd1;
          w_half        <= 1'b0;
        end
      end else if (take) begin
        if (w_gemm) begin
          if (w_sc) begin
            w_sc <= 1'b0;
          end else begin
            w_half <= !w_half;
            if (w_beats_left == '0) begin
              w_sc          <= 1'b1;
              w_beats_left  <= 10'({w_kch, 3'b000} - 10'd1);
              w_groups_left <= w_groups_left - 11'd1;
            end else begin
              w_beats_left <= w_beats_left - 10'd1;
            end
          end
        end else begin
          w_left <= w_left - 24'd1;
        end
        if (run_end) begin
          w_run <= 1'b0;
          e_rpw <= e_rpw + 1'b1;
        end
      end
    end
  end

  always_ff @(posedge clk) if (take && w_gemm && !w_sc && !w_half) w_hold <= in_data;

  // --------------------------------------------------------------------------
  // counters
  // --------------------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      wr_cnt <= '0; fr_cnt <= '0; e_wp <= '0; e_rpk <= '0;
      err_free <= 1'b0; perf_slots <= '0; perf_full <= '0;
    end else begin
      if (wr_en)  wr_cnt <= wr_cnt + 11'd1;
      if (ring_free) begin
        fr_cnt <= fr_cnt + 11'(ring_free_n);
        if (11'(ring_free_n) > live) err_free <= 1'b1;
      end
      if (e_push) e_wp  <= e_wp + 1'b1;
      if (e_popk) e_rpk <= e_rpk + 1'b1;
      if (wr_en)  perf_slots <= perf_slots + 1;
      if (in_valid && w_run && !room) perf_full <= perf_full + 1;
    end
  end
  assign ring_wptr = wr_cnt;

  // --------------------------------------------------------------------------
  // the memory: 1,024 x 1,024 bits, one write port, one read port (2 cycles)
  // --------------------------------------------------------------------------
  wire logic [9:0] waddr = wr_cnt[9:0];

  generate
    if (USE_PRIM) begin : g_prim
      // bits 36i .. 36i + 35 in RAMB36E1 i (i = 0 .. 27): 1,008 bits; bits 1,008 .. 1,023 in the RAMB18E1
      for (genvar i = 0; i < 28; i++) begin : g_b36
        wire logic [35:0] di = wr_data[36*i +: 36];
        logic [31:0] do_;
        logic [3:0]  dop;
        RAMB36E1 #(
          .RAM_MODE("TDP"), .READ_WIDTH_A(0), .WRITE_WIDTH_A(36), .READ_WIDTH_B(36), .WRITE_WIDTH_B(0),
          .DOA_REG(0), .DOB_REG(1), .WRITE_MODE_A("WRITE_FIRST"), .WRITE_MODE_B("WRITE_FIRST"),
          .RAM_EXTENSION_A("NONE"), .RAM_EXTENSION_B("NONE"), .EN_ECC_READ("FALSE"), .EN_ECC_WRITE("FALSE"),
          .SIM_COLLISION_CHECK("NONE")
        ) u_bram (
          // port A: write
          .CLKARDCLK(clk), .ENARDEN(wr_en), .REGCEAREGCE(1'b0), .RSTRAMARSTRAM(1'b0), .RSTREGARSTREG(1'b0),
          .WEA(4'b1111), .ADDRARDADDR({1'b1, waddr, 5'b11111}),
          .DIADI(di[31:0]), .DIPADIP(di[35:32]), .DOADO(), .DOPADOP(),
          // port B: read, with the output register
          .CLKBWRCLK(clk), .ENBWREN(ring_rd), .REGCEB(1'b1), .RSTRAMB(1'b0), .RSTREGB(1'b0),
          .WEBWE(8'b0), .ADDRBWRADDR({1'b1, ring_raddr, 5'b11111}),
          .DIBDI(32'b0), .DIPBDIP(4'b0), .DOBDO(do_), .DOPBDOP(dop),
          // unused
          .CASCADEINA(1'b0), .CASCADEINB(1'b0), .CASCADEOUTA(), .CASCADEOUTB(),
          .INJECTDBITERR(1'b0), .INJECTSBITERR(1'b0), .DBITERR(), .SBITERR(), .ECCPARITY(), .RDADDRECC()
        );
        assign ring_rdata[36*i +: 36] = {dop, do_};
      end
      begin : g_b18
        logic [15:0] do_;
        RAMB18E1 #(
          .RAM_MODE("TDP"), .READ_WIDTH_A(0), .WRITE_WIDTH_A(18), .READ_WIDTH_B(18), .WRITE_WIDTH_B(0),
          .DOA_REG(0), .DOB_REG(1), .WRITE_MODE_A("WRITE_FIRST"), .WRITE_MODE_B("WRITE_FIRST"),
          .SIM_COLLISION_CHECK("NONE")
        ) u_bram (
          .CLKARDCLK(clk), .ENARDEN(wr_en), .REGCEAREGCE(1'b0), .RSTRAMARSTRAM(1'b0), .RSTREGARSTREG(1'b0),
          .WEA(2'b11), .ADDRARDADDR({waddr, 4'b1111}),
          .DIADI(wr_data[1023:1008]), .DIPADIP(2'b0), .DOADO(), .DOPADOP(),
          .CLKBWRCLK(clk), .ENBWREN(ring_rd), .REGCEB(1'b1), .RSTRAMB(1'b0), .RSTREGB(1'b0),
          .WEBWE(4'b0), .ADDRBWRADDR({ring_raddr, 4'b1111}),
          .DIBDI(16'b0), .DIPBDIP(2'b0), .DOBDO(do_), .DOPBDOP()
        );
        assign ring_rdata[1023:1008] = do_;
      end
    end else begin : g_model
      logic [1023:0] mem [1024];
      logic [1023:0] q1, q2;
      always_ff @(posedge clk) begin
        if (wr_en)   mem[waddr] <= wr_data;
        if (ring_rd) q1 <= mem[ring_raddr];      // the block RAM's read (address registered)
        q2 <= q1;                                // its output register (REGCEB = 1)
      end
      assign ring_rdata = q2;
    end
  endgenerate

endmodule

`default_nettype wire
