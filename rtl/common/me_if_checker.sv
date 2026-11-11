// ============================================================================
// File: me_if_checker.sv
// Block: Matrix-engine interface contract - protocol checker (simulation only)
//
// Watches the beat port, the tile port and the bypass port of the matrix array
// and reports any break of rules P1-P10 of me_if_pkg.sv (contract v2). It
// drives nothing: connect it to the same signals as the array in any
// testbench (the array's own, the tile walker's, the drain's, the attention
// core's, and the integration benches).
//
//   me_if_checker u_chk (
//     .clk, .rst_n, .push,
//     .op_valid, .op_first, .op_last, .op_bank, .op_bypass,
//     .tile_valid, .tile_ready, .tile_acc, .byp_valid);
//
// push = 1 while the array runs attention / scan passes in push mode (P10):
// the consumer must then hold tile_ready = 1, and the credit rule P4 is not
// applied. push = 0 for weight GEMMs (handshake mode). A producer that never
// sends bypass beats ties op_bypass to 0; byp_valid then must stay 0.
// Set CHECK_LAT = 0 when the array model under test is not cycle-exact.
//
// Every break prints one "[ME_IF] ..." line; err_count holds the total, and a
// summary "ME_IF CHECKER: n tiles, b bypass beats, e protocol errors" is
// printed at $finish. Not for synthesis: the whole body is in translate_off.
// ============================================================================

`timescale 1ns / 1ps
`default_nettype none

module me_if_checker
  import me_if_pkg::*;
#(
  parameter int TILE_W    = MEI_NACC * MEI_ACC_W,
  parameter bit CHECK_LAT = 1'b1,
  parameter int MAX_MSG   = 20            // print at most this many errors
) (
  input wire logic              clk,
  input wire logic              rst_n,
  input wire logic              push,     // 1: push mode (P10)
  // beat port (tile walker / attention issuer -> array)
  input wire logic              op_valid,
  input wire logic              op_first,
  input wire logic              op_last,
  input wire logic              op_bank,
  input wire logic              op_bypass,
  // tile port (array -> drain / attention core)
  input wire logic              tile_valid,
  input wire logic              tile_ready,
  input wire logic [TILE_W-1:0] tile_acc,
  // bypass port (array -> attention core)
  input wire logic              byp_valid
);

// synthesis translate_off

  int unsigned err_count = 0;
  int unsigned tiles_out = 0;
  int unsigned byp_out   = 0;

  int unsigned cyc       = 0;
  bit          in_tile   = 1'b0;     // between op_first and op_last
  bit          cur_bank  = 1'b0;     // bank of the open tile
  bit          next_bank = 1'b0;     // bank the next tile must use (P3)
  int          in_flight = 0;        // P4
  bit          prev_hold = 1'b0;     // last cycle: tile_valid && !tile_ready
  logic [TILE_W-1:0] prev_acc;

  // P7: edge numbers at which each tile's last beat was taken, in order
  int unsigned last_q[$];
  bit          offer_seen = 1'b0;    // the current head tile has been offered
  // P9: edge numbers at which each bypass beat was taken, in order
  int unsigned byp_q[$];

  task automatic report(string msg);
    err_count++;
    if (err_count <= MAX_MSG) $display("[ME_IF] t=%0t edge %0d: %s", $time, cyc, msg);
    if (err_count == MAX_MSG) $display("[ME_IF] (further errors not printed)");
  endtask

  always @(posedge clk) begin
    cyc <= cyc + 1;

    if (!rst_n) begin
      in_tile    = 1'b0;
      next_bank  = 1'b0;
      in_flight  = 0;
      prev_hold  = 1'b0;
      offer_seen = 1'b0;
      last_q.delete();
      byp_q.delete();
    end else begin
      // ---------------- control signals must be known ----------------
      if ($isunknown({op_valid, tile_valid, tile_ready, byp_valid, push}))
        report("op_valid, tile_valid, tile_ready, byp_valid or push is X/Z");

      // ---------------- beat port: P1, P2, P3, P4, P9 ----------------
      if (op_valid) begin
        if ($isunknown({op_first, op_last, op_bank, op_bypass}))
          report("op_first, op_last, op_bank or op_bypass is X/Z on a valid beat");

        if (op_bypass) begin
          if (op_first || op_last) report("P9: bypass beat with op_first or op_last set");
          if (in_tile)             report("P9: bypass beat inside an open tile");
          byp_q.push_back(cyc);
        end else begin
          if (op_first) begin
            if (in_tile)               report("P2: op_first inside an open tile");
            if (op_bank !== next_bank) report($sformatf("P3: tile started in bank %0d, expected %0d", op_bank, next_bank));
            if (!push && in_flight >= MEI_TILES_IN_FLIGHT)
              report($sformatf("P4: a tile started with %0d already in flight (credit limit %0d)",
                               in_flight, MEI_TILES_IN_FLIGHT));
            in_tile   = 1'b1;
            cur_bank  = op_bank;
            next_bank = ~op_bank;
            in_flight++;
          end else if (!in_tile) begin
            report("P2: beat outside a tile (no op_first before it)");
          end

          if (in_tile && op_bank !== cur_bank) report("P3: op_bank changed inside a tile");

          if (op_last) begin
            if (in_tile) last_q.push_back(cyc);
            in_tile = 1'b0;
          end
        end
      end

      // ---------------- push mode: P10 ----------------
      if (push && tile_valid && !tile_ready)
        report("P10: tile offered in push mode but tile_ready is 0");

      // ---------------- tile port: P5 ----------------
      if (prev_hold) begin
        if (!tile_valid)                report("P5: tile_valid dropped before tile_ready");
        else if (tile_acc !== prev_acc) report("P5: tile_acc changed while waiting for tile_ready");
      end

      // ---------------- latency: P7 (first offer of each tile) ----------------
      if (tile_valid && !offer_seen) begin
        offer_seen = 1'b1;
        if (last_q.size() == 0) begin
          report("P6: tile offered before any tile's last beat");
        end else if (CHECK_LAT) begin
          // the offer can only be later than MEI_TILE_LAT when the bank was busy
          if (cyc - last_q[0] < MEI_TILE_LAT)
            report($sformatf("P7: tile offered %0d edges after its last beat, contract says %0d",
                             cyc - last_q[0], MEI_TILE_LAT));
        end
      end

      // ---------------- handshake: P4, P6 bookkeeping ----------------
      if (tile_valid && tile_ready) begin
        tiles_out++;
        if (in_flight == 0) report("P6: tile handshake with no tile in flight");
        else                in_flight--;
        if (last_q.size() > 0) void'(last_q.pop_front());
        offer_seen = 1'b0;
      end

      // ---------------- bypass port: P9 ----------------
      if (byp_valid) begin
        byp_out++;
        if (byp_q.size() == 0) begin
          report("P9: byp_valid with no bypass beat outstanding");
        end else begin
          if (CHECK_LAT && cyc - byp_q[0] != MEI_BYP_LAT)
            report($sformatf("P9: bypass result %0d edges after its beat, contract says %0d",
                             cyc - byp_q[0], MEI_BYP_LAT));
          void'(byp_q.pop_front());
        end
      end
      // a bypass beat whose result is overdue was lost
      if (CHECK_LAT && byp_q.size() > 0 && cyc - byp_q[0] > MEI_BYP_LAT) begin
        report("P9: bypass beat lost (no byp_valid at its latency)");
        void'(byp_q.pop_front());
      end

      prev_hold = tile_valid && !tile_ready;
      prev_acc  = tile_acc;
    end
  end

  final begin
    $display("ME_IF CHECKER: %0d tiles, %0d bypass beats, %0d protocol errors", tiles_out, byp_out, err_count);
  end

// synthesis translate_on

endmodule

`default_nettype wire
