# artix7-matrix-engine

An INT8 matrix engine for transformer inference, written in SystemVerilog for the AMD Artix-7
(XC7A200T-2FBG676C). Its core is a 512-DSP48E1 array. It runs real SmolLM2-135M layer GEMMs on the
**AMD AC701 board at 125 MHz**, and every output matches a Python reference model bit for bit.

| | |
|---|---|
| Array | **4 x 8 x 16 = 512 DSP48E1**: 4 rows (tokens sharing each weight slot) x 8 output channels per cycle x 16-deep DSP cascade per cell |
| Clock | **125 MHz** on the board (routed WNS +0.095 ns, WHS +0.036 ns, whole chip) |
| Peak | 512 MAC/cycle = 64.0 GMAC/s |
| Measured on the AC701 | **63.5 GMAC/s per GEMM (99.2 % of peak)**, **63.4 GMAC/s sustained** over 4,104 back-to-back GEMMs |
| Correctness on the board | 8 / 8 cases bit-exact; **212,387 repeat runs, 0 failures** |
| Power (Vivado estimate, SAIF) | 2.405 W, **37.9 pJ/MAC**, 26.4 GMAC/s/W at full speed |
| Numerics | INT8 x INT8, 34-bit accumulators, FP16 per-channel scales, per-row activation scales, round-half-to-even, saturation |

## Architecture

```
            weight stream (64-B beats)                      window rows (INT8)
                    |                                              |
                    v                                              v
             +--------------+   slots   +-----------------+  +-----------+
             | weight ring  |---------->|   tile walker   |<-| row store |
             | 1,024 slots  |<--frees---| (GEMM loop nest)|  |  window,  |
             +--------------+           +-----------------+  |  staging, |
                                          | beats    | scales|  residual |
                                          v          |       +-----------+
                                  +----------------+ |             ^
                                  |    me_array    | |             | outputs
                                  | 4 x 8 x 16 DSP | |             |
                                  +----------------+ |       +-----------+
                                          | tiles    +------>|   drain   |
                                          +----------------->| scale,    |
                                                             | round,    |
                                                             | saturate  |
                                                             +-----------+
        me_s1_cmd: debug-register command path (load window, run GEMM, dump results)
```

| Block | File | What it does |
|---|---|---|
| Top level | `rtl/matrix_engine/me_top.sv` | The blocks wired together. One clock and one soft reset. |
| Array | `me_array.sv`, `me_cell.sv`, `me_dsp_mac.sv`, `me_skew.sv` | 4 rows x 8 output channels x 16-deep cascade = 512 DSP48E1. Each cycle, one stationary weight slot (8 channels x 16 elements) meets 4 rows of 16 activations. Each cell sums 16 products along a DSP cascade. Double-banked 34-bit channel accumulators, tile output with a valid/ready handshake. |
| Weight ring | `me_weight_ring.sv` | 1,024 slots x 1,024 bits (28.5 BRAM36). Takes the weight stream as it lies in memory, one 64-B beat a cycle. Holds run descriptors. Slots are freed by count. |
| Tile walker | `me_tile_walker.sv` | Runs one GEMM command. For each 32-channel group: the scale slot goes to the drain, then 4 tiles of 8 channels. Each tile replays the same ring slots for every 4-row group of the window. Checks the command and reports a fault code instead of running a bad one. |
| Drain | `me_drain.sv` | Tile accumulators to outputs. Per-channel FP16 scale x per-row scale (17-bit normalised constants), variable shift with round-half-to-even, saturation to INT16 / INT32, optional residual accumulate (ACC). Two lanes, one channel pair a cycle. Sticky fault codes and a saturation counter. |
| Row store | `me_row_store.sv`, `me_bram_banks.sv`, `me_sdp_ram.sv` | Up to 24 rows on chip: the INT8 input window, INT16 / INT32 staging, the INT32 residual. A window loader and a dump stream for testing. |
| Command path | `me_s1_cmd.sv` | Debug registers. Load a window, run one GEMM, dump the results, step settings and the class table. One operation at a time; bad requests are refused. |
| Packages | `rtl/common/me_if_pkg.sv`, `me_cmd_pkg.sv`, `rtl/matrix_engine/me_pkg.sv` | The array's interface contract (widths, latencies, tags), the command word, shared constants. |
| Protocol checker | `rtl/common/me_if_checker.sv` | Simulation-only assertions on every handshake around the array. |

The array also supports an attention mode, in RTL and simulation only (it was not part of the board run):
- score and P.V passes, using the DSP pre-adder;
- bypass beats and push-mode tiles;
- a page-scan mode.

Two build switches add pipeline stages for full-chip placement, and the board build uses both:
- `ME_EAST_REG`: a register stage for the long wires into the east DSP columns;
- `ME_TILE_REG`: a registered tile output.

## Results on the FPGA (AMD AC701)

Details, logs and Vivado reports: [`results/fpga/`](results/fpga/README.md).

A small board shell (`rtl/board/`) stands in for a host and memory:
- on-chip ROMs hold the SmolLM2 weights, the input windows and the expected outputs;
- a controller drives the debug registers;
- a checker compares every output beat on chip;
- results go to a VIO (Vivado Hardware Manager).

| Case (DIP 2:0) | Layer | Rows | Result | GEMM cycles | Throughput | % of peak | Output beats compared / differ |
|---|---|---|---|---|---|---|---|
| 0 | 0 | 5 | PASS | 5,309 | 39.1 GMAC/s | 61.0 % | 90 / 0 |
| 1 | 0 | 8 | PASS | 5,309 | 62.5 GMAC/s | 97.6 % | 144 / 0 |
| 2 | 0 | 16 | PASS | 10,493 | 63.2 GMAC/s | 98.8 % | 288 / 0 |
| 3 | 0 | 24 | PASS | 15,677 | 63.5 GMAC/s | 99.2 % | 432 / 0 |
| 4-7 | 29 | 5 / 8 / 16 / 24 | PASS | same as 0-3 | same | same | same, 0 differ |

These are layer-0 and layer-29 query projections (576 x 576) of SmolLM2-135M-Instruct, with its pretrained weights quantised to INT8. The GEMM cycle counts on the board equal
the simulation's exactly.

**Additional runs:**
- **Repeat mode:** 212,387 runs, each checked, 0 failures.
- **GEMM-only loop:** 4,104 GEMMs back to back, sustained 63.4 GMAC/s; GEMMs fill 99.9 % of the loop.

**Implementation:** the whole chip including the shell, at 125 MHz.

| Metric | Value |
|---|---|
| WNS | +0.095 ns |
| WHS | +0.036 ns |
| Block RAM | 289 / 365 |
| DSP | 518 / 740 |
| LUTs | 16,175 |
| FFs | 58,450 |
| DRC errors | 0 |

**Power** (Vivado 2024.1 post-implementation estimate for the routed matrix engine alone, with switching activity from
a simulation of the GEMM on the real array; not measured on the board):

| | 5 rows | 24 rows (full speed) |
|---|---|---|
| Total on-chip power | 1.682 W | 2.405 W |
| Energy per MAC | 55.0 pJ | 37.9 pJ |
| Efficiency | 18.2 GMAC/s/W | 26.4 GMAC/s/W |

## Verification

Every block has a self-checking testbench in `sim/unit/`. The expected values come from the vectors in `vectors/`:
real SmolLM2-135M GEMMs plus edge cases, pre-generated by a bit-exact Python reference model of the engine's
numerics, which is not part of this repository. `golden/check_gemm_vectors.py` re-checks the vector files
independently.

| Script | What it runs |
|---|---|
| `sim/run_me_array.ps1` | The array on GEMM, attention (score, P.V) and page-scan cases, back to back and with random bubbles and a random drain, with the protocol checker (`-EastReg` / `-TileReg` for the build switches) |
| `sim/run_me_weight_ring.ps1` | The ring on block RAM primitives and on behavioural memory: beats every cycle or random, a consumer that keeps up or lags (the ring fills), a soft reset part-way |
| `sim/run_me_row_store.ps1` | Window load and reads on real windows, on block RAM primitives and on behavioural memories |
| `sim/run_me_s1_cmd.ps1` | The command path against models of its neighbours with random delays: 3 seeds x 400 operations |
| `sim/run_me_drain.ps1` | Real GEMM outputs (5 and 16 rows), the drain's edge cases, random gaps, a slower store |
| `sim/run_me_tile_walker.ps1` | Layer 0 (Q, K, V, O, gate/up, down) and layer 29 GEMMs, 16-row steps, fault cases; six timing modes (full rate, bus rate, ring / window latencies, stalling drain, soft reset mid-command); `-Ring` through the real weight ring; one joint run on the real 512-DSP array |
| `sim/run_me_top_s1.ps1` | `me_top` end to end, as the board runs it (stand-in array; `-Real` on the 512 DSP48E1 models) |
| `sim/run_ac701_me_board.ps1` | The whole board design from its pins (clock, reset, buttons, DIP switches, LEDs): all eight cases, repeat mode, a flipped-bit check, the GEMM-only loop |

The testbenches were also checked with mutation testing: bugs were injected into a copy of the RTL (`-RtlDir`) on purpose, and
the tests had to catch them.

Out-of-context synthesis and place-and-route scripts for each block are in `sim/impl_*.tcl` / `sim/synth_*.tcl`.

## Running it

Requires Vivado 2024.1 (xsim) on Windows PowerShell; Python 3 with NumPy for the vector scripts.

```powershell
powershell -ExecutionPolicy Bypass -File sim\run_me_array.ps1
powershell -ExecutionPolicy Bypass -File sim\run_me_tile_walker.ps1 -NoJoint   # the joint real-array run takes ~75 min
powershell -ExecutionPolicy Bypass -File sim\run_ac701_me_board.ps1
```

Each script takes `-Vivado <path to Vivado bin>` if Vivado is not at the default path in the script. Build output goes to
`%TEMP%\ac701_sim\`.

**FPGA build:** `vivado -mode batch -source sim/vivado/make_ac701_me_board_project.tcl` makes the board project. It runs
synthesis and implementation at 125 MHz and writes the bitstream only if timing is met. Set `ME_BOARD_PROJ` to choose the
project folder. To run on the board, use the Hardware Manager Tcl console:
1. source `sim/hw_ac701_me_board.tcl`;
2. `bd_open <bit> <ltx>`;
3. `bd_all`, or `bd_gemm_only 3 1000`.

## Repository layout

```
rtl/matrix_engine/   the matrix engine
rtl/common/          interface packages and the protocol checker
rtl/board/           AC701 board top, pin constraints, test shell (ROMs, controller, checker)
sim/unit/            testbenches
sim/                 run scripts (xsim), out-of-context synthesis / implementation scripts
sim/vivado/          board project, power-estimation scripts, Hardware Manager script
golden/              array vector generator, board ROM generator, vector checker
vectors/             pre-generated test vectors (SmolLM2-135M, rows 5 and 16) and the board ROMs
results/fpga/        board run logs and Vivado reports
```
