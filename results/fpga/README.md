# AC701 board run: the matrix engine alone, 2026-10-30

The matrix engine (`rtl/matrix_engine/me_top`, both build switches on: `ME_EAST_REG`, `ME_TILE_REG`) running on
the AMD AC701 (XC7A200T-2FBG676C) at 125 MHz. The board shell (`rtl/board`) stands in for a host and memory:
- it writes the command-path register sequence;
- it streams real SmolLM2-135M weights and input windows from on-chip ROMs;
- it checks every output against the reference values, bit for bit.

## Results on the board

| Case (DIP 2:0) | Layer | Rows | Result | GEMM cycles (board = simulation) | Throughput | % of 512-MAC peak | Output beats compared / differ |
|---|---|---|---|---|---|---|---|
| 0 | 0 | 5 | PASS | 5,309 | 39.1 GMAC/s | 61.0 % | 90 / 0 |
| 1 | 0 | 8 | PASS | 5,309 | 62.5 GMAC/s | 97.6 % | 144 / 0 |
| 2 | 0 | 16 | PASS | 10,493 | 63.2 GMAC/s | 98.8 % | 288 / 0 |
| 3 | 0 | 24 | PASS | 15,677 | 63.5 GMAC/s | 99.2 % | 432 / 0 |
| 4-7 | 29 | 5 / 8 / 16 / 24 | PASS | same as 0-3 | same | same | same, 0 differ |

- **Repeat mode** (every run checked): 212,387 runs, 212,387 passed, 0 failed.
- **GEMM-only loop** (case 3): 4,104 GEMMs back to back, PASS, sustained 63.4 GMAC/s; GEMMs fill 99.9 % of the loop.
- **Board logs:** `board_runs_2026-10-30.csv`, `board_console_2026-10-30.txt`.

## Implementation (whole chip, 125 MHz)

| Metric | Value | Report |
|---|---|---|
| Routed WNS | +0.095 ns | `board_routed_timing_summary.rpt` (worst path in `me_array`: an east-half weight copy to a DSP A input) |
| Routed WHS | +0.036 ns | `board_routed_timing_summary.rpt` |
| Failing endpoints | 0 | `board_routed_timing_summary.rpt` |
| Block RAM | 289 / 365 | `board_routed_utilization*.rpt` |
| DSP | 518 / 740 | `board_routed_utilization*.rpt` |
| Slice LUTs | 16,175 | `board_routed_utilization*.rpt` |
| Flip-flops | 58,450 | `board_routed_utilization*.rpt` |
| DRC | 6 warnings, 0 errors | `board_routed_drc.rpt` |

## Power (estimate, not a board measurement)

This is the Vivado 2024.1 post-implementation power of the routed `me_top` alone. The switching activity (SAIF)
comes from a simulation of the GEMM on the real array. Vivado rates its confidence High (`power_summary_r5_r24.txt`,
`me_top_power_*.rpt`).

| | 5 rows (input gaps) | 24 rows (full speed, as on the board) |
|---|---|---|
| Total on-chip power | 1.682 W | 2.405 W |
| Energy per MAC | 55.0 pJ | 37.9 pJ |
| Efficiency | 18.2 GMAC/s/W | 26.4 GMAC/s/W |

The board shell (ROMs, checker, VIO/ILA) is test equipment and is not included.

## How to reproduce

1. **Simulation:** `sim/run_ac701_me_board.ps1`.
   - The default run uses the stand-in array and covers every case, repeat mode, a flipped-bit check and the GEMM-only loop.
   - `-RealOnly` runs one case on the real DSP array.
   - The ROMs come from `vectors/board_roms` (made by `golden/gen_board_roms.py`).
2. **Vivado project, implementation and bitstream:** `sim/vivado/make_ac701_me_board_project.tcl`.
3. **On the board:** in the Hardware Manager:
   1. program the bitstream and its `.ltx`;
   2. `source sim/hw_ac701_me_board.tcl`;
   3. `bd_all`, or `bd_gemm_only`.
4. **Power estimate:**
   1. `sim/vivado/power/saif_run_r24.tcl`, in the `tb_me_top_s1` xsim run on the real array;
   2. `sim/vivado/power/power_saif_r24.tcl`, which reads the routed `me_top` checkpoint.

   The paths in these two scripts are local; adjust them first.
