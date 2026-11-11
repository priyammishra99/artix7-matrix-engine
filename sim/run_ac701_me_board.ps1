# ==============================================================================
# run_ac701_me_board.ps1 - the simulation of the whole AC701 board design for the
# matrix-engine-only run (tb_ac701_me_board): from the board's pins (SYSCLK,
# CPU_RESET, the button, the DIP switches, the LEDs), through the MMCM, the
# shell's host control, ROMs and checker, and me_top with both switches on.
#
# Stand-in array (fast): every case, repeat mode, the check catching a flipped
# output bit, and GEMM-only mode (the power measurement's loop). With -Real: one case (default sel 0) on the real me_array
# (512 DSP48E1 models; slow, about 730 cycles a minute: about 12 minutes).
#
# ROM files: vectors/board_roms (-Roms <folder> for others); they are copied to
# the run folder. Build output goes to a temp folder, not into the repository.
# Usage:  powershell -ExecutionPolicy Bypass -File sim\run_ac701_me_board.ps1
#         (optional)  -Real  [-RealCase <0-7>]   -RealOnly
#                     -Roms <folder>  -Vivado <bin>
# ==============================================================================
param(
  [string]$Vivado = 'D:\Verilog\Vivado\2024.1\bin',
  [string]$Work   = (Join-Path $env:TEMP 'ac701_sim\ac701_me_board'),
  [string]$Roms   = '',
  [switch]$Real,
  [switch]$RealOnly,
  [int]$RealCase  = 0
)
$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
if (-not $Roms) { $Roms = Join-Path $root 'vectors\board_roms' }
$me  = Join-Path $root 'rtl\matrix_engine'
$bd  = Join-Path $root 'rtl\board'
$common = @((Join-Path $root 'rtl\common\me_cmd_pkg.sv'), (Join-Path $root 'rtl\common\me_if_pkg.sv'),
            (Join-Path $me 'me_pkg.sv'))
$blocks = @('me_sdp_ram', 'me_bram_banks', 'me_row_store', 'me_weight_ring', 'me_tile_walker', 'me_drain', 'me_s1_cmd', 'me_top') |
          ForEach-Object { Join-Path $me "$_.sv" }
$board  = @('bd_rom', 'bd_streamer', 'bd_checker', 'bd_mem_side', 'bd_ctrl', 'bd_shell', 'ac701_me_board') |
          ForEach-Object { Join-Path $bd "$_.sv" }
$tb = Join-Path $root 'sim\unit\tb_ac701_me_board.sv'
$arrays = @{
  standin = @((Join-Path $root 'sim\unit\me_array_standin.sv'))
  real    = @('me_dsp_mac', 'me_cell', 'me_skew', 'me_array') | ForEach-Object { Join-Path $me "$_.sv" }
}
$which = @('standin'); if ($Real) { $which += 'real' }; if ($RealOnly) { $which = @('real') }

$rows = @(); $fail = 0
foreach ($a in $which) {
  $w = Join-Path $Work $a
  New-Item -ItemType Directory -Force $w | Out-Null
  Set-Location $w
  Copy-Item (Join-Path $Roms '*.mem') $w -Force
  Write-Host "== $a array: compile (switches: east-half stage, tile output register) =="
  & "$Vivado\xvlog.bat" (Join-Path $Vivado '..\data\verilog\src\glbl.v') *> compile_glbl.out
  & "$Vivado\xvlog.bat" -sv -d ME_EAST_REG -d ME_TILE_REG @common @($arrays[$a]) @blocks @board $tb *> compile.out
  $err = Select-String -Path compile.out -Pattern '^ERROR'
  if ($err) { $err | ForEach-Object { $_.Line }; throw "xvlog failed ($a)" }
  & "$Vivado\xelab.bat" tb_ac701_me_board glbl -L unisims_ver -timescale 1ns/1ps -debug off -s snap *> elab.out
  $err = Select-String -Path elab.out -Pattern '^ERROR'
  if ($err) { $err | ForEach-Object { $_.Line }; throw "xelab failed ($a)" }
  if ($a -eq 'standin') { "CASES 01234567", "REPEAT 1", "INJECT 1", "GEMMONLY 1" | Set-Content -Encoding ascii bd.cfg }
  else                  { "CASES $RealCase", "REPEAT 0", "INJECT 0", "GEMMONLY 0" | Set-Content -Encoding ascii bd.cfg }
  Write-Host "== $a array: run =="
  $t0 = Get-Date
  & "$Vivado\xsim.bat" snap -runall *> run.out
  $min = [math]::Round(((Get-Date) - $t0).TotalMinutes, 1)
  $res  = (Select-String -Path run.out -Pattern 'BOARD RESULTS' | Select-Object -Last 1).Line
  $pass = [bool](Select-String -Path run.out -Pattern 'VERDICT: PASS')
  if (-not $pass) { $fail++; Get-Content run.out | Select-String -Pattern '\[FAIL\]|FATAL|Fatal|ERROR' | Select-Object -First 12 | ForEach-Object { $_.Line } }
  Get-Content run.out | Select-String -Pattern '^  ' | ForEach-Object { $_.Line }
  $rows += [pscustomobject]@{ Array = $a; Minutes = $min
    Result = $(if ($res) { $res -replace 'BOARD RESULTS: ', '' } else { 'no result' }); Verdict = $(if ($pass) { 'PASS' } else { 'FAIL' }) }
}
$rows | Format-Table -AutoSize | Out-String -Width 200
if ($fail -eq 0) { Write-Host 'BOARD SIMULATION PASSED' } else { Write-Host "$fail RUN(S) FAILED"; exit 1 }
