# ==============================================================================
# run_me_top_s1.ps1 - the Stage-1 simulation: one real SmolLM2 GEMM through
# me_top (tb_me_top_s1), on the behavioural stand-in array (fast: wiring,
# handshakes, the full data path) and, with -Real, on the real me_array (512
# DSP48E1 models; slow, about 730 cycles a minute).
#
# Vectors: vectors/ (r5 = 5 rows, r16 = 16 rows).
# Build output goes to a temp folder, not into the repository.
# Usage:  powershell -ExecutionPolicy Bypass -File sim\run_me_top_s1.ps1
#         (optional)  -Real                 also the run on the real array
#                     -RealOnly             only the run on the real array
#                     -Vec <folder>  -Vivado <bin>
#                     -NRows <n>            the row set: vectors from <Vec>\r<n> (default 5)
#                     -EastReg, -TileReg    build switches (me_if_pkg v3): the array's
#                                           east-half stage, its tile output register
# ==============================================================================
param(
  [string]$Vivado = 'D:\Verilog\Vivado\2024.1\bin',
  [string]$Work   = (Join-Path $env:TEMP 'ac701_sim\me_top_s1'),
  [string]$Vec    = '',
  [int]$NRows     = 5,
  [switch]$Real,
  [switch]$RealOnly,
  [switch]$EastReg,
  [switch]$TileReg
)
$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
if (-not $Vec) { $Vec = Join-Path $root 'vectors' }
$me  = Join-Path $root 'rtl\matrix_engine'
$common = @((Join-Path $root 'rtl\common\me_cmd_pkg.sv'), (Join-Path $root 'rtl\common\me_if_pkg.sv'),
            (Join-Path $me 'me_pkg.sv'), (Join-Path $root 'rtl\common\me_if_checker.sv'))
$blocks = @('me_sdp_ram', 'me_bram_banks', 'me_row_store', 'me_weight_ring', 'me_tile_walker', 'me_drain', 'me_s1_cmd', 'me_top') |
          ForEach-Object { Join-Path $me "$_.sv" }
$tb = Join-Path $root 'sim\unit\tb_me_top_s1.sv'
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
  Write-Host "== $a array: compile =="
  & "$Vivado\xvlog.bat" (Join-Path $Vivado '..\data\verilog\src\glbl.v') *> compile_glbl.out
  $defs = @(); if ($EastReg) { $defs += '-d', 'ME_EAST_REG' }; if ($TileReg) { $defs += '-d', 'ME_TILE_REG' }
  Write-Host "switches: east-half stage $([bool]$EastReg), tile output register $([bool]$TileReg)"
  & "$Vivado\xvlog.bat" -sv @defs @common @($arrays[$a]) @blocks $tb *> compile.out
  $err = Select-String -Path compile.out -Pattern '^ERROR'
  if ($err) { $err | ForEach-Object { $_.Line }; throw "xvlog failed ($a)" }
  & "$Vivado\xelab.bat" tb_me_top_s1 glbl -L unisims_ver -timescale 1ns/1ps -debug off -s snap *> elab.out
  $err = Select-String -Path elab.out -Pattern '^ERROR'
  if ($err) { $err | ForEach-Object { $_.Line }; throw "xelab failed ($a)" }
  $runs = if ($a -eq 'standin') { @(@{rate=0; seed=1}, @{rate=1; seed=2}) } else { @(@{rate=1; seed=3}) }
  foreach ($r in $runs) {
    $vdir = (Join-Path $Vec "r$NRows").Replace('\', '/')
    "DIR $vdir", "CASE l0_q", "SEED $($r.seed)", "RATE $($r.rate)" | Set-Content -Encoding ascii s1top.cfg
    Write-Host "== $a array: run (rate $($r.rate)) =="
    $t0 = Get-Date
    $log = "run_rate$($r.rate).out"
    & "$Vivado\xsim.bat" snap -runall *> $log
    $min = [math]::Round(((Get-Date) - $t0).TotalMinutes, 1)
    $res  = (Select-String -Path $log -Pattern 'S1TOP RESULTS' | Select-Object -Last 1).Line
    $pass = [bool](Select-String -Path $log -Pattern 'VERDICT: PASS')
    if (-not $pass) { $fail++; Get-Content $log | Select-String -Pattern '\[FAIL\]|FATAL|Fatal|ERROR' | Select-Object -First 8 | ForEach-Object { $_.Line } }
    Get-Content $log | Select-String -Pattern '\w+ +done in|compare:' | ForEach-Object { $_.Line }
    $rows += [pscustomobject]@{ Array = $a; Rate = $r.rate; Minutes = $min
      Result = $(if ($res) { $res -replace 'S1TOP RESULTS: ', '' } else { 'no result' }); Verdict = $(if ($pass) { 'PASS' } else { 'FAIL' }) }
  }
}
$rows | Format-Table -AutoSize | Out-String -Width 200
if ($fail -eq 0) { Write-Host 'STAGE-1 SIMULATION PASSED' } else { Write-Host "$fail RUN(S) FAILED"; exit 1 }
