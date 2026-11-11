# ==============================================================================
# run_me_drain.ps1 - unit tests of me_drain (the GEMM path) in Vivado xsim:
# compile, then run tb_me_drain on the reference model's real GEMMs and the
# drain's edge cases (vectors/), with tiles back to back and with
# random gaps long enough for the drain to idle inside a command (bus-bound
# decode), on 5-row and 16-row steps, and once with the row store's reads a
# cycle slower (RD_LAT 3).
#
# Build output goes to a temp folder, not into the repository.
# Usage:  powershell -ExecutionPolicy Bypass -File sim\run_me_drain.ps1
#         (optional)  -Vivado D:\Verilog\Vivado\2024.1\bin
#                     -Vec <folder>         the vector folder (default: vectors/)
#                     -RtlDir <folder>      use another copy of rtl\matrix_engine
#                     -Quick                one run only (mutation tests)
# ==============================================================================
param(
  [string]$Vivado = 'D:\Verilog\Vivado\2024.1\bin',
  [string]$Work   = (Join-Path $env:TEMP 'ac701_sim\me_drain'),
  [string]$Vec    = '',
  [string]$RtlDir = '',
  [switch]$Quick
)
$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
if (-not $RtlDir) { $RtlDir = Join-Path $root 'rtl\matrix_engine' }
if (-not $Vec)    { $Vec = Join-Path $root 'vectors' }
$ifpkg = (Join-Path $root 'rtl\common\me_cmd_pkg.sv')
New-Item -ItemType Directory -Force $Work | Out-Null
Set-Location $Work

Write-Host '== compile =='
& "$Vivado\xvlog.bat" -sv $ifpkg (Join-Path $root 'rtl\common\me_if_pkg.sv') (Join-Path $RtlDir 'me_drain.sv') (Join-Path $root 'sim\unit\tb_me_drain.sv') *> compile.out
$err = Select-String -Path compile.out -Pattern '^ERROR'
if ($err) { $err | ForEach-Object { $_.Line }; throw 'xvlog failed' }
foreach ($e in @(@{s='snap'; t='tb_me_drain'}, @{s='snap_lat3'; t='tb_me_drain_lat3'})) {
  & "$Vivado\xelab.bat" $e.t -timescale 1ns/1ps -debug off -s $e.s *> "elab_$($e.s).out"
  $err = Select-String -Path "elab_$($e.s).out" -Pattern '^ERROR'
  if ($err) { $err | ForEach-Object { $_.Line }; throw "xelab $($e.s) failed" }
}

$runs = @(
  @{snap='snap';      dir='r5';  real='l0_q,l0_o,l0_down,l29_q,l29_down'; edge=1; gap=0; seed=1; what='back to back'},
  @{snap='snap';      dir='r5';  real='l0_q,l0_o,l0_down,l29_q,l29_down'; edge=1; gap=40; seed=2; what='random gaps 0-40 (idle mid-command)'},
  @{snap='snap';      dir='r16'; real='l0_q,l0_down,l15_gate_up';          edge=0; gap=0; seed=3; what='16 rows, back to back'},
  @{snap='snap_lat3'; dir='r5';  real='l0_o,l0_down';                      edge=1; gap=3; seed=4; what='store reads 3 cycles'}
)
if ($Quick) { $runs = @($runs[0]) }

Write-Host '== run =='
$rows = @(); $fail = 0
foreach ($r in $runs) {
  $d = (Join-Path $Vec $r.dir) -replace '\\', '/'
  "DIR $d", "SEED $($r.seed)", "GAP $($r.gap)", "REAL $($r.real)", "EDGE $($r.edge)" | Set-Content -Encoding ascii drain_run.cfg
  $log = "run_$($r.snap)_$($r.dir)_$($r.seed).out"
  & "$Vivado\xsim.bat" $r.snap -runall *> $log
  $res  = (Select-String -Path $log -Pattern 'DRAIN RESULTS' | Select-Object -Last 1).Line
  $pass = [bool](Select-String -Path $log -Pattern 'VERDICT: PASS')
  $rate = (Select-String -Path $log -Pattern 'l0_q flag-free +slot' | Select-Object -First 1).Line
  if (-not $pass) { $fail++; Get-Content $log | Select-String -Pattern '\[FAIL\]|FATAL|Fatal' | Select-Object -First 6 | ForEach-Object { $_.Line } }
  $rows += [pscustomobject]@{
    Run = "$($r.dir) $($r.what)"; Seed = $r.seed
    Result = $(if ($res) { $res -replace 'DRAIN RESULTS: ', '' } else { 'no result' })
    Verdict = $(if ($pass) { 'PASS' } else { 'FAIL' })
  }
}
$rows | Format-Table -AutoSize | Out-String -Width 200
if ($fail -eq 0) { Write-Host 'ALL DRAIN TESTS PASSED' } else { Write-Host "$fail RUN(S) FAILED"; exit 1 }
