# ==============================================================================
# run_me_row_store.ps1 - unit tests of me_row_store (storage only) in Vivado xsim:
# compile, then run tb_me_row_store on real windows from the reference model
# (vectors/) on the instantiated block RAM (me_bram_banks), and
# once on the behavioural memories.
#
# Build output goes to a temp folder, not into the repository.
# Usage:  powershell -ExecutionPolicy Bypass -File sim\run_me_row_store.ps1
#         (optional)  -Vivado D:\Verilog\Vivado\2024.1\bin
#                     -Vec <folder>         the vector folder (default: vectors/)
#                     -RtlDir <folder>      use another copy of rtl\matrix_engine
#                     -Quick                one case only (mutation tests)
# ==============================================================================
param(
  [string]$Vivado = 'D:\Verilog\Vivado\2024.1\bin',
  [string]$Work   = (Join-Path $env:TEMP 'ac701_sim\me_row_store'),
  [string]$Vec    = '',
  [string]$RtlDir = '',
  [switch]$Quick
)
$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
if (-not $RtlDir) { $RtlDir = Join-Path $root 'rtl\matrix_engine' }
if (-not $Vec)    { $Vec = Join-Path $root 'vectors' }
New-Item -ItemType Directory -Force $Work | Out-Null
Set-Location $Work

Write-Host '== compile =='
$glbl = Join-Path $Vivado '..\data\verilog\src\glbl.v'
& "$Vivado\xvlog.bat" $glbl *> compile_glbl.out
& "$Vivado\xvlog.bat" -sv (Join-Path $RtlDir 'me_sdp_ram.sv') (Join-Path $RtlDir 'me_bram_banks.sv') (Join-Path $RtlDir 'me_row_store.sv') (Join-Path $root 'sim\unit\tb_me_row_store.sv') *> compile.out
$err = Select-String -Path compile.out -Pattern '^ERROR'
if ($err) { $err | ForEach-Object { $_.Line }; throw 'xvlog failed' }
foreach ($e in @(@{s='snap_prim'; t='tb_me_row_store'}, @{s='snap_model'; t='tb_me_row_store_model'})) {
  & "$Vivado\xelab.bat" $e.t glbl -L unisims_ver -timescale 1ns/1ps -debug off -s $e.s *> "elab_$($e.s).out"
  $err = Select-String -Path "elab_$($e.s).out" -Pattern '^ERROR'
  if ($err) { $err | ForEach-Object { $_.Line }; throw "xelab $($e.s) failed" }
}

$runs = @(
  @{snap='snap_prim';  mem='block RAM primitives'; dir='r5';  case='l0_q';        seed=7},
  @{snap='snap_prim';  mem='block RAM primitives'; dir='r5';  case='l0_down';     seed=11},
  @{snap='snap_prim';  mem='block RAM primitives'; dir='r16'; case='l15_gate_up'; seed=13},
  @{snap='snap_model'; mem='behavioural';       dir='r5';  case='l0_q';        seed=7}
)
if ($Quick) { $runs = @($runs[0]) }

Write-Host '== run =='
$rows = @(); $fail = 0
foreach ($r in $runs) {
  $d = (Join-Path $Vec $r.dir) -replace '\\', '/'
  "DIR $d", "CASE $($r.case)", "SEED $($r.seed)" | Set-Content -Encoding ascii rs_run.cfg
  $log = "run_$($r.snap)_$($r.case).out"
  & "$Vivado\xsim.bat" $r.snap -runall *> $log
  $res  = (Select-String -Path $log -Pattern 'RS RESULTS' | Select-Object -Last 1).Line
  $pass = [bool](Select-String -Path $log -Pattern 'VERDICT: PASS')
  if (-not $pass) { $fail++; Get-Content $log | Select-String -Pattern '\[FAIL\]|FATAL' | Select-Object -First 4 | ForEach-Object { $_.Line } }
  $rows += [pscustomobject]@{
    Memory = $r.mem; 'Real window (reference model)' = "$($r.dir)/$($r.case)"; Seed = $r.seed
    Result = $(if ($res) { $res -replace 'RS RESULTS: ', '' } else { 'no result' })
    Verdict = $(if ($pass) { 'PASS' } else { 'FAIL' })
  }
}
$rows | Format-Table -AutoSize | Out-String -Width 200
if ($fail -eq 0) { Write-Host 'ALL ROW-STORE TESTS PASSED' } else { Write-Host "$fail RUN(S) FAILED"; exit 1 }
