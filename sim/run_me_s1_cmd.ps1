# ==============================================================================
# run_me_s1_cmd.ps1 - unit tests of me_s1_cmd (the Stage-1 command path) in
# Vivado xsim: compile, then run tb_me_s1_cmd (models of the walker, the drain,
# the row store and the memory side, all with random delays) with three
# seeds of 400 random operations each.
#
# Build output goes to a temp folder, not into the repository.
# Usage:  powershell -ExecutionPolicy Bypass -File sim\run_me_s1_cmd.ps1
#         (optional)  -Vivado D:\Verilog\Vivado\2024.1\bin
#                     -RtlDir <folder>      use another copy of rtl\matrix_engine
#                     -Quick                one seed only (mutation tests)
# ==============================================================================
param(
  [string]$Vivado = 'D:\Verilog\Vivado\2024.1\bin',
  [string]$Work   = (Join-Path $env:TEMP 'ac701_sim\me_s1_cmd'),
  [string]$RtlDir = '',
  [switch]$Quick
)
$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
if (-not $RtlDir) { $RtlDir = Join-Path $root 'rtl\matrix_engine' }
$ifpkg = (Join-Path $root 'rtl\common\me_cmd_pkg.sv')
New-Item -ItemType Directory -Force $Work | Out-Null
Set-Location $Work

Write-Host '== compile =='
& "$Vivado\xvlog.bat" -sv $ifpkg (Join-Path $RtlDir 'me_s1_cmd.sv') (Join-Path $root 'sim\unit\tb_me_s1_cmd.sv') *> compile.out
$err = Select-String -Path compile.out -Pattern '^ERROR'
if ($err) { $err | ForEach-Object { $_.Line }; throw 'xvlog failed' }
& "$Vivado\xelab.bat" tb_me_s1_cmd -timescale 1ns/1ps -debug off -s snap *> elab.out
$err = Select-String -Path elab.out -Pattern '^ERROR'
if ($err) { $err | ForEach-Object { $_.Line }; throw 'xelab failed' }

$seeds = @(1, 2, 3)
if ($Quick) { $seeds = @(1) }
Write-Host '== run =='
$rows = @(); $fail = 0
foreach ($s in $seeds) {
  "SEED $s", "OPS 400" | Set-Content -Encoding ascii s1_run.cfg
  $log = "run_$s.out"
  & "$Vivado\xsim.bat" snap -runall *> $log
  $res  = (Select-String -Path $log -Pattern 'S1 RESULTS' | Select-Object -Last 1).Line
  $pass = [bool](Select-String -Path $log -Pattern 'VERDICT: PASS')
  if (-not $pass) { $fail++; Get-Content $log | Select-String -Pattern '\[FAIL\]|FATAL|Fatal' | Select-Object -First 6 | ForEach-Object { $_.Line } }
  $rows += [pscustomobject]@{
    Seed = $s; Operations = 400
    Result = $(if ($res) { $res -replace 'S1 RESULTS: ', '' } else { 'no result' })
    Verdict = $(if ($pass) { 'PASS' } else { 'FAIL' })
  }
}
$rows | Format-Table -AutoSize | Out-String -Width 200
if ($fail -eq 0) { Write-Host 'ALL COMMAND-PATH TESTS PASSED' } else { Write-Host "$fail RUN(S) FAILED"; exit 1 }
