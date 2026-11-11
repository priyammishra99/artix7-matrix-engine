# ==============================================================================
# run_me_weight_ring.ps1 - unit tests of me_weight_ring in Vivado xsim: compile,
# then run tb_me_weight_ring on the block RAM primitives and on the behavioural
# memory, over the beat rate (every cycle / random), the consumer (keeps up /
# lags, so the ring fills) and a soft reset part-way.
#
# Build output goes to a temp folder, not into the repository.
# Usage:  powershell -ExecutionPolicy Bypass -File sim\run_me_weight_ring.ps1
#         (optional)  -Vivado D:\Verilog\Vivado\2024.1\bin
#                     -RtlDir <folder>      use another copy of rtl\matrix_engine
#                     -ModelOnly            the behavioural memory only (mutation tests)
#                     -Seeds 7,11           seeds (default 7)
# ==============================================================================
param(
  [string]$Vivado = 'D:\Verilog\Vivado\2024.1\bin',
  [string]$Work   = (Join-Path $env:TEMP 'ac701_sim\me_weight_ring'),
  [string]$RtlDir = '',
  [switch]$ModelOnly,
  [string]$Seeds  = '7'
)
$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
if (-not $RtlDir) { $RtlDir = Join-Path $root 'rtl\matrix_engine' }
New-Item -ItemType Directory -Force $Work | Out-Null
Set-Location $Work

Write-Host '== compile =='
$glbl = Join-Path $Vivado '..\data\verilog\src\glbl.v'
& "$Vivado\xvlog.bat" $glbl *> compile_glbl.out
& "$Vivado\xvlog.bat" -sv (Join-Path $RtlDir 'me_weight_ring.sv') (Join-Path $root 'sim\unit\tb_me_weight_ring.sv') *> compile.out
$err = Select-String -Path compile.out -Pattern '^ERROR'
if ($err) { $err | ForEach-Object { $_.Line }; throw 'xvlog failed' }
$tops = @(@{s='snap_model'; t='tb_me_weight_ring_model'; mem='behavioural'})
if (-not $ModelOnly) { $tops += @{s='snap_prim'; t='tb_me_weight_ring'; mem='RAMB36E1/RAMB18E1'} }
foreach ($e in $tops) {
  & "$Vivado\xelab.bat" $e.t glbl -L unisims_ver -timescale 1ns/1ps -debug off -s $e.s *> "elab_$($e.s).out"
  $err = Select-String -Path "elab_$($e.s).out" -Pattern '^ERROR'
  if ($err) { $err | ForEach-Object { $_.Line }; throw "xelab $($e.s) failed" }
}

Write-Host '== run =='
$rows = @(); $fail = 0
foreach ($e in $tops) {
  foreach ($seed in ($Seeds -split ',')) {
    foreach ($rate in 0, 1) { foreach ($lag in 0, 1) { foreach ($rs in 0, 1) {
      "SEED $seed", "RATE $rate", "LAG $lag", "RESET $rs" | Set-Content -Encoding ascii ring_run.cfg
      $log = "run_$($e.s)_s$($seed)_r$($rate)_l$($lag)_x$($rs).out"
      & "$Vivado\xsim.bat" $e.s -runall *> $log
      $res  = (Select-String -Path $log -Pattern 'RING RESULTS' | Select-Object -Last 1).Line
      $live = (Select-String -Path $log -Pattern '^max live' | Select-Object -Last 1).Line
      $pass = [bool](Select-String -Path $log -Pattern 'VERDICT: PASS')
      if (-not $pass) { $fail++; Get-Content $log | Select-String -Pattern '\[FAIL\]' | Select-Object -First 4 | ForEach-Object { $_.Line } }
      $rows += [pscustomobject]@{
        Memory = $e.mem; Seed = $seed; Beats = $(if ($rate) { 'random' } else { 'every cycle' })
        Consumer = $(if ($lag) { 'lags (fills)' } else { 'keeps up' }); Reset = $(if ($rs) { 'part-way' } else { '-' })
        Result = $(if ($res) { $res -replace 'RING RESULTS: ', '' } else { 'no result' })
        Live = $(if ($live) { ($live -split ',')[0] -replace 'max live ', '' } else { '' })
        Verdict = $(if ($pass) { 'PASS' } else { 'FAIL' })
      }
    } } }
  }
}
$rows | Format-Table -AutoSize | Out-String -Width 220
if ($fail -eq 0) { Write-Host 'ALL WEIGHT-RING TESTS PASSED' } else { Write-Host "$fail RUN(S) FAILED"; exit 1 }
