# ==============================================================================
# run_me_tile_walker.ps1 - tile walker (OP_GEMM) on real SmolLM2 GEMMs: compile,
# and run tb_me_tile_walker in Vivado xsim on the reference model's vectors (vectors/).
#
#   runs:   layer0   l0 q k v o gate_up down back to back, with the fault cases first
#           layer29  l29 q down
#           rows16   a 16-row step: l0 q, l0 down, l15 gate_up (4 full row groups)
#   modes:  A  one slot per cycle (array-bound), drain always ready
#           B  5 slots per 13 cycles (bus-bound), random drain, random gaps
#           L  as B with ring latency 1 and window latency 3 (the ring data delayed to match)
#           R  as B with ring latency 3 and window latency 1 (the window data delayed to match)
#           S  as B with a stalling drain (long holds: tiles wait on credits) and a late ring
#              (each tensor's run starts after its command: the walker waits for the first slot)
#           X  as B, with the core's soft reset part-way through a command (beat 7,000, tiles in
#              flight), then the whole run again: it must match exactly
#   array:  the walker's logic does not depend on the DSP models, so every run uses the
#           behavioural array except the joint run, layer0 mode A, on the real me_array
#           (512 DSP48E1 models; slow: about 75 minutes); -NoJoint skips it.
#
# Build output goes to a temp folder, not into the repository.
# Usage:  powershell -ExecutionPolicy Bypass -File sim\run_me_tile_walker.ps1
#         (optional)  -Vivado D:\Verilog\Vivado\2024.1\bin
#                     -Only layer0,rows16   run just these runs
#                     -Modes A,B            run just these modes
#                     -RtlDir <folder>      use another copy of rtl\matrix_engine
#                     -Model                every run on the behavioural array (mutation tests)
#                     -NoJoint              skip the real-array joint run
#                     -Ring                 every run through the real me_weight_ring, fed beat by
#                                           beat (modes L and R need other ring latencies: skipped)
# ==============================================================================
param(
  [string]$Vivado = 'D:\Verilog\Vivado\2024.1\bin',
  [string]$Work   = (Join-Path $env:TEMP 'ac701_sim\me_tile_walker'),
  [string[]]$Only = @(),
  [string]$Modes  = 'A,B,L,R,S,X',
  [string]$RtlDir = '',
  [string]$Vec    = '',                # vector folder (default: vectors under Work)
  [switch]$Model,                      # every run on the behavioural array (fast: mutation tests)
  [switch]$NoJoint,                     # no real-array run at all
  [switch]$Ring,                        # every run through the real me_weight_ring (behavioural array)
  [switch]$EastReg,                     # build switches (me_if_pkg v3): the array's east-half stage
  [switch]$TileReg                      # and its tile output register
)

$ErrorActionPreference = 'Stop'
$Only = @($Only | ForEach-Object { $_ -split ',' } | Where-Object { $_ })
$ModeList = @($Modes -split ',' | Where-Object { $_ })
$root = Split-Path $PSScriptRoot -Parent
if (-not $RtlDir) { $RtlDir = Join-Path $root 'rtl\matrix_engine' }
$vec = if ($Vec) { $Vec } else { Join-Path $root 'vectors' }
New-Item -ItemType Directory -Force $Work | Out-Null
Set-Location $Work

$runs = @(
  @{name='layer0';  dir='r5';   cases='l0_q,l0_k,l0_v,l0_o,l0_gate_up,l0_down'; faults=1; joint=1},
  @{name='layer29'; dir='r5';   cases='l29_q,l29_down';                         faults=0; joint=0},
  @{name='rows16';  dir='r16';  cases='l0_q,l0_down,l15_gate_up';               faults=0; joint=0}
)
$modeDefs = @(
  @{key='A'; tag='A full-rate ring, drain ready';   rate=0; drain=0; cold=0; snap='snap_model'},
  @{key='B'; tag='B bus-rate ring, random drain';   rate=1; drain=1; cold=0; snap='snap_model'},
  @{key='L'; tag='L as B, ring lat 1, window lat 3'; rate=1; drain=1; cold=0; snap='snap_lat'},
  @{key='R'; tag='R as B, ring lat 3, window lat 1'; rate=1; drain=1; cold=0; snap='snap_lat2'},
  @{key='S'; tag='S late ring, stalling drain';      rate=1; drain=2; cold=1; snap='snap_model'},
  @{key='X'; tag='X as B, soft reset at beat 7000';  rate=1; drain=1; cold=0; snap='snap_model'; reset=7000}
)

Write-Host '== compile =='
$pkg  = (Join-Path $root 'rtl\common\me_cmd_pkg.sv')
$src  = @($pkg, (Join-Path $root 'rtl\common\me_if_pkg.sv'), (Join-Path $root 'rtl\common\me_if_checker.sv'))
$src += 'me_pkg.sv', 'me_skew.sv', 'me_dsp_mac.sv', 'me_cell.sv', 'me_array.sv',
        'me_tile_walker.sv', 'me_weight_ring.sv' | ForEach-Object { Join-Path $RtlDir $_ }
$src += Join-Path $root 'sim\unit\tb_me_tile_walker.sv'
$glbl = Join-Path $Vivado '..\data\verilog\src\glbl.v'
& "$Vivado\xvlog.bat" $glbl *> compile_glbl.out
$defs = @(); if ($EastReg) { $defs += '-d', 'ME_EAST_REG' }; if ($TileReg) { $defs += '-d', 'ME_TILE_REG' }
Write-Host "switches: east-half stage $([bool]$EastReg), tile output register $([bool]$TileReg)"
& "$Vivado\xvlog.bat" -sv @defs @src *> compile.out
$err = Select-String -Path compile.out -Pattern '^ERROR'
if ($err) { $err | ForEach-Object { $_.Line }; throw 'xvlog failed' }
# the variants are wrapper tops in the testbench file (xelab.bat splits -generic_top at '=')
$elab = @(
  @{s='snap_model'; t='tb_me_tile_walker_model'},
  @{s='snap_lat';   t='tb_me_tile_walker_lat'},
  @{s='snap_lat2';  t='tb_me_tile_walker_lat2'},
  @{s='snap';       t='tb_me_tile_walker'},
  @{s='snap_ring';  t='tb_me_tile_walker_ring'}
)
if ($Model)   { $elab = @($elab | Where-Object { $_.s -eq 'snap_model' }) }
if ($Ring)    { $elab = @($elab | Where-Object { $_.s -eq 'snap_ring' }) }
if (-not $Ring) { $elab = @($elab | Where-Object { $_.s -ne 'snap_ring' }) }
if ($NoJoint) { $elab = @($elab | Where-Object { $_.s -ne 'snap' }) }
foreach ($e in $elab) {
  & "$Vivado\xelab.bat" $e.t glbl -L unisims_ver -timescale 1ns/1ps -debug off -s $e.s *> "elab_$($e.s).out"
  $err = Select-String -Path "elab_$($e.s).out" -Pattern '^ERROR'
  if ($err) { $err | ForEach-Object { $_.Line }; throw "xelab $($e.s) failed" }
}

Write-Host '== run =='
$rows = @()
$fail = 0
foreach ($r in $runs) {
  if ($Only.Count -gt 0 -and $Only -notcontains $r.name) { continue }
  foreach ($m in $modeDefs) {
    if ($ModeList -notcontains $m.key) { continue }
    if (($Model -or $Ring) -and ($m.key -eq 'L' -or $m.key -eq 'R')) { continue }
    $snap = $m.snap
    $arr  = 'stand-in'
    if ($r.joint -and $m.key -eq 'A' -and -not $Model -and -not $NoJoint -and -not $Ring) { $snap = 'snap'; $arr = 'REAL me_array' }
    if ($Ring) { $snap = 'snap_ring'; $arr = 'stand-in + REAL ring' }
    $d = (Join-Path $vec $r.dir) -replace '\\', '/'
    @("DIR $d", "CASES $($r.cases)", "FAULTS $($r.faults)", "RATE $($m.rate)", "DRAIN $($m.drain)", "COLD $($m.cold)", "RESET_AT $([int]$m.reset)", 'SEED 7') |
      Set-Content -Encoding ascii (Join-Path $Work 'tw_run.cfg')
    $log = "run_$($r.name)_$($m.key).out"
    & "$Vivado\xsim.bat" $snap -runall *> $log
    $res  = (Select-String -Path $log -Pattern 'TW RESULTS' | Select-Object -Last 1).Line
    $cyc  = (Select-String -Path $log -Pattern '^cycles' | Select-Object -Last 1).Line
    $pass = [bool](Select-String -Path $log -Pattern 'VERDICT: PASS')
    if (-not $pass) {
      $fail++
      Get-Content $log | Select-String -Pattern 'FAIL|FATAL' | Select-Object -First 6 | ForEach-Object { $_.Line }
    }
    $rows += [pscustomobject]@{
      Run = $r.name; Array = $arr; Mode = $m.tag
      Result  = $(if ($res) { $res -replace 'TW RESULTS: ', '' } else { 'no result' })
      Timing  = $(if ($cyc) { $cyc -replace '^cycles ', '' } else { '' })
      Verdict = $(if ($pass) { 'PASS' } else { 'FAIL' })
    }
  }
}
$rows | Format-Table -AutoSize | Out-String -Width 260
if ($rows.Count -eq 0) { Write-Host 'NO TEST RAN: check -Only against the run names'; exit 2 }
if ($fail -eq 0) { Write-Host 'ALL TILE-WALKER TESTS PASSED' } else { Write-Host "$fail RUN(S) FAILED"; exit 1 }
