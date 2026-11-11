# ==============================================================================
# run_me_array.ps1 - matrix engine array (contract v2): generate golden
# vectors, compile and run tb_me_array in Vivado xsim for every case, in two
# modes.
#
#   GEMM cases (handshake):   A back-to-back, drain always ready
#                             B random bubbles + random drain (exercises the credits)
#   attention / scan (push):  A back-to-back
#                             B random bubbles between and inside tiles and passes
#
# Build output goes to a temp folder, not into the repository.
# Usage:  powershell -ExecutionPolicy Bypass -File sim\run_me_array.ps1
#         (optional)  -Vivado D:\Verilog\Vivado\2024.1\bin
#                     -Only wq,pv           run just these cases
#                     -Modes A              run just these modes (A, B)
#                     -RtlDir <folder>      use another copy of rtl\matrix_engine
#                                           (used for the mutation tests)
#                     -EastReg, -TileReg    build switches (me_if_pkg v3): the east-half
#                                           stage, the tile output register
# ==============================================================================
param(
  [string]$Vivado = 'D:\Verilog\Vivado\2024.1\bin',
  [string]$Work   = (Join-Path $env:TEMP 'ac701_sim\me_array'),
  [string[]]$Only = @(),
  [string]$Modes  = 'A,B',
  [string]$RtlDir = '',
  [switch]$EastReg,
  [switch]$TileReg
)

$ErrorActionPreference = 'Stop'
$Only = @($Only | ForEach-Object { $_ -split ',' } | Where-Object { $_ })   # -File passes "a,b" as one string
$ModeList = @($Modes -split ',' | Where-Object { $_ })
$root = Split-Path $PSScriptRoot -Parent
if (-not $RtlDir) { $RtlDir = Join-Path $root 'rtl\matrix_engine' }
$vec  = Join-Path $Work 'vectors'
New-Item -ItemType Directory -Force $Work, $vec | Out-Null
Set-Location $Work

Write-Host '== golden vectors =='
python (Join-Path $root 'golden\gen_me_vectors.py') --out $vec
if ($LASTEXITCODE -ne 0) { throw 'vector generation failed' }

Write-Host '== compile =='
$src  = @((Join-Path $root 'rtl\common\me_if_pkg.sv'), (Join-Path $root 'rtl\common\me_if_checker.sv'))
$src += 'me_pkg.sv', 'me_skew.sv', 'me_dsp_mac.sv', 'me_cell.sv', 'me_array.sv' |
        ForEach-Object { Join-Path $RtlDir $_ }
$src += Join-Path $root 'sim\unit\tb_me_array.sv'
$glbl = Join-Path $Vivado '..\data\verilog\src\glbl.v'
& "$Vivado\xvlog.bat" $glbl *> compile_glbl.out
$defs = @(); if ($EastReg) { $defs += '-d', 'ME_EAST_REG' }; if ($TileReg) { $defs += '-d', 'ME_TILE_REG' }
Write-Host "switches: east-half stage $([bool]$EastReg), tile output register $([bool]$TileReg)"
& "$Vivado\xvlog.bat" -sv @defs @src *> compile.out
$err = Select-String -Path compile.out -Pattern '^ERROR'
if ($err) { $err | ForEach-Object { $_.Line }; throw 'xvlog failed' }
Select-String -Path compile.out -Pattern '^WARNING' | ForEach-Object { $_.Line }

& "$Vivado\xelab.bat" tb_me_array glbl -L unisims_ver -debug off -s snap *> elab.out
$err = Select-String -Path elab.out -Pattern '^ERROR'
if ($err) { $err | ForEach-Object { $_.Line }; throw 'xelab failed' }

Write-Host '== run =='
$rows = @()
$fail = 0
$v = $vec -replace '\\', '/'
foreach ($line in Get-Content (Join-Path $vec 'manifest.txt')) {
  $n, $nbeat, $ntile, $nbyp, $mode = $line -split '\s+'
  if ($Only.Count -gt 0 -and $Only -notcontains $n) { continue }
  foreach ($m in @(@{key='A'; gap=0; drain=0}, @{key='B'; gap=1; drain=1})) {
    if ($ModeList -notcontains $m.key) { continue }
    $tag = if ($mode -eq '1') { if ($m.key -eq 'A') { 'A push, back-to-back' } else { 'B push, bubbles' } }
           else               { if ($m.key -eq 'A') { 'A back-to-back' } else { 'B bubbles+slow drain' } }
    @("BEATS $v/me_${n}_beats.mem", "TILES $v/me_${n}_tiles.mem", "BYP $v/me_${n}_byp.mem",
      "NBEAT $nbeat", "NTILE $ntile", "NBYP $nbyp", "MODE $mode",
      "GAP $($m.gap)", "DRAIN $($m.drain)", "SEED 7") |
        Set-Content -Encoding ascii (Join-Path $Work 'me_run.cfg')
    $log = "run_${n}_$($m.key).out"
    & "$Vivado\xsim.bat" snap -runall *> $log
    $res  = (Select-String -Path $log -Pattern 'ME_ARRAY RESULTS' | Select-Object -Last 1).Line
    $pass = [bool](Select-String -Path $log -Pattern 'VERDICT: PASS')
    if (-not $pass) {
      $fail++
      Get-Content $log | Select-String -Pattern 'FAIL|FATAL|ME_IF\]' | Select-Object -First 4 | ForEach-Object { $_.Line }
    }
    $rows += [pscustomobject]@{
      Case = $n; Mode = $tag; Beats = $nbeat; Tiles = $ntile; Bypass = $nbyp
      Result = $(if ($res) { $res -replace 'ME_ARRAY RESULTS: ', '' } else { 'no result' })
      Verdict = $(if ($pass) { 'PASS' } else { 'FAIL' })
    }
  }
}
$rows | Format-Table -AutoSize | Out-String -Width 200
if ($rows.Count -eq 0) { Write-Host 'NO TEST RAN: check -Only against the case names'; exit 2 }
if ($fail -eq 0) { Write-Host 'ALL MATRIX-ARRAY TESTS PASSED' } else { Write-Host "$fail RUN(S) FAILED"; exit 1 }
