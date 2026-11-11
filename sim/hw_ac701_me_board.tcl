# ==============================================================================
# hw_ac701_me_board.tcl - the board day: program the AC701 and run the matrix-
# engine-only cases from Vivado's Hardware Manager Tcl console, through the VIO
# (ac701_me_board.sv's vio_i<n> / vio_o<n>), printing PASS/FAIL, the GEMM cycle
# count, the throughput, and (GEMM-only) the sustained rate; every run is also
# appended to bd_runs.csv in the current folder.
#
# In the Hardware Manager's Tcl console (board on, USB-JTAG cable in):
#   source <repo>/sim/hw_ac701_me_board.tcl
#   bd_open  <project>/bit/ac701_me_board.bit <project>/bit/ac701_me_board.ltx
#   bd_run 3                 ;# case 3: layer-0 W_Q, 24 rows (cases 0-3 layer 0 at 5/8/16/24 rows, 4-7 layer 29)
#   bd_all                   ;# all 8 cases, one run each
#   bd_gemm_only 3 1000      ;# GEMM-only: case 3, 1,000 GEMMs back to back (for the power reading), then the check
#   bd_show                  ;# the last run's results again
# The buttons and DIP switches still work; the VIO's override is set only while a bd_* command starts a run.
# ==============================================================================

set bd_period_ns 8.0                         ;# the core clock: 8.0 at 125 MHz (10.0 if built with CORE_DIV 10)
set bd_rows {5 8 16 24}
set bd_layer {0 0 0 0 29 29 29 29}
set bd_n 576
set bd_k 576

proc bd_open {bit ltx} {
  open_hw_manager
  catch {connect_hw_server}
  open_hw_target
  set dev [lindex [get_hw_devices xc7a200t*] 0]
  current_hw_device $dev
  set_property PROGRAM.FILE $bit $dev
  set_property PROBES.FILE $ltx $dev
  set_property FULL_PROBES.FILE $ltx $dev
  program_hw_devices $dev
  refresh_hw_device $dev
  puts "programmed [get_property NAME $dev]; VIO: [get_hw_vios]"
  bd_set o1 0
}

proc bd_probe {n} {
  foreach p [get_hw_probes -of_objects [get_hw_vios]] {
    if {[regexp "(^|/)vio_${n}(\\\[|$)" [get_property NAME $p]]} { return $p }
  }
  error "no VIO probe vio_$n"
}
proc bd_get {n} {
  set p [bd_probe $n]
  set_property INPUT_VALUE_RADIX UNSIGNED $p
  refresh_hw_vio [get_hw_vios]
  return [get_property INPUT_VALUE $p]
}
proc bd_set {n v} {
  set p [bd_probe $n]
  set_property OUTPUT_VALUE_RADIX UNSIGNED $p
  set_property OUTPUT_VALUE $v $p
  commit_hw_vio $p
}

proc bd_status {} {
  set s [bd_get i0]
  return [dict create fail [expr {$s & 1}] pass [expr {($s >> 1) & 1}] done [expr {($s >> 2) & 1}] \
          running [expr {($s >> 3) & 1}] rep [expr {($s >> 4) & 1}] sel [expr {($s >> 5) & 7}] code [expr {$s >> 8}]]
}

proc bd_wait_idle {{limit_s 60}} {
  set t0 [clock milliseconds]
  while {[dict get [bd_status] running]} {
    if {[clock milliseconds] - $t0 > 1000 * $limit_s} { puts "still running after $limit_s s"; return 0 }
    after 50
  }
  return 1
}

proc bd_start {sel rep gm} {
  bd_set o2 $sel; bd_set o3 $rep; bd_set o5 $gm; bd_set o1 1
  bd_set o0 1; bd_set o0 0
}

proc bd_show {} {
  global bd_period_ns bd_rows bd_layer bd_n bd_k
  set st [bd_status]
  set sel [dict get $st sel]
  set rows [lindex $bd_rows [expr {$sel & 3}]]
  set gc [bd_get i4]; set rc [bd_get i5]; set mism [bd_get i6]; set chk [bd_get i7]
  set gn [bd_get i15]; set lc [bd_get i16]; set gm [expr {[bd_get i14] & 1}]
  set macs [expr {double($rows) * $bd_n * $bd_k}]
  set t_us [expr {$gc * $bd_period_ns / 1000.0}]
  set gmacs [expr {$gc > 0 ? $macs / ($gc * $bd_period_ns) : 0}]
  set util [expr {$gc > 0 ? 100.0 * $macs / ($gc * 512.0) : 0}]
  puts [format "case %d: layer %d W_Q, %d rows  %s  code %02x  GEMM %d cycles = %.2f us  %.1f GMAC/s  (%.1f%% of the 512-MAC peak)" \
        $sel [lindex $bd_layer $sel] $rows [expr {[dict get $st pass] ? "PASS" : "FAIL"}] [dict get $st code] $gc $t_us $gmacs $util]
  puts [format "        output beats compared %d, differ %d; run %d cycles; saturations %d; array beats %d" \
        $chk $mism $rc [bd_get i10] [bd_get i11]]
  set line [list [clock format [clock seconds] -format %Y-%m-%dT%H:%M:%S] $sel [lindex $bd_layer $sel] $rows \
            [expr {[dict get $st pass] ? "PASS" : "FAIL"}] [dict get $st code] $gc $chk $mism $rc $gm $gn $lc]
  if {$gm} {
    set sus [expr {$lc > 0 ? $gn * $macs / ($lc * $bd_period_ns) : 0}]
    puts [format "        GEMM-only: %d GEMMs in %d cycles = %.3f ms, sustained %.1f GMAC/s, GEMMs %.1f%% of the loop" \
          $gn $lc [expr {$lc * $bd_period_ns / 1e6}] $sus [expr {$lc > 0 ? 100.0 * $gn * $gc / $lc : 0}]]
  }
  if {[dict get $st code] != 0} {
    puts [format "        first difference: %s  got/expected %s  fail line %s" [bd_get i8] [bd_get i9] [expr {[bd_get i14] >> 24}]]
  }
  set new [expr {![file exists bd_runs.csv]}]
  set fh [open bd_runs.csv a]
  if {$new} { puts $fh "time,case,layer,rows,result,code,gemm_cycles,beats_compared,beats_differ,run_cycles,gemm_only,gemms,loop_cycles" }
  puts $fh [join $line ,]
  close $fh
}

proc bd_run {sel} {
  bd_start $sel 0 0
  after 20
  bd_wait_idle
  bd_set o1 0
  bd_show
}

proc bd_all {} { for {set s 0} {$s < 8} {incr s} { bd_run $s } }

proc bd_gemm_only {sel gemms} {
  bd_start $sel 1 1
  while {[bd_get i15] < $gemms} { after 100 }
  bd_set o3 0
  bd_wait_idle
  bd_set o1 0
  bd_show
}
