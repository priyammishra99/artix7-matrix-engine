# ==============================================================================
# synth_me_tile_walker.tcl - out-of-context synthesis of me_tile_walker for the
# AC701 part (XC7A200T-2FBG676C).
#
# Usage:  vivado -mode batch -source sim/synth_me_tile_walker.tcl -tclargs <outdir> [period_ns]
# Reports: util.rpt, timing.rpt in <outdir>. 125 MHz (8.000 ns) unless a period is given.
# ==============================================================================
set outdir [lindex $argv 0]
set period [expr {[llength $argv] > 1 ? [lindex $argv 1] : 8.000}]
set root   [file normalize [file join [file dirname [info script]] ..]]
file mkdir $outdir

read_verilog -sv [list \
  $root/rtl/common/me_if_pkg.sv \
  $root/rtl/matrix_engine/me_tile_walker.sv]

synth_design -top me_tile_walker -part xc7a200tfbg676-2 -mode out_of_context
create_clock -name clk -period $period [get_ports clk]

report_utilization    -file $outdir/util.rpt
report_timing_summary -max_paths 5 -file $outdir/timing.rpt
puts "CHECK WNS: [get_property SLACK [get_timing_paths -max_paths 1 -setup]]"
puts "SYNTH_DONE"
