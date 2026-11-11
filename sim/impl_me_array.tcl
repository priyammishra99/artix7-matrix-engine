# ==============================================================================
# impl_me_array.tcl - out-of-context synthesis AND implementation (place and
# route) of me_array for the AC701 part, to measure post-route timing.
#
# Usage:  vivado -mode batch -source sim/impl_me_array.tcl -tclargs <outdir> [period_ns]
# Reports in <outdir>: util_routed.rpt, timing_routed.rpt, and the critical
# paths; prints the WNS / WHS and the DSP checks. 125 MHz (8.000 ns) unless a
# period is given.
# ==============================================================================
set outdir [lindex $argv 0]
set period [expr {[llength $argv] > 1 ? [lindex $argv 1] : 8.000}]
set root   [file normalize [file join [file dirname [info script]] ..]]
file mkdir $outdir

read_verilog -sv [list \
  $root/rtl/common/me_if_pkg.sv \
  $root/rtl/matrix_engine/me_pkg.sv \
  $root/rtl/matrix_engine/me_skew.sv \
  $root/rtl/matrix_engine/me_dsp_mac.sv \
  $root/rtl/matrix_engine/me_cell.sv \
  $root/rtl/matrix_engine/me_array.sv]

set xdc $outdir/clk.xdc
set fh [open $xdc w]
puts $fh "create_clock -name clk -period $period \[get_ports clk\]"
close $fh
read_xdc -mode out_of_context $xdc

# build switches (me_if_pkg v3), from the environment: ME_DEFINES="ME_EAST_REG ME_TILE_REG"
set defs [expr {[info exists ::env(ME_DEFINES)] ? $::env(ME_DEFINES) : ""}]
puts "CHECK switches: [expr {$defs eq "" ? "none" : $defs}]"
synth_design -top me_array -part xc7a200tfbg676-2 -mode out_of_context {*}[expr {$defs eq "" ? {} : [list -verilog_define $defs]}]
opt_design
place_design
phys_opt_design
route_design

report_utilization    -file $outdir/util_routed.rpt
report_timing_summary -max_paths 10 -file $outdir/timing_routed.rpt
report_timing -max_paths 20 -nworst 1 -sort_by group -file $outdir/paths_routed.rpt

set wns [get_property SLACK [get_timing_paths -max_paths 1 -setup]]
set whs [get_property SLACK [get_timing_paths -max_paths 1 -hold]]
puts "CHECK WNS (setup): $wns ns"
puts "CHECK WHS (hold):  $whs ns"
set p [get_timing_paths -max_paths 1 -setup]
puts "CHECK worst path: [get_property STARTPOINT_PIN $p] -> [get_property ENDPOINT_PIN $p]"
puts "CHECK DSP48E1 cells: [llength [get_cells -hierarchical -filter {REF_NAME == DSP48E1}]]"
puts "IMPL_DONE"
