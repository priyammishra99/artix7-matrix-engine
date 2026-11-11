# ==============================================================================
# impl_me_tile_walker.tcl - out-of-context synthesis AND implementation (place
# and route) of me_tile_walker (OP_GEMM) for the AC701 part.
#
# Usage:  vivado -mode batch -source sim/impl_me_tile_walker.tcl -tclargs <outdir> [period_ns] [-] [io_ns]
# Reports in <outdir>: util_synth.rpt, timing_synth.rpt, util_routed.rpt,
# timing_routed.rpt, paths_routed.rpt; prints the synthesis and post-route WNS,
# TNS, WHS, the worst path and the LUT / FF counts. 125 MHz (8.000 ns) unless a
# period is given.
# ==============================================================================
set outdir [lindex $argv 0]
set period [expr {[llength $argv] > 1 ? [lindex $argv 1] : 8.000}]
set root   [file normalize [file join [file dirname [info script]] ..]]
# io_ns > 0: the neighbours (ring, row store, array, drain, command path) take io_ns of each cycle on
# their side of every port: input delay io_ns, output delay io_ns (0: the ports are not timed)
set io_ns  [expr {[llength $argv] > 3 ? [lindex $argv 3] : 0}]
file mkdir $outdir

read_verilog -sv [list \
  $root/rtl/common/me_cmd_pkg.sv \
  $root/rtl/common/me_if_pkg.sv \
  $root/rtl/matrix_engine/me_tile_walker.sv]

set xdc $outdir/clk.xdc
set fh [open $xdc w]
puts $fh "create_clock -name clk -period $period \[get_ports clk\]"
if {$io_ns > 0} {
  puts $fh "set_input_delay  -clock clk $io_ns \[get_ports -filter {DIRECTION == IN && NAME != clk} *\]"
  puts $fh "set_output_delay -clock clk $io_ns \[get_ports -filter {DIRECTION == OUT} *\]"
}
close $fh
read_xdc -mode out_of_context $xdc

synth_design -top me_tile_walker -part xc7a200tfbg676-2 -mode out_of_context
report_utilization    -file $outdir/util_synth.rpt
report_timing_summary -max_paths 5 -file $outdir/timing_synth.rpt
set s_wns [get_property SLACK [get_timing_paths -max_paths 1 -setup]]

opt_design
place_design
phys_opt_design
route_design

report_utilization    -file $outdir/util_routed.rpt
report_timing_summary -max_paths 10 -file $outdir/timing_routed.rpt
report_timing -max_paths 20 -nworst 1 -sort_by group -file $outdir/paths_routed.rpt

set p   [get_timing_paths -max_paths 1 -setup]
set wns [get_property SLACK $p]
set whs [get_property SLACK [get_timing_paths -max_paths 1 -hold]]
set fail [get_timing_paths -max_paths 100000 -nworst 1 -slack_lesser_than 0 -setup]
set tns 0.0
foreach fp $fail { set tns [expr {$tns + [get_property SLACK $fp]}] }
puts "CHECK synthesis WNS: $s_wns ns"
puts "CHECK routed WNS (setup): $wns ns"
puts "CHECK routed WHS (hold):  $whs ns"
puts "CHECK routed TNS: [format %.3f $tns] ns over [llength $fail] failing endpoints"
puts "CHECK worst path: [get_property STARTPOINT_PIN $p] -> [get_property ENDPOINT_PIN $p] ([get_property LOGIC_LEVELS $p] levels, [format %.3f [get_property DATAPATH_DELAY $p]] ns)"
puts "CHECK LUT: [llength [get_cells -hierarchical -filter {PRIMITIVE_GROUP == LUT}]]  FF: [llength [get_cells -hierarchical -filter {PRIMITIVE_GROUP == FLOP_LATCH}]]  LUTRAM/SRL: [llength [get_cells -hierarchical -filter {PRIMITIVE_GROUP == DMEM || PRIMITIVE_GROUP == SRL}]]"
if {$io_ns > 0} {
  foreach {name from to} [list "input->register" [all_inputs] [all_registers]                                "register->output" [all_registers] [all_outputs]                                "input->output (through the walker)" [all_inputs] [all_outputs]] {
    set q [get_timing_paths -quiet -setup -max_paths 1 -from $from -to $to]
    if {[llength $q]} {
      puts "CHECK $name: slack [get_property SLACK $q] ns, [get_property STARTPOINT_PIN $q] -> [get_property ENDPOINT_PIN $q] ([get_property LOGIC_LEVELS $q] levels)"
    } else { puts "CHECK $name: no path" }
  }
  report_timing -from [all_inputs] -to [all_outputs] -max_paths 10 -nworst 1 -file $outdir/paths_in_out.rpt
}
puts "IMPL_DONE"
