# ==============================================================================
# synth_me_array.tcl - out-of-context synthesis of me_array for the AC701 part
#
# Usage:  vivado -mode batch -source sim/synth_me_array.tcl -tclargs <outdir> [period_ns]
# Reports: util.rpt, timing.rpt in <outdir>; the DSP / cascade check is printed.
# The target clock is 125 MHz (8.000 ns) unless a period is given.
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

synth_design -top me_array -part xc7a200tfbg676-2 -mode out_of_context
create_clock -name clk -period $period [get_ports clk]

report_utilization -file $outdir/util.rpt
report_utilization -hierarchical -file $outdir/util_hier.rpt
report_timing_summary -max_paths 5 -file $outdir/timing.rpt

# ---- DSP mapping and cascade check ----
set dsps [get_cells -hierarchical -filter {REF_NAME == DSP48E1}]
set casc 0
set cfg  [dict create]
foreach c $dsps {
  set pin [get_pins -quiet -of_objects $c -filter {REF_PIN_NAME == "PCIN[0]"}]
  if {$pin ne ""} {
    set net [get_nets -quiet -of_objects $pin]
    if {$net ne ""} {
      foreach dp [get_pins -quiet -leaf -of_objects $net -filter {DIRECTION == OUT}] {
        if {[string match "*PCOUT*" [get_property REF_PIN_NAME $dp]]} { incr casc }
      }
    }
  }
  set key "AREG=[get_property AREG $c] BREG=[get_property BREG $c] DREG=[get_property DREG $c] ADREG=[get_property ADREG $c] MREG=[get_property MREG $c] PREG=[get_property PREG $c] USE_DPORT=[get_property USE_DPORT $c]"
  dict incr cfg $key
}
puts "CHECK DSP48E1 cells: [llength $dsps]"
puts "CHECK DSPs whose PCIN is driven by another DSP's PCOUT: $casc"
dict for {k v} $cfg { puts "CHECK DSP config $k : $v cells" }
puts "SYNTH_DONE"
