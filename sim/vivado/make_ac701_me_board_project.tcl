# One Vivado project for the AC701 matrix-engine-only board run (Stage 1, no DDR): the whole chip, not out of
# context. Board top ac701_me_board (rtl/board) around me_top with BOTH build switches on (ME_EAST_REG
# ME_TILE_REG), the board pins (rtl/board/ac701_me_board.xdc), the VIO and ILA IPs (BD_DEBUG_IP), the ROM files
# from golden/gen_board_roms.py; synthesis, implementation at 125 MHz, and, only if timing is met, the
# bitstream and its debug-probe file. Run Simulation: the board testbench on the real array, case 0 (about 8 minutes).
# Sources are referenced in place in this repository: nothing in them is copied or edited. The ROM files (generated outside Vivado) are copied into the project.
set root  [file normalize [file join [file dirname [info script]] .. ..]]
set proj  [expr {[info exists ::env(ME_BOARD_PROJ)] ? $::env(ME_BOARD_PROJ) : [file normalize $root/../ac701_me_board_proj]}]
set roms  $root/vectors/board_roms
set defs  {ME_EAST_REG ME_TILE_REG}

create_project ac701_me_board $proj -part xc7a200tfbg676-2
set_property target_language Verilog [current_project]
set_property simulator_language Mixed [current_project]
file mkdir $proj/data
foreach f [glob $roms/bd_*.mem $roms/bd_cases.txt] { file copy -force $f $proj/data/ }
file copy -force $root/sim/vivado/sim_setup_board.tcl $proj/sim_setup_board.tcl

# ---- design sources ----
set me $root/rtl/matrix_engine
set bd $root/rtl/board
add_files -norecurse -fileset sources_1 [list \
  $root/rtl/common/me_cmd_pkg.sv $root/rtl/common/me_if_pkg.sv \
  $me/me_pkg.sv $me/me_dsp_mac.sv $me/me_cell.sv $me/me_skew.sv $me/me_array.sv \
  $me/me_sdp_ram.sv $me/me_bram_banks.sv $me/me_row_store.sv $me/me_weight_ring.sv $me/me_tile_walker.sv \
  $me/me_drain.sv $me/me_s1_cmd.sv $me/me_top.sv \
  $bd/bd_rom.sv $bd/bd_streamer.sv $bd/bd_checker.sv $bd/bd_mem_side.sv $bd/bd_ctrl.sv $bd/bd_shell.sv \
  $bd/ac701_me_board.sv]
set_property file_type SystemVerilog [get_files -of_objects [get_filesets sources_1] *.sv]
add_files -norecurse -fileset sources_1 [glob $proj/data/bd_*.mem]
set_property file_type {Memory Initialization Files} [get_files -of_objects [get_filesets sources_1] *.mem]
set_property top ac701_me_board [get_filesets sources_1]
set_property verilog_define [concat $defs BD_DEBUG_IP] [get_filesets sources_1]

# ---- the board's pins and clock ----
add_files -norecurse -fileset constrs_1 $bd/ac701_me_board.xdc

# ---- VIO: 17 values in, 6 controls out (the widths ac701_me_board.sv lists) ----
set vlnv_vio [lindex [get_ipdefs -all xilinx.com:ip:vio:*] end]
set vlnv_ila [lindex [get_ipdefs -all xilinx.com:ip:ila:*] end]
create_ip -vlnv $vlnv_vio -module_name bd_vio
set win {16 32 32 32 32 32 32 32 32 32 32 32 256 256 32 32 32}
set cfg [list CONFIG.C_NUM_PROBE_IN {17} CONFIG.C_NUM_PROBE_OUT {6} CONFIG.C_EN_PROBE_IN_ACTIVITY {0}]
for {set i 0} {$i < 17} {incr i} { lappend cfg CONFIG.C_PROBE_IN${i}_WIDTH [lindex $win $i] }
foreach {i w} {0 1 1 1 2 3 3 1 4 10 5 1} { lappend cfg CONFIG.C_PROBE_OUT${i}_WIDTH $w }
set_property -dict $cfg [get_ips bd_vio]
# ---- ILA: bd_shell's dbg, 128 bits, 1,024 samples ----
create_ip -vlnv $vlnv_ila -module_name bd_ila
set_property -dict [list CONFIG.C_NUM_OF_PROBES {1} CONFIG.C_PROBE0_WIDTH {128} CONFIG.C_DATA_DEPTH {1024} \
  CONFIG.C_TRIGIN_EN {false} CONFIG.C_INPUT_PIPE_STAGES {1}] [get_ips bd_ila]
generate_target all [get_ips {bd_vio bd_ila}]
puts "IP: $vlnv_vio, $vlnv_ila"

# the laptop's cores for every step (set inside each run's own Vivado process)
set pre $proj/threads_pre.tcl
set fh [open $pre w]; puts $fh "set_param general.maxThreads 8"; close $fh
foreach {run step} {synth_1 SYNTH_DESIGN impl_1 OPT_DESIGN impl_1 PLACE_DESIGN impl_1 PHYS_OPT_DESIGN impl_1 ROUTE_DESIGN} {
  set_property -name "STEPS.${step}.TCL.PRE" -value $pre -objects [get_runs $run]
}
set_property STEPS.PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]

# ---- simulation: the board testbench on the real array, case 0 (sim_setup_board.tcl writes its bd.cfg) ----
add_files -norecurse -fileset sim_1 [list $root/sim/unit/tb_ac701_me_board.sv]
set_property file_type SystemVerilog [get_files -of_objects [get_filesets sim_1] *.sv]
set_property verilog_define $defs [get_filesets sim_1]
set_property top tb_ac701_me_board [get_filesets sim_1]
set_property top_lib xil_defaultlib [get_filesets sim_1]
set_property -name {xsim.simulate.runtime}             -value {all} -objects [get_filesets sim_1]
set_property -name {xsim.simulate.custom_tcl}          -value $proj/sim_setup_board.tcl -objects [get_filesets sim_1]
set_property -name {xsim.elaborate.debug_level}        -value {typical} -objects [get_filesets sim_1]
set_property -name {xsim.elaborate.xelab.more_options} -value {-timescale 1ns/1ps} -objects [get_filesets sim_1]

proc util_lines {rpt} {
  set fh [open $rpt r]; set t [read $fh]; close $fh
  foreach l [split $t "\n"] {
    if {[regexp {^\| (Slice LUTs|Slice Registers|Block RAM Tile|DSPs|Bonded IOB|MMCME2_ADV|BUFGCTRL) } $l]} { puts "CHECK util $l" }
  }
}

# ---- synthesis ----
launch_runs synth_1 -jobs 4
wait_on_run synth_1
puts "SYNTH_STATUS [get_property STATUS [get_runs synth_1]]"
open_run synth_1
file mkdir $proj/reports
report_utilization    -file $proj/reports/synth_utilization.rpt
report_timing_summary -max_paths 10 -file $proj/reports/synth_timing.rpt
util_lines $proj/reports/synth_utilization.rpt
puts "CHECK synthesis WNS: [get_property SLACK [get_timing_paths -max_paths 1 -setup]]"
close_design

# ---- implementation (place and route) ----
launch_runs impl_1 -jobs 1
wait_on_run impl_1
puts "IMPL_STATUS [get_property STATUS [get_runs impl_1]]"
open_run impl_1
report_utilization    -file $proj/reports/routed_utilization.rpt
report_utilization    -hierarchical -hierarchical_depth 3 -file $proj/reports/routed_utilization_hier.rpt
report_timing_summary -max_paths 20 -file $proj/reports/routed_timing.rpt
report_drc            -file $proj/reports/routed_drc.rpt
report_clocks         -file $proj/reports/clocks.rpt
report_power          -file $proj/reports/routed_power_vectorless.rpt
util_lines $proj/reports/routed_utilization.rpt
set wns [get_property SLACK [get_timing_paths -max_paths 1 -setup]]
set whs [get_property SLACK [get_timing_paths -max_paths 1 -hold]]
set nfail [llength [get_timing_paths -max_paths 100000 -nworst 1 -slack_lesser_than 0 -setup]]
puts "CHECK routed WNS: $wns  WHS: $whs  failing endpoints: $nfail"
set p [get_timing_paths -max_paths 1 -setup]
puts "CHECK worst path: [get_property STARTPOINT_PIN $p] -> [get_property ENDPOINT_PIN $p] ([get_property GROUP $p])"
foreach c [get_clocks] { puts "CHECK clock [get_property NAME $c] period [get_property PERIOD $c] ns" }
set b36 [llength [get_cells -hierarchical -filter {REF_NAME == RAMB36E1}]]
set b18 [llength [get_cells -hierarchical -filter {REF_NAME == RAMB18E1}]]
puts "CHECK DSP48E1: [llength [get_cells -hierarchical -filter {REF_NAME == DSP48E1}]]  block RAM: $b36 RAMB36E1 + $b18 RAMB18E1 = [expr {$b36 + 0.5 * $b18}] BRAM36"
foreach sev {ERROR {CRITICAL WARNING}} {
  set v [get_drc_violations -quiet -filter "SEVERITY == \"$sev\""]
  puts "CHECK DRC $sev: [llength $v] [expr {[llength $v] ? [lsort -unique [get_property NAME $v]] : {}}]"
}
set met [expr {$wns >= 0 && $whs >= 0 && $nfail == 0}]
close_design

# ---- the bitstream and the debug probes, only with timing met ----
if {$met} {
  launch_runs impl_1 -to_step write_bitstream -jobs 1
  wait_on_run impl_1
  file mkdir $proj/bit
  foreach f [glob -nocomplain $proj/ac701_me_board.runs/impl_1/*.bit $proj/ac701_me_board.runs/impl_1/*.ltx] {
    file copy -force $f $proj/bit/
  }
  puts "CHECK bitstream: [glob -nocomplain -tails -directory $proj/bit *]"
} else {
  puts "CHECK bitstream: NOT written (timing not met)"
}

launch_simulation -simset sim_1 -mode behavioral -scripts_only
close_project
puts "PROJECT_READY $proj/ac701_me_board.xpr"
