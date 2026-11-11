# Power of the routed me_top (both switches on) with the GEMM's simulated activity: layer-0 W_Q, 24 rows,
# full speed (RATE 0), as on the board (option 3a: RTL SAIF; the 5-row run is power_saif.tcl).
# Reads the routed checkpoint directly: the me_top project is not opened, nothing in it is written or re-run.
set p D:/ac701_vivado_v2/me_top_power
open_checkpoint D:/ac701_vivado_v2/me_top/me_top.runs/impl_1/me_top_routed.dcp
read_saif -strip_path tb_me_top_s1/dut -out_file $p/read_saif_matched_r24.txt $p/gemm_l0_q_r24.saif
report_power -file $p/power_saif_r24.rpt
report_power -hierarchical_depth 2 -file $p/power_saif_r24_hier.rpt
puts "POWER_DONE"
close_design
