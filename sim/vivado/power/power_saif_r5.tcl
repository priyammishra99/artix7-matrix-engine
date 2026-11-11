# Power of the routed me_top (both switches on) with the GEMM's simulated activity (option 3a: RTL SAIF).
# The SAIF covers only the GEMM (layer-0 W_Q, 5 rows): from the command path's accept to its going idle.
set p D:/ac701_vivado_v2/me_top_power
open_project D:/ac701_vivado_v2/me_top/me_top.xpr
open_run impl_1
read_saif -strip_path tb_me_top_s1/dut -out_file $p/read_saif_matched.txt $p/gemm_l0_q.saif
report_power -file $p/power_saif.rpt
report_power -hierarchical_depth 2 -file $p/power_saif_hier.rpt
catch {report_switching_activity -file $p/switching_activity.rpt}
puts "POWER_DONE"
close_project
