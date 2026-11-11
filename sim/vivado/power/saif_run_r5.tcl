# SAIF of the DUT during the GEMM only: from the GEMM's accept (st_op = GEMM) to the command path going idle.
set out D:/ac701_vivado_v2/me_top_power/gemm_l0_q.saif
open_saif $out
add_condition -name gstart {/tb_me_top_s1/dut/u_cmd/st_op == 1} {stop}
run all
puts "SAIF_START [current_time]"
log_saif [get_objects -r /tb_me_top_s1/dut/*]
remove_conditions gstart
add_condition -name gstop {/tb_me_top_s1/dut/u_cmd/busy == 0} {stop}
run all
puts "SAIF_STOP [current_time]"
close_saif
remove_conditions gstop
run all
quit
