# xsim set-up for ac701_me_board's "Run Simulation": the board testbench (tb_ac701_me_board) from the board's pins,
# on the REAL me_array, case 0 (layer-0 W_Q, 5 rows) once: about 8 minutes. Writes the testbench's bd.cfg, makes
# sure the ROM files are in the run folder, adds the key signals under headings and runs to the end; the console
# ends with ">> VERDICT: PASS <<".
set fh [open bd.cfg w]; puts $fh "CASES 0"; puts $fh "REPEAT 0"; puts $fh "INJECT 0"; puts $fh "GEMMONLY 0"; close $fh
foreach f [glob -nocomplain [file dirname [info script]]/data/bd_*.mem] { file copy -force $f . }
proc sect {name sigs} {
  add_wave_divider $name
  foreach s $sigs { if {[catch {add_wave $s} e]} { puts "wave $s: $e" } }
}
set t /tb_ac701_me_board
set b $t/dut
set s $b/u_shell
sect "1 board pins" [list $t/sys_p $t/cpu_reset $t/btn $t/btn_n $t/dip $t/led]
sect "2 clock and reset" [list $b/locked $b/clk $b/rst_n $b/start $b/sel $b/rep $b/gm]
sect "3 controller" [list $s/u_ctrl/st $s/u_ctrl/pc $s/core_rst_n $s/csr_we $s/csr_waddr $s/csr_wdata $s/csr_raddr $s/csr_rdata]
sect "4 requests and streams" [list $s/rq_valid $s/rq_ready $s/wt_valid $s/wt_ready $s/run_valid $s/run_ready \
                                    $s/w_valid $s/w_ready $s/ld_valid $s/ld_ready $s/wq_valid $s/wq_ready $s/wq_done \
                                    $s/dp_valid $s/dp_ready]
sect "5 checker" [list $s/u_mem/u_chk/bi $s/u_mem/u_chk/rel $s/u_mem/u_chk/k $s/chk_rx $s/chk_checked $s/chk_mism]
sect "6 results" [list $s/running $s/done_any $s/last_pass $s/fail_any $s/fail_code $s/gemm_cycles $s/run_cycles \
                       $s/gemm_count $s/arr_beats]
catch {set_property radix unsigned [get_waves -r {*pc *bi *k *chk_* *_cycles *gemm_count *arr_beats}]}
run all
