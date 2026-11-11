# ==============================================================================
# ac701_me_board.xdc - AC701 pins and timing for the matrix-engine-only board run
# (ac701_me_board.sv). Pins from UG952 v1.4 and the AC701 board files (1.4), as
# checked on 2026-10-29; I/O standards as our board-proven XDCs use them.
# ==============================================================================

# ---- 200 MHz system clock (U51, 2.5 V LVDS, 100 ohm on the board): bank 34 at 1.5 V ----
# An LVDS_25 input is allowed in a bank at another VCCO with the FPGA's termination off (DIFF_TERM FALSE, the default).
set_property PACKAGE_PIN R3 [get_ports SYSCLK_P]
set_property PACKAGE_PIN P3 [get_ports SYSCLK_N]
set_property IOSTANDARD LVDS_25 [get_ports {SYSCLK_P SYSCLK_N}]
set_property DIFF_TERM FALSE [get_ports SYSCLK_P]
create_clock -name sysclk -period 5.000 [get_ports SYSCLK_P]
# the core clock (125 MHz, or 100 MHz with CORE_DIV 10) is the MMCM's CLKOUT0, derived by Vivado

# ---- CPU_RESET (SW8), active high ----
set_property PACKAGE_PIN U4 [get_ports CPU_RESET]
set_property IOSTANDARD LVCMOS15 [get_ports CPU_RESET]

# ---- push buttons: C (SW6) starts a run, N (SW3) starts GEMM-only ----
set_property PACKAGE_PIN U6 [get_ports GPIO_SW_C]
set_property PACKAGE_PIN P6 [get_ports GPIO_SW_N]
set_property IOSTANDARD LVCMOS15 [get_ports {GPIO_SW_C GPIO_SW_N}]

# ---- DIP switches (SW2): 0-2 the case, 3 repeat ----
set_property PACKAGE_PIN R8 [get_ports {GPIO_DIP_SW[0]}]
set_property PACKAGE_PIN P8 [get_ports {GPIO_DIP_SW[1]}]
set_property PACKAGE_PIN R7 [get_ports {GPIO_DIP_SW[2]}]
set_property PACKAGE_PIN R6 [get_ports {GPIO_DIP_SW[3]}]
set_property IOSTANDARD LVCMOS15 [get_ports {GPIO_DIP_SW[*]}]

# ---- user LEDs (DS2-DS5): 0 running / idle blink, 1 done, 2 PASS, 3 FAIL ----
set_property PACKAGE_PIN M26 [get_ports {GPIO_LED[0]}]
set_property PACKAGE_PIN T24 [get_ports {GPIO_LED[1]}]
set_property PACKAGE_PIN T25 [get_ports {GPIO_LED[2]}]
set_property PACKAGE_PIN R26 [get_ports {GPIO_LED[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {GPIO_LED[*]}]

# USB-UART (T19 RX, U19 TX, V19, W19; LVCMOS18): reserved for a later UART, not used here.

# ---- the buttons, switches and LEDs are slow and synchronised in the design: not timed ----
set_false_path -from [get_ports {CPU_RESET GPIO_SW_C GPIO_SW_N GPIO_DIP_SW[*]}]
set_false_path -to   [get_ports {GPIO_LED[*]}]

# ---- configuration (bank 0 at 3.3 V on the AC701) and the bitstream ----
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
