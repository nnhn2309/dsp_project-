# ============================================================================
# Synopsys Design Constraints (.sdc) cho DE10-Standard ANC
# ============================================================================

# 1. Khai bao Clock he thong 50 MHz (PIN_AF14)
create_clock -name CLOCK_50 -period 20.000 [get_ports CLOCK_50]

# 2. Khai bao Audio Bit Clock tu Codec WM8731 (~512 kHz - 1.5 MHz)
create_clock -name AUD_BCLK -period 1953.125 [get_ports AUD_BCLK]

# 3. Clock phu thuoc (Generated Clocks)
# AUD_XCK duoc tao bang cach chia 4 tu CLOCK_50 (~12.5 MHz)
create_generated_clock -name AUD_XCK -source [get_ports CLOCK_50] -divide_by 4 [get_registers {xck_div[1]}]

# 4. Rang buoc mien clock doc lap (Asynchronous Clock Groups)
set_clock_groups -asynchronous \
    -group [get_clocks {CLOCK_50 AUD_XCK}] \
    -group [get_clocks {AUD_BCLK}]

# 5. Khai bao False Path cho cac chan I/O cham (Nut bam, Cong tac, LED, 7-doan)
set_false_path -from [get_ports {KEY[*] SW[*]}] -to [all_registers]
set_false_path -from [all_registers] -to [get_ports {LEDR[*] HEX*[*]}]
