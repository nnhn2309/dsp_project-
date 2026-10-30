# ============================================================================
# SDC Timing Constraints cho DE2 ANC Project
# Device: Cyclone II EP2C35F672C6
# ============================================================================

# Clock 50 MHz chinh
create_clock -name CLOCK_50 -period 20.000 [get_ports CLOCK_50]

# Audio Bit Clock tu WM8731 (~512 kHz khi fs=8kHz, 16bit stereo)
create_clock -name AUD_BCLK -period 1953.125 [get_ports AUD_BCLK]

# Clock audio tao tu CLOCK_50 chia 4 -> 12.5 MHz
create_generated_clock -name AUD_XCK \
    -source [get_ports CLOCK_50] \
    -divide_by 4 \
    [get_registers {xck_div[1]}]

# Hai mien clock doc lap nhau (Asynchronous)
set_clock_groups -asynchronous \
    -group [get_clocks {CLOCK_50 AUD_XCK}] \
    -group [get_clocks {AUD_BCLK}]

# False path cho cac chan IO cham (khong can timing check)
set_false_path -from [get_ports {KEY[*] SW[*]}]      -to [all_registers]
set_false_path -from [all_registers] -to [get_ports {LEDR[*] LEDG[*] HEX*[*]}]
