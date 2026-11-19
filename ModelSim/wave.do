onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate -label CLOCK_50 -radix binary /testbench/CLOCK_50
add wave -noupdate -label {KEY[0] (Resetn)} -radix binary /testbench/KEY[0]
add wave -noupdate -divider {PS/2 Input}
add wave -noupdate -label key_action -radix binary /testbench/key_action
add wave -noupdate -label scan_code -radix hexadecimal /testbench/scan_code
add wave -noupdate -label ps2_clk -radix binary /testbench/ps2_clk
add wave -noupdate -label ps2_dat -radix binary /testbench/ps2_dat
add wave -noupdate -divider {Keyboard Decoder}
add wave -noupdate -label Packet -radix unsigned /testbench/U1/KB/Packet
add wave -noupdate -label ps2_rec -radix binary /testbench/U1/KB/ps2_rec
add wave -noupdate -label scancode -radix hexadecimal /testbench/U1/KB/scancode
add wave -noupdate -divider {WASD LEDs}
add wave -noupdate -label {LEDR[3:0]} -radix binary /testbench/LEDR[3:0]
add wave -noupdate -label {LED_W (LEDR[3])} -radix binary /testbench/LEDR[3]
add wave -noupdate -label {LED_A (LEDR[2])} -radix binary /testbench/LEDR[2]
add wave -noupdate -label {LED_S (LEDR[1])} -radix binary /testbench/LEDR[1]
add wave -noupdate -label {LED_D (LEDR[0])} -radix binary /testbench/LEDR[0]
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {0 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 150
configure wave -valuecolwidth 70
configure wave -justifyvalue left
configure wave -signalnamewidth 0
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ns
update
WaveRestoreZoom {0 ps} {30000 ns}
