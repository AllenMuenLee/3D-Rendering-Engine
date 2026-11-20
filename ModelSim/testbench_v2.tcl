# stop any simulation that is currently running
quit -sim

# create the default "work" library
vlib work;

# compile the Verilog source code for keyboard_v2
vlog +define+DESIM ../top.v ../keyboard_v2.v ../platonic_lut.v ../sin_lut.v ../platonic_renderer.v ../vga_adapter/*.v
vlog ../tb/PS2_keyboard.v
# compile the Verilog code of the v2 testbench
vlog testbench_v2.v
# start the Simulator with full visibility for waveforms
vsim -voptargs=+acc work.testbench_v2 -Lf 220model -Lf altera_mf_ver -Lf verilog
# show waveforms specified in wave_v2.do
do wave_v2.do
# advance the simulation the desired amount of time
run 35000 ns
