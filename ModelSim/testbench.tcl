# stop any simulation that is currently running
quit -sim

# create the default "work" library
vlib work;

# compile the Verilog source code in the parent folders
vlog ../top.v ../keyboard.v ../platonic_lut.v ../platonic_renderer.v ../vga_adapter/*.v
vlog ../tb/PS2_keyboard.v
# compile the Verilog code of the testbench
vlog testbench.v
# start the Simulator with full visibility for waveforms
vsim -voptargs=+acc work.testbench -Lf 220model -Lf altera_mf_ver -Lf verilog
# show waveforms specified in wave.do
do wave.do
# advance the simulation the desired amount of time
run 30000 ns
