# stop any simulation that is currently running
quit -sim

# create the default "work" library
vlib work;

# compile the Verilog source code in the parent folders
vlog +define+DESIM ../*.v ../object_lib/keyboard.v ../object_lib/vga_adapter.v ../object_lib/vga_address_translator.v ../object_lib/vga_controller.v
vlog ../tb/PS2_keyboard.v
# compile the Verilog code of the testbench
vlog testbench.v
# start the Simulator with full visibility for waveforms
vsim -voptargs=+acc work.testbench -Lf 220model -Lf altera_mf_ver -Lf verilog
# show waveforms specified in wave.do
do wave.do
# advance the simulation the desired amount of time
run 30000 ns
