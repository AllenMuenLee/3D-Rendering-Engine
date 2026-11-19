# stop any simulation that is currently running
quit -sim

# create the default "work" library
vlib work;

# compile the Verilog source code for the VGA adapter
vlog ../vga_adapter/*.v
# compile the VGA testbench
vlog testbench_vga.v
# start the Simulator with full visibility for waveforms
vsim -voptargs=+acc work.testbench_vga -Lf 220model -Lf altera_mf_ver -Lf verilog
# advance the simulation
run -all
