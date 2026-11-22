# stop any simulation that is currently running
quit -sim

# create the default "work" library
vlib work;

# compile the Verilog source code for the VGA adapter
vlog ../object_lib/vga_adapter.v ../object_lib/vga_address_translator.v ../object_lib/vga_controller.v
# compile the VGA testbench
vlog testbench_vga.v
# start the Simulator with full visibility for waveforms
vsim -voptargs=+acc work.testbench_vga -Lf 220model -Lf altera_mf_ver -Lf verilog
# advance the simulation
run -all
