set SOURCE_FILES=../*.v ../object_lib/vga_adapter.v ../object_lib/vga_address_translator.v ../object_lib/vga_controller.v ../object_lib/keyboard_v2.v

if exist work rmdir /S /Q work

vlib work
vlog ../tb/*.v
vlog +define+DESIM %SOURCE_FILES%
