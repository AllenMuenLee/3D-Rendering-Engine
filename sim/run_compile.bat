set SOURCE_FILES=../*.v ../vga_adapter/*.v

if exist work rmdir /S /Q work

vlib work
vlog ../tb/*.v
vlog +define+DESIM %SOURCE_FILES%
