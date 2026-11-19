`default_nettype none

// Uncomment to build for DESim simulation instead of a physical FPGA board:
// `define DESIM

module top (
    CLOCK_50,
    KEY,
    LEDR,
    PS2_CLK,
    PS2_DAT,
`ifdef DESIM
    VGA_X,
    VGA_Y,
    VGA_COLOR,
    plot
`else
    VGA_R,
    VGA_G,
    VGA_B,
    VGA_HS,
    VGA_VS,
    VGA_BLANK_N,
    VGA_SYNC_N,
    VGA_CLK
`endif
);

    // --- Screen & color settings (hardcoded to 640x480) ---
    localparam RESOLUTION       = "640x480";
    localparam nX               = 10;
    localparam nY               = 9;
    parameter  COLOR_DEPTH      = 9;         // 9, 6, or 3

    // --- Custom 3D renderer parameters (not from vga_adapter) ---
    parameter [2:0] SHAPE_ID    = 3'd2;      // 0:Tetra, 1:Octa, 2:Cube, 3:Icosa, 4:Dodeca
    parameter CULL_INWARD_NEG_Y = 1;         // 1: cull faces with ny <= 0, 0: cull ny >= 0

    input  wire       CLOCK_50;              // DE-series 50 MHz clock signal
    input  wire [3:0] KEY;                   // DE-series pushbuttons (active-low)

    output wire [9:0] LEDR;                  // DE-series LEDs
    inout  wire       PS2_CLK;               // DE-series PS/2 Clock
    inout  wire       PS2_DAT;               // DE-series PS/2 Data

`ifdef DESIM
    output wire [9:0]  VGA_X;                // DESim VGA X coordinate
    output wire [8:0]  VGA_Y;                // DESim VGA Y coordinate
    output wire [23:0] VGA_COLOR;            // DESim VGA 24-bit RGB color
    output wire        plot;                 // DESim VGA pixel write strobe
`else
    output wire [7:0]  VGA_R;                // FPGA VGA DAC Red
    output wire [7:0]  VGA_G;                // FPGA VGA DAC Green
    output wire [7:0]  VGA_B;                // FPGA VGA DAC Blue
    output wire        VGA_HS;               // FPGA VGA Horizontal Sync
    output wire        VGA_VS;               // FPGA VGA Vertical Sync
    output wire        VGA_BLANK_N;          // FPGA VGA Active-low Blanking
    output wire        VGA_SYNC_N;           // FPGA VGA Sync-on-Green (unused = 0)
    output wire        VGA_CLK;              // FPGA VGA 25 MHz Pixel Clock
`endif

    wire Resetn;
    wire ps2_rec;
    wire [7:0] scancode;
    wire object_sel;
    wire [1:0] O1_dir;
    wire [1:0] O2_dir;

    // Renderer -> VGA adapter signals (640x480: 10-bit X, 9-bit Y)
    wire [9:0] vga_x;
    wire [8:0] vga_y;
    wire [COLOR_DEPTH-1:0] vga_color;
    wire vga_write;
    wire render_done;
    wire VGA_SYNC;

    assign Resetn = KEY[0];

    // --- Custom PS/2 keyboard controller (not from vga_adapter) ---
    keyboard KB (
        .CLOCK_50(CLOCK_50),
        .Resetn(Resetn),
        .PS2_CLK(PS2_CLK),
        .PS2_DAT(PS2_DAT),
        .ps2_rec(ps2_rec),
        .scancode(scancode),
        .object_sel(object_sel),
        .O1_dir(O1_dir),
        .O2_dir(O2_dir),
        .LEDR(LEDR[3:0])
    );

    // --- Custom 3D Platonic solid renderer (not from vga_adapter) ---
    // Reads geometry/normals from platonic_lut, culls inward ny faces, and
    // streams rasterized pixels (vga_x, vga_y, vga_color, vga_write) to VGA.
    platonic_renderer #(
        .SHAPE_ID(SHAPE_ID),
        .CULL_INWARD_NEG_Y(CULL_INWARD_NEG_Y),
        .COLOR_DEPTH(COLOR_DEPTH)
    ) RENDERER (
        .clk(CLOCK_50),
        .resetn(Resetn),
        .vga_sync(VGA_SYNC),
        .vga_x(vga_x),
        .vga_y(vga_y),
        .vga_color(vga_color),
        .vga_write(vga_write),
        .done(render_done)
    );

`ifdef DESIM
    // --- Existing DESim vga_adapter instantiation ---
    wire [9:0] vga_out_x;
    wire [8:0] vga_out_y;

    vga_adapter #(
        .RESOLUTION(RESOLUTION),
        .COLOR_DEPTH(COLOR_DEPTH),
        .nX(nX),
        .nY(nY)
    ) VGA (
        .resetn(Resetn),
        .clock(CLOCK_50),
        .color(vga_color),
        .x(vga_x),
        .y(vga_y),
        .write(vga_write),
        .VGA_X(vga_out_x),
        .VGA_Y(vga_out_y),
        .VGA_COLOR(VGA_COLOR),
        .VGA_SYNC(VGA_SYNC),
        .plot(plot)
    );

    assign VGA_X     = vga_out_x;
    assign VGA_Y     = vga_out_y;
    assign LEDR[9:4] = 6'b0;
`else
    // --- Physical FPGA board VGA controller + framebuffer (not from DESim vga_adapter) ---
    vga_board_adapter #(
        .COLOR_DEPTH(COLOR_DEPTH)
    ) VGA_HW (
        .resetn(Resetn),
        .clock(CLOCK_50),
        .color(vga_color),
        .x(vga_x),
        .y(vga_y),
        .write(vga_write),
        .VGA_SYNC(VGA_SYNC),
        .VGA_R(VGA_R),
        .VGA_G(VGA_G),
        .VGA_B(VGA_B),
        .VGA_HS(VGA_HS),
        .VGA_VS(VGA_VS),
        .VGA_BLANK_N(VGA_BLANK_N),
        .VGA_SYNC_N(VGA_SYNC_N),
        .VGA_CLK(VGA_CLK)
    );

    // Status LEDs: LEDR[9]=framebuffer cleared, LEDR[8]=shape rendered, LEDR[3:0]=WASD
    assign LEDR[9]   = VGA_SYNC;
    assign LEDR[8]   = render_done;
    assign LEDR[7:4] = 4'b0;
`endif

endmodule

`default_nettype wire
