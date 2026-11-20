`default_nettype none

// Uncomment to build for DESim simulation instead of a physical FPGA board:
// `define DESIM

module top (
    CLOCK_50,
    SW,
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
    parameter  COLOR_DEPTH      = 24;        // 24 (8-bit smooth grayscale/RGB), 9, 6, or 3

    // --- Custom 3D renderer parameters (not from vga_adapter) ---
    parameter CULL_INWARD_NEG_Y = 1;                   // 1: cull faces with ny <= 0, 0: cull ny >= 0
    parameter [8:0] DEFAULT_ANGLE_X = 9'd30;           // Default X-axis rotation in degrees (0..359)
    parameter [8:0] DEFAULT_ANGLE_Y = 9'd20;           // Default Y-axis rotation in degrees (0..359)
    parameter [8:0] DEFAULT_ANGLE_Z = 9'd35;           // Default Z-axis rotation in degrees (0..359)
    parameter [8:0]  ROT_STEP_DEG      = 9'd3;         // Rotation step in degrees per key press / hold tick
    parameter [23:0] ROT_REPEAT_CYCLES = 24'd1666667;  // ~33.3 ms @ 50 MHz (30 Hz while key held)
    parameter signed [15:0] OBJ_SCALE_X = 16'sd135;    // X radius (3/4 of 180 for 16:9 widescreen stretch)
    parameter signed [15:0] OBJ_SCALE_Z = 16'sd180;    // Z/Y radius (3/8 of 480)
    // 3D point light coordinate in signed Q6.14 (1.0 = 16384), in world space
    // (unit-sphere object centered at origin; +X right, +Y toward viewer, +Z down).
    // Default: (+0.5, +2.0, -1.0).
    parameter signed [19:0] LIGHT_POS_X = 20'sd8192;    // +0.5
    parameter signed [19:0] LIGHT_POS_Y = 20'sd32768;   // +2.0
    parameter signed [19:0] LIGHT_POS_Z = -20'sd16384;  // -1.0
    parameter [14:0] LIGHT_BRIGHTNESS_MIN = 15'd1638;   // 10% brightness at flux = 0.0 (0.10 * 16384)
    parameter [14:0] LIGHT_BRIGHTNESS_MAX = 15'd13107;  // 80% brightness at flux = 1.0 (0.80 * 16384)
    parameter [8:0]  BASE_COLOR_RGB9 = 9'b111_111_111;  // Monotone white base color (grayscale shading)

    input  wire       CLOCK_50;              // DE-series 50 MHz clock signal
    input  wire [9:0] SW;                    // DE-series switches (SW[9:5] select shape)
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
    wire [3:0] wasd_held;

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
        .LEDR(LEDR[3:0]),
        .wasd_held(wasd_held)
    );

    // --- Keyboard-driven fixed screen-axis 3D rotation steps ---
    //   Screen X axis is horizontal; screen Z axis is vertical; Y axis is depth (+Y toward viewer).
    //   'w' (wasd_held[3]) / 's' (wasd_held[1]): rotate vertices around fixed screen X axis (-X / +X)
    //   'a' (wasd_held[2]) / 'd' (wasd_held[0]): rotate vertices around fixed screen Z axis (+Z / -Z)
    localparam [23:0] ROT_CNT_MAX = (ROT_REPEAT_CYCLES > 24'd0) ? (ROT_REPEAT_CYCLES - 24'd1) : 24'd0;

    reg  [3:0]  prev_wasd_held;
    reg  [23:0] rot_tick_cnt;

    wire [3:0]  wasd_rise  = wasd_held & ~prev_wasd_held;
    wire        any_wasd   = |wasd_held;
    wire        rot_tick   = any_wasd && (rot_tick_cnt == ROT_CNT_MAX);

    wire        step_x_neg = (wasd_rise[3] || rot_tick) && wasd_held[3] && !wasd_held[1]; // 'w' -> -X (tilt front up)
    wire        step_x_pos = (wasd_rise[1] || rot_tick) && wasd_held[1] && !wasd_held[3]; // 's' -> +X (tilt front down)
    wire        step_z_pos = (wasd_rise[2] || rot_tick) && wasd_held[2] && !wasd_held[0]; // 'a' -> +Z (spin front left)
    wire        step_z_neg = (wasd_rise[0] || rot_tick) && wasd_held[0] && !wasd_held[2]; // 'd' -> -Z (spin front right)

    always @(posedge CLOCK_50) begin
        if (!Resetn) begin
            prev_wasd_held <= 4'b0000;
            rot_tick_cnt   <= 24'd0;
        end else begin
            prev_wasd_held <= wasd_held;

            if (!any_wasd || rot_tick)
                rot_tick_cnt <= 24'd0;
            else
                rot_tick_cnt <= rot_tick_cnt + 24'd1;
        end
    end

    // --- Shape selection from SW[9:5] (one-hot, 2-FF synchronized) ---
    //   SW[5] (5'b00001): Tetrahedron  (3'd0)
    //   SW[6] (5'b00010): Octahedron   (3'd1)
    //   SW[7] (5'b00100): Cube         (3'd2)
    //   SW[8] (5'b01000): Icosahedron  (3'd3)
    //   SW[9] (5'b10000): Dodecahedron (3'd4)
    //   All off (5'b00000): blank screen (nothing rendered)
    //   Multiple switches on: do nothing (hold current shape/state)
    reg [4:0] sw_q1, sw_sync;
    reg       shape_enable;
    reg [2:0] shape_id;

    wire       sw_all_off = (sw_sync == 5'b00000);
    reg        sw_one_hot;
    reg  [2:0] sw_decoded_id;

    always @(*) begin
        sw_one_hot    = 1'b0;
        sw_decoded_id = 3'd0;
        case (sw_sync)
            5'b00001: begin sw_one_hot = 1'b1; sw_decoded_id = 3'd0; end // SW[5]: Tetrahedron
            5'b00010: begin sw_one_hot = 1'b1; sw_decoded_id = 3'd1; end // SW[6]: Octahedron
            5'b00100: begin sw_one_hot = 1'b1; sw_decoded_id = 3'd2; end // SW[7]: Cube
            5'b01000: begin sw_one_hot = 1'b1; sw_decoded_id = 3'd3; end // SW[8]: Icosahedron
            5'b10000: begin sw_one_hot = 1'b1; sw_decoded_id = 3'd4; end // SW[9]: Dodecahedron
            default:  begin sw_one_hot = 1'b0; sw_decoded_id = 3'd0; end // 0 or >=2 switches on
        endcase
    end

    always @(posedge CLOCK_50) begin
        if (!Resetn) begin
            sw_q1        <= 5'b00000;
            sw_sync      <= 5'b00000;
            shape_enable <= 1'b0;
            shape_id     <= 3'd0;
        end else begin
            sw_q1   <= SW[9:5];
            sw_sync <= sw_q1;

            if (sw_all_off) begin
                shape_enable <= 1'b0;
            end else if (sw_one_hot) begin
                shape_enable <= 1'b1;
                shape_id     <= sw_decoded_id;
            end
            // Multiple switches on: hold shape_enable and shape_id (do nothing)
        end
    end

    // --- Custom 3D Platonic solid renderer (not from vga_adapter) ---
    // Reads geometry/normals from platonic_lut, rotates via sin_lut, culls inward
    // ny faces, and streams rasterized pixels (vga_x, vga_y, vga_color, vga_write) to VGA.
    platonic_renderer #(
        .CULL_INWARD_NEG_Y(CULL_INWARD_NEG_Y),
        .COLOR_DEPTH(COLOR_DEPTH),
        .DEFAULT_ANGLE_X(DEFAULT_ANGLE_X),
        .DEFAULT_ANGLE_Y(DEFAULT_ANGLE_Y),
        .DEFAULT_ANGLE_Z(DEFAULT_ANGLE_Z),
        .ROT_STEP_DEG(ROT_STEP_DEG),
        .OBJ_SCALE_X(OBJ_SCALE_X),
        .OBJ_SCALE_Z(OBJ_SCALE_Z),
        .LIGHT_POS_X(LIGHT_POS_X),
        .LIGHT_POS_Y(LIGHT_POS_Y),
        .LIGHT_POS_Z(LIGHT_POS_Z),
        .LIGHT_BRIGHTNESS_MIN(LIGHT_BRIGHTNESS_MIN),
        .LIGHT_BRIGHTNESS_MAX(LIGHT_BRIGHTNESS_MAX),
        .BASE_COLOR_RGB9(BASE_COLOR_RGB9)
    ) RENDERER (
        .clk(CLOCK_50),
        .resetn(Resetn),
        .vga_sync(VGA_SYNC),
        .shape_enable(shape_enable),
        .shape_id(shape_id),
        .rot_x_pos(step_x_pos),
        .rot_x_neg(step_x_neg),
        .rot_z_pos(step_z_pos),
        .rot_z_neg(step_z_neg),
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
