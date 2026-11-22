`default_nettype none

module top #(
    parameter [21:0] KEY_DELAY_CYCLES = 22'd2500000,
    parameter [8:0]  ROT_STEP_DEG     = 9'd5,
    parameter [8:0]  DEFAULT_ANGLE_X  = 9'd30,
    parameter [8:0]  DEFAULT_ANGLE_Z  = 9'd35
) (
    input  wire       CLOCK_50,
    input  wire [9:0] SW,
    input  wire [3:0] KEY,
    output wire [9:0] LEDR,
    inout  wire       PS2_CLK,
    inout  wire       PS2_DAT,
`ifdef DESIM
    output wire [9:0]  VGA_X,
    output wire [8:0]  VGA_Y,
    output wire [23:0] VGA_COLOR,
    output wire        plot
`else
    output wire [7:0]  VGA_R,
    output wire [7:0]  VGA_G,
    output wire [7:0]  VGA_B,
    output wire        VGA_HS,
    output wire        VGA_VS,
    output wire        VGA_BLANK_N,
    output wire        VGA_SYNC_N,
    output wire        VGA_CLK
`endif
);
    wire Resetn = KEY[0];

    wire [3:0] wasd_held;

    keyboard KB (
        .CLOCK_50   (CLOCK_50),
        .Resetn     (Resetn),
        .PS2_CLK    (PS2_CLK),
        .PS2_DAT    (PS2_DAT),
        .ps2_rec    (),
        .scancode   (),
        .object_sel (),
        .O1_dir     (),
        .O2_dir     (),
        .LEDR       (LEDR[3:0]),
        .wasd_held  (wasd_held)
    );

    reg [21:0] key_cnt;
    reg [8:0]  angle_x;
    reg [8:0]  angle_z;

    always @(posedge CLOCK_50) begin
        if (!Resetn) begin
            key_cnt <= 22'd0;
            angle_x <= DEFAULT_ANGLE_X;
            angle_z <= DEFAULT_ANGLE_Z;
        end else if (wasd_held != 4'b0000) begin
            if (key_cnt == KEY_DELAY_CYCLES - 22'd1) begin
                key_cnt <= 22'd0;

                if (wasd_held[3] && !wasd_held[1])
                    angle_x <= (angle_x < ROT_STEP_DEG) ? (angle_x + 9'd360 - ROT_STEP_DEG) : (angle_x - ROT_STEP_DEG);
                else if (wasd_held[1] && !wasd_held[3])
                    angle_x <= (angle_x + ROT_STEP_DEG >= 9'd360) ? (angle_x + ROT_STEP_DEG - 9'd360) : (angle_x + ROT_STEP_DEG);

                if (wasd_held[2] && !wasd_held[0])
                    angle_z <= (angle_z + ROT_STEP_DEG >= 9'd360) ? (angle_z + ROT_STEP_DEG - 9'd360) : (angle_z + ROT_STEP_DEG);
                else if (wasd_held[0] && !wasd_held[2])
                    angle_z <= (angle_z < ROT_STEP_DEG) ? (angle_z + 9'd360 - ROT_STEP_DEG) : (angle_z - ROT_STEP_DEG);
            end else begin
                key_cnt <= key_cnt + 22'd1;
            end
        end else begin
            key_cnt <= 22'd0;
        end
    end

    reg [9:0] sw_q1, sw_sync;
    reg       shape_enable;
    reg [3:0] shape_id;

    always @(posedge CLOCK_50) begin
        if (!Resetn) begin
            sw_q1        <= 10'd0;
            sw_sync      <= 10'd0;
            shape_enable <= 1'b0;
            shape_id     <= 4'd0;
        end else begin
            sw_q1   <= SW;
            sw_sync <= sw_q1;

            case (sw_sync)
                10'b00000_00000: shape_enable <= 1'b0;
                10'b00001_00000: begin shape_enable <= 1'b1; shape_id <= 4'd0; end
                10'b00010_00000: begin shape_enable <= 1'b1; shape_id <= 4'd1; end
                10'b00100_00000: begin shape_enable <= 1'b1; shape_id <= 4'd2; end
                10'b01000_00000: begin shape_enable <= 1'b1; shape_id <= 4'd3; end
                10'b10000_00000: begin shape_enable <= 1'b1; shape_id <= 4'd4; end
                10'b00000_10000: begin shape_enable <= 1'b1; shape_id <= 4'd5; end
                10'b00000_01000: begin shape_enable <= 1'b1; shape_id <= 4'd6; end
                10'b00000_00100: begin shape_enable <= 1'b1; shape_id <= 4'd7; end
                10'b00000_00010: begin shape_enable <= 1'b1; shape_id <= 4'd8; end
                10'b00000_00001: begin shape_enable <= 1'b1; shape_id <= 4'd9; end
                default: ;
            endcase
        end
    end

    wire [9:0]  vga_x;
    wire [8:0]  vga_y;
    wire [23:0] vga_color;
    wire        vga_write;
    wire        render_done;
    wire        VGA_SYNC;

    renderer RENDERER (
        .clk          (CLOCK_50),
        .resetn       (Resetn),
        .vga_sync     (VGA_SYNC),
        .shape_enable (shape_enable),
        .shape_id     (shape_id),
        .angle_x      (angle_x),
        .angle_z      (angle_z),
        .vga_x        (vga_x),
        .vga_y        (vga_y),
        .vga_color    (vga_color),
        .vga_write    (vga_write),
        .done         (render_done)
    );

`ifdef DESIM
    vga_adapter #(
        .RESOLUTION  ("640x480"),
        .COLOR_DEPTH (24),
        .nX          (10),
        .nY          (9)
    ) VGA (
        .resetn    (Resetn),
        .clock     (CLOCK_50),
        .color     (vga_color),
        .x         (vga_x),
        .y         (vga_y),
        .write     (vga_write),
        .VGA_X     (VGA_X),
        .VGA_Y     (VGA_Y),
        .VGA_COLOR (VGA_COLOR),
        .VGA_SYNC  (VGA_SYNC),
        .plot      (plot)
    );
    assign LEDR[9:4] = 6'b0;
`else
    vga_board_adapter #(
        .COLOR_DEPTH (24)
    ) VGA_HW (
        .resetn      (Resetn),
        .clock       (CLOCK_50),
        .color       (vga_color),
        .x           (vga_x),
        .y           (vga_y),
        .write       (vga_write),
        .VGA_SYNC    (VGA_SYNC),
        .VGA_R       (VGA_R),
        .VGA_G       (VGA_G),
        .VGA_B       (VGA_B),
        .VGA_HS      (VGA_HS),
        .VGA_VS      (VGA_VS),
        .VGA_BLANK_N (VGA_BLANK_N),
        .VGA_SYNC_N  (VGA_SYNC_N),
        .VGA_CLK     (VGA_CLK)
    );
    assign LEDR[9]   = VGA_SYNC;
    assign LEDR[8]   = render_done;
    assign LEDR[7:4] = 4'b0;
`endif
endmodule

`default_nettype wire
