`default_nettype none

module renderer #(
    parameter integer       NUM_POINTS = 2048,

    parameter signed [10:0] LIGHT_X    = 11'sd64,
    parameter signed [10:0] LIGHT_Y    = 11'sd256,
    parameter signed [10:0] LIGHT_Z    = -11'sd128,
    parameter [7:0]         MIN_BRIGHT = 8'd26,
    parameter [7:0]         MAX_BRIGHT = 8'd230,
`ifdef DESIM
    parameter               LUT_FILE   = "../shape_lut.hex"
`else
    parameter               LUT_FILE   = "shape_lut.hex"
`endif
) (
    input  wire        clk,
    input  wire        resetn,
    input  wire        vga_sync,
    input  wire        shape_enable,
    input  wire [3:0]  shape_id,
    input  wire [8:0]  angle_x,
    input  wire [8:0]  angle_z,
    output reg  [9:0]  vga_x,
    output reg  [8:0]  vga_y,
    output reg  [23:0] vga_color,
    output reg         vga_write,
    output reg         done
);
    localparam [1:0]
        S_IDLE       = 2'd0,
        S_CLEAR      = 2'd1,
        S_MATRIX_MUL = 2'd2,
        S_RENDER     = 2'd3;

    reg [1:0]  state;
    reg        sub;
    reg [10:0] idx;
    reg [9:0]  clear_x;
    reg [8:0]  clear_y;
    reg        screen_dirty;

    reg       prev_enable;
    reg [3:0] prev_shape_id;
    reg [8:0] prev_angle_x;
    reg [8:0] prev_angle_z;

    wire need_update = (shape_enable != prev_enable) ||
                       (shape_enable && (
                           (shape_id != prev_shape_id) ||
                           (angle_x  != prev_angle_x)  ||
                           (angle_z  != prev_angle_z)
                       ));

    wire signed [15:0] sx, cx, sz, cz;
    sin_lut LUT_SX (.angle(angle_x),         .sin_val(sx));
    sin_lut LUT_CX (.angle(angle_x + 9'd90), .sin_val(cx));
    sin_lut LUT_SZ (.angle(angle_z),         .sin_val(sz));
    sin_lut LUT_CZ (.angle(angle_z + 9'd90), .sin_val(cz));

    wire signed [8:0] base_px, base_py, base_pz;
    wire signed [8:0] base_nx, base_ny, base_nz;
    shape_lut #(
        .NUM_POINTS (NUM_POINTS),
        .INIT_FILE  (LUT_FILE)
    ) BASE_LUT (
        .clk       (clk),
        .shape_id  (prev_shape_id),
        .point_idx (idx),
        .px (base_px), .py (base_py), .pz (base_pz),
        .nx (base_nx), .ny (base_ny), .nz (base_nz)
    );

    reg signed [15:0] r00, r01, r02;
    reg signed [15:0] r10, r11, r12;
    reg signed [15:0] r20, r21, r22;

    wire signed [31:0] sz_cx = sz * cx;
    wire signed [31:0] sz_sx = sz * sx;
    wire signed [31:0] cz_cx = cz * cx;
    wire signed [31:0] cz_sx = cz * sx;

    function signed [8:0] mat_row_mul;
        input signed [15:0] m0, m1, m2;
        input signed [8:0]  v0, v1, v2;
        reg   signed [26:0] acc;
        begin
            acc = (m0 * v0) + (m1 * v1) + (m2 * v2) + 27'sd8192;
            mat_row_mul = acc >>> 14;
        end
    endfunction

    (* ramstyle = "M10K" *) reg [53:0] updated_lut [0:NUM_POINTS-1];
    reg signed [8:0] cur_px, cur_py, cur_pz;
    reg signed [8:0] cur_nx, cur_ny, cur_nz;

    wire signed [11:0] dx = LIGHT_X - cur_px;
    wire signed [11:0] dy = LIGHT_Y - cur_py;
    wire signed [11:0] dz = LIGHT_Z - cur_pz;

    wire signed [19:0] dot_x   = cur_nx * dx;
    wire signed [19:0] dot_y   = cur_ny * dy;
    wire signed [19:0] dot_z   = cur_nz * dz;
    wire signed [21:0] dot_val = dot_x + dot_y + dot_z;

    wire [22:0] bright_scaled = dot_val[14:0] * (MAX_BRIGHT - MIN_BRIGHT);
    wire [7:0]  brightness =
        (dot_val <= 22'sd0)     ? MIN_BRIGHT :
        (dot_val >= 22'sd32768) ? MAX_BRIGHT :
                                  (MIN_BRIGHT + bright_scaled[22:15]);

    wire signed [10:0] screen_x = 11'sd320 + cur_px;
    wire signed [10:0] screen_y = 11'sd240 + cur_pz;

    always @(posedge clk) begin
        if (!resetn) begin
            state         <= S_IDLE;
            sub           <= 1'b0;
            idx           <= 11'd0;
            clear_x       <= 10'd0;
            clear_y       <= 9'd0;
            screen_dirty  <= 1'b0;
            prev_enable   <= 1'b0;
            prev_shape_id <= 4'd0;
            prev_angle_x  <= 9'd0;
            prev_angle_z  <= 9'd0;
            vga_x         <= 10'd0;
            vga_y         <= 9'd0;
            vga_color     <= 24'd0;
            vga_write     <= 1'b0;
            done          <= 1'b0;
        end else begin
            vga_write <= 1'b0;

            case (state)

                S_IDLE: begin
                    if (vga_sync && need_update) begin
                        done          <= 1'b0;
                        prev_enable   <= shape_enable;
                        prev_shape_id <= shape_id;
                        prev_angle_x  <= angle_x;
                        prev_angle_z  <= angle_z;

                        r00 <=  cz;
                        r01 <= -(sz_cx >>> 14);
                        r02 <=  (sz_sx >>> 14);
                        r10 <=  sz;
                        r11 <=  (cz_cx >>> 14);
                        r12 <= -(cz_sx >>> 14);
                        r20 <=  16'sd0;
                        r21 <=  sx;
                        r22 <=  cx;

                        idx <= 11'd0;
                        sub <= 1'b0;

                        if (shape_enable) begin
                            state <= S_MATRIX_MUL;
                        end else if (screen_dirty) begin
                            clear_x <= 10'd0;
                            clear_y <= 9'd0;
                            state   <= S_CLEAR;
                        end
                    end
                end

                S_MATRIX_MUL: begin
                    if (!sub) begin
                        sub <= 1'b1;
                    end else begin
                        updated_lut[idx] <= {
                            mat_row_mul(r00, r01, r02, base_px, base_py, base_pz),
                            mat_row_mul(r10, r11, r12, base_px, base_py, base_pz),
                            mat_row_mul(r20, r21, r22, base_px, base_py, base_pz),
                            mat_row_mul(r00, r01, r02, base_nx, base_ny, base_nz),
                            mat_row_mul(r10, r11, r12, base_nx, base_ny, base_nz),
                            mat_row_mul(r20, r21, r22, base_nx, base_ny, base_nz)
                        };
                        sub <= 1'b0;

                        if (idx == NUM_POINTS - 1) begin
                            idx <= 11'd0;
                            if (screen_dirty) begin
                                clear_x <= 10'd0;
                                clear_y <= 9'd0;
                                state   <= S_CLEAR;
                            end else begin
                                state   <= S_RENDER;
                            end
                        end else begin
                            idx <= idx + 11'd1;
                        end
                    end
                end

                S_CLEAR: begin
                    vga_x     <= clear_x;
                    vga_y     <= clear_y;
                    vga_color <= 24'd0;
                    vga_write <= 1'b1;

                    if (clear_x < 10'd639) begin
                        clear_x <= clear_x + 10'd1;
                    end else if (clear_y < 9'd479) begin
                        clear_x <= 10'd0;
                        clear_y <= clear_y + 9'd1;
                    end else begin
                        screen_dirty <= 1'b0;
                        idx          <= 11'd0;
                        sub          <= 1'b0;
                        if (prev_enable)
                            state <= S_RENDER;
                        else begin
                            done  <= 1'b1;
                            state <= S_IDLE;
                        end
                    end
                end

                S_RENDER: begin
                    if (!sub) begin

                        {cur_px, cur_py, cur_pz, cur_nx, cur_ny, cur_nz} <= updated_lut[idx];
                        sub <= 1'b1;
                    end else begin

                        if (cur_ny > 9'sd0) begin
                            vga_x        <= screen_x[9:0];
                            vga_y        <= screen_y[8:0];
                            vga_color    <= {brightness, brightness, brightness};
                            vga_write    <= 1'b1;
                            screen_dirty <= 1'b1;
                        end

                        sub <= 1'b0;
                        if (idx == NUM_POINTS - 1) begin
                            done  <= 1'b1;
                            state <= S_IDLE;
                        end else begin
                            idx <= idx + 11'd1;
                        end
                    end
                end
            endcase
        end
    end
endmodule

`default_nettype wire
