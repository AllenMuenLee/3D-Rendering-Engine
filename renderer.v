`default_nettype none

module renderer #(
    parameter signed [15:0] SIN_DELTA  = 16'sd1428,
    parameter signed [15:0] COS_DELTA  = 16'sd16322,
    parameter integer       MAX_TRIS   = 256,
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
    input  wire              shape_enable,
    input  wire [3:0]        shape_id,
    input  wire              rot_req,
    input  wire signed [1:0] rot_x,
    input  wire signed [1:0] rot_z,
    output reg  [9:0]        vga_x,
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

    reg [1:0] state;
    reg [1:0] sub;
    reg [7:0] idx;
    reg       screen_dirty;

    reg       prev_enable;
    reg [3:0] prev_shape_id;
    reg [3:0] next_shape_id;
    reg       prev_rot_req;
    reg       load_base;

    wire shape_changed = (shape_enable != prev_enable) ||
                         (shape_enable && (shape_id != prev_shape_id));
    wire rot_pending   = (rot_req != prev_rot_req);
    wire need_update   = shape_changed || (shape_enable && rot_pending);

    wire signed [15:0] sx = (rot_x == 2'sd1)  ?  SIN_DELTA :
                            (rot_x == -2'sd1) ? -SIN_DELTA : 16'sd0;
    wire signed [15:0] cx = (rot_x == 2'sd0)  ?  16'sd16384 : COS_DELTA;
    wire signed [15:0] sz = (rot_z == 2'sd1)  ?  SIN_DELTA :
                            (rot_z == -2'sd1) ? -SIN_DELTA : 16'sd0;
    wire signed [15:0] cz = (rot_z == 2'sd0)  ?  16'sd16384 : COS_DELTA;

    wire [7:0]        num_tris;
    wire signed [8:0] bv0x, bv0y, bv0z;
    wire signed [8:0] bv1x, bv1y, bv1z;
    wire signed [8:0] bv2x, bv2y, bv2z;
    wire signed [8:0] bnx,  bny,  bnz;

    shape_lut #(
        .MAX_TRIS  (MAX_TRIS),
        .INIT_FILE (LUT_FILE)
    ) BASE_LUT (
        .clk      (clk),
        .shape_id (prev_shape_id),
        .tri_idx  (idx),
        .num_tris (num_tris),
        .v0x (bv0x), .v0y (bv0y), .v0z (bv0z),
        .v1x (bv1x), .v1y (bv1y), .v1z (bv1z),
        .v2x (bv2x), .v2y (bv2y), .v2z (bv2z),
        .nx  (bnx),  .ny  (bny),  .nz  (bnz)
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
            mat_row_mul = acc[22:14];
        end
    endfunction

    (* ramstyle = "M10K" *) reg [107:0] updated_lut [0:MAX_TRIS-1];
    reg [107:0] upd_rdata;

    wire upd_we = (state == S_MATRIX_MUL) && (sub == 2'd1);
    wire [107:0] upd_wdata = load_base ? {
        bv0x, bv0y, bv0z,
        bv1x, bv1y, bv1z,
        bv2x, bv2y, bv2z,
        bnx,  bny,  bnz
    } : {
        mat_row_mul(r00, r01, r02, cv0x, cv0y, cv0z),
        mat_row_mul(r10, r11, r12, cv0x, cv0y, cv0z),
        mat_row_mul(r20, r21, r22, cv0x, cv0y, cv0z),
        mat_row_mul(r00, r01, r02, cv1x, cv1y, cv1z),
        mat_row_mul(r10, r11, r12, cv1x, cv1y, cv1z),
        mat_row_mul(r20, r21, r22, cv1x, cv1y, cv1z),
        mat_row_mul(r00, r01, r02, cv2x, cv2y, cv2z),
        mat_row_mul(r10, r11, r12, cv2x, cv2y, cv2z),
        mat_row_mul(r20, r21, r22, cv2x, cv2y, cv2z),
        mat_row_mul(r00, r01, r02, cur_nx, cur_ny, cur_nz),
        mat_row_mul(r10, r11, r12, cur_nx, cur_ny, cur_nz),
        mat_row_mul(r20, r21, r22, cur_nx, cur_ny, cur_nz)
    };

    always @(posedge clk) begin
        if (upd_we)
            updated_lut[idx] <= upd_wdata;
        upd_rdata <= updated_lut[idx];
    end

    wire signed [8:0] cv0x, cv0y, cv0z;
    wire signed [8:0] cv1x, cv1y, cv1z;
    wire signed [8:0] cv2x, cv2y, cv2z;
    wire signed [8:0] cur_nx, cur_ny, cur_nz;
    assign {cv0x, cv0y, cv0z, cv1x, cv1y, cv1z, cv2x, cv2y, cv2z, cur_nx, cur_ny, cur_nz} = upd_rdata;

    reg signed [10:0] scan_x, scan_y;

    wire signed [11:0] dx = LIGHT_X - scan_x;
    wire signed [11:0] dy = LIGHT_Y - cv0y;
    wire signed [11:0] dz = LIGHT_Z - scan_y;

    wire signed [19:0] dot_x   = cur_nx * dx;
    wire signed [19:0] dot_y   = cur_ny * dy;
    wire signed [19:0] dot_z   = cur_nz * dz;
    wire signed [21:0] dot_val = dot_x + dot_y + dot_z;

    wire [22:0] bright_scaled = dot_val[14:0] * (MAX_BRIGHT - MIN_BRIGHT);
    wire [7:0]  brightness =
        (dot_val <= 22'sd0)     ? MIN_BRIGHT :
        (dot_val >= 22'sd32768) ? MAX_BRIGHT :
                                  (MIN_BRIGHT + bright_scaled[22:15]);

    wire signed [8:0] min_x01 = (cv0x < cv1x) ? cv0x : cv1x;
    wire signed [8:0] min_x   = (min_x01 < cv2x) ? min_x01 : cv2x;
    wire signed [8:0] max_x01 = (cv0x > cv1x) ? cv0x : cv1x;
    wire signed [8:0] max_x   = (max_x01 > cv2x) ? max_x01 : cv2x;

    wire signed [8:0] min_y01 = (cv0z < cv1z) ? cv0z : cv1z;
    wire signed [8:0] min_y   = (min_y01 < cv2z) ? min_y01 : cv2z;
    wire signed [8:0] max_y01 = (cv0z > cv1z) ? cv0z : cv1z;
    wire signed [8:0] max_y   = (max_y01 > cv2z) ? max_y01 : cv2z;

    reg signed [10:0] box_min_x, box_max_x, box_max_y;

    wire signed [9:0] d10x = cv1x - cv0x;
    wire signed [9:0] d10y = cv1z - cv0z;
    wire signed [9:0] d21x = cv2x - cv1x;
    wire signed [9:0] d21y = cv2z - cv1z;
    wire signed [9:0] d02x = cv0x - cv2x;
    wire signed [9:0] d02y = cv0z - cv2z;

    wire signed [10:0] p0x = scan_x - cv0x;
    wire signed [10:0] p0y = scan_y - cv0z;
    wire signed [10:0] p1x = scan_x - cv1x;
    wire signed [10:0] p1y = scan_y - cv1z;
    wire signed [10:0] p2x = scan_x - cv2x;
    wire signed [10:0] p2y = scan_y - cv2z;

    wire signed [21:0] e0 = (d10x * p0y) - (d10y * p0x);
    wire signed [21:0] e1 = (d21x * p1y) - (d21y * p1x);
    wire signed [21:0] e2 = (d02x * p2y) - (d02y * p2x);

    wire inside_tri = ((e0 >= 22'sd0) && (e1 >= 22'sd0) && (e2 >= 22'sd0)) ||
                      ((e0 <= 22'sd0) && (e1 <= 22'sd0) && (e2 <= 22'sd0));

    always @(posedge clk) begin
        if (!resetn) begin
            state         <= S_IDLE;
            sub           <= 2'd0;
            idx           <= 8'd0;
            screen_dirty  <= 1'b0;
            prev_enable   <= 1'b0;
            prev_shape_id <= 4'd0;
            next_shape_id <= 4'd0;
            prev_rot_req  <= 1'b0;
            load_base     <= 1'b0;
            scan_x        <= 11'sd0;
            scan_y        <= 11'sd0;
            box_min_x     <= 11'sd0;
            box_max_x     <= 11'sd0;
            box_max_y     <= 11'sd0;
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
                        next_shape_id <= shape_id;
                        prev_rot_req  <= rot_req;
                        load_base     <= shape_changed;

                        r00 <=  cz;
                        r01 <= -$signed(sz_cx[29:14]);
                        r02 <=  $signed(sz_sx[29:14]);
                        r10 <=  sz;
                        r11 <=  $signed(cz_cx[29:14]);
                        r12 <= -$signed(cz_sx[29:14]);
                        r20 <=  16'sd0;
                        r21 <=  sx;
                        r22 <=  cx;

                        idx <= 8'd0;
                        sub <= 2'd0;

                        if (screen_dirty) begin
                            state <= S_CLEAR;
                        end else begin
                            prev_shape_id <= shape_id;
                            if (shape_enable)
                                state <= S_MATRIX_MUL;
                            else
                                done  <= 1'b1;
                        end
                    end
                end

                S_MATRIX_MUL: begin
                    if (sub == 2'd0) begin
                        sub <= 2'd1;
                    end else begin
                        sub <= 2'd0;
                        if (idx + 8'd1 >= num_tris) begin
                            idx   <= 8'd0;
                            state <= S_RENDER;
                        end else begin
                            idx <= idx + 8'd1;
                        end
                    end
                end

                S_CLEAR: begin
                    case (sub)
                        2'd0: begin
                            sub <= 2'd1;
                        end

                        2'd1: begin
                            if (cur_ny > 9'sd0) begin
                                scan_x    <= min_x;
                                scan_y    <= min_y;
                                box_min_x <= min_x;
                                box_max_x <= max_x;
                                box_max_y <= max_y;
                                sub       <= 2'd2;
                            end else begin
                                sub <= 2'd0;
                                if (idx + 8'd1 >= num_tris) begin
                                    idx           <= 8'd0;
                                    screen_dirty  <= 1'b0;
                                    prev_shape_id <= next_shape_id;
                                    if (prev_enable)
                                        state <= S_MATRIX_MUL;
                                    else begin
                                        done  <= 1'b1;
                                        state <= S_IDLE;
                                    end
                                end else begin
                                    idx <= idx + 8'd1;
                                end
                            end
                        end

                        2'd2: begin
                            if (inside_tri) begin
                                vga_x     <= scan_x[9:0];
                                vga_y     <= scan_y[8:0];
                                vga_color <= 24'd0;
                                vga_write <= 1'b1;
                            end

                            if (scan_x < box_max_x) begin
                                scan_x <= scan_x + 11'sd1;
                            end else if (scan_y < box_max_y) begin
                                scan_x <= box_min_x;
                                scan_y <= scan_y + 11'sd1;
                            end else begin
                                sub <= 2'd0;
                                if (idx + 8'd1 >= num_tris) begin
                                    idx           <= 8'd0;
                                    screen_dirty  <= 1'b0;
                                    prev_shape_id <= next_shape_id;
                                    if (prev_enable)
                                        state <= S_MATRIX_MUL;
                                    else begin
                                        done  <= 1'b1;
                                        state <= S_IDLE;
                                    end
                                end else begin
                                    idx <= idx + 8'd1;
                                end
                            end
                        end

                        default: sub <= 2'd0;
                    endcase
                end

                S_RENDER: begin
                    case (sub)
                        2'd0: begin
                            sub <= 2'd1;
                        end

                        2'd1: begin
                            if (cur_ny > 9'sd0) begin
                                scan_x     <= min_x;
                                scan_y     <= min_y;
                                box_min_x  <= min_x;
                                box_max_x  <= max_x;
                                box_max_y  <= max_y;
                                sub        <= 2'd2;
                            end else begin
                                sub <= 2'd0;
                                if (idx + 8'd1 >= num_tris) begin
                                    done  <= 1'b1;
                                    state <= S_IDLE;
                                end else begin
                                    idx <= idx + 8'd1;
                                end
                            end
                        end

                        2'd2: begin
                            if (inside_tri) begin
                                vga_x        <= 11'sd320 + scan_x;
                                vga_y        <= 11'sd240 + scan_y;
                                vga_color    <= {brightness, brightness, brightness};
                                vga_write    <= 1'b1;
                                screen_dirty <= 1'b1;
                            end

                            if (scan_x < box_max_x) begin
                                scan_x <= scan_x + 11'sd1;
                            end else if (scan_y < box_max_y) begin
                                scan_x <= box_min_x;
                                scan_y <= scan_y + 11'sd1;
                            end else begin
                                sub <= 2'd0;
                                if (idx + 8'd1 >= num_tris) begin
                                    done  <= 1'b1;
                                    state <= S_IDLE;
                                end else begin
                                    idx <= idx + 8'd1;
                                end
                            end
                        end

                        default: sub <= 2'd0;
                    endcase
                end
            endcase
        end
    end
endmodule

`default_nettype wire
