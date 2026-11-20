`default_nettype none

/*  Synthesizable 3D Platonic Solid Renderer (VGA - 640x480)
 *  Maintains a 3x3 world-space orientation matrix M (Q2.16) initialized to
 *  Rz(DEFAULT_ANGLE_Z) * Ry(DEFAULT_ANGLE_Y) * Rx(DEFAULT_ANGLE_X).
 *  Incremental rotation pulses (rot_x_pos/neg, rot_z_pos/neg) left-multiply M
 *  by Rx(+-ROT_STEP_DEG) or Rz(+-ROT_STEP_DEG) and re-orthonormalize M, so
 *  vertices and face normals always rotate around the fixed screen axes
 *  (X = horizontal screen axis, Z = vertical screen axis, Y = depth axis)
 *  rather than rotating the coordinate axes with the object.
 *
 *  Iterates through all triangles of the selected Platonic solid (shape_id),
 *  transforms normals and vertices by the latched frame matrix active_M,
 *  culls faces whose rotated normal vector Y points inward, projects rotated
 *  vertices as (x = rot_x, z = rot_z, ignoring rot_y), scans from the
 *  left-bottom to the top-right of each triangle's bounding box, checks
 *  point-in-triangle via 2D edge functions, and for each rendered point P on
 *  the triangle computes its 3D coordinate via barycentric interpolation,
 *  normalizes (LIGHT_POS - P) to a unit vector, computes per-pixel flux as
 *  dot(N_rot, normalize(LIGHT_POS - P)), and writes shaded pixels through
 *  vga_adapter / vga_board_adapter.
 *
 *  Supports dynamic shape switching and blanking at runtime:
 *    - When shape_enable is 0, clears the screen (if dirty) and renders nothing.
 *    - When shape_enable is 1 and shape_id changes, clears the screen (if dirty)
 *      and renders the newly selected shape.
 *
 *  Note: Uses clocked FSMs (always @(posedge clk)) rather than behavioral
 *  for-loops with @(posedge clk) so Quartus can elaborate and synthesize it.
 *
 *  shape_id encoding:
 *    3'd0 : Tetrahedron  ( 4 verts,  4 tris,  4 face groups)
 *    3'd1 : Octahedron   ( 6 verts,  8 tris,  8 face groups)
 *    3'd2 : Cube         ( 8 verts, 12 tris,  6 face groups)
 *    3'd3 : Icosahedron  (12 verts, 20 tris, 20 face groups)
 *    3'd4 : Dodecahedron (20 verts, 36 tris, 12 face groups)
 *
 *  Point light coordinate (LIGHT_POS_X, LIGHT_POS_Y, LIGHT_POS_Z):
 *    Signed Q6.14 3D world-space coordinate (1.0 = 16384, range [-32, +32)).
 *    Default: (+0.5, +2.0, -1.0).
 */
module platonic_renderer #(
    parameter               CULL_INWARD_NEG_Y    = 1,                // 1: cull ny <= 0; 0: cull ny >= 0
    parameter               COLOR_DEPTH          = 24,               // 24 (8-bit/ch smooth), 9, 6, or 3
    parameter [8:0]         DEFAULT_ANGLE_X      = 9'd0,             // Initial X rotation angle (degrees, 0..359)
    parameter [8:0]         DEFAULT_ANGLE_Y      = 9'd0,             // Initial Y rotation angle (degrees, 0..359)
    parameter [8:0]         DEFAULT_ANGLE_Z      = 9'd0,             // Initial Z rotation angle (degrees, 0..359)
    parameter [8:0]         ROT_STEP_DEG         = 9'd3,             // Incremental rotation step (degrees, 0..359)
    parameter signed [15:0] OBJ_SCALE_X          = 16'sd135,         // X radius (3/4 of 180 for 16:9 widescreen stretch)
    parameter signed [15:0] OBJ_SCALE_Z          = 16'sd180,         // Z/Y radius (3/8 of 480)
    parameter signed [19:0] LIGHT_POS_X          = 20'sd8192,        // Point light X in Q6.14 (+0.5, 1.0 = 16384)
    parameter signed [19:0] LIGHT_POS_Y          = 20'sd32768,       // Point light Y in Q6.14 (+2.0)
    parameter signed [19:0] LIGHT_POS_Z          = -20'sd16384,      // Point light Z in Q6.14 (-1.0)
    parameter [14:0]        LIGHT_BRIGHTNESS_MIN = 15'd1638,         // 10% brightness at flux = 0.0 (0.10 * 16384)
    parameter [14:0]        LIGHT_BRIGHTNESS_MAX = 15'd13107,        // 80% brightness at flux = 1.0 (0.80 * 16384)
    parameter [8:0]         BASE_COLOR_RGB9      = 9'b111_111_111    // Monotone base color (white -> grayscale)
) (
    input  wire                   clk,
    input  wire                   resetn,
    input  wire                   vga_sync,
    input  wire                   shape_enable, // 1: render shape_id, 0: blank screen
    input  wire [2:0]             shape_id,
    input  wire                   rot_x_pos,    // 1-cycle pulse: rotate +ROT_STEP_DEG around fixed screen X axis
    input  wire                   rot_x_neg,    // 1-cycle pulse: rotate -ROT_STEP_DEG around fixed screen X axis
    input  wire                   rot_z_pos,    // 1-cycle pulse: rotate +ROT_STEP_DEG around fixed screen Z axis
    input  wire                   rot_z_neg,    // 1-cycle pulse: rotate -ROT_STEP_DEG around fixed screen Z axis
    output reg  [9:0]             vga_x,
    output reg  [8:0]             vga_y,
    output reg  [COLOR_DEPTH-1:0] vga_color,
    output reg                    vga_write,
    output reg                    done
);

    // -------------------------------------------------------------------------
    // Custom 3D Screen Constants (hardcoded to 640x480, not from vga_adapter)
    // -------------------------------------------------------------------------
    // Center the unit-sphere object at (320, 240) with radii (OBJ_SCALE_X, OBJ_SCALE_Z).
    localparam signed [11:0] CX        = 12'sd320;
    localparam signed [11:0] CY        = 12'sd240;
    localparam signed [11:0] MAX_X_LIM = 12'sd639;
    localparam signed [11:0] MAX_Z_LIM = 12'sd479;

    localparam [8:0] INIT_ANGLE_X  = (DEFAULT_ANGLE_X >= 9'd360) ? (DEFAULT_ANGLE_X - 9'd360) : DEFAULT_ANGLE_X;
    localparam [8:0] INIT_ANGLE_Y  = (DEFAULT_ANGLE_Y >= 9'd360) ? (DEFAULT_ANGLE_Y - 9'd360) : DEFAULT_ANGLE_Y;
    localparam [8:0] INIT_ANGLE_Z  = (DEFAULT_ANGLE_Z >= 9'd360) ? (DEFAULT_ANGLE_Z - 9'd360) : DEFAULT_ANGLE_Z;
    localparam [8:0] ROT_STEP_NORM = (ROT_STEP_DEG    >= 9'd360) ? (ROT_STEP_DEG    - 9'd360) : ROT_STEP_DEG;

    localparam signed [17:0] Q16_ONE = 18'sd65536; // 1.0 in signed Q2.16

    // -------------------------------------------------------------------------
    // World-Space 3x3 Rotation Matrix Engine (signed Q2.16, fixed screen axes)
    // -------------------------------------------------------------------------
    // Left-multiplying M by Rx(dtheta) rotates every vertex/normal around the
    // fixed horizontal screen X axis (leaving its screen X unchanged).
    // Left-multiplying M by Rz(dtheta) rotates every vertex/normal around the
    // fixed vertical screen Z axis (leaving its screen Z unchanged).
    localparam [3:0]
        M_INIT_TRIG = 4'd0,  // Load sin/cos for DEFAULT_ANGLE_X/Y/Z and ROT_STEP_DEG
        M_ROT_STEP  = 4'd1,  // Left-multiply two rows of M by 2D rotation (3 cols x 2 substeps)
        M_ORTHO_D01 = 4'd2,  // d01 = dot(row0, row1)
        M_ORTHO_SUB = 4'd3,  // row1 = row1 - d01 * row0
        M_ORTHO_S0  = 4'd4,  // k0 = (3 - dot(row0, row0)) / 2
        M_ORTHO_N0  = 4'd5,  // row0 = row0 * k0
        M_ORTHO_S1  = 4'd6,  // k1 = (3 - dot(row1, row1)) / 2
        M_ORTHO_N1  = 4'd7,  // row1 = row1 * k1
        M_ORTHO_C0  = 4'd8,  // row2[0] = (row0 x row1)[0]
        M_ORTHO_C1  = 4'd9,  // row2[1] = (row0 x row1)[1]
        M_ORTHO_C2  = 4'd10, // row2[2] = (row0 x row1)[2]; commit mat_ver
        M_IDLE      = 4'd11; // Wait for pending X/Z screen-axis rotation steps

    localparam [1:0]
        AXIS_X = 2'd0,
        AXIS_Y = 2'd1,
        AXIS_Z = 2'd2;

    reg [3:0] mat_state;
    reg       mat_ready;
    reg [7:0] mat_ver;
    reg [2:0] trig_step;
    reg [1:0] init_phase;
    reg [1:0] rot_axis;
    reg [1:0] rot_col;
    reg       rot_sub;

    reg signed [15:0] init_sin_x, init_cos_x;
    reg signed [15:0] init_sin_y, init_cos_y;
    reg signed [15:0] init_sin_z, init_cos_z;
    reg signed [15:0] step_sin,   step_cos;
    reg signed [15:0] cur_sin,    cur_cos;

    // Live orientation matrix M (Q2.16)
    reg signed [17:0] m00, m01, m02; // Row 0 -> screen X
    reg signed [17:0] m10, m11, m12; // Row 1 -> depth Y
    reg signed [17:0] m20, m21, m22; // Row 2 -> screen Z
    reg signed [17:0] rot_tmp_a;
    reg signed [17:0] ortho_coeff;

    // Queued signed step counts so pulses arriving mid-update are never dropped
    reg signed [3:0] pend_x_cnt;
    reg signed [3:0] pend_z_cnt;

    wire signed [3:0] inc_x = (rot_x_pos && !rot_x_neg) ?  4'sd1 :
                              (rot_x_neg && !rot_x_pos) ? -4'sd1 : 4'sd0;
    wire signed [3:0] inc_z = (rot_z_pos && !rot_z_neg) ?  4'sd1 :
                              (rot_z_neg && !rot_z_pos) ? -4'sd1 : 4'sd0;

    wire mat_busy = !mat_ready || (mat_state != M_IDLE) ||
                    (pend_x_cnt != 4'sd0) || (pend_z_cnt != 4'sd0);

    reg  [8:0]         trig_angle;
    wire signed [15:0] sin_lut_val;

    always @(*) begin
        case (trig_step)
            3'd0:    trig_angle = INIT_ANGLE_X;
            3'd1:    trig_angle = INIT_ANGLE_X + 9'd90;
            3'd2:    trig_angle = INIT_ANGLE_Y;
            3'd3:    trig_angle = INIT_ANGLE_Y + 9'd90;
            3'd4:    trig_angle = INIT_ANGLE_Z;
            3'd5:    trig_angle = INIT_ANGLE_Z + 9'd90;
            3'd6:    trig_angle = ROT_STEP_NORM;
            3'd7:    trig_angle = ROT_STEP_NORM + 9'd90;
            default: trig_angle = 9'd0;
        endcase
    end

    sin_lut TRIG_LUT (
        .angle   (trig_angle),
        .sin_val (sin_lut_val)
    );

    // Select matrix row pair (rowA, rowB) at column rot_col for 2D left-rotation:
    //   AXIS_X: (row1, row2) -> (row1*c - row2*s, row1*s + row2*c)
    //   AXIS_Y: (row2, row0) -> (row2*c - row0*s, row2*s + row0*c)
    //   AXIS_Z: (row0, row1) -> (row0*c - row1*s, row0*s + row1*c)
    reg signed [17:0] rot_a_val, rot_b_val;
    always @(*) begin
        case ({rot_axis, rot_col})
            {AXIS_X, 2'd0}: begin rot_a_val = m10; rot_b_val = m20; end
            {AXIS_X, 2'd1}: begin rot_a_val = m11; rot_b_val = m21; end
            {AXIS_X, 2'd2}: begin rot_a_val = m12; rot_b_val = m22; end
            {AXIS_Y, 2'd0}: begin rot_a_val = m20; rot_b_val = m00; end
            {AXIS_Y, 2'd1}: begin rot_a_val = m21; rot_b_val = m01; end
            {AXIS_Y, 2'd2}: begin rot_a_val = m22; rot_b_val = m02; end
            {AXIS_Z, 2'd0}: begin rot_a_val = m00; rot_b_val = m10; end
            {AXIS_Z, 2'd1}: begin rot_a_val = m01; rot_b_val = m11; end
            {AXIS_Z, 2'd2}: begin rot_a_val = m02; rot_b_val = m12; end
            default:        begin rot_a_val = 18'sd0; rot_b_val = 18'sd0; end
        endcase
    end

    // Shared 3-multiplier datapath for matrix rotation and Gram-Schmidt/Newton orthonormalization
    reg signed [17:0] mu0, mu1, mu2;
    reg signed [17:0] mv0, mv1, mv2;

    wire signed [17:0] cur_cos_ext = {{2{cur_cos[15]}}, cur_cos};
    wire signed [17:0] cur_sin_ext = {{2{cur_sin[15]}}, cur_sin};

    always @(*) begin
        mu0 = 18'sd0; mv0 = 18'sd0;
        mu1 = 18'sd0; mv1 = 18'sd0;
        mu2 = 18'sd0; mv2 = 18'sd0;
        case (mat_state)
            M_ROT_STEP: begin
                mu0 = rot_a_val;
                mu1 = rot_b_val;
                if (!rot_sub) begin
                    mv0 = cur_cos_ext;
                    mv1 = cur_sin_ext;
                end else begin
                    mv0 = cur_sin_ext;
                    mv1 = cur_cos_ext;
                end
            end
            M_ORTHO_D01: begin
                mu0 = m00; mv0 = m10;
                mu1 = m01; mv1 = m11;
                mu2 = m02; mv2 = m12;
            end
            M_ORTHO_SUB: begin
                mu0 = m00; mv0 = ortho_coeff;
                mu1 = m01; mv1 = ortho_coeff;
                mu2 = m02; mv2 = ortho_coeff;
            end
            M_ORTHO_S0: begin
                mu0 = m00; mv0 = m00;
                mu1 = m01; mv1 = m01;
                mu2 = m02; mv2 = m02;
            end
            M_ORTHO_N0: begin
                mu0 = m00; mv0 = ortho_coeff;
                mu1 = m01; mv1 = ortho_coeff;
                mu2 = m02; mv2 = ortho_coeff;
            end
            M_ORTHO_S1: begin
                mu0 = m10; mv0 = m10;
                mu1 = m11; mv1 = m11;
                mu2 = m12; mv2 = m12;
            end
            M_ORTHO_N1: begin
                mu0 = m10; mv0 = ortho_coeff;
                mu1 = m11; mv1 = ortho_coeff;
                mu2 = m12; mv2 = ortho_coeff;
            end
            M_ORTHO_C0: begin
                mu0 = m01; mv0 = m12;
                mu1 = m02; mv1 = m11;
            end
            M_ORTHO_C1: begin
                mu0 = m02; mv0 = m10;
                mu1 = m00; mv1 = m12;
            end
            M_ORTHO_C2: begin
                mu0 = m00; mv0 = m11;
                mu1 = m01; mv1 = m10;
            end
            default: ;
        endcase
    end

    wire signed [35:0] mp0 = mu0 * mv0;
    wire signed [35:0] mp1 = mu1 * mv1;
    wire signed [35:0] mp2 = mu2 * mv2;

    // 2D rotation outputs (Q2.16 * Q2.14 -> Q2.16 via >> 14 with rounding)
    wire signed [36:0] rot_diff  = $signed({mp0[35], mp0}) - $signed({mp1[35], mp1}) + 37'sd8192;
    wire signed [36:0] rot_sum   = $signed({mp0[35], mp0}) + $signed({mp1[35], mp1}) + 37'sd8192;
    wire signed [17:0] rot_res_a = rot_diff[31:14];
    wire signed [17:0] rot_res_b = rot_sum[31:14];

    // Orthonormalization outputs (Q2.16 * Q2.16 -> Q2.16 via >> 16 with rounding)
    wire signed [37:0] dot_sum   = $signed({{2{mp0[35]}}, mp0}) +
                                   $signed({{2{mp1[35]}}, mp1}) +
                                   $signed({{2{mp2[35]}}, mp2}) + 38'sd32768;
    wire signed [17:0] dot_q16   = dot_sum[33:16];
    wire signed [19:0] nr_diff   = 20'sd196608 - $signed({{2{dot_q16[17]}}, dot_q16});
    wire signed [17:0] nr_scale  = nr_diff[18:1];

    wire signed [36:0] sc0_sum   = $signed({mp0[35], mp0}) + 37'sd32768;
    wire signed [36:0] sc1_sum   = $signed({mp1[35], mp1}) + 37'sd32768;
    wire signed [36:0] sc2_sum   = $signed({mp2[35], mp2}) + 37'sd32768;
    wire signed [17:0] sc0_q16   = sc0_sum[33:16];
    wire signed [17:0] sc1_q16   = sc1_sum[33:16];
    wire signed [17:0] sc2_q16   = sc2_sum[33:16];

    wire signed [36:0] cross_sum = $signed({mp0[35], mp0}) - $signed({mp1[35], mp1}) + 37'sd32768;
    wire signed [17:0] cross_q16 = cross_sum[33:16];

    // Saturating signed 4-bit pending step helper (clamps to [-7, +7])
    function signed [3:0] sat_add4;
        input signed [3:0] cur_val;
        input signed [3:0] delta;
        reg signed [4:0] tmp;
        begin
            tmp = $signed({cur_val[3], cur_val}) + $signed({delta[3], delta});
            if (tmp > 5'sd7)       sat_add4 = 4'sd7;
            else if (tmp < -5'sd7) sat_add4 = -4'sd7;
            else                   sat_add4 = tmp[3:0];
        end
    endfunction

    always @(posedge clk) begin
        if (!resetn) begin
            mat_state   <= M_INIT_TRIG;
            mat_ready   <= 1'b0;
            mat_ver     <= 8'd0;
            trig_step   <= 3'd0;
            init_phase  <= 2'd0;
            rot_axis    <= AXIS_X;
            rot_col     <= 2'd0;
            rot_sub     <= 1'b0;
            init_sin_x  <= 16'sd0;
            init_cos_x  <= 16'sd16384;
            init_sin_y  <= 16'sd0;
            init_cos_y  <= 16'sd16384;
            init_sin_z  <= 16'sd0;
            init_cos_z  <= 16'sd16384;
            step_sin    <= 16'sd0;
            step_cos    <= 16'sd16384;
            cur_sin     <= 16'sd0;
            cur_cos     <= 16'sd16384;
            m00         <= Q16_ONE; m01 <= 18'sd0;  m02 <= 18'sd0;
            m10         <= 18'sd0;  m11 <= Q16_ONE; m12 <= 18'sd0;
            m20         <= 18'sd0;  m21 <= 18'sd0;  m22 <= Q16_ONE;
            rot_tmp_a   <= 18'sd0;
            ortho_coeff <= 18'sd0;
            pend_x_cnt  <= 4'sd0;
            pend_z_cnt  <= 4'sd0;
        end else begin
            // Default: accumulate any incoming rotation step pulses
            pend_x_cnt <= sat_add4(pend_x_cnt, inc_x);
            pend_z_cnt <= sat_add4(pend_z_cnt, inc_z);

            case (mat_state)
                M_INIT_TRIG: begin
                    case (trig_step)
                        3'd0: begin init_sin_x <= sin_lut_val; trig_step <= 3'd1; end
                        3'd1: begin init_cos_x <= sin_lut_val; trig_step <= 3'd2; end
                        3'd2: begin init_sin_y <= sin_lut_val; trig_step <= 3'd3; end
                        3'd3: begin init_cos_y <= sin_lut_val; trig_step <= 3'd4; end
                        3'd4: begin init_sin_z <= sin_lut_val; trig_step <= 3'd5; end
                        3'd5: begin init_cos_z <= sin_lut_val; trig_step <= 3'd6; end
                        3'd6: begin step_sin   <= sin_lut_val; trig_step <= 3'd7; end
                        3'd7: begin
                            step_cos   <= sin_lut_val;
                            trig_step  <= 3'd0;
                            init_phase <= 2'd0;
                            rot_axis   <= AXIS_X;
                            rot_col    <= 2'd0;
                            rot_sub    <= 1'b0;
                            cur_sin    <= init_sin_x;
                            cur_cos    <= init_cos_x;
                            mat_state  <= M_ROT_STEP;
                        end
                    endcase
                end

                M_ROT_STEP: begin
                    if (!rot_sub) begin
                        rot_tmp_a <= rot_res_a;
                        rot_sub   <= 1'b1;
                    end else begin
                        case ({rot_axis, rot_col})
                            {AXIS_X, 2'd0}: begin m10 <= rot_tmp_a; m20 <= rot_res_b; end
                            {AXIS_X, 2'd1}: begin m11 <= rot_tmp_a; m21 <= rot_res_b; end
                            {AXIS_X, 2'd2}: begin m12 <= rot_tmp_a; m22 <= rot_res_b; end
                            {AXIS_Y, 2'd0}: begin m20 <= rot_tmp_a; m00 <= rot_res_b; end
                            {AXIS_Y, 2'd1}: begin m21 <= rot_tmp_a; m01 <= rot_res_b; end
                            {AXIS_Y, 2'd2}: begin m22 <= rot_tmp_a; m02 <= rot_res_b; end
                            {AXIS_Z, 2'd0}: begin m00 <= rot_tmp_a; m10 <= rot_res_b; end
                            {AXIS_Z, 2'd1}: begin m01 <= rot_tmp_a; m11 <= rot_res_b; end
                            {AXIS_Z, 2'd2}: begin m02 <= rot_tmp_a; m12 <= rot_res_b; end
                            default: ;
                        endcase

                        rot_sub <= 1'b0;
                        if (rot_col < 2'd2) begin
                            rot_col <= rot_col + 2'd1;
                        end else begin
                            rot_col <= 2'd0;
                            if (!mat_ready) begin
                                if (init_phase == 2'd0) begin
                                    init_phase <= 2'd1;
                                    rot_axis   <= AXIS_Y;
                                    cur_sin    <= init_sin_y;
                                    cur_cos    <= init_cos_y;
                                end else if (init_phase == 2'd1) begin
                                    init_phase <= 2'd2;
                                    rot_axis   <= AXIS_Z;
                                    cur_sin    <= init_sin_z;
                                    cur_cos    <= init_cos_z;
                                end else begin
                                    mat_state  <= M_ORTHO_D01;
                                end
                            end else begin
                                mat_state <= M_ORTHO_D01;
                            end
                        end
                    end
                end

                M_ORTHO_D01: begin
                    ortho_coeff <= dot_q16;
                    mat_state   <= M_ORTHO_SUB;
                end

                M_ORTHO_SUB: begin
                    m10       <= m10 - sc0_q16;
                    m11       <= m11 - sc1_q16;
                    m12       <= m12 - sc2_q16;
                    mat_state <= M_ORTHO_S0;
                end

                M_ORTHO_S0: begin
                    ortho_coeff <= nr_scale;
                    mat_state   <= M_ORTHO_N0;
                end

                M_ORTHO_N0: begin
                    m00       <= sc0_q16;
                    m01       <= sc1_q16;
                    m02       <= sc2_q16;
                    mat_state <= M_ORTHO_S1;
                end

                M_ORTHO_S1: begin
                    ortho_coeff <= nr_scale;
                    mat_state   <= M_ORTHO_N1;
                end

                M_ORTHO_N1: begin
                    m10       <= sc0_q16;
                    m11       <= sc1_q16;
                    m12       <= sc2_q16;
                    mat_state <= M_ORTHO_C0;
                end

                M_ORTHO_C0: begin
                    m20       <= cross_q16;
                    mat_state <= M_ORTHO_C1;
                end

                M_ORTHO_C1: begin
                    m21       <= cross_q16;
                    mat_state <= M_ORTHO_C2;
                end

                M_ORTHO_C2: begin
                    m22       <= cross_q16;
                    mat_ready <= 1'b1;
                    mat_ver   <= mat_ver + 8'd1;
                    mat_state <= M_IDLE;
                end

                M_IDLE: begin
                    if (pend_x_cnt > 4'sd0) begin
                        pend_x_cnt <= sat_add4(pend_x_cnt - 4'sd1, inc_x);
                        rot_axis   <= AXIS_X;
                        rot_col    <= 2'd0;
                        rot_sub    <= 1'b0;
                        cur_sin    <= step_sin;
                        cur_cos    <= step_cos;
                        mat_state  <= M_ROT_STEP;
                    end else if (pend_x_cnt < 4'sd0) begin
                        pend_x_cnt <= sat_add4(pend_x_cnt + 4'sd1, inc_x);
                        rot_axis   <= AXIS_X;
                        rot_col    <= 2'd0;
                        rot_sub    <= 1'b0;
                        cur_sin    <= -step_sin;
                        cur_cos    <= step_cos;
                        mat_state  <= M_ROT_STEP;
                    end else if (pend_z_cnt > 4'sd0) begin
                        pend_z_cnt <= sat_add4(pend_z_cnt - 4'sd1, inc_z);
                        rot_axis   <= AXIS_Z;
                        rot_col    <= 2'd0;
                        rot_sub    <= 1'b0;
                        cur_sin    <= step_sin;
                        cur_cos    <= step_cos;
                        mat_state  <= M_ROT_STEP;
                    end else if (pend_z_cnt < 4'sd0) begin
                        pend_z_cnt <= sat_add4(pend_z_cnt + 4'sd1, inc_z);
                        rot_axis   <= AXIS_Z;
                        rot_col    <= 2'd0;
                        rot_sub    <= 1'b0;
                        cur_sin    <= -step_sin;
                        cur_cos    <= step_cos;
                        mat_state  <= M_ROT_STEP;
                    end
                end

                default: begin
                    mat_state <= M_IDLE;
                end
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Custom Rasterizer FSM States (not from vga_adapter)
    // -------------------------------------------------------------------------
    localparam [3:0]
        S_IDLE         = 4'd0,  // Wait for vga_sync / check if clear or new shape needed
        S_CLEAR        = 4'd1,  // Clear 640x480 screen to black before blanking or redraw
        S_START_SHAPE  = 4'd2,  // Latch cycle so LUT outputs reflect active_shape_id
        S_INV_AREA     = 4'd3,  // Compute normalized reciprocal of |area2| for barycentric 3D point
        S_ROT_X        = 4'd4,  // Compute rotated X = row0 . v
        S_ROT_Y        = 4'd5,  // Compute rotated Y = row1 . v
        S_ROT_Z        = 4'd6,  // Compute rotated Z = row2 . v
        S_ROT_DONE     = 4'd7,  // Latch rotated normal or vertex
        S_SETUP_BBOX   = 4'd8,  // Compute clamped triangle bounding box; start at left-bottom
        S_RASTER       = 4'd9,  // Scan left-bottom -> top-right; feed per-pixel point-light pipeline
        S_NEXT_TRI     = 4'd10, // Advance to next triangle in the LUT
        S_DONE         = 4'd11, // All triangles processed; hold done until target changes
        S_RASTER_DRAIN = 4'd12; // Drain remaining in-flight pixels from point-light pipeline

    localparam [1:0]
        ROT_NORM = 2'd0,
        ROT_V0   = 2'd1,
        ROT_V1   = 2'd2,
        ROT_V2   = 2'd3;

    reg [3:0] state;
    reg       active_enable;
    reg [2:0] active_shape_id;
    reg [7:0] active_mat_ver;
    reg       screen_dirty;

    // Frame-latched snapshot of M so mid-frame key presses never tear the current frame
    reg signed [17:0] a00, a01, a02;
    reg signed [17:0] a10, a11, a12;
    reg signed [17:0] a20, a21, a22;

    // Shape/blanking changes abort an in-flight render immediately; rotation changes
    // wait for the current frame to finish in S_DONE before clearing and redrawing.
    wire shape_changed  = (shape_enable != active_enable) ||
                          (shape_enable && (shape_id != active_shape_id));
    wire rot_changed    = shape_enable && !mat_busy && (mat_ver != active_mat_ver);
    wire target_changed = shape_changed || rot_changed;

    // -------------------------------------------------------------------------
    // Custom Geometry & Normal LUT Instantiation (platonic_lut, not vga_adapter)
    // -------------------------------------------------------------------------
    reg  [4:0]         lut_vert_addr;
    reg  [5:0]         tri_idx;
    wire [4:0]         num_verts;
    wire [5:0]         num_tris;
    wire [4:0]         num_face_groups;
    wire signed [15:0] vx, vy, vz;
    wire signed [15:0] nx, ny, nz;
    wire [4:0]         tri_v0, tri_v1, tri_v2, tri_face_group;

    platonic_lut LUT (
        .shape_id       (active_shape_id),
        .vert_addr      (lut_vert_addr),
        .face_addr      (tri_face_group),
        .tri_addr       (tri_idx),
        .num_verts      (num_verts),
        .num_tris       (num_tris),
        .num_face_groups(num_face_groups),
        .vx             (vx),
        .vy             (vy),
        .vz             (vz),
        .nx             (nx),
        .ny             (ny),
        .nz             (nz),
        .tri_v0         (tri_v0),
        .tri_v1         (tri_v1),
        .tri_v2         (tri_v2),
        .tri_face_group (tri_face_group)
    );

    // -------------------------------------------------------------------------
    // 3D Matrix-Vector Transform Unit (active_M * v_lut -> signed Q2.14)
    // -------------------------------------------------------------------------
    reg  [1:0]         rot_target;
    reg  signed [15:0] rot_x, rot_y, rot_z;

    wire signed [15:0] rot_in_x = (rot_target == ROT_NORM) ? nx : vx;
    wire signed [15:0] rot_in_y = (rot_target == ROT_NORM) ? ny : vy;
    wire signed [15:0] rot_in_z = (rot_target == ROT_NORM) ? nz : vz;

    reg signed [17:0] mv_r0, mv_r1, mv_r2;
    always @(*) begin
        case (state)
            S_ROT_X: begin mv_r0 = a00; mv_r1 = a01; mv_r2 = a02; end
            S_ROT_Y: begin mv_r0 = a10; mv_r1 = a11; mv_r2 = a12; end
            S_ROT_Z: begin mv_r0 = a20; mv_r1 = a21; mv_r2 = a22; end
            default: begin mv_r0 = 18'sd0; mv_r1 = 18'sd0; mv_r2 = 18'sd0; end
        endcase
    end

    wire signed [33:0] mv_p0      = mv_r0 * rot_in_x;
    wire signed [33:0] mv_p1      = mv_r1 * rot_in_y;
    wire signed [33:0] mv_p2      = mv_r2 * rot_in_z;
    wire signed [35:0] mv_sum     = $signed({{2{mv_p0[33]}}, mv_p0}) +
                                    $signed({{2{mv_p1[33]}}, mv_p1}) +
                                    $signed({{2{mv_p2[33]}}, mv_p2}) +
                                    36'sd32768;
    wire signed [15:0] mv_out_q14 = mv_sum[31:16];

    // -------------------------------------------------------------------------
    // Custom 2D Projection (x = rot_x, z = rot_z, ignore rot_y), Culling & Edge Test
    // (not from vga_adapter)
    // -------------------------------------------------------------------------
    reg [4:0]             cur_v1_idx, cur_v2_idx;
    reg signed [15:0]     tri_nx, tri_ny, tri_nz; // Rotated outward unit face normal (Q2.14)
    reg signed [15:0]     v0_x, v0_y, v0_z;       // Rotated 3D triangle vertex 0 (Q2.14)
    reg signed [15:0]     v1_x, v1_y, v1_z;       // Rotated 3D triangle vertex 1 (Q2.14)
    reg signed [15:0]     v2_x, v2_y, v2_z;       // Rotated 3D triangle vertex 2 (Q2.14)
    reg signed [16:0]     dv02_x, dv02_y, dv02_z; // v0 - v2 (signed Q3.14)
    reg signed [16:0]     dv12_x, dv12_y, dv12_z; // v1 - v2 (signed Q3.14)
    reg signed [11:0]     x0, z0, x1, z1, x2, z2;
    reg signed [11:0]     bbox_min_x, bbox_max_x;
    reg signed [11:0]     bbox_min_z, bbox_max_z;
    reg signed [11:0]     scan_x, scan_z;

    // Normalized reciprocal of |area2| for barycentric 3D point interpolation:
    //   norm_area = abs_area2 << area_shift (in [2^23, 2^24))
    //   div_quot  = floor(2^38 / norm_area) (in [16384, 32768], 16-bit unsigned)
    reg [4:0]             area_shift;
    reg [23:0]            norm_area;
    reg [24:0]            div_rem;
    reg [15:0]            div_quot;
    reg [4:0]             div_cnt;

    // Direct 2D orthographic projection of rotated vertex: x = rot_x, z = rot_z.
    // Bit slice [25:14] divides Q2.14 (rot_x * OBJ_SCALE_X, rot_z * OBJ_SCALE_Z) by 2^14.
    wire signed [31:0] vx_scaled = rot_x * OBJ_SCALE_X;
    wire signed [31:0] vz_scaled = rot_z * OBJ_SCALE_Z;
    wire signed [11:0] proj_x    = CX + $signed(vx_scaled[25:14]);
    wire signed [11:0] proj_z    = CY + $signed(vz_scaled[25:14]);

    // Rotated normal vector Y inward check
    wire normal_inward = (CULL_INWARD_NEG_Y != 0) ? (rot_y <= 16'sd0) : (rot_y >= 16'sd0);

    // Triangle axis-aligned bounding box in (x, z), clamped to [0..639, 0..479]
    wire signed [11:0] min_x_01        = (x0 < x1) ? x0 : x1;
    wire signed [11:0] min_x_unclamped = (min_x_01 < x2) ? min_x_01 : x2;
    wire signed [11:0] max_x_01        = (x0 > x1) ? x0 : x1;
    wire signed [11:0] max_x_unclamped = (max_x_01 > x2) ? max_x_01 : x2;

    wire signed [11:0] min_z_01        = (z0 < z1) ? z0 : z1;
    wire signed [11:0] min_z_unclamped = (min_z_01 < z2) ? min_z_01 : z2;
    wire signed [11:0] max_z_01        = (z0 > z1) ? z0 : z1;
    wire signed [11:0] max_z_unclamped = (max_z_01 > z2) ? max_z_01 : z2;

    wire signed [11:0] min_x_clamped = (min_x_unclamped < 12'sd0) ? 12'sd0 :
                                       ((min_x_unclamped > MAX_X_LIM) ? MAX_X_LIM : min_x_unclamped);
    wire signed [11:0] max_x_clamped = (max_x_unclamped < 12'sd0) ? 12'sd0 :
                                       ((max_x_unclamped > MAX_X_LIM) ? MAX_X_LIM : max_x_unclamped);
    wire signed [11:0] min_z_clamped = (min_z_unclamped < 12'sd0) ? 12'sd0 :
                                       ((min_z_unclamped > MAX_Z_LIM) ? MAX_Z_LIM : min_z_unclamped);
    wire signed [11:0] max_z_clamped = (max_z_unclamped < 12'sd0) ? 12'sd0 :
                                       ((max_z_unclamped > MAX_Z_LIM) ? MAX_Z_LIM : max_z_unclamped);

    // Signed 2x area in (x, z) plane (0 indicates a degenerate edge-on triangle)
    wire signed [23:0] area2      = ((x1 - x0) * (z2 - z0)) - ((z1 - z0) * (x2 - x0));
    wire signed [23:0] abs_area_s = (area2 < 24'sd0) ? (-area2) : area2;
    wire [23:0]        abs_area2  = abs_area_s;

    // Leading-zero shift to normalize abs_area2 into [2^23, 2^24)
    reg [4:0] area_shift_comb;
    always @(*) begin
        if      (abs_area2[23]) area_shift_comb = 5'd0;
        else if (abs_area2[22]) area_shift_comb = 5'd1;
        else if (abs_area2[21]) area_shift_comb = 5'd2;
        else if (abs_area2[20]) area_shift_comb = 5'd3;
        else if (abs_area2[19]) area_shift_comb = 5'd4;
        else if (abs_area2[18]) area_shift_comb = 5'd5;
        else if (abs_area2[17]) area_shift_comb = 5'd6;
        else if (abs_area2[16]) area_shift_comb = 5'd7;
        else if (abs_area2[15]) area_shift_comb = 5'd8;
        else if (abs_area2[14]) area_shift_comb = 5'd9;
        else if (abs_area2[13]) area_shift_comb = 5'd10;
        else if (abs_area2[12]) area_shift_comb = 5'd11;
        else if (abs_area2[11]) area_shift_comb = 5'd12;
        else if (abs_area2[10]) area_shift_comb = 5'd13;
        else if (abs_area2[9])  area_shift_comb = 5'd14;
        else if (abs_area2[8])  area_shift_comb = 5'd15;
        else if (abs_area2[7])  area_shift_comb = 5'd16;
        else if (abs_area2[6])  area_shift_comb = 5'd17;
        else if (abs_area2[5])  area_shift_comb = 5'd18;
        else if (abs_area2[4])  area_shift_comb = 5'd19;
        else if (abs_area2[3])  area_shift_comb = 5'd20;
        else if (abs_area2[2])  area_shift_comb = 5'd21;
        else if (abs_area2[1])  area_shift_comb = 5'd22;
        else                    area_shift_comb = 5'd23;
    end
    wire [23:0] norm_area_comb = abs_area2 << area_shift_comb;

    // 2D edge functions at (scan_x, scan_z): point is inside or on boundary iff
    // all three cross products share the same sign.
    // Barycentric weights: w0 = e1 / area2 (for V0), w1 = e2 / area2 (for V1), w2 = e0 / area2 (for V2).
    wire signed [23:0] e0 = ((x1 - x0) * (scan_z - z0)) - ((z1 - z0) * (scan_x - x0));
    wire signed [23:0] e1 = ((x2 - x1) * (scan_z - z1)) - ((z2 - z1) * (scan_x - x1));
    wire signed [23:0] e2 = ((x0 - x2) * (scan_z - z2)) - ((z0 - z2) * (scan_x - x2));

    wire inside_tri = ((e0 >= 24'sd0) && (e1 >= 24'sd0) && (e2 >= 24'sd0)) ||
                      ((e0 <= 24'sd0) && (e1 <= 24'sd0) && (e2 <= 24'sd0));

    // -------------------------------------------------------------------------
    // Per-Pixel Point-Light Flux Pipeline (9 stages, 1 pixel/cycle throughput)
    //   For each inside_tri rendering point P = (px, py, pz):
    //     D = LIGHT_POS - P
    //     U = D / ||D||    (unit vector via even-shift norm + LUT + 1 NR step)
    //     flux = dot(N_rot, U)
    // -------------------------------------------------------------------------
    localparam signed [33:0] DOT_ONE_Q28  = 34'sd268435456;
    localparam signed [33:0] DOT_HALF_Q14 = 34'sd8192;

    // 1 / sqrt(M) initial-guess LUT for M in [1.0, 4.0) (idx = m[15:8] in 64..255).
    // Output is unsigned Q1.15 (1.0 = 32768), refined to <1.4 LSB by 1 NR iteration.
    function [15:0] inv_sqrt_lut_fn;
        input [7:0] idx;
        begin
            case (idx)
                8'd 64: inv_sqrt_lut_fn = 16'd32641; 8'd 65: inv_sqrt_lut_fn = 16'd32391; 8'd 66: inv_sqrt_lut_fn = 16'd32146; 8'd 67: inv_sqrt_lut_fn = 16'd31907;
                8'd 68: inv_sqrt_lut_fn = 16'd31673; 8'd 69: inv_sqrt_lut_fn = 16'd31445; 8'd 70: inv_sqrt_lut_fn = 16'd31221; 8'd 71: inv_sqrt_lut_fn = 16'd31002;
                8'd 72: inv_sqrt_lut_fn = 16'd30787; 8'd 73: inv_sqrt_lut_fn = 16'd30577; 8'd 74: inv_sqrt_lut_fn = 16'd30371; 8'd 75: inv_sqrt_lut_fn = 16'd30169;
                8'd 76: inv_sqrt_lut_fn = 16'd29972; 8'd 77: inv_sqrt_lut_fn = 16'd29778; 8'd 78: inv_sqrt_lut_fn = 16'd29587; 8'd 79: inv_sqrt_lut_fn = 16'd29401;
                8'd 80: inv_sqrt_lut_fn = 16'd29217; 8'd 81: inv_sqrt_lut_fn = 16'd29038; 8'd 82: inv_sqrt_lut_fn = 16'd28861; 8'd 83: inv_sqrt_lut_fn = 16'd28688;
                8'd 84: inv_sqrt_lut_fn = 16'd28518; 8'd 85: inv_sqrt_lut_fn = 16'd28350; 8'd 86: inv_sqrt_lut_fn = 16'd28186; 8'd 87: inv_sqrt_lut_fn = 16'd28024;
                8'd 88: inv_sqrt_lut_fn = 16'd27866; 8'd 89: inv_sqrt_lut_fn = 16'd27709; 8'd 90: inv_sqrt_lut_fn = 16'd27556; 8'd 91: inv_sqrt_lut_fn = 16'd27405;
                8'd 92: inv_sqrt_lut_fn = 16'd27256; 8'd 93: inv_sqrt_lut_fn = 16'd27110; 8'd 94: inv_sqrt_lut_fn = 16'd26966; 8'd 95: inv_sqrt_lut_fn = 16'd26825;
                8'd 96: inv_sqrt_lut_fn = 16'd26686; 8'd 97: inv_sqrt_lut_fn = 16'd26548; 8'd 98: inv_sqrt_lut_fn = 16'd26413; 8'd 99: inv_sqrt_lut_fn = 16'd26280;
                8'd100: inv_sqrt_lut_fn = 16'd26149; 8'd101: inv_sqrt_lut_fn = 16'd26020; 8'd102: inv_sqrt_lut_fn = 16'd25893; 8'd103: inv_sqrt_lut_fn = 16'd25767;
                8'd104: inv_sqrt_lut_fn = 16'd25644; 8'd105: inv_sqrt_lut_fn = 16'd25522; 8'd106: inv_sqrt_lut_fn = 16'd25402; 8'd107: inv_sqrt_lut_fn = 16'd25283;
                8'd108: inv_sqrt_lut_fn = 16'd25167; 8'd109: inv_sqrt_lut_fn = 16'd25051; 8'd110: inv_sqrt_lut_fn = 16'd24938; 8'd111: inv_sqrt_lut_fn = 16'd24826;
                8'd112: inv_sqrt_lut_fn = 16'd24715; 8'd113: inv_sqrt_lut_fn = 16'd24606; 8'd114: inv_sqrt_lut_fn = 16'd24498; 8'd115: inv_sqrt_lut_fn = 16'd24392;
                8'd116: inv_sqrt_lut_fn = 16'd24287; 8'd117: inv_sqrt_lut_fn = 16'd24184; 8'd118: inv_sqrt_lut_fn = 16'd24081; 8'd119: inv_sqrt_lut_fn = 16'd23980;
                8'd120: inv_sqrt_lut_fn = 16'd23881; 8'd121: inv_sqrt_lut_fn = 16'd23782; 8'd122: inv_sqrt_lut_fn = 16'd23685; 8'd123: inv_sqrt_lut_fn = 16'd23589;
                8'd124: inv_sqrt_lut_fn = 16'd23494; 8'd125: inv_sqrt_lut_fn = 16'd23400; 8'd126: inv_sqrt_lut_fn = 16'd23307; 8'd127: inv_sqrt_lut_fn = 16'd23216;
                8'd128: inv_sqrt_lut_fn = 16'd23125; 8'd129: inv_sqrt_lut_fn = 16'd23036; 8'd130: inv_sqrt_lut_fn = 16'd22947; 8'd131: inv_sqrt_lut_fn = 16'd22860;
                8'd132: inv_sqrt_lut_fn = 16'd22774; 8'd133: inv_sqrt_lut_fn = 16'd22688; 8'd134: inv_sqrt_lut_fn = 16'd22604; 8'd135: inv_sqrt_lut_fn = 16'd22520;
                8'd136: inv_sqrt_lut_fn = 16'd22437; 8'd137: inv_sqrt_lut_fn = 16'd22356; 8'd138: inv_sqrt_lut_fn = 16'd22275; 8'd139: inv_sqrt_lut_fn = 16'd22195;
                8'd140: inv_sqrt_lut_fn = 16'd22116; 8'd141: inv_sqrt_lut_fn = 16'd22037; 8'd142: inv_sqrt_lut_fn = 16'd21960; 8'd143: inv_sqrt_lut_fn = 16'd21883;
                8'd144: inv_sqrt_lut_fn = 16'd21808; 8'd145: inv_sqrt_lut_fn = 16'd21732; 8'd146: inv_sqrt_lut_fn = 16'd21658; 8'd147: inv_sqrt_lut_fn = 16'd21585;
                8'd148: inv_sqrt_lut_fn = 16'd21512; 8'd149: inv_sqrt_lut_fn = 16'd21440; 8'd150: inv_sqrt_lut_fn = 16'd21368; 8'd151: inv_sqrt_lut_fn = 16'd21298;
                8'd152: inv_sqrt_lut_fn = 16'd21228; 8'd153: inv_sqrt_lut_fn = 16'd21159; 8'd154: inv_sqrt_lut_fn = 16'd21090; 8'd155: inv_sqrt_lut_fn = 16'd21022;
                8'd156: inv_sqrt_lut_fn = 16'd20955; 8'd157: inv_sqrt_lut_fn = 16'd20888; 8'd158: inv_sqrt_lut_fn = 16'd20822; 8'd159: inv_sqrt_lut_fn = 16'd20757;
                8'd160: inv_sqrt_lut_fn = 16'd20692; 8'd161: inv_sqrt_lut_fn = 16'd20628; 8'd162: inv_sqrt_lut_fn = 16'd20564; 8'd163: inv_sqrt_lut_fn = 16'd20501;
                8'd164: inv_sqrt_lut_fn = 16'd20439; 8'd165: inv_sqrt_lut_fn = 16'd20377; 8'd166: inv_sqrt_lut_fn = 16'd20316; 8'd167: inv_sqrt_lut_fn = 16'd20255;
                8'd168: inv_sqrt_lut_fn = 16'd20195; 8'd169: inv_sqrt_lut_fn = 16'd20135; 8'd170: inv_sqrt_lut_fn = 16'd20076; 8'd171: inv_sqrt_lut_fn = 16'd20017;
                8'd172: inv_sqrt_lut_fn = 16'd19959; 8'd173: inv_sqrt_lut_fn = 16'd19902; 8'd174: inv_sqrt_lut_fn = 16'd19845; 8'd175: inv_sqrt_lut_fn = 16'd19788;
                8'd176: inv_sqrt_lut_fn = 16'd19732; 8'd177: inv_sqrt_lut_fn = 16'd19676; 8'd178: inv_sqrt_lut_fn = 16'd19621; 8'd179: inv_sqrt_lut_fn = 16'd19566;
                8'd180: inv_sqrt_lut_fn = 16'd19512; 8'd181: inv_sqrt_lut_fn = 16'd19458; 8'd182: inv_sqrt_lut_fn = 16'd19405; 8'd183: inv_sqrt_lut_fn = 16'd19352;
                8'd184: inv_sqrt_lut_fn = 16'd19299; 8'd185: inv_sqrt_lut_fn = 16'd19247; 8'd186: inv_sqrt_lut_fn = 16'd19196; 8'd187: inv_sqrt_lut_fn = 16'd19144;
                8'd188: inv_sqrt_lut_fn = 16'd19093; 8'd189: inv_sqrt_lut_fn = 16'd19043; 8'd190: inv_sqrt_lut_fn = 16'd18993; 8'd191: inv_sqrt_lut_fn = 16'd18943;
                8'd192: inv_sqrt_lut_fn = 16'd18894; 8'd193: inv_sqrt_lut_fn = 16'd18845; 8'd194: inv_sqrt_lut_fn = 16'd18797; 8'd195: inv_sqrt_lut_fn = 16'd18749;
                8'd196: inv_sqrt_lut_fn = 16'd18701; 8'd197: inv_sqrt_lut_fn = 16'd18653; 8'd198: inv_sqrt_lut_fn = 16'd18606; 8'd199: inv_sqrt_lut_fn = 16'd18560;
                8'd200: inv_sqrt_lut_fn = 16'd18513; 8'd201: inv_sqrt_lut_fn = 16'd18467; 8'd202: inv_sqrt_lut_fn = 16'd18422; 8'd203: inv_sqrt_lut_fn = 16'd18376;
                8'd204: inv_sqrt_lut_fn = 16'd18331; 8'd205: inv_sqrt_lut_fn = 16'd18287; 8'd206: inv_sqrt_lut_fn = 16'd18242; 8'd207: inv_sqrt_lut_fn = 16'd18198;
                8'd208: inv_sqrt_lut_fn = 16'd18155; 8'd209: inv_sqrt_lut_fn = 16'd18111; 8'd210: inv_sqrt_lut_fn = 16'd18068; 8'd211: inv_sqrt_lut_fn = 16'd18025;
                8'd212: inv_sqrt_lut_fn = 16'd17983; 8'd213: inv_sqrt_lut_fn = 16'd17941; 8'd214: inv_sqrt_lut_fn = 16'd17899; 8'd215: inv_sqrt_lut_fn = 16'd17857;
                8'd216: inv_sqrt_lut_fn = 16'd17816; 8'd217: inv_sqrt_lut_fn = 16'd17775; 8'd218: inv_sqrt_lut_fn = 16'd17734; 8'd219: inv_sqrt_lut_fn = 16'd17694;
                8'd220: inv_sqrt_lut_fn = 16'd17654; 8'd221: inv_sqrt_lut_fn = 16'd17614; 8'd222: inv_sqrt_lut_fn = 16'd17574; 8'd223: inv_sqrt_lut_fn = 16'd17535;
                8'd224: inv_sqrt_lut_fn = 16'd17496; 8'd225: inv_sqrt_lut_fn = 16'd17457; 8'd226: inv_sqrt_lut_fn = 16'd17418; 8'd227: inv_sqrt_lut_fn = 16'd17380;
                8'd228: inv_sqrt_lut_fn = 16'd17342; 8'd229: inv_sqrt_lut_fn = 16'd17304; 8'd230: inv_sqrt_lut_fn = 16'd17267; 8'd231: inv_sqrt_lut_fn = 16'd17229;
                8'd232: inv_sqrt_lut_fn = 16'd17192; 8'd233: inv_sqrt_lut_fn = 16'd17155; 8'd234: inv_sqrt_lut_fn = 16'd17119; 8'd235: inv_sqrt_lut_fn = 16'd17082;
                8'd236: inv_sqrt_lut_fn = 16'd17046; 8'd237: inv_sqrt_lut_fn = 16'd17010; 8'd238: inv_sqrt_lut_fn = 16'd16974; 8'd239: inv_sqrt_lut_fn = 16'd16939;
                8'd240: inv_sqrt_lut_fn = 16'd16904; 8'd241: inv_sqrt_lut_fn = 16'd16869; 8'd242: inv_sqrt_lut_fn = 16'd16834; 8'd243: inv_sqrt_lut_fn = 16'd16799;
                8'd244: inv_sqrt_lut_fn = 16'd16765; 8'd245: inv_sqrt_lut_fn = 16'd16731; 8'd246: inv_sqrt_lut_fn = 16'd16697; 8'd247: inv_sqrt_lut_fn = 16'd16663;
                8'd248: inv_sqrt_lut_fn = 16'd16629; 8'd249: inv_sqrt_lut_fn = 16'd16596; 8'd250: inv_sqrt_lut_fn = 16'd16563; 8'd251: inv_sqrt_lut_fn = 16'd16530;
                8'd252: inv_sqrt_lut_fn = 16'd16497; 8'd253: inv_sqrt_lut_fn = 16'd16465; 8'd254: inv_sqrt_lut_fn = 16'd16432; 8'd255: inv_sqrt_lut_fn = 16'd16400;
                default: inv_sqrt_lut_fn = 16'd32768;
            endcase
        end
    endfunction

    function signed [15:0] clamp_q14;
        input signed [37:0] val;
        begin
            if (val > 38'sd16384)       clamp_q14 = 16'sd16384;
            else if (val < -38'sd16384) clamp_q14 = -16'sd16384;
            else                        clamp_q14 = val[15:0];
        end
    endfunction

    // Stage 1: latched pixel coords + normalized edge magnitudes
    reg        p1_valid;
    reg [9:0]  p1_x;
    reg [8:0]  p1_z;
    reg [19:0] p1_e1_s, p1_e2_s;

    // Stage 2: barycentric weights w0, w1 in unsigned Q1.15 [0..32768]
    reg        p2_valid;
    reg [9:0]  p2_x;
    reg [8:0]  p2_z;
    reg [15:0] p2_w0, p2_w1;

    // Stage 3: displacement vector D = LIGHT_POS - P (signed Q7.14)
    reg               p3_valid;
    reg [9:0]         p3_x;
    reg [8:0]         p3_z;
    reg signed [20:0] p3_dx, p3_dy, p3_dz;

    // Stage 4: squared distance ||D||^2 (unsigned 42-bit, scale 2^28)
    reg               p4_valid;
    reg [9:0]         p4_x;
    reg [8:0]         p4_z;
    reg signed [20:0] p4_dx, p4_dy, p4_dz;
    reg [41:0]        p4_dist_sq;

    // Stage 5: even-shift normalized mantissa m in [1.0, 4.0) (Q2.14) & LUT y0 (Q1.15)
    reg               p5_valid;
    reg [9:0]         p5_x;
    reg [8:0]         p5_z;
    reg signed [20:0] p5_dx, p5_dy, p5_dz;
    reg [15:0]        p5_m, p5_y0;
    reg [4:0]         p5_pair_idx;
    reg               p5_zero;

    // Stage 6: y0^2 (Q1.15)
    reg               p6_valid;
    reg [9:0]         p6_x;
    reg [8:0]         p6_z;
    reg signed [20:0] p6_dx, p6_dy, p6_dz;
    reg [15:0]        p6_m, p6_y0, p6_y0_sq;
    reg [4:0]         p6_pair_idx;
    reg               p6_zero;

    // Stage 7: Newton-Raphson factor (3 - m * y0^2) / 2 (Q1.15)
    reg               p7_valid;
    reg [9:0]         p7_x;
    reg [8:0]         p7_z;
    reg signed [20:0] p7_dx, p7_dy, p7_dz;
    reg [15:0]        p7_y0, p7_nr;
    reg [4:0]         p7_pair_idx;
    reg               p7_zero;

    // Stage 8: refined inv_sqrt(m) = y1 (Q1.15)
    reg               p8_valid;
    reg [9:0]         p8_x;
    reg [8:0]         p8_z;
    reg signed [20:0] p8_dx, p8_dy, p8_dz;
    reg [15:0]        p8_y1;
    reg [4:0]         p8_pair_idx;
    reg               p8_zero;

    // Stage 9: unit vector U = (LIGHT_POS - P) / ||LIGHT_POS - P|| (signed Q2.14)
    reg               p9_valid;
    reg [9:0]         p9_x;
    reg [8:0]         p9_z;
    reg signed [15:0] p9_ux, p9_uy, p9_uz;

    // Stage 10: clamped diffuse flux dot(N_rot, U) in unsigned Q1.14 [0..16384]
    reg        p10_valid;
    reg [9:0]  p10_x;
    reg [8:0]  p10_z;
    reg [14:0] p10_diffuse_q14;

    wire pipe_run  = ((state == S_RASTER) || (state == S_RASTER_DRAIN)) && !shape_changed;
    wire pipe_busy = p1_valid | p2_valid | p3_valid | p4_valid | p5_valid |
                     p6_valid | p7_valid | p8_valid | p9_valid | p10_valid;

    // Stage 0 -> Stage 1 combinational edge scaling
    wire signed [23:0] abs_e1_s = (e1 < 24'sd0) ? (-e1) : e1;
    wire signed [23:0] abs_e2_s = (e2 < 24'sd0) ? (-e2) : e2;
    wire [23:0]        abs_e1   = abs_e1_s;
    wire [23:0]        abs_e2   = abs_e2_s;
    wire [23:0]        e1_sh24  = abs_e1 << area_shift;
    wire [23:0]        e2_sh24  = abs_e2 << area_shift;

    // Stage 1 -> Stage 2 combinational barycentric weights (Q1.15)
    wire [35:0] w0_prod = (p1_e1_s * div_quot) + 36'd262144;
    wire [35:0] w1_prod = (p1_e2_s * div_quot) + 36'd262144;

    // Stage 2 -> Stage 3 combinational 3D point P = V2 + w0*(V0-V2) + w1*(V1-V2) & D = L - P
    wire signed [16:0] w0_s = $signed({1'b0, p2_w0});
    wire signed [16:0] w1_s = $signed({1'b0, p2_w1});

    wire signed [33:0] px_t0  = w0_s * dv02_x;
    wire signed [33:0] px_t1  = w1_s * dv12_x;
    wire signed [34:0] px_sum = $signed({px_t0[33], px_t0}) + $signed({px_t1[33], px_t1}) + 35'sd16384;
    wire signed [16:0] px_17  = $signed({v2_x[15], v2_x}) + $signed(px_sum[31:15]);

    wire signed [33:0] py_t0  = w0_s * dv02_y;
    wire signed [33:0] py_t1  = w1_s * dv12_y;
    wire signed [34:0] py_sum = $signed({py_t0[33], py_t0}) + $signed({py_t1[33], py_t1}) + 35'sd16384;
    wire signed [16:0] py_17  = $signed({v2_y[15], v2_y}) + $signed(py_sum[31:15]);

    wire signed [33:0] pz_t0  = w0_s * dv02_z;
    wire signed [33:0] pz_t1  = w1_s * dv12_z;
    wire signed [34:0] pz_sum = $signed({pz_t0[33], pz_t0}) + $signed({pz_t1[33], pz_t1}) + 35'sd16384;
    wire signed [16:0] pz_17  = $signed({v2_z[15], v2_z}) + $signed(pz_sum[31:15]);

    wire signed [20:0] d_x_comb = $signed({LIGHT_POS_X[19], LIGHT_POS_X}) - $signed({{4{px_17[16]}}, px_17});
    wire signed [20:0] d_y_comb = $signed({LIGHT_POS_Y[19], LIGHT_POS_Y}) - $signed({{4{py_17[16]}}, py_17});
    wire signed [20:0] d_z_comb = $signed({LIGHT_POS_Z[19], LIGHT_POS_Z}) - $signed({{4{pz_17[16]}}, pz_17});

    // Stage 3 -> Stage 4 combinational squared distance ||D||^2
    wire signed [41:0] dx_sq_s      = p3_dx * p3_dx;
    wire signed [41:0] dy_sq_s      = p3_dy * p3_dy;
    wire signed [41:0] dz_sq_s      = p3_dz * p3_dz;
    wire [41:0]        dist_sq_comb = dx_sq_s[41:0] + dy_sq_s[41:0] + dz_sq_s[41:0];

    // Stage 4 -> Stage 5 combinational even-shift normalization & LUT lookup
    reg [4:0] pair_idx_comb;
    always @(*) begin
        if      (p4_dist_sq[41:40] != 2'b00) pair_idx_comb = 5'd20;
        else if (p4_dist_sq[39:38] != 2'b00) pair_idx_comb = 5'd19;
        else if (p4_dist_sq[37:36] != 2'b00) pair_idx_comb = 5'd18;
        else if (p4_dist_sq[35:34] != 2'b00) pair_idx_comb = 5'd17;
        else if (p4_dist_sq[33:32] != 2'b00) pair_idx_comb = 5'd16;
        else if (p4_dist_sq[31:30] != 2'b00) pair_idx_comb = 5'd15;
        else if (p4_dist_sq[29:28] != 2'b00) pair_idx_comb = 5'd14;
        else if (p4_dist_sq[27:26] != 2'b00) pair_idx_comb = 5'd13;
        else if (p4_dist_sq[25:24] != 2'b00) pair_idx_comb = 5'd12;
        else if (p4_dist_sq[23:22] != 2'b00) pair_idx_comb = 5'd11;
        else if (p4_dist_sq[21:20] != 2'b00) pair_idx_comb = 5'd10;
        else if (p4_dist_sq[19:18] != 2'b00) pair_idx_comb = 5'd9;
        else if (p4_dist_sq[17:16] != 2'b00) pair_idx_comb = 5'd8;
        else if (p4_dist_sq[15:14] != 2'b00) pair_idx_comb = 5'd7;
        else if (p4_dist_sq[13:12] != 2'b00) pair_idx_comb = 5'd6;
        else if (p4_dist_sq[11:10] != 2'b00) pair_idx_comb = 5'd5;
        else if (p4_dist_sq[9:8]   != 2'b00) pair_idx_comb = 5'd4;
        else if (p4_dist_sq[7:6]   != 2'b00) pair_idx_comb = 5'd3;
        else if (p4_dist_sq[5:4]   != 2'b00) pair_idx_comb = 5'd2;
        else if (p4_dist_sq[3:2]   != 2'b00) pair_idx_comb = 5'd1;
        else                                 pair_idx_comb = 5'd0;
    end
    wire [5:0]  norm_sh = 6'd40 - {pair_idx_comb, 1'b0};
    wire [41:0] s_norm  = p4_dist_sq << norm_sh;
    wire [15:0] m_comb  = s_norm[41:26];
    wire [15:0] y0_comb = inv_sqrt_lut_fn(m_comb[15:8]);

    // Stage 5 -> Stage 6 -> Stage 7 -> Stage 8 Newton-Raphson inv_sqrt
    wire [31:0] y0_sq_prod = (p5_y0 * p5_y0) + 32'd16384;
    wire [31:0] my2_prod   = (p6_m * p6_y0_sq) + 32'd8192;
    wire [16:0] my2_q15    = my2_prod[30:14];
    wire [16:0] nr_num     = 17'd98304 - my2_q15 + 17'd1;
    wire [31:0] y1_prod    = (p7_y0 * p7_nr) + 32'd16384;

    // Stage 8 -> Stage 9 combinational unit vector U = D / ||D|| (signed Q2.14)
    wire signed [16:0] y1_s    = $signed({1'b0, p8_y1});
    wire signed [37:0] ux_prod = p8_dx * y1_s;
    wire signed [37:0] uy_prod = p8_dy * y1_s;
    wire signed [37:0] uz_prod = p8_dz * y1_s;
    wire [4:0]         u_sh    = p8_pair_idx + 5'd1;
    wire signed [37:0] u_rnd   = 38'sd1 << p8_pair_idx;
    wire signed [37:0] ux_sh   = (ux_prod + u_rnd) >>> u_sh;
    wire signed [37:0] uy_sh   = (uy_prod + u_rnd) >>> u_sh;
    wire signed [37:0] uz_sh   = (uz_prod + u_rnd) >>> u_sh;

    // Stage 9 -> Stage 10: Per-pixel flux = dot(N_rot, U) in Q6.28 -> clamped Q1.14 [0..16384]
    wire signed [31:0] flux_x     = tri_nx * p9_ux;
    wire signed [31:0] flux_y     = tri_ny * p9_uy;
    wire signed [31:0] flux_z     = tri_nz * p9_uz;
    wire signed [33:0] flux_q28   = $signed({{2{flux_x[31]}}, flux_x}) +
                                    $signed({{2{flux_y[31]}}, flux_y}) +
                                    $signed({{2{flux_z[31]}}, flux_z});
    wire signed [33:0] flux_rnd   = flux_q28 + DOT_HALF_Q14;

    wire [14:0] px_diffuse_q14 =
        (flux_q28 <= 34'sd0)      ? 15'd0 :
        (flux_q28 >= DOT_ONE_Q28) ? 15'd16384 :
                                    flux_rnd[28:14];

    // Stage 10 -> Output: Linearly map flux [0.0 .. 1.0] evenly onto
    // [LIGHT_BRIGHTNESS_MIN .. LIGHT_BRIGHTNESS_MAX] (default 10% .. 80%).
    localparam [14:0] LIGHT_BRIGHTNESS_SPAN =
        (LIGHT_BRIGHTNESS_MAX >= LIGHT_BRIGHTNESS_MIN) ?
        (LIGHT_BRIGHTNESS_MAX - LIGHT_BRIGHTNESS_MIN) : 15'd0;

    wire [29:0] px_scaled_prod    = (p10_diffuse_q14 * LIGHT_BRIGHTNESS_SPAN) + 30'd8192;
    wire [15:0] px_brightness_sum = {1'b0, LIGHT_BRIGHTNESS_MIN} + {1'b0, px_scaled_prod[28:14]};
    wire [14:0] px_brightness_q14 =
        (px_brightness_sum >= 16'd16384) ? 15'd16384 : px_brightness_sum[14:0];

    // -------------------------------------------------------------------------
    // Monotone Base Color Shading (not from vga_adapter)
    // -------------------------------------------------------------------------
    // Scale 8-bit expanded BASE_COLOR_RGB9 by per-pixel Q1.14 brightness [0..16384]
    wire [2:0]  base_r       = BASE_COLOR_RGB9[8:6];
    wire [2:0]  base_g       = BASE_COLOR_RGB9[5:3];
    wire [2:0]  base_b       = BASE_COLOR_RGB9[2:0];
    wire [7:0]  base_r8      = {base_r, base_r, base_r[2:1]};
    wire [7:0]  base_g8      = {base_g, base_g, base_g[2:1]};
    wire [7:0]  base_b8      = {base_b, base_b, base_b[2:1]};
    wire [22:0] lit_r8_prod  = (base_r8 * px_brightness_q14) + 23'd8192;
    wire [22:0] lit_g8_prod  = (base_g8 * px_brightness_q14) + 23'd8192;
    wire [22:0] lit_b8_prod  = (base_b8 * px_brightness_q14) + 23'd8192;
    wire [23:0] shaded_rgb24 = {lit_r8_prod[21:14], lit_g8_prod[21:14], lit_b8_prod[21:14]};

    wire [17:0] lit_r3_prod  = (base_r * px_brightness_q14) + 18'd8192;
    wire [17:0] lit_g3_prod  = (base_g * px_brightness_q14) + 18'd8192;
    wire [17:0] lit_b3_prod  = (base_b * px_brightness_q14) + 18'd8192;
    wire [8:0]  shaded_rgb9  = {lit_r3_prod[16:14], lit_g3_prod[16:14], lit_b3_prod[16:14]};

    wire [COLOR_DEPTH-1:0] px_color;
    generate
        if (COLOR_DEPTH == 24) begin : gen_col_24
            assign px_color = shaded_rgb24;
        end else if (COLOR_DEPTH == 9) begin : gen_col_9
            assign px_color = shaded_rgb9;
        end else if (COLOR_DEPTH == 6) begin : gen_col_6
            assign px_color = {shaded_rgb9[8:7], shaded_rgb9[5:4], shaded_rgb9[2:1]};
        end else begin : gen_col_3
            assign px_color = {shaded_rgb9[8], shaded_rgb9[5], shaded_rgb9[2]};
        end
    endgenerate

    // Clocked 10-stage per-pixel point-light pipeline registers
    always @(posedge clk) begin
        if (!resetn || !pipe_run) begin
            p1_valid        <= 1'b0;
            p1_x            <= 10'd0; p1_z <= 9'd0; p1_e1_s <= 20'd0; p1_e2_s <= 20'd0;
            p2_valid        <= 1'b0;
            p2_x            <= 10'd0; p2_z <= 9'd0; p2_w0 <= 16'd0; p2_w1 <= 16'd0;
            p3_valid        <= 1'b0;
            p3_x            <= 10'd0; p3_z <= 9'd0; p3_dx <= 21'sd0; p3_dy <= 21'sd0; p3_dz <= 21'sd0;
            p4_valid        <= 1'b0;
            p4_x            <= 10'd0; p4_z <= 9'd0; p4_dx <= 21'sd0; p4_dy <= 21'sd0; p4_dz <= 21'sd0;
            p4_dist_sq      <= 42'd0;
            p5_valid        <= 1'b0;
            p5_x            <= 10'd0; p5_z <= 9'd0; p5_dx <= 21'sd0; p5_dy <= 21'sd0; p5_dz <= 21'sd0;
            p5_m            <= 16'd0; p5_y0 <= 16'd0; p5_pair_idx <= 5'd0; p5_zero <= 1'b0;
            p6_valid        <= 1'b0;
            p6_x            <= 10'd0; p6_z <= 9'd0; p6_dx <= 21'sd0; p6_dy <= 21'sd0; p6_dz <= 21'sd0;
            p6_m            <= 16'd0; p6_y0 <= 16'd0; p6_y0_sq <= 16'd0; p6_pair_idx <= 5'd0; p6_zero <= 1'b0;
            p7_valid        <= 1'b0;
            p7_x            <= 10'd0; p7_z <= 9'd0; p7_dx <= 21'sd0; p7_dy <= 21'sd0; p7_dz <= 21'sd0;
            p7_y0           <= 16'd0; p7_nr <= 16'd0; p7_pair_idx <= 5'd0; p7_zero <= 1'b0;
            p8_valid        <= 1'b0;
            p8_x            <= 10'd0; p8_z <= 9'd0; p8_dx <= 21'sd0; p8_dy <= 21'sd0; p8_dz <= 21'sd0;
            p8_y1           <= 16'd0; p8_pair_idx <= 5'd0; p8_zero <= 1'b0;
            p9_valid        <= 1'b0;
            p9_x            <= 10'd0; p9_z <= 9'd0; p9_ux <= 16'sd0; p9_uy <= 16'sd0; p9_uz <= 16'sd0;
            p10_valid       <= 1'b0;
            p10_x           <= 10'd0; p10_z <= 9'd0; p10_diffuse_q14 <= 15'd0;
        end else begin
            // Stage 1: accept inside_tri pixels while actively scanning in S_RASTER
            p1_valid    <= (state == S_RASTER) && inside_tri;
            p1_x        <= scan_x[9:0];
            p1_z        <= scan_z[8:0];
            p1_e1_s     <= e1_sh24[23:4];
            p1_e2_s     <= e2_sh24[23:4];

            // Stage 2: barycentric weights w0, w1
            p2_valid    <= p1_valid;
            p2_x        <= p1_x;
            p2_z        <= p1_z;
            p2_w0       <= (w0_prod[35:19] > 17'd32768) ? 16'd32768 : w0_prod[34:19];
            p2_w1       <= (w1_prod[35:19] > 17'd32768) ? 16'd32768 : w1_prod[34:19];

            // Stage 3: 3D point P -> displacement D = LIGHT_POS - P
            p3_valid    <= p2_valid;
            p3_x        <= p2_x;
            p3_z        <= p2_z;
            p3_dx       <= d_x_comb;
            p3_dy       <= d_y_comb;
            p3_dz       <= d_z_comb;

            // Stage 4: squared distance ||D||^2
            p4_valid    <= p3_valid;
            p4_x        <= p3_x;
            p4_z        <= p3_z;
            p4_dx       <= p3_dx;
            p4_dy       <= p3_dy;
            p4_dz       <= p3_dz;
            p4_dist_sq  <= dist_sq_comb;

            // Stage 5: even-shift normalize & LUT initial guess y0
            p5_valid    <= p4_valid;
            p5_x        <= p4_x;
            p5_z        <= p4_z;
            p5_dx       <= p4_dx;
            p5_dy       <= p4_dy;
            p5_dz       <= p4_dz;
            p5_m        <= m_comb;
            p5_y0       <= y0_comb;
            p5_pair_idx <= pair_idx_comb;
            p5_zero     <= (p4_dist_sq == 42'd0);

            // Stage 6: y0^2
            p6_valid    <= p5_valid;
            p6_x        <= p5_x;
            p6_z        <= p5_z;
            p6_dx       <= p5_dx;
            p6_dy       <= p5_dy;
            p6_dz       <= p5_dz;
            p6_m        <= p5_m;
            p6_y0       <= p5_y0;
            p6_y0_sq    <= y0_sq_prod[30:15];
            p6_pair_idx <= p5_pair_idx;
            p6_zero     <= p5_zero;

            // Stage 7: NR factor (3 - m * y0^2) / 2
            p7_valid    <= p6_valid;
            p7_x        <= p6_x;
            p7_z        <= p6_z;
            p7_dx       <= p6_dx;
            p7_dy       <= p6_dy;
            p7_dz       <= p6_dz;
            p7_y0       <= p6_y0;
            p7_nr       <= nr_num[16:1];
            p7_pair_idx <= p6_pair_idx;
            p7_zero     <= p6_zero;

            // Stage 8: refined inv_sqrt(m) = y1
            p8_valid    <= p7_valid;
            p8_x        <= p7_x;
            p8_z        <= p7_z;
            p8_dx       <= p7_dx;
            p8_dy       <= p7_dy;
            p8_dz       <= p7_dz;
            p8_y1       <= y1_prod[30:15];
            p8_pair_idx <= p7_pair_idx;
            p8_zero     <= p7_zero;

            // Stage 9: unit vector U = D / ||D|| (signed Q2.14)
            p9_valid    <= p8_valid;
            p9_x        <= p8_x;
            p9_z        <= p8_z;
            p9_ux       <= p8_zero ? 16'sd0 : clamp_q14(ux_sh);
            p9_uy       <= p8_zero ? 16'sd0 : clamp_q14(uy_sh);
            p9_uz       <= p8_zero ? 16'sd0 : clamp_q14(uz_sh);

            // Stage 10: clamped diffuse flux dot(N_rot, U) in Q1.14
            p10_valid       <= p9_valid;
            p10_x           <= p9_x;
            p10_z           <= p9_z;
            p10_diffuse_q14 <= px_diffuse_q14;
        end
    end

    // -------------------------------------------------------------------------
    // Custom Synthesizable Rasterizer FSM (not from vga_adapter)
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (!resetn) begin
            state           <= S_IDLE;
            active_enable   <= 1'b0;
            active_shape_id <= 3'd0;
            active_mat_ver  <= 8'd0;
            screen_dirty    <= 1'b0;
            a00             <= Q16_ONE; a01 <= 18'sd0;  a02 <= 18'sd0;
            a10             <= 18'sd0;  a11 <= Q16_ONE; a12 <= 18'sd0;
            a20             <= 18'sd0;  a21 <= 18'sd0;  a22 <= Q16_ONE;
            rot_target      <= ROT_NORM;
            rot_x           <= 16'sd0;
            rot_y           <= 16'sd0;
            rot_z           <= 16'sd0;
            lut_vert_addr   <= 5'd0;
            tri_idx         <= 6'd0;
            cur_v1_idx      <= 5'd0;
            cur_v2_idx      <= 5'd0;
            tri_nx          <= 16'sd0;
            tri_ny          <= 16'sd0;
            tri_nz          <= 16'sd0;
            v0_x            <= 16'sd0; v0_y <= 16'sd0; v0_z <= 16'sd0;
            v1_x            <= 16'sd0; v1_y <= 16'sd0; v1_z <= 16'sd0;
            v2_x            <= 16'sd0; v2_y <= 16'sd0; v2_z <= 16'sd0;
            dv02_x          <= 17'sd0; dv02_y <= 17'sd0; dv02_z <= 17'sd0;
            dv12_x          <= 17'sd0; dv12_y <= 17'sd0; dv12_z <= 17'sd0;
            area_shift      <= 5'd0;
            norm_area       <= 24'd0;
            div_rem         <= 25'd0;
            div_quot        <= 16'd0;
            div_cnt         <= 5'd0;
            x0              <= 12'sd0;
            z0              <= 12'sd0;
            x1              <= 12'sd0;
            z1              <= 12'sd0;
            x2              <= 12'sd0;
            z2              <= 12'sd0;
            bbox_min_x      <= 12'sd0;
            bbox_max_x      <= 12'sd0;
            bbox_min_z      <= 12'sd0;
            bbox_max_z      <= 12'sd0;
            scan_x          <= 12'sd0;
            scan_z          <= 12'sd0;
            vga_x           <= 10'd0;
            vga_y           <= 9'd0;
            vga_color       <= {COLOR_DEPTH{1'b0}};
            vga_write       <= 1'b0;
            done            <= 1'b0;
        end else begin
            case (state)
                S_IDLE: begin
                    vga_write <= 1'b0;
                    done      <= 1'b0;
                    tri_idx   <= 6'd0;
                    if (vga_sync) begin
                        if (screen_dirty) begin
                            scan_x <= 12'sd0;
                            scan_z <= 12'sd0;
                            state  <= S_CLEAR;
                        end else if (shape_enable && !mat_busy) begin
                            active_enable   <= 1'b1;
                            active_shape_id <= shape_id;
                            active_mat_ver  <= mat_ver;
                            a00 <= m00; a01 <= m01; a02 <= m02;
                            a10 <= m10; a11 <= m11; a12 <= m12;
                            a20 <= m20; a21 <= m21; a22 <= m22;
                            state           <= S_START_SHAPE;
                        end else if (!shape_enable) begin
                            active_enable <= 1'b0;
                            state         <= S_IDLE;
                        end
                    end
                end

                S_CLEAR: begin
                    vga_x     <= scan_x[9:0];
                    vga_y     <= scan_z[8:0];
                    vga_color <= {COLOR_DEPTH{1'b0}};
                    vga_write <= 1'b1;
                    done      <= 1'b0;
                    if (scan_x < MAX_X_LIM) begin
                        scan_x <= scan_x + 12'sd1;
                    end else if (scan_z < MAX_Z_LIM) begin
                        scan_x <= 12'sd0;
                        scan_z <= scan_z + 12'sd1;
                    end else begin
                        screen_dirty  <= 1'b0;
                        active_enable <= 1'b0;
                        state         <= S_IDLE;
                    end
                end

                S_START_SHAPE: begin
                    vga_write <= 1'b0;
                    if (shape_changed) begin
                        state <= S_IDLE;
                    end else if (num_tris == 6'd0) begin
                        state <= S_DONE;
                    end else begin
                        rot_target <= ROT_NORM;
                        state      <= S_ROT_X;
                    end
                end

                S_ROT_X: begin
                    vga_write <= 1'b0;
                    if (shape_changed) begin
                        state <= S_IDLE;
                    end else begin
                        rot_x <= mv_out_q14;
                        state <= S_ROT_Y;
                    end
                end

                S_ROT_Y: begin
                    vga_write <= 1'b0;
                    if (shape_changed) begin
                        state <= S_IDLE;
                    end else begin
                        rot_y <= mv_out_q14;
                        state <= S_ROT_Z;
                    end
                end

                S_ROT_Z: begin
                    vga_write <= 1'b0;
                    if (shape_changed) begin
                        state <= S_IDLE;
                    end else begin
                        rot_z <= mv_out_q14;
                        state <= S_ROT_DONE;
                    end
                end

                S_ROT_DONE: begin
                    vga_write <= 1'b0;
                    if (shape_changed) begin
                        state <= S_IDLE;
                    end else begin
                        case (rot_target)
                            ROT_NORM: begin
                                if (normal_inward) begin
                                    state <= S_NEXT_TRI;
                                end else begin
                                    cur_v1_idx    <= tri_v1;
                                    cur_v2_idx    <= tri_v2;
                                    tri_nx        <= rot_x;
                                    tri_ny        <= rot_y;
                                    tri_nz        <= rot_z;
                                    lut_vert_addr <= tri_v0;
                                    rot_target    <= ROT_V0;
                                    state         <= S_ROT_X;
                                end
                            end
                            ROT_V0: begin
                                x0            <= proj_x;
                                z0            <= proj_z;
                                v0_x          <= rot_x;
                                v0_y          <= rot_y;
                                v0_z          <= rot_z;
                                lut_vert_addr <= cur_v1_idx;
                                rot_target    <= ROT_V1;
                                state         <= S_ROT_X;
                            end
                            ROT_V1: begin
                                x1            <= proj_x;
                                z1            <= proj_z;
                                v1_x          <= rot_x;
                                v1_y          <= rot_y;
                                v1_z          <= rot_z;
                                lut_vert_addr <= cur_v2_idx;
                                rot_target    <= ROT_V2;
                                state         <= S_ROT_X;
                            end
                            ROT_V2: begin
                                x2    <= proj_x;
                                z2    <= proj_z;
                                v2_x  <= rot_x;
                                v2_y  <= rot_y;
                                v2_z  <= rot_z;
                                state <= S_SETUP_BBOX;
                            end
                        endcase
                    end
                end

                S_SETUP_BBOX: begin
                    vga_write <= 1'b0;
                    if (shape_changed) begin
                        state <= S_IDLE;
                    end else if ((area2 == 24'sd0) ||
                        (max_x_unclamped < 12'sd0) || (min_x_unclamped > MAX_X_LIM) ||
                        (max_z_unclamped < 12'sd0) || (min_z_unclamped > MAX_Z_LIM)) begin
                        state <= S_NEXT_TRI;
                    end else begin
                        bbox_min_x <= min_x_clamped;
                        bbox_max_x <= max_x_clamped;
                        bbox_min_z <= min_z_clamped;
                        bbox_max_z <= max_z_clamped;
                        // Start at left-bottom: (min_x, max_z)
                        scan_x     <= min_x_clamped;
                        scan_z     <= max_z_clamped;
                        // Precompute vertex deltas for barycentric 3D point interpolation
                        dv02_x     <= $signed({v0_x[15], v0_x}) - $signed({v2_x[15], v2_x});
                        dv02_y     <= $signed({v0_y[15], v0_y}) - $signed({v2_y[15], v2_y});
                        dv02_z     <= $signed({v0_z[15], v0_z}) - $signed({v2_z[15], v2_z});
                        dv12_x     <= $signed({v1_x[15], v1_x}) - $signed({v2_x[15], v2_x});
                        dv12_y     <= $signed({v1_y[15], v1_y}) - $signed({v2_y[15], v2_y});
                        dv12_z     <= $signed({v1_z[15], v1_z}) - $signed({v2_z[15], v2_z});
                        // Initialize 16-cycle restoring divider for floor(2^38 / norm_area)
                        area_shift <= area_shift_comb;
                        norm_area  <= norm_area_comb;
                        div_rem    <= 25'd8388608; // 2^23
                        div_quot   <= 16'd0;
                        div_cnt    <= 5'd15;
                        state      <= S_INV_AREA;
                    end
                end

                S_INV_AREA: begin
                    vga_write <= 1'b0;
                    if (shape_changed) begin
                        state <= S_IDLE;
                    end else begin
                        if (div_rem >= {1'b0, norm_area}) begin
                            div_quot <= {div_quot[14:0], 1'b1};
                            div_rem  <= (div_rem - {1'b0, norm_area}) << 1;
                        end else begin
                            div_quot <= {div_quot[14:0], 1'b0};
                            div_rem  <= div_rem << 1;
                        end

                        if (div_cnt == 5'd0)
                            state <= S_RASTER;
                        else
                            div_cnt <= div_cnt - 5'd1;
                    end
                end

                S_RASTER: begin
                    if (shape_changed) begin
                        vga_write <= 1'b0;
                        state     <= S_IDLE;
                    end else begin
                        if (p10_valid) begin
                            vga_x        <= p10_x;
                            vga_y        <= p10_z;
                            vga_color    <= px_color;
                            vga_write    <= 1'b1;
                            screen_dirty <= 1'b1;
                        end else begin
                            vga_write <= 1'b0;
                        end

                        // Scan left -> right, bottom -> top up to top-right (max_x, min_z)
                        if (scan_x < bbox_max_x) begin
                            scan_x <= scan_x + 12'sd1;
                        end else if (scan_z > bbox_min_z) begin
                            scan_x <= bbox_min_x;
                            scan_z <= scan_z - 12'sd1;
                        end else begin
                            state <= S_RASTER_DRAIN;
                        end
                    end
                end

                S_RASTER_DRAIN: begin
                    if (shape_changed) begin
                        vga_write <= 1'b0;
                        state     <= S_IDLE;
                    end else begin
                        if (p10_valid) begin
                            vga_x        <= p10_x;
                            vga_y        <= p10_z;
                            vga_color    <= px_color;
                            vga_write    <= 1'b1;
                            screen_dirty <= 1'b1;
                        end else begin
                            vga_write <= 1'b0;
                        end

                        if (!pipe_busy)
                            state <= S_NEXT_TRI;
                    end
                end

                S_NEXT_TRI: begin
                    vga_write <= 1'b0;
                    if (shape_changed) begin
                        state <= S_IDLE;
                    end else if (tri_idx + 6'd1 < num_tris) begin
                        tri_idx    <= tri_idx + 6'd1;
                        rot_target <= ROT_NORM;
                        state      <= S_ROT_X;
                    end else begin
                        state <= S_DONE;
                    end
                end

                S_DONE: begin
                    vga_write <= 1'b0;
                    if (target_changed) begin
                        done  <= 1'b0;
                        state <= S_IDLE;
                    end else begin
                        done  <= 1'b1;
                    end
                end

                default: begin
                    vga_write <= 1'b0;
                    state     <= S_IDLE;
                end
            endcase
        end
    end

endmodule

`default_nettype wire
