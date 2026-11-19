`default_nettype none

/*  Synthesizable 3D Platonic Solid Renderer (VGA - 640x480)
 *  Iterates through all triangles of the selected Platonic solid (SHAPE_ID),
 *  culls faces whose normal vector Y points inward, projects vertices directly
 *  as (x = vx, z = vz, ignoring vy), scans from the left-bottom to the
 *  top-right of each triangle's bounding box, checks point-in-triangle via 2D
 *  edge functions, and writes matching pixels through vga_adapter.
 *
 *  Note: Uses a clocked FSM (always @(posedge clk)) rather than behavioral
 *  for-loops with @(posedge clk) so Quartus can elaborate and synthesize it.
 *
 *  SHAPE_ID encoding:
 *    3'd0 : Tetrahedron  ( 4 verts,  4 tris,  4 face groups)
 *    3'd1 : Octahedron   ( 6 verts,  8 tris,  8 face groups)
 *    3'd2 : Cube         ( 8 verts, 12 tris,  6 face groups)
 *    3'd3 : Icosahedron  (12 verts, 20 tris, 20 face groups)
 *    3'd4 : Dodecahedron (20 verts, 36 tris, 12 face groups)
 */
module platonic_renderer #(
    parameter [2:0] SHAPE_ID          = 3'd2,
    parameter       CULL_INWARD_NEG_Y = 1,    // 1: cull ny <= 0; 0: cull ny >= 0
    parameter       COLOR_DEPTH       = 9     // 9, 6, or 3
) (
    input  wire                   clk,
    input  wire                   resetn,
    input  wire                   vga_sync,
    output reg  [9:0]             vga_x,
    output reg  [8:0]             vga_y,
    output reg  [COLOR_DEPTH-1:0] vga_color,
    output reg                    vga_write,
    output reg                    done
);

    // -------------------------------------------------------------------------
    // Custom 3D Screen Constants (hardcoded to 640x480, not from vga_adapter)
    // -------------------------------------------------------------------------
    // Center the unit-sphere object at (320, 240) with radius 180 (3/8 of 480).
    localparam signed [11:0] CX        = 12'sd320;
    localparam signed [11:0] CY        = 12'sd240;
    localparam signed [15:0] OBJ_SCALE = 16'sd180;
    localparam signed [11:0] MAX_X_LIM = 12'sd639;
    localparam signed [11:0] MAX_Z_LIM = 12'sd479;

    // -------------------------------------------------------------------------
    // Custom Rasterizer FSM States (not from vga_adapter)
    // -------------------------------------------------------------------------
    localparam [3:0]
        S_IDLE       = 4'd0,  // Wait for vga_sync from vga_adapter before drawing
        S_CHECK_TRI  = 4'd1,  // Check face normal Y and cull inward-facing triangles
        S_LOAD_V0    = 4'd2,  // Latch projected (x0, z0) for vertex 0
        S_LOAD_V1    = 4'd3,  // Latch projected (x1, z1) for vertex 1
        S_LOAD_V2    = 4'd4,  // Latch projected (x2, z2) for vertex 2
        S_SETUP_BBOX = 4'd5,  // Compute clamped triangle bounding box; start at left-bottom
        S_RASTER     = 4'd6,  // Scan left-bottom -> top-right and test point-in-triangle
        S_NEXT_TRI   = 4'd7,  // Advance to next triangle in the LUT
        S_DONE       = 4'd8;  // All triangles processed; hold done high

    reg [3:0] state;

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
        .shape_id       (SHAPE_ID),
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
    // Custom 2D Projection (x = vx, z = vz, ignore vy), Culling & Edge Test
    // (not from vga_adapter)
    // -------------------------------------------------------------------------
    reg [4:0]         cur_v1_idx, cur_v2_idx, cur_face_group;
    reg signed [11:0] x0, z0, x1, z1, x2, z2;
    reg signed [11:0] bbox_min_x, bbox_max_x;
    reg signed [11:0] bbox_min_z, bbox_max_z;
    reg signed [11:0] scan_x, scan_z;

    // Direct 2D orthographic projection: x = x, z = z, ignore y (vy unused).
    // Bit slice [25:14] divides Q2.14 (vx * OBJ_SCALE, vz * OBJ_SCALE) by 2^14.
    wire signed [31:0] vx_scaled = vx * OBJ_SCALE;
    wire signed [31:0] vz_scaled = vz * OBJ_SCALE;
    wire signed [11:0] proj_x    = CX + vx_scaled[25:14];
    wire signed [11:0] proj_z    = CY + vz_scaled[25:14];

    // Normal vector Y inward check
    wire normal_inward = (CULL_INWARD_NEG_Y != 0) ? (ny <= 16'sd0) : (ny >= 16'sd0);

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
    wire signed [23:0] area2 = ((x1 - x0) * (z2 - z0)) - ((z1 - z0) * (x2 - x0));

    // 2D edge functions at (scan_x, scan_z): point is inside or on boundary iff
    // all three cross products share the same sign
    wire signed [23:0] e0 = ((x1 - x0) * (scan_z - z0)) - ((z1 - z0) * (scan_x - x0));
    wire signed [23:0] e1 = ((x2 - x1) * (scan_z - z1)) - ((z2 - z1) * (scan_x - x1));
    wire signed [23:0] e2 = ((x0 - x2) * (scan_z - z2)) - ((z0 - z2) * (scan_x - x2));

    wire inside_tri = ((e0 >= 24'sd0) && (e1 >= 24'sd0) && (e2 >= 24'sd0)) ||
                      ((e0 <= 24'sd0) && (e1 <= 24'sd0) && (e2 <= 24'sd0));

    // -------------------------------------------------------------------------
    // Custom Face-Group Color Palette (not from vga_adapter)
    // -------------------------------------------------------------------------
    reg [8:0] palette_rgb9;
    always @(*) begin
        case (cur_face_group)
            5'd0:  palette_rgb9 = 9'b111_010_010; // coral red
            5'd1:  palette_rgb9 = 9'b010_111_010; // bright green
            5'd2:  palette_rgb9 = 9'b010_101_111; // sky blue
            5'd3:  palette_rgb9 = 9'b111_111_010; // yellow
            5'd4:  palette_rgb9 = 9'b111_010_111; // magenta
            5'd5:  palette_rgb9 = 9'b010_111_111; // cyan
            5'd6:  palette_rgb9 = 9'b111_101_001; // orange
            5'd7:  palette_rgb9 = 9'b101_011_111; // violet
            5'd8:  palette_rgb9 = 9'b111_111_111; // white
            5'd9:  palette_rgb9 = 9'b101_111_010; // lime
            5'd10: palette_rgb9 = 9'b111_011_101; // rose
            5'd11: palette_rgb9 = 9'b011_111_101; // mint
            5'd12: palette_rgb9 = 9'b111_110_100; // warm gold
            5'd13: palette_rgb9 = 9'b100_110_111; // periwinkle
            5'd14: palette_rgb9 = 9'b111_100_100; // salmon
            5'd15: palette_rgb9 = 9'b100_111_111; // ice cyan
            5'd16: palette_rgb9 = 9'b110_111_100; // pale chartreuse
            5'd17: palette_rgb9 = 9'b111_100_111; // orchid
            5'd18: palette_rgb9 = 9'b110_101_010; // amber
            5'd19: palette_rgb9 = 9'b011_101_110; // cerulean
            default: palette_rgb9 = 9'b111_111_111;
        endcase
    end

    wire [COLOR_DEPTH-1:0] face_color =
        (COLOR_DEPTH == 9) ? palette_rgb9 :
        (COLOR_DEPTH == 6) ? {palette_rgb9[8:7], palette_rgb9[5:4], palette_rgb9[2:1]} :
                             {palette_rgb9[8],   palette_rgb9[5],   palette_rgb9[2]};

    // -------------------------------------------------------------------------
    // Custom Synthesizable Rasterizer FSM (not from vga_adapter)
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (!resetn) begin
            state          <= S_IDLE;
            lut_vert_addr  <= 5'd0;
            tri_idx        <= 6'd0;
            cur_v1_idx     <= 5'd0;
            cur_v2_idx     <= 5'd0;
            cur_face_group <= 5'd0;
            x0             <= 12'sd0;
            z0             <= 12'sd0;
            x1             <= 12'sd0;
            z1             <= 12'sd0;
            x2             <= 12'sd0;
            z2             <= 12'sd0;
            bbox_min_x     <= 12'sd0;
            bbox_max_x     <= 12'sd0;
            bbox_min_z     <= 12'sd0;
            bbox_max_z     <= 12'sd0;
            scan_x         <= 12'sd0;
            scan_z         <= 12'sd0;
            vga_x          <= 10'd0;
            vga_y          <= 9'd0;
            vga_color      <= {COLOR_DEPTH{1'b0}};
            vga_write      <= 1'b0;
            done           <= 1'b0;
        end else begin
            case (state)
                S_IDLE: begin
                    vga_write <= 1'b0;
                    done      <= 1'b0;
                    tri_idx   <= 6'd0;
                    if (vga_sync) begin
                        if (num_tris == 6'd0)
                            state <= S_DONE;
                        else
                            state <= S_CHECK_TRI;
                    end
                end

                S_CHECK_TRI: begin
                    vga_write <= 1'b0;
                    if (normal_inward) begin
                        state <= S_NEXT_TRI;
                    end else begin
                        cur_v1_idx     <= tri_v1;
                        cur_v2_idx     <= tri_v2;
                        cur_face_group <= tri_face_group;
                        lut_vert_addr  <= tri_v0;
                        state          <= S_LOAD_V0;
                    end
                end

                S_LOAD_V0: begin
                    x0            <= proj_x;
                    z0            <= proj_z;
                    lut_vert_addr <= cur_v1_idx;
                    state         <= S_LOAD_V1;
                end

                S_LOAD_V1: begin
                    x1            <= proj_x;
                    z1            <= proj_z;
                    lut_vert_addr <= cur_v2_idx;
                    state         <= S_LOAD_V2;
                end

                S_LOAD_V2: begin
                    x2    <= proj_x;
                    z2    <= proj_z;
                    state <= S_SETUP_BBOX;
                end

                S_SETUP_BBOX: begin
                    if ((area2 == 24'sd0) ||
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
                        state      <= S_RASTER;
                    end
                end

                S_RASTER: begin
                    if (inside_tri) begin
                        vga_x     <= scan_x[9:0];
                        vga_y     <= scan_z[8:0];
                        vga_color <= face_color;
                        vga_write <= 1'b1;
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
                        state <= S_NEXT_TRI;
                    end
                end

                S_NEXT_TRI: begin
                    vga_write <= 1'b0;
                    if (tri_idx + 6'd1 < num_tris) begin
                        tri_idx <= tri_idx + 6'd1;
                        state   <= S_CHECK_TRI;
                    end else begin
                        state <= S_DONE;
                    end
                end

                S_DONE: begin
                    vga_write <= 1'b0;
                    done      <= 1'b1;
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
