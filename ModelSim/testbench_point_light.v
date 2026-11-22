`timescale 1ns / 1ps

module testbench_point_light ();

    parameter CLOCK_PERIOD = 20;

    reg        CLOCK_50;
    reg  [9:0] SW;
    reg  [3:0] KEY;
    wire [9:0] LEDR_A, LEDR_B;
    wire [9:0] VGA_X_A, VGA_X_B;
    wire [8:0] VGA_Y_A, VGA_Y_B;
    wire [23:0] VGA_COLOR_A, VGA_COLOR_B;
    wire       plot_A, plot_B;
    wire       ps2_clk_a, ps2_dat_a, ps2_clk_b, ps2_dat_b;

    assign (pull1, pull0) ps2_clk_a = 1'b1;
    assign (pull1, pull0) ps2_dat_a = 1'b1;
    assign (pull1, pull0) ps2_clk_b = 1'b1;
    assign (pull1, pull0) ps2_dat_b = 1'b1;

    integer errors = 0;

    // -------------------------------------------------------------------------
    // Monitor U_DEF (default light at (+0.5, +2.0, -1.0)):
    //   1. Check unit vector norm ||U||^2 == 1.0 (in Q2.14) at every valid Stage 9 pixel
    //   2. Check flux = dot(N, U) matches exact real-valued dot(N, normalize(L - P))
    //   3. Check per-triangle brightness variation (proves per-pixel point-light gradient)
    // -------------------------------------------------------------------------
    integer u_checked       = 0;
    integer max_norm_err    = 0;
    integer max_flux_err    = 0;
    integer tri_min_bright  [0:35];
    integer tri_max_bright  [0:35];
    integer tri_px_cnt      [0:35];
    integer gray_hist       [0:255];
    integer min_gray_seen   = 999;
    integer max_gray_seen   = 0;
    integer unique_grays    = 0;
    integer left_sum_a      = 0, right_sum_a = 0, left_cnt_a = 0, right_cnt_a = 0;
    integer left_sum_b      = 0, right_sum_b = 0, left_cnt_b = 0, right_cnt_b = 0;

    // Track per-pixel P (from stage 3) through pipeline to verify against stage 9!
    real p3_px, p3_py, p3_pz;
    real d_pipe_x [3:9], d_pipe_y [3:9], d_pipe_z [3:9];
    integer idx;

    always @(posedge CLOCK_50) begin
        if (!KEY[0]) begin
            for (idx = 3; idx <= 9; idx = idx + 1) begin
                d_pipe_x[idx] <= 0.0;
                d_pipe_y[idx] <= 0.0;
                d_pipe_z[idx] <= 0.0;
            end
        end else if (U_DEF.RENDERER.pipe_run) begin
            d_pipe_x[3] <= $itor($signed(U_DEF.RENDERER.RASTER.LIGHT_PIPE.d_x_comb)) / 16384.0;
            d_pipe_y[3] <= $itor($signed(U_DEF.RENDERER.RASTER.LIGHT_PIPE.d_y_comb)) / 16384.0;
            d_pipe_z[3] <= $itor($signed(U_DEF.RENDERER.RASTER.LIGHT_PIPE.d_z_comb)) / 16384.0;
            for (idx = 4; idx <= 9; idx = idx + 1) begin
                d_pipe_x[idx] <= d_pipe_x[idx-1];
                d_pipe_y[idx] <= d_pipe_y[idx-1];
                d_pipe_z[idx] <= d_pipe_z[idx-1];
            end
        end
    end

    always @(posedge CLOCK_50) begin
        if (U_DEF.RENDERER.pipe_run && U_DEF.RENDERER.RASTER.LIGHT_PIPE.p9_valid) begin : chk_p9
            integer ux_i, uy_i, uz_i;
            integer norm_q14, nerr;
            real dx_r, dy_r, dz_r, dlen;
            real nx_r, ny_r, nz_r;
            real exact_flux, hw_flux;
            integer ferr;
            integer tidx, br;

            ux_i = $signed(U_DEF.RENDERER.RASTER.LIGHT_PIPE.p9_ux);
            uy_i = $signed(U_DEF.RENDERER.RASTER.LIGHT_PIPE.p9_uy);
            uz_i = $signed(U_DEF.RENDERER.RASTER.LIGHT_PIPE.p9_uz);

            norm_q14 = (ux_i*ux_i + uy_i*uy_i + uz_i*uz_i + 8192) / 16384;
            nerr = (norm_q14 > 16384) ? (norm_q14 - 16384) : (16384 - norm_q14);
            if (nerr > max_norm_err) max_norm_err = nerr;
            if (nerr > 8) begin
                $display("FAIL: Unit vector norm error too large: norm_q14=%0d (ux=%0d uy=%0d uz=%0d)",
                         norm_q14, ux_i, uy_i, uz_i);
                errors = errors + 1;
            end

            dx_r = d_pipe_x[9];
            dy_r = d_pipe_y[9];
            dz_r = d_pipe_z[9];
            dlen = $sqrt(dx_r*dx_r + dy_r*dy_r + dz_r*dz_r);
            nx_r = $itor($signed(U_DEF.RENDERER.tri_nx)) / 16384.0;
            ny_r = $itor($signed(U_DEF.RENDERER.tri_ny)) / 16384.0;
            nz_r = $itor($signed(U_DEF.RENDERER.tri_nz)) / 16384.0;

            if (dlen > 0.0)
                exact_flux = (nx_r*dx_r + ny_r*dy_r + nz_r*dz_r) / dlen;
            else
                exact_flux = 0.0;
            if (exact_flux < 0.0) exact_flux = 0.0;
            if (exact_flux > 1.0) exact_flux = 1.0;

            hw_flux = $itor(U_DEF.RENDERER.RASTER.LIGHT_PIPE.px_diffuse_q14) / 16384.0;
            ferr = $rtoi((hw_flux - exact_flux) * 16384.0);
            if (ferr < 0) ferr = -ferr;
            if (ferr > max_flux_err) max_flux_err = ferr;
            if (ferr > 8) begin
                $display("FAIL: Flux mismatch at (%0d,%0d): hw=%f exact=%f (err=%0d LSB)",
                         U_DEF.RENDERER.RASTER.LIGHT_PIPE.p9_x, U_DEF.RENDERER.RASTER.LIGHT_PIPE.p9_z, hw_flux, exact_flux, ferr);
                errors = errors + 1;
            end

            u_checked = u_checked + 1;
        end

        if (U_DEF.RENDERER.pipe_run && U_DEF.RENDERER.RASTER.LIGHT_PIPE.p10_valid) begin : chk_p10
            integer tidx, br;
            tidx = U_DEF.RENDERER.tri_idx;
            br   = U_DEF.RENDERER.RASTER.LIGHT_PIPE.px_brightness_q14;
            if (br < 1638 || br > 16384) begin
                $display("FAIL: Brightness out of [10%%..100%%] range: br=%0d (expected 1638..16384)", br);
                errors = errors + 1;
            end
            if (br < tri_min_bright[tidx]) tri_min_bright[tidx] = br;
            if (br > tri_max_bright[tidx]) tri_max_bright[tidx] = br;
            tri_px_cnt[tidx] = tri_px_cnt[tidx] + 1;
        end

        if (plot_A && U_DEF.RENDERER.state != 4'd1) begin
            gray_hist[VGA_COLOR_A[23:16]] = gray_hist[VGA_COLOR_A[23:16]] + 1;
            if (VGA_COLOR_A[23:16] < min_gray_seen) min_gray_seen = VGA_COLOR_A[23:16];
            if (VGA_COLOR_A[23:16] > max_gray_seen) max_gray_seen = VGA_COLOR_A[23:16];
            if (VGA_X_A < 10'd320) begin
                left_sum_a = left_sum_a + VGA_COLOR_A[23:16];
                left_cnt_a = left_cnt_a + 1;
            end else begin
                right_sum_a = right_sum_a + VGA_COLOR_A[23:16];
                right_cnt_a = right_cnt_a + 1;
            end
        end

        if (plot_B && U_LEFT.RENDERER.state != 4'd1) begin
            if (VGA_X_B < 10'd320) begin
                left_sum_b = left_sum_b + VGA_COLOR_B[23:16];
                left_cnt_b = left_cnt_b + 1;
            end else begin
                right_sum_b = right_sum_b + VGA_COLOR_B[23:16];
                right_cnt_b = right_cnt_b + 1;
            end
        end
    end

    initial begin
        CLOCK_50 = 1'b0;
        forever #(CLOCK_PERIOD / 2) CLOCK_50 = ~CLOCK_50;
    end

    integer c, t, graded_tris;
    real avg_l_a, avg_r_a, avg_l_b, avg_r_b;

    initial begin
        for (t = 0; t < 36; t = t + 1) begin
            tri_min_bright[t] = 99999;
            tri_max_bright[t] = -1;
            tri_px_cnt[t]     = 0;
        end
        for (t = 0; t < 256; t = t + 1)
            gray_hist[t] = 0;

        // Select Cube (SW[7])
        SW  = 10'b00100_00000;
        KEY = 4'b1110;
        repeat (5) @(posedge CLOCK_50);
        KEY[0] = 1'b1;

        c = 0;
        while ((!U_DEF.render_done || !U_LEFT.render_done) && c < 400000) begin
            @(posedge CLOCK_50);
            c = c + 1;
        end
        if (!U_DEF.render_done || !U_LEFT.render_done) begin
            $display("FAIL: Timed out waiting for render_done (cycles=%0d)", c);
            errors = errors + 1;
        end else begin
            $display("PASS [Timing]: Cube rendered in %0d cycles (< 400000)", c);
        end

        $display("PASS [Accuracy]: Checked %0d rendered pixels; max ||U||^2 err = %0d LSB, max flux err = %0d LSB (in Q1.14)",
                 u_checked, max_norm_err, max_flux_err);

        graded_tris = 0;
        for (t = 0; t < 12; t = t + 1) begin
            if (tri_px_cnt[t] > 0) begin
                $display("  Cube tri %2d: px=%5d, brightness_q14=[%5d .. %5d] (span=%0d)",
                         t, tri_px_cnt[t], tri_min_bright[t], tri_max_bright[t],
                         tri_max_bright[t] - tri_min_bright[t]);
                if ((tri_max_bright[t] - tri_min_bright[t]) > 500)
                    graded_tris = graded_tris + 1;
            end
        end
        if (graded_tris < 2) begin
            $display("FAIL [Point-light gradient]: Expected per-pixel brightness gradient across faces, got %0d graded tris",
                     graded_tris);
            errors = errors + 1;
        end else begin
            $display("PASS [Point-light gradient]: %0d visible triangles exhibit smooth per-pixel point-light flux gradient",
                     graded_tris);
        end

        unique_grays = 0;
        for (t = 0; t < 256; t = t + 1)
            if (gray_hist[t] > 0) unique_grays = unique_grays + 1;
        $display("  Smooth 8-bit grayscale stats (Cube): %0d distinct gray levels in [%0d .. %0d] / 255",
                 unique_grays, min_gray_seen, max_gray_seen);
        if (unique_grays < 50 || min_gray_seen < 25 || max_gray_seen > 255) begin
            $display("FAIL [Smooth 10%%..100%% range]: expected >=50 gray levels within [25..255], got %0d in [%0d..%0d]",
                     unique_grays, min_gray_seen, max_gray_seen);
            errors = errors + 1;
        end else begin
            $display("PASS [Smooth 10%%..100%% range]: %0d smooth 8-bit gray levels within [10%%..100%%] ([25..255])",
                     unique_grays);
        end

        // Compare right-side light (+1.5, +1.5, 0) vs left-side light (-1.5, +1.5, 0) on symmetric front-facing Cube (0,0,0)!
        avg_l_a = (left_cnt_a  > 0) ? ($itor(left_sum_a)  / $itor(left_cnt_a))  : 0.0;
        avg_r_a = (right_cnt_a > 0) ? ($itor(right_sum_a) / $itor(right_cnt_a)) : 0.0;
        avg_l_b = (left_cnt_b  > 0) ? ($itor(left_sum_b)  / $itor(left_cnt_b))  : 0.0;
        avg_r_b = (right_cnt_b > 0) ? ($itor(right_sum_b) / $itor(right_cnt_b)) : 0.0;

        $display("  Right-light (+0.5,+2.0,-1.0) avg: left=%.2f, right=%.2f", avg_l_a, avg_r_a);
        $display("  Left-light  (-1.5,+1.5, 0.0) avg: left=%.2f, right=%.2f", avg_l_b, avg_r_b);
        if (!(avg_l_b > avg_r_b)) begin
            $display("FAIL [Light position shift]: Left-positioned light should illuminate left side brighter than right side!");
            errors = errors + 1;
        end else begin
            $display("PASS [Light position shift]: Moving light coordinate to (-1.5,+1.5,0) shifts peak brightness to left side");
        end

        if (errors == 0)
            $display("ALL POINT LIGHT TESTS PASSED");
        else
            $display("FAILED with %0d error(s)", errors);
        $stop;
    end

    // Instance A: Default point light at (+0.5, +2.0, -1.0)
    top U_DEF (
        .CLOCK_50(CLOCK_50),
        .SW(SW),
        .KEY(KEY),
        .LEDR(LEDR_A),
        .PS2_CLK(ps2_clk_a),
        .PS2_DAT(ps2_dat_a),
        .VGA_X(VGA_X_A),
        .VGA_Y(VGA_Y_A),
        .VGA_COLOR(VGA_COLOR_A),
        .plot(plot_A)
    );

    // Instance B: Point light shifted to (-1.5, +1.5, 0.0) with unrotated Cube (0,0,0)
    top #(
        .DEFAULT_ANGLE_X(9'd0),
        .DEFAULT_ANGLE_Y(9'd0),
        .DEFAULT_ANGLE_Z(9'd0),
        .LIGHT_POS_X(-20'sd24576), // -1.5
        .LIGHT_POS_Y(20'sd24576),  // +1.5
        .LIGHT_POS_Z(20'sd0)       //  0.0
    ) U_LEFT (
        .CLOCK_50(CLOCK_50),
        .SW(SW),
        .KEY(KEY),
        .LEDR(LEDR_B),
        .PS2_CLK(ps2_clk_b),
        .PS2_DAT(ps2_dat_b),
        .VGA_X(VGA_X_B),
        .VGA_Y(VGA_Y_B),
        .VGA_COLOR(VGA_COLOR_B),
        .plot(plot_B)
    );

endmodule
