`timescale 1ns / 1ps

module testbench_wasd_rot ();

    parameter CLOCK_PERIOD = 20;

    reg        CLOCK_50;
    reg  [9:0] SW;
    reg  [3:0] KEY;
    wire [9:0] LEDR;
    wire [9:0] VGA_X;
    wire [8:0] VGA_Y;
    wire [23:0] VGA_COLOR;
    wire       plot;
    wire       ps2_clk, ps2_dat;
    reg        key_action = 0;
    reg  [7:0] scan_code  = 0;

    integer errors = 0;
    integer clear_writes = 0;
    integer shape_writes = 0;
    reg [31:0] frame_hash = 32'd0;
    reg [31:0] base_hash  = 32'd0;
    reg [31:0] rot_z_hash = 32'd0;
    reg [31:0] rot_x_hash = 32'd0;

    reg [3:0] prev_state;
    always @(posedge CLOCK_50) begin
        prev_state <= U1.RENDERER.state;
        if (plot) begin
            if (prev_state == 4'd1) begin
                clear_writes = clear_writes + 1;
            end else begin
                shape_writes = shape_writes + 1;
                frame_hash = frame_hash ^ ({13'd0, VGA_X, VGA_Y} * 32'd2654435761) ^ VGA_COLOR;
            end
        end
    end

    task reset_counters;
        begin
            clear_writes = 0;
            shape_writes = 0;
            frame_hash   = 32'd0;
        end
    endtask

    task send_byte;
        input [7:0] code;
        begin
            @(posedge CLOCK_50);
            key_action <= 1'b1; scan_code <= code;
            @(posedge CLOCK_50);
            key_action <= 1'b0;
            repeat (90) @(posedge CLOCK_50);
        end
    endtask

    task tap_key;
        input [7:0] code;
        begin
            send_byte(code);
            send_byte(8'hF0);
            send_byte(code);
        end
    endtask

    task wait_render_done;
        input integer max_cycles;
        integer c;
        begin
            c = 0;
            repeat (10) @(posedge CLOCK_50);
            while (!U1.render_done && c < max_cycles) begin
                @(posedge CLOCK_50);
                c = c + 1;
            end
            if (!U1.render_done) begin
                $display("FAIL: Timed out waiting for render_done after %0d cycles", max_cycles);
                errors = errors + 1;
            end
        end
    endtask

    task wait_mat_idle;
        integer c;
        begin
            c = 0;
            repeat (2) @(posedge CLOCK_50);
            while (U1.RENDERER.mat_busy && c < 2000) begin
                @(posedge CLOCK_50);
                c = c + 1;
            end
            if (U1.RENDERER.mat_busy) begin
                $display("FAIL: Timed out waiting for mat_busy=0");
                errors = errors + 1;
            end
        end
    endtask

    // Helper: check that the live 3x3 matrix M is orthonormal to within tolerance (Q2.16)
    task check_orthonormal;
        input [24*8-1:0] label;
        reg signed [63:0] r00, r01, r02, r10, r11, r12, r20, r21, r22;
        reg signed [63:0] d00, d11, d22, d01, d02, d12;
        reg signed [63:0] max_err, e;
        begin
            r00 = U1.RENDERER.m00; r01 = U1.RENDERER.m01; r02 = U1.RENDERER.m02;
            r10 = U1.RENDERER.m10; r11 = U1.RENDERER.m11; r12 = U1.RENDERER.m12;
            r20 = U1.RENDERER.m20; r21 = U1.RENDERER.m21; r22 = U1.RENDERER.m22;
            d00 = (r00*r00 + r01*r01 + r02*r02 + 64'sd32768) >>> 16;
            d11 = (r10*r10 + r11*r11 + r12*r12 + 64'sd32768) >>> 16;
            d22 = (r20*r20 + r21*r21 + r22*r22 + 64'sd32768) >>> 16;
            d01 = (r00*r10 + r01*r11 + r02*r12 + 64'sd32768) >>> 16;
            d02 = (r00*r20 + r01*r21 + r02*r22 + 64'sd32768) >>> 16;
            d12 = (r10*r20 + r11*r21 + r12*r22 + 64'sd32768) >>> 16;
            max_err = 0;
            e = (d00 > 64'sd65536) ? (d00 - 64'sd65536) : (64'sd65536 - d00); if (e > max_err) max_err = e;
            e = (d11 > 64'sd65536) ? (d11 - 64'sd65536) : (64'sd65536 - d11); if (e > max_err) max_err = e;
            e = (d22 > 64'sd65536) ? (d22 - 64'sd65536) : (64'sd65536 - d22); if (e > max_err) max_err = e;
            e = (d01 < 0) ? -d01 : d01; if (e > max_err) max_err = e;
            e = (d02 < 0) ? -d02 : d02; if (e > max_err) max_err = e;
            e = (d12 < 0) ? -d12 : d12; if (e > max_err) max_err = e;
            if (max_err > 64) begin
                $display("FAIL [%0s orthonormality]: max_err=%0d LSB (d00=%0d d11=%0d d22=%0d d01=%0d d02=%0d d12=%0d)",
                         label, max_err, d00, d11, d22, d01, d02, d12);
                errors = errors + 1;
            end else begin
                $display("PASS [%0s orthonormality]: max_err=%0d LSB in Q2.16", label, max_err);
            end
        end
    endtask

    // Helper: evaluate cube vertices under current M and check which screen coordinate stayed fixed
    task check_fixed_axis_step;
        input [28*8-1:0] label;
        input [7:0]      key_code;
        input            expect_x_fixed; // 1: X fixed (W/S around X); 0: Z fixed (A/D around Z)
        integer i, sx, sy, sz;
        integer vx_b [0:7], vy_b [0:7], vz_b [0:7];
        integer vx_a [0:7], vy_a [0:7], vz_a [0:7];
        integer max_dx, max_dz, dx, dz;
        integer m00_c, m01_c, m02_c, m10_c, m11_c, m12_c, m20_c, m21_c, m22_c;
        begin
            wait_mat_idle();
            m00_c = U1.RENDERER.m00; m01_c = U1.RENDERER.m01; m02_c = U1.RENDERER.m02;
            m10_c = U1.RENDERER.m10; m11_c = U1.RENDERER.m11; m12_c = U1.RENDERER.m12;
            m20_c = U1.RENDERER.m20; m21_c = U1.RENDERER.m21; m22_c = U1.RENDERER.m22;
            for (i = 0; i < 8; i = i + 1) begin
                sx = (i[0]) ? 9459 : -9459;
                sy = (i[1]) ? 9459 : -9459;
                sz = (i[2]) ? 9459 : -9459;
                vx_b[i] = (m00_c * sx + m01_c * sy + m02_c * sz + 32768) >>> 16;
                vy_b[i] = (m10_c * sx + m11_c * sy + m12_c * sz + 32768) >>> 16;
                vz_b[i] = (m20_c * sx + m21_c * sy + m22_c * sz + 32768) >>> 16;
            end

            tap_key(key_code);
            wait_mat_idle();

            m00_c = U1.RENDERER.m00; m01_c = U1.RENDERER.m01; m02_c = U1.RENDERER.m02;
            m10_c = U1.RENDERER.m10; m11_c = U1.RENDERER.m11; m12_c = U1.RENDERER.m12;
            m20_c = U1.RENDERER.m20; m21_c = U1.RENDERER.m21; m22_c = U1.RENDERER.m22;
            max_dx = 0;
            max_dz = 0;
            for (i = 0; i < 8; i = i + 1) begin
                sx = (i[0]) ? 9459 : -9459;
                sy = (i[1]) ? 9459 : -9459;
                sz = (i[2]) ? 9459 : -9459;
                vx_a[i] = (m00_c * sx + m01_c * sy + m02_c * sz + 32768) >>> 16;
                vy_a[i] = (m10_c * sx + m11_c * sy + m12_c * sz + 32768) >>> 16;
                vz_a[i] = (m20_c * sx + m21_c * sy + m22_c * sz + 32768) >>> 16;
                dx = (vx_a[i] >= vx_b[i]) ? (vx_a[i] - vx_b[i]) : (vx_b[i] - vx_a[i]);
                dz = (vz_a[i] >= vz_b[i]) ? (vz_a[i] - vz_b[i]) : (vz_b[i] - vz_a[i]);
                if (dx > max_dx) max_dx = dx;
                if (dz > max_dz) max_dz = dz;
            end

            if (expect_x_fixed) begin
                if (max_dx > 4 || max_dz < 500) begin
                    $display("FAIL [%0s]: expected fixed X (max_dx<=4, max_dz>=500), got max_dx=%0d max_dz=%0d",
                             label, max_dx, max_dz);
                    errors = errors + 1;
                end else begin
                    $display("PASS [%0s]: screen X fixed (max_dx=%0d), Z rotated (max_dz=%0d)",
                             label, max_dx, max_dz);
                end
            end else begin
                if (max_dz > 4 || max_dx < 500) begin
                    $display("FAIL [%0s]: expected fixed Z (max_dz<=4, max_dx>=500), got max_dx=%0d max_dz=%0d",
                             label, max_dx, max_dz);
                    errors = errors + 1;
                end else begin
                    $display("PASS [%0s]: screen Z fixed (max_dz=%0d), X rotated (max_dx=%0d)",
                             label, max_dz, max_dx);
                end
            end
        end
    endtask

    initial begin
        CLOCK_50 = 1'b0;
        forever #(CLOCK_PERIOD / 2) CLOCK_50 = ~CLOCK_50;
    end

    initial begin
        // Reset and select Cube (SW[7])
        SW  = 10'b00100_00000;
        KEY = 4'b1110;
        reset_counters();
        repeat (5) @(posedge CLOCK_50);
        KEY[0] = 1'b1;

        wait_render_done(400000);
        check_orthonormal("Initial default");
        base_hash = frame_hash;
        if (shape_writes == 0) begin
            $display("FAIL [Initial Cube]: 0 shape pixels drawn");
            errors = errors + 1;
        end else begin
            $display("PASS [Initial Cube]: %0d pixels drawn, hash=%08h", shape_writes, base_hash);
        end

        // 1. Tap 'a' (8'h1C): rotates vertices around fixed vertical Z axis (+5 deg)
        reset_counters();
        check_fixed_axis_step("Tap A (+Z fixed vert)", 8'h1C, 1'b0);
        wait_render_done(800000);
        rot_z_hash = frame_hash;
        if (clear_writes != 307200 || shape_writes == 0 || rot_z_hash === base_hash) begin
            $display("FAIL [Tap A redraw]: clear=%0d shape=%0d hash_same=%b",
                     clear_writes, shape_writes, (rot_z_hash === base_hash));
            errors = errors + 1;
        end else begin
            $display("PASS [Tap A redraw]: cleared=%0d, shape=%0d, pixels updated", clear_writes, shape_writes);
        end

        // Verify stays stopped after release
        begin : hold_check
            reg [7:0] ver_before;
            ver_before = U1.RENDERER.mat_ver;
            repeat (400) @(posedge CLOCK_50);
            if (U1.RENDERER.mat_ver !== ver_before) begin
                $display("FAIL [After A release]: mat_ver changed from %0d to %0d", ver_before, U1.RENDERER.mat_ver);
                errors = errors + 1;
            end else begin
                $display("PASS [After A release]: rotation stopped cleanly");
            end
        end

        // 2. Tap 'd' (8'h23): rotates around fixed vertical Z axis (-5 deg), returns to base image
        reset_counters();
        check_fixed_axis_step("Tap D (-Z fixed vert)", 8'h23, 1'b0);
        wait_render_done(800000);
        if (frame_hash !== base_hash) begin
            $display("FAIL [Tap D return]: hash=%08h base=%08h", frame_hash, base_hash);
            errors = errors + 1;
        end else begin
            $display("PASS [Tap D return]: returned to exact base frame hash %08h", frame_hash);
        end

        // 3. Tap 'w' (8'h1D): rotates vertices around fixed horizontal X axis (-5 deg)
        reset_counters();
        check_fixed_axis_step("Tap W (-X fixed horiz)", 8'h1D, 1'b1);
        wait_render_done(800000);
        rot_x_hash = frame_hash;
        if (clear_writes != 307200 || shape_writes == 0 || rot_x_hash === base_hash) begin
            $display("FAIL [Tap W redraw]: clear=%0d shape=%0d hash_same=%b",
                     clear_writes, shape_writes, (rot_x_hash === base_hash));
            errors = errors + 1;
        end else begin
            $display("PASS [Tap W redraw]: cleared=%0d, shape=%0d, pixels updated", clear_writes, shape_writes);
        end

        // 4. Tap 's' (8'h1B): rotates around fixed horizontal X axis (+5 deg), returns to base image
        reset_counters();
        check_fixed_axis_step("Tap S (+X fixed horiz)", 8'h1B, 1'b1);
        wait_render_done(800000);
        if (frame_hash !== base_hash) begin
            $display("FAIL [Tap S return]: hash=%08h base=%08h", frame_hash, base_hash);
            errors = errors + 1;
        end else begin
            $display("PASS [Tap S return]: returned to exact base frame hash %08h", frame_hash);
        end

        // 5. CRITICAL TEST: Rotate 90 deg around Z (18 x A), then press W -> MUST still rotate around
        //    fixed horizontal screen X axis (screen X unchanged for all vertices, NOT rotated body axis!)
        repeat (18) tap_key(8'h1C);
        wait_mat_idle();
        check_fixed_axis_step("After 90d Z, Tap W (-X)", 8'h1D, 1'b1);
        check_orthonormal("After 90d Z + W");

        // 6. CRITICAL TEST: Reset, rotate 90 deg around X (18 x W), then press A -> MUST still rotate
        //    around fixed vertical screen Z axis (screen Z unchanged for all vertices!)
        KEY[0] = 1'b0;
        repeat (5) @(posedge CLOCK_50);
        KEY[0] = 1'b1;
        wait_mat_idle();
        repeat (18) tap_key(8'h1D);
        wait_mat_idle();
        check_fixed_axis_step("After 90d X, Tap A (+Z)", 8'h1C, 1'b0);
        check_orthonormal("After 90d X + A");

        // 7. Hold 'a' + 'w' simultaneously for multiple ticks -> matrix updates repeatedly
        begin : multi_hold
            reg [7:0] ver_start;
            ver_start = U1.RENDERER.mat_ver;
            send_byte(8'h1C); // hold A
            send_byte(8'h1D); // hold W
            repeat (600) @(posedge CLOCK_50);
            send_byte(8'hF0); send_byte(8'h1C);
            send_byte(8'hF0); send_byte(8'h1D);
            wait_mat_idle();
            if (U1.RENDERER.mat_ver - ver_start < 8'd4) begin
                $display("FAIL [Hold A+W]: expected >=4 matrix updates, got %0d",
                         U1.RENDERER.mat_ver - ver_start);
                errors = errors + 1;
            end else begin
                $display("PASS [Hold A+W]: simultaneous rotation applied %0d updates",
                         U1.RENDERER.mat_ver - ver_start);
            end
            check_orthonormal("After Hold A+W");
        end

        // 8. Opposite keys cancel: hold A + D together -> matrix must not change during overlap
        begin : cancel_ad
            reg [7:0] ver_before;
            send_byte(8'h1C); // press A
            send_byte(8'h23); // press D too
            wait_mat_idle();
            ver_before = U1.RENDERER.mat_ver;
            repeat (400) @(posedge CLOCK_50);
            if (U1.RENDERER.mat_ver !== ver_before) begin
                $display("FAIL [Hold A+D cancel]: mat_ver changed from %0d to %0d",
                         ver_before, U1.RENDERER.mat_ver);
                errors = errors + 1;
            end else begin
                $display("PASS [Hold A+D cancel]: matrix held steady at ver=%0d", ver_before);
            end
            send_byte(8'hF0); send_byte(8'h1C);
            send_byte(8'hF0); send_byte(8'h23);
        end

        // 9. Opposite keys cancel: hold W + S together -> matrix must not change during overlap
        begin : cancel_ws
            reg [7:0] ver_before;
            send_byte(8'h1D); // press W
            send_byte(8'h1B); // press S too
            wait_mat_idle();
            ver_before = U1.RENDERER.mat_ver;
            repeat (400) @(posedge CLOCK_50);
            if (U1.RENDERER.mat_ver !== ver_before) begin
                $display("FAIL [Hold W+S cancel]: mat_ver changed from %0d to %0d",
                         ver_before, U1.RENDERER.mat_ver);
                errors = errors + 1;
            end else begin
                $display("PASS [Hold W+S cancel]: matrix held steady at ver=%0d", ver_before);
            end
            send_byte(8'hF0); send_byte(8'h1D);
            send_byte(8'hF0); send_byte(8'h1B);
        end

        if (errors == 0)
            $display("ALL WASD ROTATION TESTS PASSED");
        else
            $display("FAILED with %0d error(s)", errors);
        $stop;
    end

    // Override ROT_STEP_DEG=5 and ROT_REPEAT_CYCLES=250 for fast deterministic simulation
    top #(
        .ROT_STEP_DEG(9'd5),
        .ROT_REPEAT_CYCLES(24'd250)
    ) U1 (
        .CLOCK_50(CLOCK_50),
        .SW(SW),
        .KEY(KEY),
        .LEDR(LEDR),
        .PS2_CLK(ps2_clk),
        .PS2_DAT(ps2_dat),
        .VGA_X(VGA_X),
        .VGA_Y(VGA_Y),
        .VGA_COLOR(VGA_COLOR),
        .plot(plot)
    );

    PS2_keyboard U2 (CLOCK_50, KEY[0], key_action, scan_code, ps2_clk, ps2_dat);

endmodule
