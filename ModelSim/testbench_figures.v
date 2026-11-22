`timescale 1ns / 1ps

module testbench_figures ();

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

    assign (pull1, pull0) ps2_clk = 1'b1;
    assign (pull1, pull0) ps2_dat = 1'b1;

    integer errors = 0;
    integer total_writes = 0;
    integer clear_writes = 0;
    integer shape_writes = 0;
    integer color_writes = 0;
    integer non_mono_writes = 0;
    integer min_x, max_x, min_y, max_y;

    // Virtual 640x480 framebuffer to capture exact RTL-rendered pixels for visual inspection
    reg [7:0] fb [0:307199];

    reg [3:0] prev_state;
    always @(posedge CLOCK_50) begin
        prev_state <= U1.RENDERER.state;
        if (plot) begin
            total_writes = total_writes + 1;
            fb[VGA_Y * 640 + VGA_X] = VGA_COLOR[23:16];
            if (prev_state == 4'd1) begin
                clear_writes = clear_writes + 1;
                if (VGA_COLOR !== 24'h000000) begin
                    $display("FAIL: Non-black pixel %06h written during S_CLEAR", VGA_COLOR);
                    errors = errors + 1;
                end
            end else begin
                shape_writes = shape_writes + 1;
                if (VGA_COLOR != 24'h000000)
                    color_writes = color_writes + 1;
                if (VGA_COLOR[23:16] !== VGA_COLOR[15:8] || VGA_COLOR[15:8] !== VGA_COLOR[7:0])
                    non_mono_writes = non_mono_writes + 1;
                if (VGA_X < min_x) min_x = VGA_X;
                if (VGA_X > max_x) max_x = VGA_X;
                if (VGA_Y < min_y) min_y = VGA_Y;
                if (VGA_Y > max_y) max_y = VGA_Y;
            end
        end
    end

    task clear_fb;
        integer k;
        begin
            for (k = 0; k < 307200; k = k + 1)
                fb[k] = 8'd0;
        end
    endtask

    task dump_pgm;
        input [256*8-1:0] filename;
`ifdef DUMP_PGM
        integer fd, y, x;
        begin
            fd = $fopen(filename, "wb");
            if (fd != 0) begin
                $fwrite(fd, "P5\n640 480\n255\n");
                for (y = 0; y < 480; y = y + 1) begin
                    for (x = 0; x < 640; x = x + 1) begin
                        $fwrite(fd, "%c", fb[y * 640 + x]);
                    end
                end
                $fclose(fd);
            end
        end
`else
        begin
        end
`endif
    endtask

    task reset_counters;
        begin
            total_writes = 0;
            clear_writes = 0;
            shape_writes = 0;
            color_writes = 0;
            non_mono_writes = 0;
            min_x = 1000;
            max_x = 0;
            min_y = 1000;
            max_y = 0;
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

    task check_depth_sorted;
        input [16*8-1:0] label;
        integer i, vc, sort_errs;
        begin
            vc = U1.RENDERER.vis_cnt;
            sort_errs = 0;
            for (i = 1; i < vc; i = i + 1) begin
                if (U1.RENDERER.DEPTH_SORTER.vis_tri_depth[i - 1] > U1.RENDERER.DEPTH_SORTER.vis_tri_depth[i]) begin
                    sort_errs = sort_errs + 1;
                end
            end
            if (vc == 0 || sort_errs != 0) begin
                $display("FAIL [%0s depth sort]: vis_cnt=%0d sort_errs=%0d", label, vc, sort_errs);
                errors = errors + 1;
            end else begin
                $display("PASS [%0s depth sort]: %0d visible tris strictly sorted back-to-front", label, vc);
            end
        end
    endtask

    initial begin
        CLOCK_50 = 1'b0;
        forever #(CLOCK_PERIOD / 2) CLOCK_50 = ~CLOCK_50;
    end

    initial begin
        clear_fb();
        SW  = 10'b00000_00000;
        KEY = 4'b1110;
        reset_counters();
        repeat (5) @(posedge CLOCK_50);
        KEY[0] = 1'b1;
        repeat (200) @(posedge CLOCK_50);

        // 1. SW[4] = Person (shape_id = 5)
        reset_counters();
        SW = 10'b00000_10000;
        wait_render_done(600000);
        if (U1.RENDERER.active_shape_id !== 4'd5 || clear_writes != 0 || shape_writes == 0 ||
            color_writes != shape_writes || non_mono_writes != 0) begin
            $display("FAIL [SW[4] Person]: id=%0d clear=%0d shape=%0d lit=%0d non_mono=%0d",
                     U1.RENDERER.active_shape_id, clear_writes, shape_writes, color_writes, non_mono_writes);
            errors = errors + 1;
        end else begin
            $display("PASS [SW[4] Person]: id=%0d pixels=%0d bbox=%0dx%0d",
                     U1.RENDERER.active_shape_id, shape_writes, max_x - min_x, max_y - min_y);
        end
        check_depth_sorted("Person");
        dump_pgm("rtl_sw4_person.pgm");

        // 2. Multiple switches across SW[5] + SW[4] -> hold Person, 0 writes
        reset_counters();
        SW = 10'b00001_10000;
        repeat (500) @(posedge CLOCK_50);
        if (total_writes != 0 || U1.RENDERER.active_shape_id !== 4'd5 || U1.render_done !== 1'b1) begin
            $display("FAIL [Multi SW[5]+SW[4]]: writes=%0d id=%0d done=%b",
                     total_writes, U1.RENDERER.active_shape_id, U1.render_done);
            errors = errors + 1;
        end else begin
            $display("PASS [Multi SW[5]+SW[4]]: held id=%0d with 0 writes", U1.RENDERER.active_shape_id);
        end

        // 3. SW[3] = NVIDIA Logo (shape_id = 6)
        reset_counters();
        SW = 10'b00000_01000;
        wait_render_done(900000);
        if (U1.RENDERER.active_shape_id !== 4'd6 || clear_writes != 307200 || shape_writes == 0 ||
            color_writes != shape_writes || non_mono_writes != 0) begin
            $display("FAIL [SW[3] NVIDIA Logo]: id=%0d clear=%0d shape=%0d lit=%0d non_mono=%0d",
                     U1.RENDERER.active_shape_id, clear_writes, shape_writes, color_writes, non_mono_writes);
            errors = errors + 1;
        end else begin
            $display("PASS [SW[3] NVIDIA Logo]: id=%0d pixels=%0d bbox=%0dx%0d",
                     U1.RENDERER.active_shape_id, shape_writes, max_x - min_x, max_y - min_y);
        end
        check_depth_sorted("NVIDIA Logo");
        dump_pgm("rtl_sw3_nvidia.pgm");

        // 4. SW[2] = Rocket (shape_id = 7)
        reset_counters();
        SW = 10'b00000_00100;
        wait_render_done(900000);
        if (U1.RENDERER.active_shape_id !== 4'd7 || clear_writes != 307200 || shape_writes == 0 ||
            color_writes != shape_writes || non_mono_writes != 0) begin
            $display("FAIL [SW[2] Rocket]: id=%0d clear=%0d shape=%0d lit=%0d non_mono=%0d",
                     U1.RENDERER.active_shape_id, clear_writes, shape_writes, color_writes, non_mono_writes);
            errors = errors + 1;
        end else begin
            $display("PASS [SW[2] Rocket]: id=%0d pixels=%0d bbox=%0dx%0d",
                     U1.RENDERER.active_shape_id, shape_writes, max_x - min_x, max_y - min_y);
        end
        check_depth_sorted("Rocket");
        dump_pgm("rtl_sw2_rocket.pgm");

        // 5. SW[1] = Jet (shape_id = 8)
        reset_counters();
        SW = 10'b00000_00010;
        wait_render_done(900000);
        if (U1.RENDERER.active_shape_id !== 4'd8 || clear_writes != 307200 || shape_writes == 0 ||
            color_writes != shape_writes || non_mono_writes != 0) begin
            $display("FAIL [SW[1] Jet]: id=%0d clear=%0d shape=%0d lit=%0d non_mono=%0d",
                     U1.RENDERER.active_shape_id, clear_writes, shape_writes, color_writes, non_mono_writes);
            errors = errors + 1;
        end else begin
            $display("PASS [SW[1] Jet]: id=%0d pixels=%0d bbox=%0dx%0d",
                     U1.RENDERER.active_shape_id, shape_writes, max_x - min_x, max_y - min_y);
        end
        check_depth_sorted("Jet");
        dump_pgm("rtl_sw1_jet.pgm");

        // 6. SW[0] = Star (shape_id = 9)
        reset_counters();
        SW = 10'b00000_00001;
        wait_render_done(900000);
        if (U1.RENDERER.active_shape_id !== 4'd9 || clear_writes != 307200 || shape_writes == 0 ||
            color_writes != shape_writes || non_mono_writes != 0) begin
            $display("FAIL [SW[0] Star]: id=%0d clear=%0d shape=%0d lit=%0d non_mono=%0d",
                     U1.RENDERER.active_shape_id, clear_writes, shape_writes, color_writes, non_mono_writes);
            errors = errors + 1;
        end else begin
            $display("PASS [SW[0] Star]: id=%0d pixels=%0d bbox=%0dx%0d",
                     U1.RENDERER.active_shape_id, shape_writes, max_x - min_x, max_y - min_y);
        end
        check_depth_sorted("Star");
        dump_pgm("rtl_sw0_star.pgm");

        // 7. Multiple switches within SW[4:0] (SW[1] + SW[0]) -> hold Star, 0 writes
        reset_counters();
        SW = 10'b00000_00011;
        repeat (500) @(posedge CLOCK_50);
        if (total_writes != 0 || U1.RENDERER.active_shape_id !== 4'd9 || U1.render_done !== 1'b1) begin
            $display("FAIL [Multi SW[1]+SW[0]]: writes=%0d id=%0d done=%b",
                     total_writes, U1.RENDERER.active_shape_id, U1.render_done);
            errors = errors + 1;
        end else begin
            $display("PASS [Multi SW[1]+SW[0]]: held id=%0d with 0 writes", U1.RENDERER.active_shape_id);
        end

        if (errors == 0)
            $display("ALL CUSTOM FIGURE TESTS PASSED");
        else
            $display("FAILED with %0d error(s)", errors);
        $stop;
    end

    top U1 (
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

endmodule
