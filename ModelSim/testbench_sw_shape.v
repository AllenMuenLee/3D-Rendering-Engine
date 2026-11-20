`timescale 1ns / 1ps

module testbench_sw_shape ();

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

    // vga_write is registered in platonic_renderer, so track previous-cycle state
    reg [3:0] prev_state;
    always @(posedge CLOCK_50) begin
        prev_state <= U1.RENDERER.state;
        if (plot) begin
            total_writes = total_writes + 1;
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
            // Allow 2-FF sync + decode + FSM start (10 cycles) before checking done
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

    task wait_clear_to_idle;
        input integer max_cycles;
        integer c;
        begin
            c = 0;
            repeat (10) @(posedge CLOCK_50);
            while ((U1.RENDERER.state != 4'd0 || U1.RENDERER.screen_dirty) && c < max_cycles) begin
                @(posedge CLOCK_50);
                c = c + 1;
            end
            repeat (10) @(posedge CLOCK_50);
            if (U1.RENDERER.state != 4'd0 || U1.RENDERER.screen_dirty) begin
                $display("FAIL: Timed out waiting for blank idle (state=%0d dirty=%b)",
                         U1.RENDERER.state, U1.RENDERER.screen_dirty);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        CLOCK_50 = 1'b0;
        forever #(CLOCK_PERIOD / 2) CLOCK_50 = ~CLOCK_50;
    end

    initial begin
        // 1. Reset with all switches off -> nothing should render
        SW  = 10'b00000_00000;
        KEY = 4'b1110;
        reset_counters();
        repeat (5) @(posedge CLOCK_50);
        KEY[0] = 1'b1;
        repeat (500) @(posedge CLOCK_50);
        if (total_writes != 0 || U1.render_done !== 1'b0) begin
            $display("FAIL [1: All off after reset]: writes=%0d done=%b", total_writes, U1.render_done);
            errors = errors + 1;
        end else begin
            $display("PASS [1: All off after reset]: 0 pixels written, blank screen");
        end

        // 2. While blank, turn on multiple switches (SW[9] and SW[5]) -> does nothing
        reset_counters();
        SW = 10'b10001_00000;
        repeat (500) @(posedge CLOCK_50);
        if (total_writes != 0 || U1.shape_enable !== 1'b0) begin
            $display("FAIL [2: Multiple switches while blank]: writes=%0d shape_enable=%b", total_writes, U1.shape_enable);
            errors = errors + 1;
        end else begin
            $display("PASS [2: Multiple switches while blank]: does nothing (0 pixels written)");
        end

        // 3. Turn on SW[5] only (Tetrahedron, shape_id=0) -> draws without clearing clean screen
        reset_counters();
        SW = 10'b00001_00000;
        wait_render_done(400000);
        if (U1.RENDERER.active_shape_id !== 3'd0 || clear_writes != 0 || shape_writes == 0 ||
            color_writes != shape_writes || non_mono_writes != 0) begin
            $display("FAIL [3: SW[5] Tetrahedron]: active_id=%0d clear=%0d shape=%0d lit=%0d non_mono=%0d",
                     U1.RENDERER.active_shape_id, clear_writes, shape_writes, color_writes, non_mono_writes);
            errors = errors + 1;
        end else begin
            $display("PASS [3: SW[5] Tetrahedron]: id=%0d, pixels=%0d (all lit mono), bbox=%0dx%0d",
                     U1.RENDERER.active_shape_id, shape_writes, max_x - min_x, max_y - min_y);
        end

        // 4. Turn on SW[6] while SW[5] is still on -> multiple switches -> does nothing!
        reset_counters();
        SW = 10'b00011_00000;
        repeat (500) @(posedge CLOCK_50);
        if (total_writes != 0 || U1.RENDERER.active_shape_id !== 3'd0 || U1.render_done !== 1'b1) begin
            $display("FAIL [4: Multiple switches while Tetra active]: writes=%0d active_id=%0d done=%b",
                     total_writes, U1.RENDERER.active_shape_id, U1.render_done);
            errors = errors + 1;
        end else begin
            $display("PASS [4: Multiple switches while Tetra active]: does nothing (held id=%0d, 0 writes)",
                     U1.RENDERER.active_shape_id);
        end

        // 5. Turn off SW[5] leaving SW[6] on (Octahedron, id=1) -> clears 307200 black + draws Octahedron
        reset_counters();
        SW = 10'b00010_00000;
        wait_render_done(800000);
        if (U1.RENDERER.active_shape_id !== 3'd1 || clear_writes != 307200 || shape_writes == 0 ||
            color_writes != shape_writes || non_mono_writes != 0) begin
            $display("FAIL [5: SW[6] Octahedron]: active_id=%0d clear=%0d shape=%0d lit=%0d non_mono=%0d",
                     U1.RENDERER.active_shape_id, clear_writes, shape_writes, color_writes, non_mono_writes);
            errors = errors + 1;
        end else begin
            $display("PASS [5: SW[6] Octahedron]: cleared=%0d, id=%0d, pixels=%0d (all lit mono), bbox=%0dx%0d",
                     clear_writes, U1.RENDERER.active_shape_id, shape_writes, max_x - min_x, max_y - min_y);
        end

        // 6. Switch to SW[7] (Cube, id=2)
        reset_counters();
        SW = 10'b00100_00000;
        wait_render_done(800000);
        if (U1.RENDERER.active_shape_id !== 3'd2 || clear_writes != 307200 || shape_writes == 0 ||
            color_writes != shape_writes || non_mono_writes != 0 || (max_x - min_x) >= (max_y - min_y)) begin
            $display("FAIL [6: SW[7] Cube]: active_id=%0d clear=%0d shape=%0d lit=%0d non_mono=%0d bbox=%0dx%0d",
                     U1.RENDERER.active_shape_id, clear_writes, shape_writes, color_writes, non_mono_writes,
                     max_x - min_x, max_y - min_y);
            errors = errors + 1;
        end else begin
            $display("PASS [6: SW[7] Cube]: cleared=%0d, id=%0d, pixels=%0d (all lit mono), bbox=%0dx%0d",
                     clear_writes, U1.RENDERER.active_shape_id, shape_writes, max_x - min_x, max_y - min_y);
        end

        // 7. Switch to SW[8] (Icosahedron, id=3)
        reset_counters();
        SW = 10'b01000_00000;
        wait_render_done(800000);
        if (U1.RENDERER.active_shape_id !== 3'd3 || clear_writes != 307200 || shape_writes == 0 ||
            color_writes != shape_writes || non_mono_writes != 0) begin
            $display("FAIL [7: SW[8] Icosahedron]: active_id=%0d clear=%0d shape=%0d lit=%0d non_mono=%0d",
                     U1.RENDERER.active_shape_id, clear_writes, shape_writes, color_writes, non_mono_writes);
            errors = errors + 1;
        end else begin
            $display("PASS [7: SW[8] Icosahedron]: cleared=%0d, id=%0d, pixels=%0d (all lit mono), bbox=%0dx%0d",
                     clear_writes, U1.RENDERER.active_shape_id, shape_writes, max_x - min_x, max_y - min_y);
        end

        // 8. Switch to SW[9] (Dodecahedron, id=4)
        reset_counters();
        SW = 10'b10000_00000;
        wait_render_done(800000);
        if (U1.RENDERER.active_shape_id !== 3'd4 || clear_writes != 307200 || shape_writes == 0 ||
            color_writes != shape_writes || non_mono_writes != 0) begin
            $display("FAIL [8: SW[9] Dodecahedron]: active_id=%0d clear=%0d shape=%0d lit=%0d non_mono=%0d",
                     U1.RENDERER.active_shape_id, clear_writes, shape_writes, color_writes, non_mono_writes);
            errors = errors + 1;
        end else begin
            $display("PASS [8: SW[9] Dodecahedron]: cleared=%0d, id=%0d, pixels=%0d (all lit mono), bbox=%0dx%0d",
                     clear_writes, U1.RENDERER.active_shape_id, shape_writes, max_x - min_x, max_y - min_y);
        end

        // 9. Turn all switches off -> clears screen to black and renders nothing
        reset_counters();
        SW = 10'b00000_00000;
        wait_clear_to_idle(400000);
        if (clear_writes != 307200 || shape_writes != 0 || U1.render_done !== 1'b0) begin
            $display("FAIL [9: All switches off]: clear=%0d shape=%0d done=%b",
                     clear_writes, shape_writes, U1.render_done);
            errors = errors + 1;
        end else begin
            $display("PASS [9: All switches off]: cleared=%0d black pixels, 0 shape pixels, idle blank",
                     clear_writes);
        end

        // 10. Stay all off for 500 more cycles -> verify no further writes
        reset_counters();
        repeat (500) @(posedge CLOCK_50);
        if (total_writes != 0) begin
            $display("FAIL [10: Stay blank]: unexpected %0d writes", total_writes);
            errors = errors + 1;
        end else begin
            $display("PASS [10: Stay blank]: 0 further writes");
        end

        if (errors == 0)
            $display("ALL SWITCH SHAPE TESTS PASSED");
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
