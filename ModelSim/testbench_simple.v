`timescale 1ns / 1ns
`default_nettype none

module testbench_simple;
    reg        CLOCK_50 = 1'b0;
    reg  [9:0] SW       = 10'd0;
    reg  [3:0] KEY      = 4'b0000;
    wire [9:0] LEDR;
    reg        ps2_clk_drv = 1'b1;
    reg        ps2_dat_drv = 1'b1;
    wire       PS2_CLK = ps2_clk_drv;
    wire       PS2_DAT = ps2_dat_drv;

    wire [9:0]  VGA_X;
    wire [8:0]  VGA_Y;
    wire [23:0] VGA_COLOR;
    wire        plot;

    always #10 CLOCK_50 = ~CLOCK_50;

    // Override KEY_DELAY_CYCLES for fast simulation (represents 0.05s in hardware)
    top #(
        .KEY_DELAY_CYCLES (22'd20),
        .ROT_STEP_DEG     (9'd15)
    ) DUT (
        .CLOCK_50  (CLOCK_50),
        .SW        (SW),
        .KEY       (KEY),
        .LEDR      (LEDR),
        .PS2_CLK   (PS2_CLK),
        .PS2_DAT   (PS2_DAT),
        .VGA_X     (VGA_X),
        .VGA_Y     (VGA_Y),
        .VGA_COLOR (VGA_COLOR),
        .plot      (plot)
    );

    // Send one 11-bit PS/2 frame (START=0, 8 data bits LSB-first, odd parity, STOP=1)
    task send_ps2_byte;
        input [7:0] data;
        integer i;
        reg [10:0] pkt;
        begin
            pkt = {1'b1, ~(^data), data, 1'b0};
            for (i = 0; i < 11; i = i + 1) begin
                ps2_dat_drv = pkt[i];
                repeat (8) @(posedge CLOCK_50);
                ps2_clk_drv = 1'b0;
                repeat (8) @(posedge CLOCK_50);
                ps2_clk_drv = 1'b1;
                repeat (8) @(posedge CLOCK_50);
            end
            ps2_dat_drv = 1'b1;
            repeat (10) @(posedge CLOCK_50);
        end
    endtask

    integer lit_pixels;
    integer min_bright, max_bright;
    integer errors;

    always @(posedge CLOCK_50) begin
        if (plot && VGA_COLOR != 24'd0) begin
            lit_pixels = lit_pixels + 1;
            if (VGA_COLOR[7:0] < min_bright) min_bright = VGA_COLOR[7:0];
            if (VGA_COLOR[7:0] > max_bright) max_bright = VGA_COLOR[7:0];
        end
    end

    task wait_idle;
        begin
            // Let 2-FF SW synchronizer + shape_enable + renderer S_IDLE react
            repeat (10) @(posedge CLOCK_50);
            wait (DUT.render_done == 1'b1 && !DUT.RENDERER.need_update);
            repeat (5) @(posedge CLOCK_50);
        end
    endtask

    initial begin
        errors = 0;
        lit_pixels = 0;
        min_bright = 255;
        max_bright = 0;

        // Reset
        KEY[0] = 1'b0;
        repeat (5) @(posedge CLOCK_50);
        KEY[0] = 1'b1;
        repeat (5) @(posedge CLOCK_50);

        // 1. Select Cube (SW[7]) and wait for initial render
        SW = 10'b00100_00000;
        wait_idle();
        $display("[TEST 1] Cube rendered: lit_pixels=%0d, brightness=[%0d..%0d]",
                 lit_pixels, min_bright, max_bright);
        if (lit_pixels < 500 || max_bright <= min_bright) begin
            $display("FAIL: Cube did not render with shaded pixels!");
            errors = errors + 1;
        end

        // 2. Press 'D' (8'h23), let counter expire, release, and wait for final render
        lit_pixels = 0;
        min_bright = 255;
        max_bright = 0;
        send_ps2_byte(8'h23);
        repeat (30) @(posedge CLOCK_50);
        send_ps2_byte(8'hF0);
        send_ps2_byte(8'h23);
        wait_idle();
        $display("[TEST 2] After 'D' press: angle_z=%0d (expected != 35), lit_pixels=%0d, brightness=[%0d..%0d]",
                 DUT.angle_z, lit_pixels, min_bright, max_bright);
        if (DUT.angle_z == 9'd35 || lit_pixels < 500) begin
            $display("FAIL: Rotation did not update angle or re-render!");
            errors = errors + 1;
        end

        // 3. Switch to Star (SW[0])
        lit_pixels = 0;
        min_bright = 255;
        max_bright = 0;
        SW = 10'b00000_00001;
        wait_idle();
        $display("[TEST 3] Star rendered: lit_pixels=%0d, brightness=[%0d..%0d]",
                 lit_pixels, min_bright, max_bright);
        if (lit_pixels < 500) begin
            $display("FAIL: Star did not render!");
            errors = errors + 1;
        end

        // 4. Turn all switches off -> blank screen
        lit_pixels = 0;
        SW = 10'd0;
        wait_idle();
        $display("[TEST 4] Blank screen: lit_pixels=%0d (expected 0)", lit_pixels);
        if (lit_pixels != 0) begin
            $display("FAIL: Blank screen drew non-black pixels!");
            errors = errors + 1;
        end

        if (errors == 0)
            $display("ALL TESTS PASSED!");
        else
            $display("FAILED with %0d error(s).", errors);
        $stop;
    end
endmodule

`default_nettype wire
