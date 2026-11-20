`timescale 1ns / 1ps

module testbench_v2 ();

    parameter CLOCK_PERIOD = 20;

    reg CLOCK_50;
    reg [9:0] SW = 10'b0;
    reg [3:0] KEY;
    wire [9:0] LEDR;
    wire [9:0] VGA_X;
    wire [8:0] VGA_Y;
    wire [23:0] VGA_COLOR;
    wire plot;
    wire ps2_clk, ps2_dat;
    reg key_action = 0;
    reg [7:0] scan_code = 0;

    integer errors = 0;

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

    task expect_ledr;
        input [16*8-1:0] desc;
        input [3:0] expected;
        begin
            if (LEDR[3:0] !== expected) begin
                $display("FAIL %0s: expected LEDR[3:0]=%b, got %b",
                         desc, expected, LEDR[3:0]);
                errors = errors + 1;
            end else begin
                $display("PASS %0s -> LEDR[3:0]=%b (W=%b A=%b S=%b D=%b)",
                         desc, LEDR[3:0], LEDR[3], LEDR[2], LEDR[1], LEDR[0]);
            end
        end
    endtask

    initial begin
        CLOCK_50 = 1'b0;
        forever #(CLOCK_PERIOD / 2) CLOCK_50 = ~CLOCK_50;
    end

    initial begin
        KEY = 4'b1110; // assert active-low reset KEY[0]
        @(posedge CLOCK_50);
        KEY[0] = 1'b1;
        repeat (5) @(posedge CLOCK_50);

        // Press W and D simultaneously
        send_byte(8'h1D);
        expect_ledr("Press W", 4'b1000);

        send_byte(8'h23);
        expect_ledr("Hold W + Press D", 4'b1001);

        // Press A and S too (all four held)
        send_byte(8'h1C);
        expect_ledr("Hold W,D + A", 4'b1101);

        send_byte(8'h1B);
        expect_ledr("Hold W,A,D + S", 4'b1111);

        // Extended key (Up arrow E0 75) and unrelated key Z (1A) should not disturb WASD
        send_byte(8'hE0);
        send_byte(8'h75);
        expect_ledr("Ext Up press", 4'b1111);
        send_byte(8'hE0);
        send_byte(8'hF0);
        send_byte(8'h75);
        expect_ledr("Ext Up release", 4'b1111);

        // Release W while A, S, D remain held
        send_byte(8'hF0);
        send_byte(8'h1D);
        expect_ledr("Release W", 4'b0111);

        // Release S while A, D remain held
        send_byte(8'hF0);
        send_byte(8'h1B);
        expect_ledr("Release S", 4'b0101);

        // Release A and D
        send_byte(8'hF0);
        send_byte(8'h1C);
        expect_ledr("Release A", 4'b0001);

        send_byte(8'hF0);
        send_byte(8'h23);
        expect_ledr("Release D", 4'b0000);

        if (errors == 0)
            $display("ALL MULTI-KEY TESTS PASSED");
        else
            $display("FAILED with %0d error(s)", errors);
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

    PS2_keyboard U2 (CLOCK_50, KEY[0], key_action, scan_code, ps2_clk, ps2_dat);

endmodule
