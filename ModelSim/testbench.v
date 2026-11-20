`timescale 1ns / 1ps

module testbench ();

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

    task check_key;
        input [8*8-1:0] label;
        input [7:0] code;
        input [3:0] expected_ledr;
        begin
            send_byte(code);
            if (LEDR[3:0] !== expected_ledr) begin
                $display("FAIL %0s (0x%02h): expected LEDR[3:0]=%b, got %b",
                         label, code, expected_ledr, LEDR[3:0]);
                errors = errors + 1;
            end else begin
                $display("PASS %0s (0x%02h) -> LEDR[3:0]=%b (W=%b A=%b S=%b D=%b)",
                         label, code, LEDR[3:0], LEDR[3], LEDR[2], LEDR[1], LEDR[0]);
            end

            // Send PS/2 break sequence (F0 + code) and verify LED stays latched
            send_byte(8'hF0);
            send_byte(code);
            if (LEDR[3:0] !== expected_ledr) begin
                $display("FAIL %0s release (0x%02h): expected LEDR[3:0]=%b, got %b",
                         label, code, expected_ledr, LEDR[3:0]);
                errors = errors + 1;
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

        check_key("Key W", 8'h1D, 4'b1000); // LEDR[3]
        check_key("Key A", 8'h1C, 4'b0100); // LEDR[2]
        check_key("Key S", 8'h1B, 4'b0010); // LEDR[1]
        check_key("Key D", 8'h23, 4'b0001); // LEDR[0]
        check_key("Key Z", 8'h1A, 4'b0000); // None

        if (errors == 0)
            $display("ALL TESTS PASSED (W->LEDR[3], A->LEDR[2], S->LEDR[1], D->LEDR[0])");
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
