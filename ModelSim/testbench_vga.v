`timescale 1ns / 1ps

module testbench_vga ();

    parameter CLOCK_PERIOD = 20;
    parameter RESOLUTION   = "160x120";
    parameter COLOR_DEPTH  = 9;
    localparam nX = 8;
    localparam nY = 7;

    reg CLOCK_50;
    reg resetn;
    reg [COLOR_DEPTH-1:0] color;
    reg [nX-1:0] x;
    reg [nY-1:0] y;
    reg write;

    wire [nX-1:0] VGA_X;
    wire [nY-1:0] VGA_Y;
    wire [23:0] VGA_COLOR;
    wire VGA_SYNC;
    wire plot;

    integer errors = 0;

    task check_pixel;
        input [nX-1:0] exp_x;
        input [nY-1:0] exp_y;
        input [COLOR_DEPTH-1:0] in_color;
        input [23:0] exp_rgb;
        input exp_plot;
        begin
            x     <= exp_x;
            y     <= exp_y;
            color <= in_color;
            write <= exp_plot;
            @(posedge CLOCK_50);
            #1;
            if (VGA_X !== exp_x || VGA_Y !== exp_y || VGA_COLOR !== exp_rgb || plot !== exp_plot || VGA_SYNC !== 1'b1) begin
                $display("FAIL x=%0d y=%0d color=%b: got X=%0d Y=%0d RGB=%06h plot=%b sync=%b",
                         exp_x, exp_y, in_color, VGA_X, VGA_Y, VGA_COLOR, plot, VGA_SYNC);
                errors = errors + 1;
            end else begin
                $display("PASS x=%0d y=%0d color=%b -> RGB=%06h plot=%b sync=%b",
                         VGA_X, VGA_Y, in_color, VGA_COLOR, plot, VGA_SYNC);
            end
        end
    endtask

    initial begin
        CLOCK_50 = 1'b0;
        forever #(CLOCK_PERIOD / 2) CLOCK_50 = ~CLOCK_50;
    end

    initial begin
        resetn = 1'b0;
        x = 0; y = 0; color = 0; write = 1'b0;
        repeat (2) @(posedge CLOCK_50);
        resetn = 1'b1;

        // 9-bit color: R=3'b111, G=3'b000, B=3'b000 -> FC_00_00 (vga_convert replicates top 6 bits)
        check_pixel(8'd10, 7'd20, 9'b111_000_000, 24'hFC0000, 1'b1);
        // 9-bit color: R=3'b000, G=3'b111, B=3'b000 -> 00_FC_00
        check_pixel(8'd80, 7'd60, 9'b000_111_000, 24'h00FC00, 1'b1);
        // 9-bit color: R=3'b101, G=3'b010, B=3'b001 -> B4_48_24
        check_pixel(8'd159, 7'd119, 9'b101_010_001, 24'hB44824, 1'b0);

        if (errors == 0)
            $display("ALL VGA ADAPTER TESTS PASSED");
        else
            $display("FAILED with %0d error(s)", errors);
        $stop;
    end

    vga_adapter #(
        .RESOLUTION(RESOLUTION),
        .COLOR_DEPTH(COLOR_DEPTH),
        .nX(nX),
        .nY(nY)
    ) DUT (
        .resetn(resetn),
        .clock(CLOCK_50),
        .color(color),
        .x(x),
        .y(y),
        .write(write),
        .VGA_X(VGA_X),
        .VGA_Y(VGA_Y),
        .VGA_COLOR(VGA_COLOR),
        .VGA_SYNC(VGA_SYNC),
        .plot(plot)
    );

endmodule
