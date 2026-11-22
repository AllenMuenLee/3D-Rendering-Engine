`default_nettype none

/*  PS/2 keyboard interface.
 *  Synchronizes and filters PS2_CLK and PS2_DAT to CLOCK_50, shifts in 11-bit
 *  PS/2 frames (START=0, d0..d7, odd PARITY, STOP=1), ignores F0 break sequences
 *  and E0 extended prefixes, pulses ps2_rec on each make code, latches the 8-bit
 *  scancode, and decodes object selection, direction, and WASD LED indicators.
 */
module keyboard (
    CLOCK_50,
    Resetn,
    PS2_CLK,
    PS2_DAT,
    ps2_rec,
    scancode,
    object_sel,
    O1_dir,
    O2_dir,
    LEDR,
    wasd_held
);
    input wire CLOCK_50;
    input wire Resetn;
    input wire PS2_CLK;
    input wire PS2_DAT;
    output reg ps2_rec;
    output reg [7:0] scancode;
    output wire object_sel;
    output wire [1:0] O1_dir;
    output wire [1:0] O2_dir;
    output reg [3:0] LEDR;
    output reg [3:0] wasd_held;     // {W, A, S, D} active while each key is held

    reg ps2_clk_q1, PS2_CLK_S;      // synchronized PS2_CLK
    reg ps2_dat_q1, PS2_DAT_S;      // synchronized PS2_DAT
    reg ps2_dat_d1, ps2_dat_d2;     // aligned with 2-cycle PS2_CLK filter
    reg [1:0] ps2_clk_hist;         // 2-cycle glitch filter history for PS2_CLK_S
    reg ps2_clk_filt, prev_ps2_clk; // filtered PS2_CLK and previous cycle value
    wire negedge_ps2_clk;           // falling edge of filtered PS2_CLK
    reg [10:0] frame;               // 11-bit PS2 frame: {STOP, PARITY, d7..d0, START}
    reg [3:0] Packet;               // number of bits received in current frame (0..11)
    reg [15:0] idle_cnt;            // idle counter to resync Packet after >1 ms bus idle
    reg break_code;                 // set after receiving 8'hF0 break prefix
    reg ext_code;                   // set after receiving 8'hE0 extended prefix
    wire valid_frame;               // framing + odd parity check

    // two-FF synchronizers for PS2 signals (PS2 idles high)
    always @(posedge CLOCK_50) begin
        if (!Resetn) begin
            ps2_clk_q1 <= 1'b1;
            PS2_CLK_S  <= 1'b1;
            ps2_dat_q1 <= 1'b1;
            PS2_DAT_S  <= 1'b1;
            ps2_dat_d1 <= 1'b1;
            ps2_dat_d2 <= 1'b1;
        end else begin
            ps2_clk_q1 <= PS2_CLK;
            PS2_CLK_S  <= ps2_clk_q1;
            ps2_dat_q1 <= PS2_DAT;
            PS2_DAT_S  <= ps2_dat_q1;
            ps2_dat_d1 <= PS2_DAT_S;
            ps2_dat_d2 <= ps2_dat_d1;
        end
    end

    // 2-sample glitch filter and falling-edge detector on PS2_CLK_S
    always @(posedge CLOCK_50) begin
        if (!Resetn) begin
            ps2_clk_hist <= 2'b11;
            ps2_clk_filt <= 1'b1;
            prev_ps2_clk <= 1'b1;
        end else begin
            ps2_clk_hist <= {ps2_clk_hist[0], PS2_CLK_S};
            if (ps2_clk_hist == 2'b11)
                ps2_clk_filt <= 1'b1;
            else if (ps2_clk_hist == 2'b00)
                ps2_clk_filt <= 1'b0;
            prev_ps2_clk <= ps2_clk_filt;
        end
    end

    assign negedge_ps2_clk = (prev_ps2_clk & !ps2_clk_filt);

    // count how long filtered PS2_CLK stays high to auto-resync after any partial frame
    always @(posedge CLOCK_50) begin
        if (!Resetn || !ps2_clk_filt)
            idle_cnt <= 16'd0;
        else if (idle_cnt < 16'd50000)
            idle_cnt <= idle_cnt + 16'd1;
    end

    // shift in 11-bit PS2 frame and count bits
    always @(posedge CLOCK_50) begin
        if (!Resetn) begin
            frame  <= 11'b0;
            Packet <= 4'd0;
        end else if (Packet == 4'd11) begin
            Packet <= 4'd0;
        end else if (negedge_ps2_clk) begin
            frame  <= {ps2_dat_d2, frame[10:1]};
            Packet <= Packet + 4'd1;
        end else if (idle_cnt == 16'd50000) begin
            Packet <= 4'd0;
        end
    end

    // validate START=0, STOP=1, and odd parity
    assign valid_frame = (frame[0] == 1'b0) && (frame[10] == 1'b1) && ((^frame[9:1]) == 1'b1);

    // capture make scancode immediately on key press; track F0 break sequences and E0 prefix
    always @(posedge CLOCK_50) begin
        if (!Resetn) begin
            scancode   <= 8'b0;
            break_code <= 1'b0;
            ext_code   <= 1'b0;
            ps2_rec    <= 1'b0;
            wasd_held  <= 4'b0000;
        end else begin
            ps2_rec <= 1'b0;
            if (Packet == 4'd11 && valid_frame) begin
                if (frame[8:1] == 8'hF0) begin
                    break_code <= 1'b1;
                end else if (frame[8:1] == 8'hE0) begin
                    ext_code <= 1'b1;
                end else if (ext_code) begin
                    ext_code   <= 1'b0;
                    break_code <= 1'b0;
                end else if (break_code) begin
                    break_code <= 1'b0;
                    case (frame[8:1])
                        8'h1D: wasd_held[3] <= 1'b0; // 'w' released
                        8'h1C: wasd_held[2] <= 1'b0; // 'a' released
                        8'h1B: wasd_held[1] <= 1'b0; // 's' released
                        8'h23: wasd_held[0] <= 1'b0; // 'd' released
                        default: ;
                    endcase
                end else begin
                    scancode <= frame[8:1];
                    ps2_rec  <= 1'b1;
                    case (frame[8:1])
                        8'h1D: wasd_held[3] <= 1'b1; // 'w' pressed
                        8'h1C: wasd_held[2] <= 1'b1; // 'a' pressed
                        8'h1B: wasd_held[1] <= 1'b1; // 's' pressed
                        8'h23: wasd_held[0] <= 1'b1; // 'd' pressed
                        default: ;
                    endcase
                end
            end
        end
    end

    // select object according to which PS2 key was pressed.
    // scancode[4] == 1 for a/s/w/z and 0 for d/f/r/c
    assign object_sel = scancode[4];

    // decode movement direction from scancode
    assign O1_dir = scancode[1:0];               // ps2 key identifier (for a, s, w, z)
    assign O2_dir = {scancode[3], scancode[1]};  // ps2 key identifier (for d, f, r, c)

    // decode WASD keys to one-hot LED indicators: {W, A, S, D}
    always @(*) begin
        case (scancode)
            8'h1D:   LEDR = 4'b1000; // 'w'
            8'h1C:   LEDR = 4'b0100; // 'a'
            8'h1B:   LEDR = 4'b0010; // 's'
            8'h23:   LEDR = 4'b0001; // 'd'
            default: LEDR = 4'b0000;
        endcase
    end

endmodule

`default_nettype wire
