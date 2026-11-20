`default_nettype none

/*  Hardware VGA Controller & Framebuffer Adapter (640x480 @ 60 Hz)
 *  For physical DE-series FPGA boards (e.g., DE1-SoC / Cyclone V) compiled
 *  in Quartus. Replaces the DESim-only vga_adapter when driving a real monitor.
 *
 *  Features:
 *    - Derives a 25 MHz pixel tick from CLOCK_50 (50 MHz).
 *    - Infers a single-clock M10K dual-port framebuffer (640 x 480 pixels).
 *    - Automatically clears the framebuffer to black after power-up or reset
 *      before asserting VGA_SYNC, so platonic_renderer draws onto a clean
 *      black background without requiring a .mif file.
 *    - Outputs standard 640x480 @ 60 Hz VGA timing and 8-bit RGB DAC signals.
 */
module vga_board_adapter #(
    parameter COLOR_DEPTH = 24 // 24 (8-bit smooth grayscale BRAM), 9, 6, or 3
) (
    input  wire                   resetn,
    input  wire                   clock,       // 50 MHz input clock
    input  wire [COLOR_DEPTH-1:0] color,
    input  wire [9:0]             x,
    input  wire [8:0]             y,
    input  wire                   write,
    output reg                    VGA_SYNC,    // 1 once framebuffer clear is done
    output wire [7:0]             VGA_R,
    output wire [7:0]             VGA_G,
    output wire [7:0]             VGA_B,
    output reg                    VGA_HS,
    output reg                    VGA_VS,
    output reg                    VGA_BLANK_N,
    output wire                   VGA_SYNC_N,
    output wire                   VGA_CLK
);

    localparam integer SCREEN_W     = 640;
    localparam integer SCREEN_H     = 480;
    localparam integer TOTAL_PIXELS = SCREEN_W * SCREEN_H; // 307,200
    localparam [18:0]  LAST_PIXEL   = 19'd307199;

    // 640x480 @ 60 Hz timing constants (25 MHz pixel clock)
    localparam [9:0] H_ACTIVE = 10'd640;
    localparam [9:0] H_FP     = 10'd16;
    localparam [9:0] H_SYNC   = 10'd96;
    localparam [9:0] H_LAST   = 10'd799;

    localparam [9:0] V_ACTIVE = 10'd480;
    localparam [9:0] V_FP     = 10'd10;
    localparam [9:0] V_SYNC   = 10'd2;
    localparam [9:0] V_LAST   = 10'd524;

    // -------------------------------------------------------------------------
    // Framebuffer BRAM (307,200 x VRAM_DEPTH bits -> infers M10K on Cyclone V)
    //   COLOR_DEPTH == 24 stores 8-bit grayscale intensity (2.46 Mbits, 256 levels)
    //   so smooth shading fits comfortably inside Cyclone V M10K BRAM.
    // -------------------------------------------------------------------------
    localparam integer VRAM_DEPTH = (COLOR_DEPTH == 24) ? 8 : COLOR_DEPTH;
    (* ramstyle = "M10K" *) reg [VRAM_DEPTH-1:0] vram [0:TOTAL_PIXELS-1];

    // Power-up / reset framebuffer clear state
    reg        clearing   = 1'b1;
    reg [18:0] clear_addr = 19'd0;

    // 25 MHz pixel tick & VGA DAC clock (rising edge centered in pixel period)
    reg pixel_tick = 1'b0;
    reg vga_clk_r  = 1'b0;

    always @(posedge clock) begin
        pixel_tick <= ~pixel_tick;
        vga_clk_r  <= ~pixel_tick;
    end

    assign VGA_CLK    = vga_clk_r;
    assign VGA_SYNC_N = 1'b0;

    // Horizontal & vertical scan counters
    reg [9:0] h_cnt = 10'd0;
    reg [9:0] v_cnt = 10'd0;

    // Write address: y * 640 + x = (y << 9) + (y << 7) + x (shift-add, no DSP)
    wire [18:0] wr_pixel_addr = {1'b0, y, 9'b0} + {3'b0, y, 7'b0} + {9'b0, x};
    wire        wr_in_bounds  = (x < H_ACTIVE) && (y < V_ACTIVE[8:0]);

    wire [VRAM_DEPTH-1:0] color_packed;
    generate
        if (COLOR_DEPTH == 24) begin : gen_pack_24
            // Exact identity when R == G == B (grayscale), Rec.601-approx luma otherwise
            wire [9:0] luma_sum = {2'b0, color[23:16]} + {1'b0, color[15:8], 1'b0} + {2'b0, color[7:0]};
            assign color_packed = luma_sum[9:2];
        end else begin : gen_pack_raw
            assign color_packed = color;
        end
    endgenerate

    wire                  vram_we    = clearing ? 1'b1 : (VGA_SYNC && write && wr_in_bounds);
    wire [18:0]           vram_waddr = clearing ? clear_addr : wr_pixel_addr;
    wire [VRAM_DEPTH-1:0] vram_wdata = clearing ? {VRAM_DEPTH{1'b0}} : color_packed;

    // Read address: v_cnt * 640 + h_cnt (only valid inside active 640x480 window)
    wire        active_video = (h_cnt < H_ACTIVE) && (v_cnt < V_ACTIVE);
    wire [18:0] rd_addr = active_video
        ? ({1'b0, v_cnt[8:0], 9'b0} + {3'b0, v_cnt[8:0], 7'b0} + {9'b0, h_cnt})
        : 19'd0;

    reg [VRAM_DEPTH-1:0] fb_q = {VRAM_DEPTH{1'b0}};

    // Clear controller & VGA sync/read pipeline
    always @(posedge clock) begin
        if (!resetn) begin
            clearing    <= 1'b1;
            clear_addr  <= 19'd0;
            VGA_SYNC    <= 1'b0;
        end else if (clearing) begin
            VGA_SYNC <= 1'b0;
            if (clear_addr == LAST_PIXEL) begin
                clearing <= 1'b0;
                VGA_SYNC <= 1'b1;
            end else begin
                clear_addr <= clear_addr + 19'd1;
            end
        end else begin
            VGA_SYNC <= 1'b1;
        end
    end

    // Single-clock dual-port BRAM access + aligned VGA sync outputs
    always @(posedge clock) begin
        if (vram_we)
            vram[vram_waddr] <= vram_wdata;

        if (pixel_tick) begin
            fb_q        <= vram[rd_addr];
            VGA_BLANK_N <= active_video;
            VGA_HS      <= !((h_cnt >= (H_ACTIVE + H_FP)) &&
                             (h_cnt <  (H_ACTIVE + H_FP + H_SYNC)));
            VGA_VS      <= !((v_cnt >= (V_ACTIVE + V_FP)) &&
                             (v_cnt <  (V_ACTIVE + V_FP + V_SYNC)));

            if (h_cnt == H_LAST) begin
                h_cnt <= 10'd0;
                if (v_cnt == V_LAST)
                    v_cnt <= 10'd0;
                else
                    v_cnt <= v_cnt + 10'd1;
            end else begin
                h_cnt <= h_cnt + 10'd1;
            end
        end
    end

    // Expand stored pixel (24->8-bit grayscale, or 9/6/3-bit RGB) to 8-bit DAC RGB
    wire [7:0] r8, g8, b8;
    generate
        if (COLOR_DEPTH == 24) begin : gen_exp_24
            assign r8 = fb_q;
            assign g8 = fb_q;
            assign b8 = fb_q;
        end else if (COLOR_DEPTH == 9) begin : gen_exp_9
            assign r8 = {fb_q[8:6], fb_q[8:6], fb_q[8:7]};
            assign g8 = {fb_q[5:3], fb_q[5:3], fb_q[5:4]};
            assign b8 = {fb_q[2:0], fb_q[2:0], fb_q[2:1]};
        end else if (COLOR_DEPTH == 6) begin : gen_exp_6
            assign r8 = {fb_q[5:4], fb_q[5:4], fb_q[5:4], fb_q[5:4]};
            assign g8 = {fb_q[3:2], fb_q[3:2], fb_q[3:2], fb_q[3:2]};
            assign b8 = {fb_q[1:0], fb_q[1:0], fb_q[1:0], fb_q[1:0]};
        end else begin : gen_exp_3
            assign r8 = {8{fb_q[2]}};
            assign g8 = {8{fb_q[1]}};
            assign b8 = {8{fb_q[0]}};
        end
    endgenerate

    assign VGA_R = VGA_BLANK_N ? r8 : 8'd0;
    assign VGA_G = VGA_BLANK_N ? g8 : 8'd0;
    assign VGA_B = VGA_BLANK_N ? b8 : 8'd0;

endmodule

`default_nettype wire
