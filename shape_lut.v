`default_nettype none

module shape_lut #(
    parameter integer NUM_SHAPES = 6,
    parameter integer MAX_TRIS   = 256,
`ifdef DESIM
    parameter         INIT_FILE  = "../shape_lut.hex"
`else
    parameter         INIT_FILE  = "shape_lut.hex"
`endif
) (
    input  wire               clk,
    input  wire [3:0]         shape_id,
    input  wire [7:0]         tri_idx,
    output reg  [7:0]         num_tris,
    output wire signed [8:0]  v0x,
    output wire signed [8:0]  v0y,
    output wire signed [8:0]  v0z,
    output wire signed [8:0]  v1x,
    output wire signed [8:0]  v1y,
    output wire signed [8:0]  v1z,
    output wire signed [8:0]  v2x,
    output wire signed [8:0]  v2y,
    output wire signed [8:0]  v2z,
    output wire signed [8:0]  nx,
    output wire signed [8:0]  ny,
    output wire signed [8:0]  nz
);
    localparam integer TOTAL_ENTRIES = NUM_SHAPES * MAX_TRIS;

    (* ramstyle = "M10K" *) reg [107:0] rom [0:TOTAL_ENTRIES-1];
    reg [107:0] rom_q;

    initial begin
        $readmemh(INIT_FILE, rom);
    end

    always @(*) begin
        case (shape_id)
            4'd0:    num_tris = 8'd4;
            4'd1:    num_tris = 8'd8;
            4'd2:    num_tris = 8'd12;
            4'd3:    num_tris = 8'd20;
            4'd4:    num_tris = 8'd36;
            4'd5:    num_tris = 8'd172;
            default: num_tris = 8'd0;
        endcase
    end

    wire [10:0] rom_addr = ({7'd0, shape_id} * MAX_TRIS) + {3'd0, tri_idx};

    always @(posedge clk) begin
        rom_q <= rom[rom_addr];
    end

    assign {v0x, v0y, v0z, v1x, v1y, v1z, v2x, v2y, v2z, nx, ny, nz} = rom_q;
endmodule

`default_nettype wire
