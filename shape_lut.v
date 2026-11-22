`default_nettype none

module shape_lut #(
    parameter integer NUM_SHAPES = 10,
    parameter integer NUM_POINTS = 2048,
`ifdef DESIM
    parameter         INIT_FILE  = "../shape_lut.hex"
`else
    parameter         INIT_FILE  = "shape_lut.hex"
`endif
) (
    input  wire               clk,
    input  wire [3:0]         shape_id,
    input  wire [10:0]        point_idx,
    output reg  signed [8:0]  px,
    output reg  signed [8:0]  py,
    output reg  signed [8:0]  pz,
    output reg  signed [8:0]  nx,
    output reg  signed [8:0]  ny,
    output reg  signed [8:0]  nz
);
    localparam integer TOTAL_ENTRIES = NUM_SHAPES * NUM_POINTS;

    (* ramstyle = "M10K" *) reg [53:0] rom [0:TOTAL_ENTRIES-1];

    initial begin
        $readmemh(INIT_FILE, rom);
    end

    wire [14:0] rom_addr = ({11'd0, shape_id} * NUM_POINTS) + {4'd0, point_idx};

    always @(posedge clk) begin
        {px, py, pz, nx, ny, nz} <= rom[rom_addr];
    end
endmodule

`default_nettype wire
