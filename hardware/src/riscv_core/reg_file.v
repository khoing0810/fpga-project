module reg_file (
    input clk,
    input we, // RegWEn
    input [4:0] ra1, ra2, wa, //rs1, rs2, rd
    input [31:0] wd, // data to write
    output [31:0] rd1, rd2 //R[rs1], R[rs2]
);
    parameter DEPTH = 32;
    reg [31:0] mem [0:31];
    reg [31:0] rd1_reg;
    reg [31:0] rd2_reg;
    initial begin
        // TODO: double check these on lab computers!
        genvar i;
        for (i = 0; i < DEPTH; i = i + 1) begin
            mem[i] = 0;
        end
        rd1_reg = 0;
        rd2_reg = 0;
    end
    assign rd1 = rd1_reg;
    assign rd2 = rd2_reg;
    always @(negedge clk) begin
        //read
        rd1_reg <= mem[ra1];
        rd2_reg <= mem[ra2];
    end
    always @(posedge clk) begin
        //write
        if (we && wa != 5'd0) begin
            mem[wa] <= wd;
        end
    end
endmodule
