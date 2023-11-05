module imm_gen
(input [31:0] inst,
input [2:0] imm_sel, // change if necessary
output [31:0] imm
);
reg imm_reg = 0;
always (*)  begin
    imm_reg = 0;
    case (imm_sel): 
        3'b000: imm_reg = {{20{inst[31]}}, inst[31:20]}; // imm_sel == 0 == I-TYPE
        3'b001: imm_reg = {{20{inst[31]}}, inst[31:25], inst[11:7]}; // imm_sel == 1 == S-TYPE
        3'b010: imm_reg = {{12{inst[31]}}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0}; // imm_sel == 2 == B_TYPE
        3'b011: imm_reg = {inst[31:12], 12'b0}; // imm_sel == 3 == U-TYPE
        3'b100: imm_reg = {{20{inst[31]}}, inst[31], inst[19:12], inst[20], inst[30:21], 1'b0}; // imm_sel == 4 == J-TYPE
        default: imm_reg = 0;
    endcase   
end
assign imm = imm_reg;
endmodule