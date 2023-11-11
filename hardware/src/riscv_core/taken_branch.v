`include "opcode.vh"

module taken_branch (
    //Module checks if given the instruction, BrEq and BrLT, if the branch should be taken
    input [31:0] inst,
    input br_eq,
    input br_lt,
    output taken // 1: taken, 0: not taken
);

wire [2:0] funct3;

assign funct3 = inst[14:12];

assign taken = (inst[6:0] == `OPC_BRANCH) && ((funct3 == `FNC_BEQ && br_eq) ? 1 :
               ((funct3 == `FNC_BGE || funct3 == `FNC_BGEU) && !br_lt) ? 1 :
               ((funct3 == `FNC_BLT || funct3 == `FNC_BLTU) &&  br_lt) ? 1 :
               (funct3 == `FNC_BNE && !br_eq) ? 1 : 0);
endmodule