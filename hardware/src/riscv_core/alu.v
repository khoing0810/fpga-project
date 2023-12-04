`include "alu_code.vh"
module alu (
    input [31:0] a_val,
    input [31:0] b_val,
    input [3:0] alu_sel,

    output [31:0] out_val
);

reg signed [31:0] a_val_signed;
reg signed [31:0] b_val_signed;
reg [31:0] out_reg;

always @(*) begin
    case (alu_sel) 
        `ALU_ADD: out_reg = a_val + b_val;
        `ALU_SUB: out_reg = a_val - b_val;
        `ALU_AND: out_reg = a_val & b_val;
        `ALU_OR: out_reg = a_val | b_val;
        `ALU_XOR: out_reg = a_val ^ b_val;
        `ALU_SLT: begin
            a_val_signed = a_val;
            b_val_signed = b_val;
            out_reg = (a_val_signed < b_val_signed) ? 1 : 0;
        end
        `ALU_SLTU: out_reg = a_val < b_val ? 1 : 0;
        `ALU_SLL: out_reg = a_val << b_val[4:0];
        `ALU_SRL: out_reg = a_val >> b_val[4:0];
        `ALU_SRA: begin
            a_val_signed = a_val;
            b_val_signed = b_val;
            out_reg = a_val_signed >>> b_val_signed[4:0];
        end
        `ALU_BSEL: out_reg = b_val;
        default: out_reg = 0;
    endcase
end

// ALUsels to make: add, sub, and, or, xor, slt, sltu, sll, srl, sra, bsel
assign out_val = out_reg;
    
endmodule