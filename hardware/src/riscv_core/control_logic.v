`include "opcode.vh"
`include "alu_code.vh"

module control_logic ( // ALL BASE SIGNALS ONLY (forwarding logic to be handled in CPU)
    input [31:0] inst,

    output reg reg_wen,
    output reg [2:0] imm_sel,
    output reg br_un,
    output reg a_sel,
    output reg b_sel,
    output reg [3:0] alu_sel,
    output reg [1:0] wb_sel,
    output reg [2:0] csr_sel,
    output reg csr_wen
);

wire [6:0] funct7;
wire [2:0] funct3;

assign funct7 = inst[31:25];
assign funct3 = inst[14:12];


always @(*) begin
    case (inst[6:0]) // opcode
        `OPC_ARI_RTYPE: begin // R-type
            reg_wen = 1;
            imm_sel = 3'b000; // shouldnt use immediate! so at 3'b000 should be NOP/nothing
            br_un = 0;
            a_sel = 0;
            b_sel = 0;
            alu_sel = (funct3 == 3'b000 & funct7 == 7'd0) ? `ALU_ADD :
                          (funct3 == 3'b000) ? `ALU_SUB : // add if 0 /sub if != 0
                          (funct3 == 3'b001) ? `ALU_SLL : // sll
                          (funct3 == 3'b010) ? `ALU_SLT : // slt
                          (funct3 == 3'b011) ? `ALU_SLTU : // sltu
                          (funct3 == 3'b100) ? `ALU_XOR : // xor
                          (funct3 == 3'b101 & funct7 == 7'd0) ? `ALU_SRL :
                          (funct3 == 3'b101) ? `ALU_SRA : // srl if 0 / sra if != 0
                          (funct3 == 3'b110) ? `ALU_OR : // or
                          (funct3 == 3'b111) ? `ALU_AND : // and
                          4'b0000;
            wb_sel = 2'b00;
            csr_sel = 3'b000;
            csr_wen = 0;
        end
        `OPC_ARI_ITYPE: begin // I-type
            reg_wen = 1;
            imm_sel = 3'b000;
            br_un = 0;
            a_sel = 0;
            b_sel = 1;
            alu_sel = (funct3 == 3'b000) ? `ALU_ADD : // add if 0 /sub if != 0
                          (funct3 == 3'b001) ? `ALU_SLL : // sll
                          (funct3 == 3'b010) ? `ALU_SLT : // slt
                          (funct3 == 3'b011) ? `ALU_SLTU : // sltu
                          (funct3 == 3'b100) ? `ALU_XOR : // xor
                          (funct3 == 3'b101 & funct7 == 7'd0) ? `ALU_SRL : 
                          (funct3 == 3'b101) ? `ALU_SRA : // srl if 0 / sra if != 0
                          (funct3 == 3'b110) ? `ALU_OR : // or
                          (funct3 == 3'b111) ? `ALU_AND : // and
                          4'b0000;
            wb_sel = 2'b00;
            csr_sel = 3'b000;
            csr_wen = 0;
        end
        `OPC_LOAD: begin // load
            reg_wen = 1;
            imm_sel = 3'b000;
            br_un = 0;
            a_sel = 0;
            b_sel = 1;
            alu_sel = `ALU_ADD;
            wb_sel = 2'b01;
            csr_sel = 3'b000;
            csr_wen = 0;
        end
        `OPC_STORE: begin // store
            reg_wen = 0;
            imm_sel = 3'b001;
            br_un = 0;
            a_sel = 0;
            b_sel = 1;
            alu_sel = `ALU_ADD;
            wb_sel = 2'b00;
            csr_sel = 3'b000;
            csr_wen = 0;
        end
        `OPC_BRANCH: begin // branch
            reg_wen = 0;
            imm_sel = 3'b010;
            br_un = funct3 == 3'b110 || funct3 == 3'b111;
            a_sel = 1;
            b_sel = 1;
            alu_sel = `ALU_ADD;
            wb_sel = 2'b00;
            csr_sel = 3'b000;
            csr_wen = 0;
        end
        `OPC_LUI: begin // lui
            reg_wen = 1;
            imm_sel = 3'b011;
            br_un = 0;
            a_sel = 0;
            b_sel = 1;
            alu_sel = `ALU_BSEL;
            wb_sel = 2'b00;
            csr_sel = 3'b000;
            csr_wen = 0;
        end
        `OPC_AUIPC: begin // auipc
            reg_wen = 1;
            imm_sel = 3'b011;
            br_un = 0;
            a_sel = 1;
            b_sel = 1;
            alu_sel = `ALU_ADD;
            wb_sel = 2'b00;
            csr_sel = 3'b000;
            csr_wen = 0;
        end
        `OPC_JAL: begin // jal
            reg_wen = 1;
            imm_sel = 3'b100;
            br_un = 0;
            a_sel = 1;
            b_sel = 1;
            alu_sel = `ALU_ADD;
            wb_sel = 2'b10;
            csr_sel = 3'b000;
            csr_wen = 0;
        end
        `OPC_JALR: begin // jalr
            reg_wen = 1;
            imm_sel = 3'b000;
            br_un = 0;
            a_sel = 0;
            b_sel = 1;
            alu_sel = `ALU_ADD;
            wb_sel = 2'b10;
            csr_sel = 3'b000;
            csr_wen = 0;
        end
        // CSR case
        `OPC_CSR: begin
            case (funct3)
                3'b001: begin // csrrw
                    reg_wen = 0;
                    imm_sel = 3'b101;
                    br_un = 0;
                    a_sel = 0;
                    b_sel = 1;
                    alu_sel = `ALU_ADD;
                    wb_sel = 2'b00;
                    csr_sel = 3'b000;
                    csr_wen = 1;
                end
                3'b101: begin // csrrwi
                    reg_wen = 0;
                    imm_sel = 3'b101;
                    br_un = 0;
                    a_sel = 0;
                    b_sel = 1;
                    alu_sel = `ALU_ADD;
                    wb_sel = 2'b00;
                    csr_sel = 3'b001;
                    csr_wen = 1;
                end
                default: begin
                    reg_wen = 0;
                    imm_sel = 3'b101;
                    br_un = 0;
                    a_sel = 0;
                    b_sel = 1;
                    alu_sel = `ALU_ADD;
                    wb_sel = 2'b00;
                    csr_sel = 3'b001;
                    csr_wen = 1;
                end
            endcase
        end
        default: begin
            reg_wen = 0;
            imm_sel = 3'b000;
            br_un = 0;
            a_sel = 0;
            b_sel = 1;
            alu_sel = `ALU_ADD;
            wb_sel = 2'b00;
            csr_sel = 3'b000;
            csr_wen = 0;
        end
    endcase
end

endmodule