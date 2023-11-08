module control_logic (
    input [31:0] inst,
    input [31:0] pc,
    input br_eq,
    input br_lt,

    output reg_wen,
    output [2:0] imm_sel,
    output br_un,
    output [1:0] a_sel,
    output [1:0] b_sel,
    output [3:0] alu_sel,
    output mem_wen, //MAY NOT NEED
    output mem_sel,
    output wb_sel,
    output csr_sel,
    output csr_wen
);

localparam [3:0] ALU_ADD = 4'b0000;
localparam [3:0] ALU_SUB = 4'b0001;
localparam [3:0] ALU_AND = 4'b0010;
localparam [3:0] ALU_OR = 4'b0011;
localparam [3:0] ALU_XOR = 4'b0100;
localparam [3:0] ALU_SLT = 4'b0101;
localparam [3:0] ALU_SLTU = 4'b0110;
localparam [3:0] ALU_SLL = 4'b0111;
localparam [3:0] ALU_SRL = 4'b1000;
localparam [3:0] ALU_SRA = 4'b1001;
localparam [3:0] ALU_BSEL = 4'b1010;

wire [6:0] funct7;
wire [3:0] funct3;

assign funct7 = inst[31:25];
assign funct3 = inst[14:12];

reg reg_wen_reg = 0;
reg [2:0] imm_sel_reg = 0;
reg br_un_reg = 0;
reg a_sel_reg = 0;
reg b_sel_reg = 0;
reg [3:0] alu_sel_reg = 0;
reg mem_wen_reg = 0; //MAY NOT NEED
reg mem_sel_reg = 0;
reg wb_sel_reg = 0;
reg csr_sel_reg = 0;
reg csr_wen_reg = 0;
reg pc_sel_reg;

always @(*) begin
    case (inst[6:0])
        7'b0110011: begin // R-type
            reg_wen_reg = 1;
            imm_sel_reg = 3'b000; // shouldnt use immediate! so at 3'b000 should be NOP/nothing
            br_un_reg = 0;
            a_sel_reg = 2'b00;
            b_sel_reg = 2'b10;
            alu_sel_reg = (funct3 == 3'b000 & funct7 == 7'd0) ? ALU_ADD :
                          (funct3 == 3'b000) ? ALU_SUB : // add if 0 /sub if != 0
                          (funct3 == 3'b001) ? ALU_SLL : // sll
                          (funct3 == 3'b010) ? ALU_SLT : // slt
                          (funct3 == 3'b011) ? ALU_SLTU : // sltu
                          (funct3 == 3'b100) ? ALU_XOR : // xor
                          (funct3 == 3'b101 & funct7 == 7'd0) ? ALU_SRL :
                          (funct3 == 3'b101) ? ALU_SRA : // srl if 0 / sra if != 0
                          (funct3 == 3'b110) ? ALU_OR : // or
                          (funct3 == 3'b111) ? ALU_AND : // and
                          4'b0000;
            mem_wen_reg = 0;
            mem_sel_reg = 0;
            wb_sel_reg = 2'b01;
            csr_sel_reg = 3'b000;
            csr_wen_reg = 0;
        end
        7'b0010011: begin // I-type
            reg_wen_reg = 1;
            imm_sel_reg = 3'b000;
            br_un_reg = 0;
            a_sel_reg = 2'b00;
            b_sel_reg = 2'b01;
            alu_sel_reg = (funct3 == 3'b000) ? ALU_ADD : // add if 0 /sub if != 0
                          (funct3 == 3'b001) ? ALU_SLL : // sll
                          (funct3 == 3'b010) ? ALU_SLT : // slt
                          (funct3 == 3'b011) ? ALU_SLTU : // sltu
                          (funct3 == 3'b100) ? ALU_XOR : // xor
                          (funct3 == 3'b101 & funct7 == 7'd0) ? ALU_SRL : 
                          (funct3 == 3'b101) ? ALU_SRA : // srl if 0 / sra if != 0
                          (funct3 == 3'b110) ? ALU_OR : // or
                          (funct3 == 3'b111) ? ALU_AND : // and
                          4'b0000;
            mem_wen_reg = 0;
            mem_sel_reg = 0;
            wb_sel_reg = 2'b01;
            csr_sel_reg = 3'b000;
            csr_wen_reg = 0;
        end
        7'b0000011: begin // load
            reg_wen_reg = 1;
            imm_sel_reg = 3'b000;
            br_un_reg = 0;
            a_sel_reg = 2'b00;
            b_sel_reg = 2'b10;
            alu_sel_reg = ALU_ADD;
            mem_wen_reg = 0;
            mem_sel_reg = 0;
            wb_sel_reg = 2'b00;
            csr_sel_reg = 3'b000;
            csr_wen_reg = 0;
        end
        7'b0100011: begin // store
            reg_wen_reg = 0;
            imm_sel_reg = 3'b001;
            br_un_reg = 0;
            a_sel_reg = 2'b00;
            b_sel_reg = 2'b01;
            alu_sel_reg = 4'b0000;
            mem_wen_reg = 1;
            mem_sel_reg = 0;
            wb_sel_reg = 2'b00;
            csr_sel_reg = 3'b000;
            csr_wen_reg = 0;
        end
        7'b1100011: begin // branch
            reg_wen_reg = 0;
            imm_sel_reg = 3'b010;
            br_un_reg = 0;
            a_sel_reg = 2'b0;
            b_sel_reg = 2'b01;
            alu_sel_reg = 4'b0000;
            mem_wen_reg = 0;
            mem_sel_reg = 0;
            wb_sel_reg = 2'b00;
            csr_sel_reg = 3'b000;
            csr_wen_reg = 0;
        end
        7'b0010111: begin // lui
            reg_wen_reg = 1;
            imm_sel_reg = 3'b011;
            br_un_reg = 0;
            a_sel_reg = 2'b0;
            b_sel_reg = 2'b01;
            alu_sel_reg = 4'b0000;
            mem_wen_reg = 0;
            mem_sel_reg = 0;
            wb_sel_reg = 2'b00;
            csr_sel_reg = 3'b000;
            csr_wen_reg = 0;
        end
        7'b0110111: begin // auipc
            reg_wen_reg = 1;
            imm_sel_reg = 3'b011;
            br_un_reg = 0;
            a_sel_reg = 2'b0;
            b_sel_reg = 2'b01;
            alu_sel_reg = 4'b0000;
            mem_wen_reg = 0;
            mem_sel_reg = 0;
            wb_sel_reg = 2'b00;
            csr_sel_reg = 3'b000;
            csr_wen_reg = 0;
        end
        7'b1101111: begin // jal
            reg_wen_reg = 1;
            imm_sel_reg = 3'b100;
            br_un_reg = 0;
            a_sel_reg = 2'b0;
            b_sel_reg = 2'b01;
            alu_sel_reg = 4'b0000;
            mem_wen_reg = 0;
            mem_sel_reg = 0;
            wb_sel_reg = 2'b00;
            csr_sel_reg = 3'b000;
            csr_wen_reg = 0;
        end
        7'b1100111: begin // jalr
            pc_sel_reg = 3'b011;
            reg_wen_reg = 1;
            imm_sel_reg = 3'b000;
            br_un_reg = 0;
            a_sel_reg = 2'b0;
            b_sel_reg = 2'b1;
            alu_sel_reg = 4'b0000;
            mem_wen_reg = 0;
            mem_sel_reg = 0;
            wb_sel_reg = 2'b00;
            csr_sel_reg = 3'b000;
            csr_wen_reg = 0;
        end
        // CSR case
        7'b1110011: begin
            case (inst[31:20])
                12'b000000000000: begin // ecall
                    reg_wen_reg = 0;
                    imm_sel_reg = 3'b000;
                    br_un_reg = 0;
                    a_sel_reg = 2'b0;
                    b_sel_reg = 2'b1;
                    alu_sel_reg = 4'b0000;
                    mem_wen_reg = 0;
                    mem_sel_reg = 0;
                    wb_sel_reg = 2'b00;
                    csr_sel_reg = 3'b000;
                    csr_wen_reg = 0;
                end
                12'b000000000001: begin // ebreak
                    reg_wen_reg = 0;
                    imm_sel_reg = 3'b000;
                    br_un_reg = 0;
                    a_sel_reg = 2'b0;
                    b_sel_reg = 2'b1;
                    alu_sel_reg = 4'b0000;
                    mem_wen_reg = 0;
                    mem_sel_reg = 0;
                    wb_sel_reg = 2'b00;
                    csr_sel_reg = 3'b000;
                    csr_wen_reg = 0;
                end
                12'b000000000010: begin // uret
                    reg_wen_reg = 0;
                    imm_sel_reg = 3'b000;
                    br_un_reg = 0;
                    a_sel_reg = 2'b0;
                    b_sel_reg = 2'b1;
                    alu_sel_reg = 4'b0000;
                    mem_wen_reg = 0;
                    mem_sel_reg = 0;
                    wb_sel_reg = 2'b00;
                    csr_sel_reg = 3'b000;
                    csr_wen_reg = 0;
                end
                12'b000100000010: begin // sret
                    reg_wen_reg = 0;
                    imm_sel_reg = 3'b000;
                    br_un_reg = 0;
                    a_sel_reg = 2'b0;
                    b_sel_reg = 2'b1;
                    alu_sel_reg = 4'b0000;
                    mem_wen_reg = 0;
                    mem_sel_reg = 0;
                    wb_sel_reg = 2'b00;
                    csr_sel_reg = 3'b000;
                    csr_wen_reg = 0;
                end
            endcase
        end
        default: begin
            reg_wen_reg = 0;
            imm_sel_reg = 3'b000;
            br_un_reg = 0;
            a_sel_reg = 2'b0;
            b_sel_reg = 2'b1;
            alu_sel_reg = 4'b0000;
            mem_wen_reg = 0;
            mem_sel_reg = 0;
            wb_sel_reg = 2'b00;
            csr_sel_reg = 3'b000;
            csr_wen_reg = 0;
        end
    endcase
end

assign reg_wen = reg_wen_reg;
assign imm_sel = imm_sel_reg;
assign br_un = br_un_reg;
assign a_sel = a_sel_reg;
assign b_sel = b_sel_reg;
assign alu_sel = alu_sel_reg;
assign mem_wen = mem_wen_reg; //MAY NOT NEED
assign mem_sel = mem_sel_reg;
assign wb_sel = wb_sel_reg;
assign csr_sel = csr_sel_reg;
assign csr_wen = csr_wen_reg;

endmodule