`timescale 1ns/1ns
module control_tb();
    
    // inputs
    reg [31:0] inst;
    reg [31:0] pc;
    reg br_eq;
    reg br_lt;

    // outputs
    wire reg_wen;
    wire [2:0] imm_sel;
    wire br_un;
    wire [1:0] a_sel;
    wire [1:0] b_sel;
    wire [3:0] alu_sel;
    wire mem_wen; //MAY NOT NEED
    wire mem_sel;
    wire wb_sel;
    wire csr_sel;
    wire csr_wen;

    // Instantiate Unit Under Test (UUT)
    control_logic dut (
        .inst(inst),
        .pc(pc),
        .br_eq(br_eq),
        .br_lt(br_lt),
        .reg_wen(reg_wen),
        .imm_sel(imm_sel),
        .br_un(br_un),
        .a_sel(a_sel),
        .b_sel(b_sel),
        .alu_sel(alu_sel),
        .mem_wen(mem_wen),
        .wb_sel(wb_sel),
        .csr_sel(csr_sel),
        .csr_wen(csr_wen)
    );

    initial begin // Assumed no pipeline!
        // R-Type Test
        inst = 32'h00200013;
        pc = 4;

        assert(imm_sel == 3'b000) else $fatal("Test failed! imm_sel is not correct for instruction %04x", inst);
        assert(alu_sel == 4'b0000) else $fatal("Test failed! alu_sel is not correct for instruction %04x", inst);
        assert(a_sel == 2'd0) else $fatal("Test failed! a_sel is not correct for instruction %04x", inst);
        assert(b_sel == 2'd1) else $fatal("Test failed! b_sel is not correct for instruction %04x", inst);
        // I-Type Test

        // J-Type Test

        // B-Type Test
    end
endmodule