`timescale 1ns/1ns
`include "opcode.vh"
module control_tb();
    
    // inputs
    reg [31:0] inst;
    reg [31:0] pc;
    // reg br_eq;
    // reg br_lt;

    // outputs
    wire reg_wen;
    wire [2:0] imm_sel;
    wire br_un;
    wire [1:0] a_sel;
    wire [1:0] b_sel;
    wire [3:0] alu_sel;
    wire mem_wen; //MAY NOT NEED
    wire mem_sel;
    wire [1:0] wb_sel;
    wire csr_sel;
    wire csr_wen;

    // Instantiate Unit Under Test (UUT)
    control_logic dut (
        .inst(inst),
        .pc(pc),
        // .br_eq(br_eq),
        // .br_lt(br_lt),
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
        `ifdef IVERILOG
            $dumpfile("imm_gen_tb.fst");
            $dumpvars(0, imm_gen_tb);
        `endif
        `ifndef IVERILOG
            $vcdpluson;
        `endif
        // I-Type Test
        inst = 32'h00200013; // addi x0, x0, 2
        pc = 4;

        #(2)

        assert(imm_sel == 3'b000) else $fatal("Test failed! imm_sel is not correct for instruction %04x", inst);
        assert(alu_sel == 4'b0000) else $fatal("Test failed! alu_sel is not correct for instruction %04x", inst);
        assert(a_sel == 1'd0) else $fatal("Test failed! a_sel is not correct for instruction %04x", inst);
        assert(b_sel == 1'd1) else $fatal("Test failed! b_sel is not correct for instruction %04x", inst);
        assert(wb_sel == 2'd0) else $fatal("Test failed! wb_sel is not correct for instruction %04x", inst);

        inst = 32'h008100E7; // jalr x1 x2 8
        #(2)
        assert(inst[6:0] == `OPC_JALR) else $fatal("Instruction is not JALR!");
        #(2)
        assert(imm_sel == 3'b000) else $fatal("Test failed! imm_sel is not correct for instruction %04x", inst);
        assert(alu_sel == 4'b0000) else $fatal("Test failed! alu_sel is not correct for instruction %04x", inst);
        assert(a_sel == 1'd0) else $fatal("Test failed! a_sel is not correct for instruction %04x", inst);
        assert(b_sel == 1'd1) else $fatal("Test failed! b_sel is not correct for instruction %04x", inst);
        assert(wb_sel == 2'd2) else $fatal("Test failed! wb_sel is not correct for instruction %04x", inst);

        
        // R-Type Test
        inst = 32'h00308133; // add x2, x1, x3

        #(2)
        assert(imm_sel == 3'b000) else $fatal("Test failed! imm_sel is not correct for instruction %04x", inst);
        assert(alu_sel == 4'b0000) else $fatal("Test failed! alu_sel is not correct for instruction %04x", inst);
        assert(a_sel == 1'd0) else $fatal("Test failed! a_sel is not correct for instruction %04x", inst);
        assert(b_sel == 1'd0) else $fatal("Test failed! b_sel is not correct for instruction %04x", inst);
        assert(wb_sel == 2'd0) else $fatal("Test failed! wb_sel is not correct for instruction %04x", inst);

        // J-Type Test
        inst = 32'h008000EF; // jal x1 8
        
        #(2)
        assert(imm_sel == 3'b100) else $fatal("Test failed! imm_sel is not correct for instruction %04x. imm_sel is %d", inst, imm_sel);
        assert(alu_sel == 4'b0000) else $fatal("Test failed! alu_sel is not correct for instruction %04x", inst);
        assert(a_sel == 1'd1) else $fatal("Test failed! a_sel is not correct for instruction %04x", inst);
        assert(b_sel == 1'd1) else $fatal("Test failed! b_sel is not correct for instruction %04x", inst);
        assert(wb_sel == 2'd2) else $fatal("Test failed! wb_sel is not correct for instruction %04x. curr wb_sel: %d\n", inst, wb_sel);
        assert(reg_wen == 1) else $fatal("Test failed! wb_sel is not correct for instruction %04x. curr wb_sel: %d\n", inst, wb_sel);

        // B-Type Test
        // beq, blt, bltu (make sure to check for br_un)
        inst = 32'h00310863; // beq x2 x3 16
        #(2)
        assert(imm_sel == 3'b010) else $fatal("Test failed! imm_sel is not correct for instruction %04x. imm_sel is %d", inst, imm_sel);
        assert(alu_sel == 4'b0000) else $fatal("Test failed! alu_sel is not correct for instruction %04x", inst);
        assert(a_sel == 1'd1) else $fatal("Test failed! a_sel is not correct for instruction %04x", inst);
        assert(b_sel == 1'd1) else $fatal("Test failed! b_sel is not correct for instruction %04x", inst);
        assert(reg_wen == 0) else $fatal("Test failed! reg_wen is not correct for instruction %04x.", reg_wen);

        inst = 32'h02316063; // bltu x2 x3 32
        #(2)
        assert(imm_sel == 3'b010) else $fatal("Test failed! imm_sel is not correct for instruction %04x. imm_sel is %d", inst, imm_sel);
        assert(alu_sel == 4'b0000) else $fatal("Test failed! alu_sel is not correct for instruction %04x", inst);
        assert(a_sel == 1'd1) else $fatal("Test failed! a_sel is not correct for instruction %04x", inst);
        assert(b_sel == 1'd1) else $fatal("Test failed! b_sel is not correct for instruction %04x", inst);
        assert(reg_wen == 0) else $fatal("Test failed! reg_wen is not correct for instruction %04x.", reg_wen);
        assert(br_un == 1) else $fatal("br_un should be 1");

        // S-Type Test
        // sw
        inst = 32'h00112223; // sw x1 4(x2)
        #(2)

        // L-Type Test
        // lw
        // CSR-Type Test
        // csrrw, csrrwi (make sure to check for csr_wen)

        // u-type
        // lui (do not check a_sel), auipc if possible


        $display("---Test passed!---");
        $finish;
    end
endmodule