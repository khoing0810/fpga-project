// make a testbench for the ALU module
`timescale 1ns/1ns
module alu_tb();
    
    // Inputs
    reg signed [31:0] a;
    reg signed [31:0] b;
    reg [3:0] op;

    // Outputs
    wire signed [31:0] out; // 'signed' just for representation, doesn't affect functionality

    // Instantiate the Unit Under Test (UUT)
    alu dut (
        .a_val(a), 
        .b_val(b), 
        .alu_sel(op), 
        .out_val(out)
    );

    initial begin
        // ADD
        a = 1;
        b = 2;
        op = 0;
        #1;
        $display("a = %d, b = %d, op = add, out = %d", a, b, out);
        assert(out == 3) else $fatal("Test failed!");

        a = -100;
        b = 2;
        op = 0;
        #1;
        $display("a = %d, b = %d, op = add, out = %d", a, b, out);
        assert(out == -98) else $fatal("Test failed!");

        // SUB
        a = 1;
        b = 2;
        op = 1;
        #1;
        $display("a = %d, b = %d, op = sub, out = %d", a, b, out);
        assert(out == -1) else $fatal("Test failed!");

        // SLT
        a = -5;
        b = -4;
        op = 5;
        #1;
        $display("a = %d, b = %d, op = slt, out = %d", a, b, out);
        assert(out == 1) else $fatal("Test failed!");

        // SLTU
        a = -1;
        b = 1;
        op = 6;
        #1;
        $display("a = %d, b = %d, op = sltu, out = %d", a, b, out);
        assert(out == 0) else $fatal("Test failed!");

        // SRA
        a = -8;
        b = 1;
        op = 9;
        #1;
        $display("a = %d, b = %d, op = sra, out = %d", a,b,out);
        assert(out == -4) else $fatal("Test failed!");

        a = 8;
        b = 2;
        op = 9;
        #1;
        $display("a = %d, b = %d, op = sra, out = %d", a,b,out);
        assert(out == 2) else $fatal("Test failed!");



        $display("Test passed!");
    end


endmodule