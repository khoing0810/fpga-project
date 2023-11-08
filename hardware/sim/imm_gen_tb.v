`timescale 1ns/1ns

module imm_gen_tb();

// Inputs
reg [31:0] inst;
reg [2:0] imm_sel; 

// Outputs
wire [31:0] imm;

imm_gen imm_gen(
    .inst(inst), 
    .imm_sel(imm_sel), 
    .imm(imm)
    );

initial begin
    `ifdef IVERILOG
        $dumpfile("imm_gen_tb.fst");
        $dumpvars(0, imm_gen_tb);
    `endif
    `ifndef IVERILOG
        $vcdpluson;
    `endif

     // Initialize Inputs
    imm_sel = 3'b000;
    inst = 32'b0;

    #(2);

    // Test I-type instruction 
    imm_sel = 3'b000; 
    inst = 32'h00300113; // addi x2, x0, 3
    #(2);
    assert (imm == 32'd3) else $error("Test failed! Expected: %d, actual: %d. instruction in hex: %.4x", 32'd3, imm, inst);

    // Test B-type instruction
    imm_sel = 3'b010; 
    inst = 32'h00628263; // beq x1, x2, 4
    #(2);
    assert (imm == 32'd4) else $error("Test failed! Expected: %d, actual: %d. instruction in hex: %.4x", 32'd4, imm, inst);

    // Test S-type instruction
    imm_sel = 3'b001; 
    inst = 32'h001120A3; // sw x1, 1(x2)
    #(2);
    assert (imm == 32'd1) else $error("Test failed! Expected: %d, actual: %d. instruction in hex: %.4x", 32'd4, imm, inst);

    // Test U-type instruction
    imm_sel = 3'b011; 
    inst = 32'h00001137; // lui x2, 0x1
    #(2);
    assert (imm == 32'b1 << 12) else $error("Test failed! Expected: %d, actual: %d. instruction in hex: %.4x",  32'b1 << 12, imm, inst);

    // Test J-type instruction
    imm_sel = 3'b100;
    inst = 32'h004000EF; // jal x1 4
    #(2);
    assert (imm == 32'd4) else $error("Expected: %d, actual: %d", 32'd4, imm);

    // Finish simulation
    $finish;
end
endmodule