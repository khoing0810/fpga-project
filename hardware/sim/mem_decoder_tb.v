`include "opcode.vh"

module mem_decoder_tb();

// Inputs
reg [31:0] inst;
reg [31:0] imm;
reg [31:0]  mem_mux;

// Outputs
wire  [3:0]   mem_wen;   
wire  [31:0]  mem_mux_wb;

initial begin
    `ifdef IVERILOG
        $dumpfile("imm_gen_tb.fst");
        $dumpvars(0, imm_gen_tb);
    `endif
    `ifndef IVERILOG
        $vcdpluson;
    `endif

    // Finish simulation
    $finish;
end
endmodule