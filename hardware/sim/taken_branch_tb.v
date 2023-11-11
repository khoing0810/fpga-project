`timescale 1ns/1ns

module taken_branch_tb();

// inputs
reg [31:0] inst;
reg br_eq;
reg br_lt;

// output
reg taken;

taken_branch taken_branch (
    .inst(inst),
    .br_eq(br_eq),
    .br_lt(br_lt),
    .taken(taken)
);

initial begin
    `ifdef IVERILOG
        $dumpfile("imm_gen_tb.fst");
        $dumpvars(0, imm_gen_tb);
    `endif
    `ifndef IVERILOG
        $vcdpluson;
    `endif

    // blt x1 x2 12
    inst = 32'h0020C663;

    br_eq = 1;
    br_lt = 0;
    #(2)
    assert(!taken) else $error("Test failed. br_eq: %d | br_lt: %d. Expected taken: %d", br_eq, br_lt, !taken);

    br_eq = 0;
    br_lt = 1;
    #(2)
    assert(taken) else $error("Test failed. br_eq: %d | br_lt: %d. Expected taken: %d", br_eq, br_lt, taken);

    br_eq = 0;
    br_lt = 0;
    #(2)
    assert(!taken) else $error("Test failed. br_eq: %d | br_lt: %d. Expected taken: %d", br_eq, br_lt, !taken);

    // bltu x1 x2 12
    inst = 32'h0020E663;
    #(2)
    br_eq = 1;
    br_lt = 0;
    #(2)
    assert(!taken) else $error("Test failed. br_eq: %d | br_lt: %d. Expected taken: %d", br_eq, br_lt, !taken);

    br_eq = 0;
    br_lt = 1;
    #(2)
    assert(taken) else $error("Test failed. br_eq: %d | br_lt: %d. Expected taken: %d", br_eq, br_lt, taken);

    br_eq = 0;
    br_lt = 0;
    #(2)
    assert(!taken) else $error("Test failed. br_eq: %d | br_lt: %d. Expected taken: %d", br_eq, br_lt, !taken);

    // bge x1 x2 12
    inst = 32'h0020D663;
    #(2)
    br_eq = 1;
    br_lt = 0;
    #(2)
    assert(taken) else $error("Test failed. br_eq: %d | br_lt: %d. Expected taken: %d", br_eq, br_lt, taken);

    br_eq = 0;
    br_lt = 1;
    #(2)
    assert(!taken) else $error("Test failed. br_eq: %d | br_lt: %d. Expected taken: %d", br_eq, br_lt, !taken);

    br_eq = 0;
    br_lt = 0;
    #(2)
    assert(taken) else $error("Test failed. br_eq: %d | br_lt: %d. Expected taken: %d", br_eq, br_lt, taken);

    // bgeu x1 x2 12
    inst = 32'h0020F663;
    #(2)
    br_eq = 1;
    br_lt = 0;
    #(2)
    assert(taken) else $error("Test failed. br_eq: %d | br_lt: %d. Expected taken: %d", br_eq, br_lt, taken);

    br_eq = 0;
    br_lt = 1;
    #(2)
    assert(!taken) else $error("Test failed. br_eq: %d | br_lt: %d. Expected taken: %d", br_eq, br_lt, !taken);

    br_eq = 0;
    br_lt = 0;
    #(2)
    assert(taken) else $error("Test failed. br_eq: %d | br_lt: %d. Expected taken: %d", br_eq, br_lt, taken);

    // beq x1 x2 12
    inst = 32'h00208663;
    br_eq = 1;
    br_lt = 0;
    #(2)
    assert(taken) else $error("Test failed. br_eq: %d | br_lt: %d. Expected taken: %d", br_eq, br_lt, taken);

    br_eq = 0;
    br_lt = 1;
    #(2)
    assert(!taken) else $error("Test failed. br_eq: %d | br_lt: %d. Expected taken: %d", br_eq, br_lt, !taken);

    br_eq = 0;
    br_lt = 0;
    #(2)
    assert(!taken) else $error("Test failed. br_eq: %d | br_lt: %d. Expected taken: %d", br_eq, br_lt, !taken);

    // bne x1 x2 12
    inst = 32'h00209663;
    br_eq = 1;
    br_lt = 0;
    #(2)
    assert(!taken) else $error("Test failed. br_eq: %d | br_lt: %d. Expected taken: %d", br_eq, br_lt, !taken); // FAILED

    br_eq = 0;
    br_lt = 1;
    #(2)
    assert(taken) else $error("Test failed. br_eq: %d | br_lt: %d. Expected taken: %d", br_eq, br_lt, taken); // FAILED

    br_eq = 0;
    br_lt = 0;
    #(2)
    assert(taken) else $error("Test failed. br_eq: %d | br_lt: %d. Expected taken: %d", br_eq, br_lt, taken); // FAILED

    $display("Test passed!");
    // finish simulation
    $finish;
end
endmodule