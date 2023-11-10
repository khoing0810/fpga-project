`include "opcode.vh"
module mem_decoder (
    input   [31:0]  inst,
    input   [31:0]  imm,
    input   [31:0]  mem_mux,
    output  [3:0]   mem_wen,    
    output  [31:0]  mem_mux_wb
);

assign funct3 = inst[14:12];

always @(*) begin
     /*
    if (inst[6:0] == `OPC_STORE) begin
            case (funct3)
                `FNC_SB: mem_wen = 4'b0001 << (imm % 4);
                `FNC_SH: mem_wen = 4'b0011 << (imm % 4);
                `FNC_SW: mem_wen = 4'b1111;
                default: mem_wen = 4'b0000;
            endcase
    end
    else if (inst[6:0] == `OPC_LOAD) begin
        case (funct3)
            `FNC_LB: mem_mux_wb = {{24{mem_mux[(imm_gen_id2ex % 4)*8+7]}}, 
                                    mem_mux[(imm_gen_id2ex % 4)*8+7:(imm_gen_id2ex % 4)*8]};
            `FNC_LH: mem_mux_wb = {{16{mem_mux[(imm_gen_id2ex % 4)*8+15]}},
                                    mem_mux[(imm_gen_id2ex % 4)*8+15:(imm_gen_id2ex % 4)*8]};
            `FNC_LW: mem_mux_wb = mem_mux;
            default: mem_mux_wb = 0;
        endcase
    end
    else begin
        mem_wen = 4'b0000;
        mem_mux = 0;
    end
    */
end

endmodule