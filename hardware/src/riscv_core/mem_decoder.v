`include "opcode.vh"
module mem_decoder (
    input   [31:0]  inst,
    input   [31:0]  imm,
    input   [31:0]  mem_mux,
    output reg [3:0]   mem_wen,    
    output reg [31:0]  mem_mux_wb
);

always @(*) begin
    if (inst[6:0] == `OPC_STORE) begin
        case (inst[14:12])
            `FNC_SB: begin
                mem_wen = 4'b0001 << imm[1:0];
                mem_mux_wb = {24'd0, mem_mux[7:0]} << (imm[1:0] * 8); // shift 1 byte = shift 8 bits
            end
            `FNC_SH: begin
                mem_wen = 4'b0011 << imm[1:0];
                mem_mux_wb = {24'd0, mem_mux[15:0]} << (imm[1:0] * 8); // shift 1 byte = shift 8 bits
            end
            `FNC_SW: begin
                mem_wen = 4'b1111;
                mem_mux_wb = mem_mux;
            end
            default: mem_wen = 4'b0000;
        endcase
    end
    else if (inst[6:0] == `OPC_LOAD) begin
        case (inst[14:12])
            `FNC_LB: begin
                case (imm[1:0]) 
                    2'b00: mem_mux_wb = {{24{mem_mux[7]}}, mem_mux[7:0]};
                    2'b01: mem_mux_wb = {{24{mem_mux[15]}}, mem_mux[15:8]};
                    2'b10: mem_mux_wb = {{24{mem_mux[23]}}, mem_mux[23:16]};
                    2'b11: mem_mux_wb = {{24{mem_mux[31]}}, mem_mux[31:24]};
                endcase
            end
            `FNC_LBU: begin
                case (imm[1:0]) 
                    2'b00: mem_mux_wb = {{24{1'b0}}, mem_mux[7:0]};
                    2'b01: mem_mux_wb = {{24{1'b0}}, mem_mux[15:8]};
                    2'b10: mem_mux_wb = {{24{1'b0}}, mem_mux[23:16]};
                    2'b11: mem_mux_wb = {{24{1'b0}}, mem_mux[31:24]};
                endcase
            end
            `FNC_LH: begin
                case (imm[1:0]) 
                    2'b00: mem_mux_wb = {{16{mem_mux[15]}}, mem_mux[15:0]};
                    2'b01: mem_mux_wb = {{16{mem_mux[23]}}, mem_mux[23:8]};
                    2'b10: mem_mux_wb = {{16{mem_mux[31]}}, mem_mux[31:16]};
                    default: mem_mux_wb = 32'd0;
                endcase
            end
            `FNC_LHU: begin
                case (imm[1:0]) 
                    2'b00: mem_mux_wb = {{16{1'b0}}, mem_mux[15:0]};
                    2'b01: mem_mux_wb = {{16{1'b0}}, mem_mux[23:8]};
                    2'b10: mem_mux_wb = {{16{1'b0}}, mem_mux[31:16]};
                    default: mem_mux_wb = 32'd0;
                endcase
            end
            `FNC_LW: mem_mux_wb = mem_mux;
            default: mem_mux_wb = 32'd0;
        endcase
        mem_wen = 4'b0000;
    end
    else begin
        mem_wen = 4'b0000;
        mem_mux_wb = 0;
    end
end
endmodule