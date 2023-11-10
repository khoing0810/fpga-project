`include "opcode.vh"
module cpu #(
    parameter CPU_CLOCK_FREQ = 50_000_000,
    parameter RESET_PC = 32'h4000_0000,
    parameter BAUD_RATE = 115200
) (
    input clk,
    input rst,
    input serial_in,
    output serial_out
);
    localparam NOP = 32'h0000_0013;

    wire [11:0] bios_addra, bios_addrb;
    wire [31:0] bios_douta, bios_doutb;
    wire bios_ena, bios_enb;

    // MUXs (could remove aluwb_mux and just use wb_mux)
    reg [31:0] pc_mux; // mux between pc+4, alu_out, and jump_addr (done in combinational logic)
    wire [31:0] mem_mux, a_mux, b_mux, wb_mux, aluwb_mux; 
    wire [31:0] rs1_exmux, rs2_exmux, inst_mux, rs1_mux, rs2_mux; 
    // rs1_exmux = mux between rs1_id2ex and aluwb_mux
    // rs2_exmux = mux between rs2_id2ex and aluwb_mux
    // inst_mux = mux between bios_douta and imem_doutb and NOP
    // rs1_mux = mux between ra1 and aluwb_mux
    // rs2_mux = mux between ra2 and aluwb_mux



    // Signals/values determined in CPU
    wire [2:0] pc_sel, mem_sel;
    wire nop_sel;
    wire rs1ex_sel, rs2ex_sel, aluwb_sel, rs1_fwd_sel, rs2_fwd_sel; // will be a mux between the actual A/B mux and the forwarding mux
    // rs1ex_sel = mux between rs1_id2ex and aluwb_mux
    // rs2ex_sel = mux between rs2_id2ex and aluwb_mux
    // aluwb_sel = mux between wb_mux and alu_fwd
    // rs1_fwd_sel = mux between ra1 and aluwb_mux
    // rs2_fwd_sel = mux between ra2 and aluwb_mux
    wire [31:0] alu_out, alu_fwd;
    wire [31:0] imm;
    
    // PIPELINE REGISTERS
    reg [31:0] inst [3:0]; // use this to hold 3 instrutions in the pipeline, and the instruction that just finished
    reg [31:0] rs1_id2ex; // rs1 value from the ID stage
    reg [31:0] rs1_id2ex_nomux; // rs1 (non-muxed) value from the ID stage
    reg [31:0] rs2_id2ex; // rs2 value from the ID stage

    reg [31:0] imm_gen_id2ex; // immediate from the ID stage
    
    reg [31:0] pc_id2ex; // PC from the ID stage
    reg [31:0] pc_ex2mw; // PC from the EX stage
    
    reg [31:0] alu_ex2mw; // ALU result from the EX stage

    // Extra registers
    reg [31:0] pc = RESET_PC; // program counter
    reg [31:0] cycle_count = 0; // number of cycles executed
    reg [31:0] instruction_counter = 0; // number of cycles executed
    reg [31:0] tohost_csr = 0; // tohost CSR

   

    // Control signals
    wire [2:0] imm_sel;
    wire reg_wen;
    wire br_un;
    wire a_sel;
    wire b_sel;
    wire [3:0] alu_sel;
    wire [3:0] mem_wen; // MAY NEED TO CALCULATE IN CPU ITSELF
    wire [2:0] csr_sel;
    wire [1:0] wb_sel;
    wire csr_wen;
    
    
    initial begin
        // code that will be executed at the beginning of the simulation/when the reset signal is asserted
        inst[0] = NOP;
        inst[1] = NOP;
        inst[2] = NOP;
        inst[3] = NOP;
    end
    
    bios_mem bios_mem (
      .clk(clk),
      .ena(bios_ena),
      .addra(bios_addra),
      .douta(bios_douta),
      .enb(bios_enb),
      .addrb(bios_addrb),
      .doutb(bios_doutb)
    );

    // Data Memory
    // Synchronous read: read takes one cycle
    // Synchronous write: write takes one cycle
    // Write-byte-enable: select which of the four bytes to write
    wire [13:0] dmem_addr;
    wire [31:0] dmem_din, dmem_dout;
    wire [3:0] dmem_we;
    wire dmem_en;
    dmem dmem (
      .clk(clk),
      .en(dmem_en),
      .we(dmem_we),
      .addr(dmem_addr),
      .din(dmem_din),
      .dout(dmem_dout)
    );

    // Instruction Memory
    // Synchronous read: read takes one cycle
    // Synchronous write: write takes one cycle
    // Write-byte-enable: select which of the four bytes to write
    wire [31:0] imem_dina, imem_doutb;
    wire [13:0] imem_addra, imem_addrb;
    wire [3:0] imem_wea;
    wire imem_ena;
    imem imem (
      .clk(clk),
      .ena(imem_ena),
      .wea(imem_wea),
      .addra(imem_addra),
      .dina(imem_dina),
      .addrb(imem_addrb),
      .doutb(imem_doutb)
    );

    // Register file
    // Asynchronous read: read data is available in the same cycle
    // Synchronous write: write takes one cycle
    wire we;
    wire [4:0] ra1, ra2, wa;
    wire [31:0] wd;
    wire [31:0] rd1, rd2;
    reg_file rf (
        .clk(clk),
        .we(we),
        .ra1(ra1), .ra2(ra2), .wa(wa),
        .wd(wd),
        .rd1(rd1), .rd2(rd2)
    );

    // On-chip UART
    //// UART Receiver
    wire [7:0] uart_rx_data_out;
    wire uart_rx_data_out_valid;
    wire uart_rx_data_out_ready;
    //// UART Transmitter
    wire [7:0] uart_tx_data_in;
    wire uart_tx_data_in_valid;
    wire uart_tx_data_in_ready;
    uart #(
        .CLOCK_FREQ(CPU_CLOCK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) on_chip_uart (
        .clk(clk),
        .reset(rst),

        .serial_in(serial_in),
        .data_out(uart_rx_data_out),
        .data_out_valid(uart_rx_data_out_valid),
        .data_out_ready(uart_rx_data_out_ready),

        .serial_out(serial_out),
        .data_in(uart_tx_data_in),
        .data_in_valid(uart_tx_data_in_valid),
        .data_in_ready(uart_tx_data_in_ready)
    );


    // TODO: Your code to implement a fully functioning RISC-V core
    // Add as many modules as you want
    // Feel free to move the memory modules around

    // MAKE SURE regwen comes from the instruction used for mem/wb

    always @(posedge clk) begin
      if (rst) begin
        //   bios_ena <= 0;
        //   bios_addrb <= 0;
        //   bios_doutb <= 0;
        pc <= RESET_PC;
        inst[0] <= NOP;
        inst[1] <= NOP;
        inst[2] <= NOP;
        inst[3] <= NOP;
      end
      else begin
          inst[1] <= inst[0]; // move the instruction from the IF stage to the ID stage
          inst[2] <= inst[1]; // move the instruction from the ID stage to the EX stage
          inst[3] <= inst[2]; // move the instruction from the EX stage to the MEM stage
          // (inst[0] is set in always (*)) // inst[0] <= inst_mux;

          // PC
          pc <= pc_mux;
          pc_id2ex <= pc;
          pc_ex2mw <= pc_id2ex;
          
          // ID to EX
          rs1_id2ex_nomux <= rd1;
          rs1_id2ex <= rd1; // TODO: add forwarding logic
          rs2_id2ex <= rd2; // TODO: add forwarding logic
          imm_gen_id2ex <= imm;
        
          // EX to MEM
          alu_ex2mw <= alu_out;

          // Instruction and cycle counters
          if (inst[3] != NOP) begin
            instruction_counter <= instruction_counter + 1;
          end
          cycle_count <= cycle_count + 1;
      end
    end

    // ID stage
    control_logic control_logic1 ( // only need imm_sel since that is the only signal that we need in this stage
        .inst(inst[0]),
        .imm_sel(imm_sel),
        .reg_wen(),
        .br_un(),
        .a_sel(),
        .b_sel(),
        .alu_sel(),
        .mem_wen(),
        .mem_sel(),
        .wb_sel(),
        .csr_sel(),
        .csr_wen()
    );

    // EX stage
    control_logic control_logic2 (
        .inst(inst[1]),
        .alu_sel(alu_sel),
        .mem_wen(mem_wen), // should put mem signals in the EX stage because we need to set that value before we get to the mem stage
        .mem_sel(mem_sel),
        .a_sel(a_sel),
        .b_sel(b_sel),
        .imm_sel(),
        .reg_wen(),
        .br_un(),
        .wb_sel(),
        .csr_sel(),
        .csr_wen()
    ); 

    // MEM+WB+IF stage
    control_logic control_logic3 (
        .inst(inst[2]),
        .reg_wen(we),
        .wb_sel(wb_sel),
        .csr_sel(csr_sel),
        .csr_wen(csr_wen),
        .imm_sel(),
        .br_un(),
        .a_sel(),
        .b_sel(),
        .alu_sel(),
        .mem_wen(),
        .mem_sel()
    );

    

/*
    wire [31:0] mem_mux_wb;
    
    always @(*) begin
        if (inst[6:0] == `OPC_STORE) begin
            case (funct3)
                `FNC_SB: mem_wen = 4'b0001 << (imm_gen_id2ex % 4);
                `FNC_SH: mem_wen = 4'b0011 << (imm_gen_id2ex % 4);
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
        
    end
*/
    // IF/ID stage signals/values/modules 
    assign pc_sel = 3'd0; //TODO: change for hazards
    //assign pc_mux = (pc_sel == 3'd0) ? pc + 4 : (pc_sel == 3'd1) ? alu_out: RESET_PC; // TODO: add jump_addr, branch_addr
    assign bios_addra = pc[11:0];
    assign imem_addrb = pc[13:0];
    assign nop_sel = 1; //TODO: change for hazards
    assign inst_mux = rst ? NOP : (nop_sel == 1'b1) ? (pc[30] == 1'b1 ? bios_douta : imem_doutb) : NOP; 
    
    assign rs1_fwd_sel = 0; /*(rs1_id2ex == 0) ? 1'd0 : (rs1_id2ex == wa) ? 1'd1 : (rs1_id2ex == wa) ? 1'd2 : 1'd0;*/
    assign rs2_fwd_sel = 0; /*(rs2_id2ex == 0) ? 1'd0 : (rs2_id2ex == wa) ? 1'd1 : (rs2_id2ex == wa) ? 1'd2 : 1'd0;*/
    assign ra1 = inst_mux[19:15];
    assign ra2 = inst_mux[24:20];
    

    assign aluwb_mux = (aluwb_sel == 1'd0) ? wb_mux : alu_fwd;
    assign rs1_mux = (rs1_fwd_sel == 1'd0) ? rs1_id2ex : aluwb_mux;
    assign rs2_mux = (rs2_fwd_sel == 1'd0) ? rs2_id2ex : aluwb_mux;

    imm_gen imm_gen (
        .inst(inst_mux),
        .imm_sel(imm_sel), //TODO
        .imm(imm)
    );

    always @(*) begin // Used because of combinational logic
        pc_mux = rst ? RESET_PC: (pc_sel == 3'd0) ? pc + 1 : (pc_sel == 3'd1) ? alu_out: RESET_PC;
        inst[0] = inst_mux;
    end

    // EX/intoMEM stage signals/values/modules
    assign a_mux = (a_sel == 1'd0) ? rs1_exmux : pc_id2ex;
    assign b_mux = (b_sel == 1'd0) ? rs2_exmux : imm_gen_id2ex;

    // EX forwarding signals
    assign rs1ex_sel = 0; /*(rs1_id2ex == 0) ? 1'd0 : (rs1_id2ex == wa) ? 1'd1 : (rs1_id2ex == wa) ? 1'd2 : 1'd0;*/
    assign rs2ex_sel = 0; /*(rs2_id2ex == 0) ? 1'd0 : (rs2_id2ex == wa) ? 1'd1 : (rs2_id2ex == wa) ? 1'd2 : 1'd0;*/
    assign aluwb_sel = 0; /*(wb_sel == 2'd0) ? 1'd0 : (wb_sel == 2'd1) ? 1'd1 : 1'd0;*/

    assign rs1_exmux = (rs1ex_sel == 1'd0) ? rs1_id2ex : aluwb_mux;
    assign rs2_exmux = (rs2ex_sel == 1'd0) ? rs2_id2ex : aluwb_mux;

    alu alu (
        .a_val(a_mux),
        .b_val(b_mux),
        .alu_sel(alu_sel),
        .out_val(alu_out)
    );

    bcmp bcmp (
        .a_val(rs1_exmux), // TODO: NEEDS TO BE THE FORWARDED VALUE
        .b_val(rs2_exmux), // TODO: NEEDS TO BE THE FORWARDED VALUE
        .br_un(br_un),
        .br_eq(), //TODO: use this values
        .br_lt() //TODO: use this value
    );

    assign dmem_addr = alu_out[13:0];
    assign dmem_din = rs2_exmux;

    // MEM/WB stage signals/values/modules
    assign alu_fwd = alu_ex2mw;
    assign mem_mux = (mem_sel == 3'd1) ? dmem_dout : dmem_dout; // TODO: 0: bios doutb, 1: dmem_dout, 2: cycle_counter, 3: inst_counter, 4: uart_dout 
    assign wb_mux = (wb_sel == 2'd0) ? alu_ex2mw : (wb_sel == 2'd1) ? mem_mux : pc_ex2mw + 1; // TODO: 0: mem_mux, 1: ALU fwd, 2: pc+4 
    assign wd = wb_mux; // we are writing the value from the wb_mux to the register file
    assign wa = inst[2][11:7]; // we are writing to the rd register (reg_wen) already set

endmodule
