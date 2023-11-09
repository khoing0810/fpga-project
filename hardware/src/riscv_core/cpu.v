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

    // BIOS Memory
    // Synchronous read: read takes one cycle
    // Synchronous write: write takes one cycle
    wire [11:0] bios_addra, bios_addrb;
    wire [31:0] bios_douta, bios_doutb;
    wire bios_ena, bios_enb;
    wire pc_mux, pc_sel, mem_mux, mem_sel;
    wire a_mux, a_sel, b_mux, b_sel;
    wire a_fwdmux, a_fwd_sel, b_fwdmux, b_fwd_sel; // will be a mux between the actual A/B mux and the forwarding mux
    wire wb_mux, wb_sel, aluwb_mux, aluwb_sel;
    wire alu_fwd;
    wire [31:0] alu_out;
    wire [31:0] inst_mux;
    
    // PIPELINE REGISTERS
    reg [31:0] inst [3:0]; // use this to hold 3 instrutions in the pipeline, and the instruction that just finished
        //[0] = if2id (right after fetch)
        //[1] = id2ex (right after decode)
        //[2] = ex2mw (right after execute)
        //[3] = mem2wb (done executing)

    // reg [31:0] inst_if2id [3:0]; 
    // reg [31:0] inst_id2ex [3:0];
    // reg [31:0] inst_ex2mw [3:0];
    reg [31:0] inst_mem = 0; // what is this?

    reg [31:0] pc = 32'h4000_0000; // program counter
    reg [31:0] cycle_count = 0; // number of cycles executed
    reg [31:0] instruction_counter = 0; // number of cycles executed

    reg [31:0] rs1_id2ex; // rs1 value from the ID stage
    reg [31:0] rs2_id2ex; // rs2 value from the ID stage

    reg [31:0] imm_gen_id2ex; // immediate from the ID stage
    
    reg [31:0] pc_id2ex; // PC from the ID stage
    reg [31:0] pc_ex2mw; // PC from the EX stage
    
    reg [31:0] alu_ex2mw; // ALU result from the EX stage

    // CSR
    reg [31:0] tohost_csr = 0;
    
    initial begin
        
        // inst_if2id[0] = NOP;
        // inst_if2id[1] = NOP;
        // inst_if2id[2] = NOP;
        // inst_if2id[3] = NOP;

        // inst_id2ex[0] = NOP;
        // inst_id2ex[1] = NOP;
        // inst_id2ex[2] = NOP;
        // inst_id2ex[3] = NOP;

        // inst_ex2mw[0] = NOP;
        // inst_ex2mw[1] = NOP;
        // inst_ex2mw[2] = NOP;
        // inst_ex2mw[3] = NOP;
        

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
    // assign dmem_addr = alu_out[13:0];
    // assign dmem_din = rs2_id2ex;


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

    alu alu ( // TODO: replace the argument values
        .a_val(0),
        .b_val(0),
        .alu_sel(alu_sel),
        .out_val(alu_out)
    );

    reg [2:0] imm;

    //TODO: add all the signals that we need to get from the control logic
    wire [2:0] imm_sel;
    wire reg_wen; 
    wire [2:0] imm_sel;
    wire br_un;
    wire a_sel;
    wire b_sel;
    wire [3:0] alu_sel;
    wire [3:0] mem_wen; // MAY NEED TO CALCULATE IN CPU ITSELF
    wire mem_sel;
    wire [1:0] wb_sel;
    wire csr_sel;
    wire csr_wen;

    imm_gen imm_gen (
        .inst(inst[0]),
        .imm_sel(imm_sel), //TODO
        .imm(imm)
    );

    // TODO: Your code to implement a fully functioning RISC-V core
    // Add as many modules as you want
    // Feel free to move the memory modules around

    // MAKE SURE regwen comes from the instruction used for mem/wb

    always @(posedge clk) begin
      if (rst) begin
          bios_ena <= 0;
          bios_addrb <= 0;
          bios_doutb <= 0;
      end
      else begin
          pc <= pc_mux;
          rs1_id2ex <= rd1;
          rs2_id2ex <= rd2;
          imm_gen_id2ex <= imm;
        
          inst[1] <= inst[0];
          inst[2] <= inst[1];
          inst[3] <= inst[2];
          inst[0] <= imem_doutb;

          if (inst[4] != NOP) begin
            instruction_counter <= instruction_counter + 1;
          end
          cycle_counter <= cycle_counter + 1;
          
        //   inst_ex2mw[3] <= inst_ex2mw[2];
        //   inst_ex2mw[2] <= inst_ex2mw[1];
        //   inst_ex2mw[1] <= inst_ex2mw[0];
        //   inst_ex2mw[0] <= inst_id2ex[3];

        //   inst_id2ex[3] <= inst_id2ex[2];
        //   inst_id2ex[2] <= inst_id2ex[1];
        //   inst_id2ex[1] <= inst_id2ex[0];
        //   inst_id2ex[0] <= inst_if2id[3];

        //   inst_if2id[3] <= inst_if2id[2];
        //   inst_if2id[2] <= inst_if2id[1];
        //   inst_if2id[1] <= inst_if2id[0];
        //   inst_if2id[0] <= NOP; // TODO: feed the next instruction!
      end
    end

    // ID stage
    control_logic control_logic1 ( // only need imm_sel since that is the only signal that we need in this stage
        .inst(inst[0]),
        .imm_sel(imm_sel),
    );

    // EX stage
    control_logic control_logic2 (
        .inst(inst[1]),
        .a_sel(a_sel),
        .b_sel(b_sel),
        .alu_sel(alu_sel),
        .mem_wen(mem_wen), // should put mem signals in the EX stage because we need to set that value before we get to the mem stage
        .mem_sel(mem_sel),
    ); 

    // MEM+WB+IF stage
    control_logic control_logic3 (
        .inst(inst[2]),
        .reg_wen(we),
        .wb_sel(wb_sel),
        .csr_sel(csr_sel),
        .csr_wen(csr_wen)
    );

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

    // MUXs
    assign pc_mux = (pc_sel == 3'd0) ? pc + 4 : (pc_sel == 3'd1) ? alu.out_val: RESET_PC; // TODO: PC+4, ALU, jump_addr, branch_addr, RESET_PC
    assign mem_mux = (mem_sel == 3'd1) ? dmem_dout : dmem_dout; // TODO: 0: bios doutb, 1: dmem_dout, 2: cycle_counter, 3: inst_counter, 4: uart_dout 
    assign a_mux = (a_sel == 2'd0) ? rs1_id2ex : (a_sel == 2'd1) ? aluwb_mux : (a_sel == 2'd2) ? pc_id2ex + 4 : NOP; // TODO: 0: rs1 (fwd ALU/WB vs. rs1 mux), 1: ALU/WB, 2: pc+4 (pc in execute stage)
    assign b_mux = (b_sel == 2'd0) ? rs2_id2ex : (b_sel == 2'd1) ? aluwb_mux : (b_sel == 2'd2) ? imm : NOP; // TODO: 0: rs2 (fwd ALU/WB vs. rs2 mux), 1: ALU/WB, 2: ImmGen
    assign wb_mux = (wb_sel == 2'd0) ? mem_mux : mem_mux; // TODO: 0: mem_mux, 1: ALU fwd, 2: pc+4 
    // For forwarding ALU-WB and WB
    assign aluwb_mux = (aluwb_sel == 1'd0) ? wb_mux : alu_fwd; // TODO: 0: wb_mux, 1: ALU fwd
    
    //assign alu_fwd to the value of the ALU pipeline register after mem
    assign alu_fwd = alu_ex2mw;

    assign bios_addra = pc_mux;


    assign bios_addra = pc_mux[11:0];
    assign imem_addrb = pc_mux[13:0];

    assign inst_mux = pc[30] == 1'b1 ? imem_doutb : bios_douta; 
    

endmodule
