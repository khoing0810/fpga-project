`include "opcode.vh"
module cpu #(
    parameter CPU_CLOCK_FREQ = 50_000_000,
    parameter RESET_PC = 32'h4000_0000,
    parameter BAUD_RATE = 115200
) (
    input clk,
    input rst,
    input serial_in,
    input [3:0] BUTTONS,
    input [1:0] SWITCHES,
    input empty,
    input tx_ack,

    output reg [5:0] LEDS,
    output reg [23:0] car_fcw,
    output reg [23:0] mod_fcw,
    output reg [4:0] mod_shift,
    output reg note_en,
    output reg tx_en,
    output rd_en,
    output serial_out
);
    localparam NOP = 32'h0000_0013;

    wire [11:0] bios_addra, bios_addrb;
    wire [31:0] bios_douta, bios_doutb;
    wire bios_ena, bios_enb;

    // MUXs (could remove aluwb_mux and just use wb_mux)
    reg [31:0] pc_mux; // mux between pc+4, alu_out, and jump_addr (done in combinational logic)
    wire [31:0] mem_mux, a_mux, b_mux, wb_mux, aluwb_mux, mem_mux_wb; 
    wire [31:0] rs1_exmux, rs2_exmux, inst_mux, bios_imem_mux, rs1_mux, rs2_mux; 
    // rs1_exmux = mux between rs1_id2ex and aluwb_mux
    // rs2_exmux = mux between rs2_id2ex and aluwb_mux
    // inst_mux = mux between (bios_douta and imem_doutb) and NOP
    // rs1_mux = mux between ra1 and aluwb_mux
    // rs2_mux = mux between ra2 and aluwb_mux
    wire [31:0] csr_mux, csr_we_mux;

    wire [31:0] inst_0, inst_1, inst_2, inst_3;



    // Signals/values determined in CPU
    wire [2:0] pc_sel, mem_sel;
    wire nop_sel;
    wire rs1ex_sel, rs2ex_sel, aluwb_sel, rs1_fwd_sel, rs2_fwd_sel; // will be a mux between the actual A/B mux and the forwarding mux
    // rs1ex_sel = mux between rs1_id2ex and aluwb_mux
    // rs2ex_sel = mux between rs2_id2ex and aluwb_mux
    // aluwb_sel = mux between wb_mux and alu_fwd
    // rs1_fwd_sel = mux between ra1 and aluwb_mux
    // rs2_fwd_sel = mux between ra2 and aluwb_mux
    wire [31:0] alu_out;
    wire [31:0] imm;
    
    // PIPELINE REGISTERS
    reg [31:0] inst [3:0]; // use this to hold 3 instrutions in the pipeline, and the instruction that just finished
    reg [31:0] rs1_id2ex; // rs1 value from the ID stage
    reg [31:0] rs1_id2ex_nomux; // rs1 (non-muxed) value from the ID stage
    reg [31:0] rs2_id2ex; // rs2 value from the ID stage

    reg [31:0] imm_gen_id2ex; // immediate from the ID stage
    reg [31:0] imm_gen_ex2mw; // immediate from the EX stage
    
    reg [31:0] pc_id2ex; // PC from the ID stage
    reg [31:0] pc_ex2mw; // PC from the EX stage
    
    reg [31:0] alu_ex2mw; // ALU result from the EX stage

    reg [31:0] rs2_exmux_to_mem; // rs2 value from the EX stage (muxed)

    // Extra registers
    reg [31:0] pc = RESET_PC; // program counter
    reg [31:0] cycle_counter = 0; // number of cycles executed
    reg [31:0] instruction_counter = 0; // number of cycles executed
    reg [31:0] tohost_csr = 0; // tohost CSR

    //DEBUGGING
    assign inst_0 = inst[0];
    assign inst_1 = inst[1];
    assign inst_2 = inst[2];
    assign inst_3 = inst[3];

   

    // Control signals
    wire [2:0] imm_sel;
    wire reg_wen;
    wire br_un;
    wire a_sel;
    wire b_sel;
    wire [3:0] alu_sel;
    reg [3:0] mem_wen;
    wire [2:0] csr_sel;
    wire [1:0] wb_sel;
    wire csr_wen;
    wire br_eq, br_lt, taken;
    
    
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

    wire [31:0] uart_dout;
    
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

    // MAKE SURE regwen comes from the instruction used for mem/wb

    always @(posedge clk) begin
      if (rst) begin
        pc <= RESET_PC;
        inst[1] <= NOP;
        inst[2] <= NOP;
        inst[3] <= NOP;
        LEDS <= 6'd0;
        instruction_counter <= 0;
        cycle_counter <= 0;
      end
      else begin
          inst[1] <= inst[0]; // move the instruction from the IF stage to the ID stage
          inst[2] <= inst[1]; // move the instruction from the ID stage to the EX stage
          inst[3] <= inst[2]; // move the instruction from the EX stage to the MEM stage

          // PC
          pc <= pc_mux;
          pc_id2ex <= pc;
          pc_ex2mw <= pc_id2ex;
          
          // ID to EX
          rs1_id2ex_nomux <= rd1;
          rs1_id2ex <= rs1_mux;
          rs2_id2ex <= rs2_mux;
          imm_gen_id2ex <= imm;
        
          // EX to MEM
          rs2_exmux_to_mem <= rs2_exmux;
          alu_ex2mw <= alu_out;
          imm_gen_ex2mw <= imm_gen_id2ex;

          // Instruction and cycle counters
          if (alu_out == 32'h80000018 && (inst[1][6:0] == `OPC_STORE && inst[1][14:12] == `FNC_SW)) begin
            instruction_counter <= 0;
            cycle_counter <= 0;
          end
          else if (inst[1] == NOP) begin
            cycle_counter <= cycle_counter + 1;
          end
          else begin
            cycle_counter <= cycle_counter + 1;
            instruction_counter <= instruction_counter + 1;
          end

          if (alu_out == 32'h80000030 && inst[1][6:0] == `OPC_STORE) begin
              LEDS <= rs2_exmux[5:0];
          end
          if (alu_out == 32'h80000100 && inst[1][6:0] == `OPC_STORE) begin
             car_fcw <= rs2_exmux[23:0];
          end
          if (alu_out == 32'h80000200 && inst[1][6:0] == `OPC_STORE) begin
             mod_fcw <= rs2_exmux[23:0];
          end
          if (alu_out == 32'h80000204 && inst[1][6:0] == `OPC_STORE) begin
             mod_shift <= rs2_exmux[4:0];
          end
          if (alu_out == 32'h80000208 && inst[1][6:0] == `OPC_STORE) begin
             note_en <= rs2_exmux[0];
          end
          if (alu_out == 32'h80000210 && inst[1][6:0] == `OPC_STORE) begin
             tx_en <= rs2_exmux[0];
          end
          // CSR
          tohost_csr <= csr_we_mux;
      end
    end

    // ID stage
    control_logic control_logicID ( // only need imm_sel since that is the only signal that we need in this stage
        .inst(inst[0]),
        .imm_sel(imm_sel),
        .reg_wen(),
        .br_un(),
        .a_sel(),
        .b_sel(),
        .alu_sel(),
        .mem_sel(),
        .wb_sel(),
        .csr_sel(),
        .csr_wen()
    );

    // EX stage
    control_logic control_logicEX (
        .inst(inst[1]),
        .alu_sel(alu_sel),
        .mem_sel(),
        .a_sel(a_sel),
        .b_sel(b_sel),
        .imm_sel(),
        .reg_wen(),
        .br_un(br_un),
        .wb_sel(),
        .csr_sel(csr_sel),
        .csr_wen(csr_wen)
    ); 

    // MEM+WB+IF stage
    control_logic control_logicWB (
        .inst(inst[2]),
        .reg_wen(we),
        .wb_sel(wb_sel),
        .csr_sel(),
        .csr_wen(),
        .imm_sel(),
        .br_un(),
        .a_sel(),
        .b_sel(),
        .alu_sel(),
        .mem_sel()
    );

    // IF/ID stage signals/values/modules 
    always @(*) begin // Used because of combinational logic
        pc_mux = rst ? RESET_PC: 
                    (pc_sel == 3'd0) ? pc + 4: // go to the next instruction
                    (pc_sel == 3'd1) ? alu_out: // jump to the address in the rs1 register + imm (JALR)
                    (pc_sel == 3'd2) ? pc + imm: // jump to the address in PC + imm (JAL handling, always taken BRANCH prediction)
                    (pc_sel == 3'd3) ? pc : // bubble (JALR handling)
                    (pc_sel == 3'd4) ? pc_id2ex + 4: // branch not taken (BRANCH handling)
                    RESET_PC;
        inst[0] = inst_mux;
    end
    assign pc_sel = (inst[0][6:0] == `OPC_JAL || inst[0][6:0] == `OPC_BRANCH) ? 3'd2: 
                        (inst[0][6:0] == `OPC_JALR) ? 3'd3: 
                        (inst[1][6:0] == `OPC_JALR) ? 3'd1:
                        (inst[1][6:0] == `OPC_BRANCH && !taken) ? 3'd4:
                        3'd0;
    assign bios_addra = pc_mux[13:2];
    assign bios_ena = (pc_mux[30] == 1'b1) ? 1'b1 : 1'b0;
    assign imem_addrb = pc_mux[15:2];
    assign bios_imem_mux = (pc[30] == 1'b1) ? bios_douta : imem_doutb;
    assign nop_sel = ((inst[1][6:0] == `OPC_JALR || (inst[1][6:0] == `OPC_BRANCH && !taken))) ? 1'b1 : 1'b0;
        // nop_sel = 1 if the current instruction is a branch or jalr and the next instruction is not a branch or jalr
    assign inst_mux = (nop_sel == 1'b0) ? bios_imem_mux : NOP;
    
    //ID forwarding signals (used to handle instructions two cycles apart)
    assign rs1_fwd_sel = inst[0][19:15] == wa && we == 1 && wa != 5'd0 ? 1'd1 : 1'd0;
    /*(rs1_id2ex == 0) ? 1'd0 : (rs1_id2ex == wa) ? 1'd1 : (rs1_id2ex == wa) ? 1'd2 : 1'd0;*/
    assign rs2_fwd_sel = inst[0][24:20] == wa && we == 1 && wa != 5'd0 ? 1'd1 : 1'd0;
    /*(rs2_id2ex == 0) ? 1'd0 : (rs2_id2ex == wa) ? 1'd1 : (rs2_id2ex == wa) ? 1'd2 : 1'd0;*/
    assign ra1 = inst_mux[19:15];
    assign ra2 = inst_mux[24:20];
    

    assign aluwb_mux = wb_mux;
    assign rs1_mux = (rs1_fwd_sel == 1'd0) ? rd1 : aluwb_mux;
    assign rs2_mux = (rs2_fwd_sel == 1'd0) ? rd2 : aluwb_mux;

    imm_gen imm_gen (
        .inst(inst_mux),
        .imm_sel(imm_sel),
        .imm(imm)
    );

    // EX/intoMEM stage signals/values/modules
    assign a_mux = (a_sel == 1'd0) ? rs1_exmux : pc_id2ex;
    assign b_mux = (b_sel == 1'd0) ? rs2_exmux : imm_gen_id2ex;

    // EX forwarding signals (used to handle instructions one cycle apart)
    assign rs1ex_sel = inst[1][19:15] == wa && we == 1 && wa != 5'd0 ? 1'd1 : 1'd0;
    /* (rs1_id2ex == 0) ? 1'd0 : (rs1_id2ex == wa) ? 1'd1 : (rs1_id2ex == wa) ? 1'd2 : 1'd0;*/
    assign rs2ex_sel = inst[1][24:20] == wa && we == 1 && wa != 5'd0 ? 1'd1 : 1'd0; /* (rs2_id2ex == 0) ? 1'd0 : (rs2_id2ex == wa) ? 1'd1 : (rs2_id2ex == wa) ? 1'd2 : 1'd0;*/
    assign aluwb_sel = 0; /* (wb_sel == 2'd0) ? 1'd0 : (wb_sel == 2'd1) ? 1'd1 : 1'd0;*/

    assign rs1_exmux = (rs1ex_sel == 1'd0) ? rs1_id2ex : aluwb_mux;
    assign rs2_exmux = (rs2ex_sel == 1'd0) ? rs2_id2ex : aluwb_mux;

    alu alu (
        .a_val(a_mux),
        .b_val(b_mux),
        .alu_sel(alu_sel),
        .out_val(alu_out)
    );

    bcmp bcmp (
        .a_val(rs1_exmux),
        .b_val(rs2_exmux),
        .br_un(br_un),
        .br_eq(br_eq),
        .br_lt(br_lt)
    );
    
    taken_branch taken_branch (
        .inst(inst[1]),
        .br_eq(br_eq),
        .br_lt(br_lt),
        .taken(taken)
    );
    mem_decoder mem_decoder_pre ( // used to set mem_wen and dmem_din before the dmem_dout is set
        .inst(inst[1]),
        .imm(alu_out),
        .mem_mux(rs2_exmux),
        .mem_wen(mem_wen),
        .mem_mux_wb(dmem_din)
    );

    assign dmem_addr = alu_out[15:2];
    assign dmem_en = (inst[1][6:0] == `OPC_STORE || inst[1][6:0] == `OPC_LOAD) && (alu_out[31:30] == 2'd0 && alu_out[28] == 1'b1) ? 1 : 0;
    assign dmem_we = mem_wen;

    assign bios_addrb = alu_out[13:2];
    assign bios_enb = (alu_out[31:28] == 4'b0100);

    assign imem_addra = alu_out[15:2];
    assign imem_dina = rs2_exmux;
    assign imem_ena = (inst[1][6:0] == `OPC_STORE || inst[1][6:0] == `OPC_LOAD) && (pc_id2ex[30] == 1'b1) && (alu_out[31:29] == 3'b001);
    assign imem_wea = mem_wen;

    assign rd_en = (inst[1][6:0] == `OPC_LOAD && alu_out == 32'h80000024);



    assign uart_rx_data_out_ready = alu_out == 32'h80000004 && inst[1][6:0] == `OPC_LOAD;
    assign uart_tx_data_in_valid = alu_out == 32'h80000008 && inst[1][6:0] == `OPC_STORE;
    assign uart_tx_data_in = uart_tx_data_in_valid ? rs2_exmux[7:0] : 0;
    assign uart_dout = alu_ex2mw == 32'h80000000 ? {30'b0, uart_rx_data_out_valid, uart_tx_data_in_ready}:
                       alu_ex2mw == 32'h80000004 ? {24'b0, uart_rx_data_out} :
                       alu_ex2mw == 32'h80000020 ? {31'd0, empty} :
                       alu_ex2mw == 32'h80000024 ? {29'd0, BUTTONS[2:0]} :
                       alu_ex2mw == 32'h80000028 ? {30'd0, SWITCHES[1:0]} :
                       alu_ex2mw == 32'h80000214 ? {31'd0, tx_ack} :
                       32'd0;

    // MEM/WB stage signals/values/modules
    //assign mem_mux = (mem_sel == 3'd1) ? dmem_dout : dmem_dout; // TODO: 0: bios doutb, 1: dmem_dout, 2: cycle_counter, 3: inst_counter, 4: uart_dout, 5: imem_doutb
    assign mem_sel = alu_ex2mw[31:28] == 4'b0100 ? 3'd0: 
                     (alu_ex2mw[31:30] == 2'b00 && alu_ex2mw[28] == 1'b1) ? 3'd1:
                     (alu_ex2mw == 32'h80000010) ? 3'd2:
                     (alu_ex2mw == 32'h80000014) ? 3'd3:
                     (alu_ex2mw[31:28] == 4'b1000) ? 3'd4: 3'd1;

    assign mem_mux = (mem_sel == 3'd0) ? bios_doutb :
                     (mem_sel == 3'd1) ? dmem_dout :
                     (mem_sel == 3'd2) ? cycle_counter :
                     (mem_sel == 3'd3) ? instruction_counter :
                     (mem_sel == 3'd4) ? uart_dout : dmem_dout;
    assign wb_mux = (wb_sel == 2'd0) ? alu_ex2mw : (wb_sel == 2'd1) ? mem_mux_wb : pc_ex2mw + 4; // TODO: 0: mem_mux, 1: ALU fwd, 2: pc+4 
    assign wd = wb_mux; // we are writing the value from the wb_mux to the register file
    assign wa = inst[2][11:7]; // we are writing to the rd register (reg_wen) already set
    
    mem_decoder mem_decoder_post ( // used to set mem_mux_wb after the dmem_dout is set
        .inst(inst[2]),
        .imm(alu_ex2mw),
        .mem_mux(mem_mux),
        .mem_wen(),
        .mem_mux_wb(mem_mux_wb)
    );

    // CSR
    assign csr_mux = (csr_sel != 3'd0) ? imm_gen_id2ex : rs1_exmux;
    assign csr_we_mux = csr_wen ? csr_mux : tohost_csr;

// ==================== ASSERTIONS ====================
// PC RESET
property PCReset;
    @(posedge clk) (rst) |-> ##1 (pc == RESET_PC);
endproperty
PCReset_check: assume property(PCReset);

// SW
property swMask;
    @(posedge clk) (inst_0[6:0] == `OPC_STORE && inst_0[14:12] == `FNC_SW) |-> ##1 (mem_wen == 4'b1111);
endproperty
swMask_check: assume property(swMask);

// SH
property shMask1;
    @(posedge clk) (inst_1[6:0] == `OPC_STORE && inst_1[14:12] == `FNC_SH && alu_out[1:0] == 2'b00) |-> (mem_wen == 4'b0011);
endproperty
shMask1_check: assert property(shMask1) else $error("Assertion failed. mem_wen = %d, expected = %d", mem_wen, 4'b0011);

property shMask2;
    @(posedge clk) (inst_1[6:0] == `OPC_STORE && inst_1[14:12] == `FNC_SH && alu_out[1:0] == 2'b01) |-> (mem_wen == 4'b0110);
endproperty
shMask2_check: assert property(shMask2) else $error("Assertion failed. mem_wen = %d, expected = %d", mem_wen, 4'b0110);

property shMask3;
    @(posedge clk) (inst_1[6:0] == `OPC_STORE && inst_1[14:12] == `FNC_SH && alu_out[1:0] == 2'b10) |-> (mem_wen == 4'b1100);
endproperty
shMask3_check: assert property(shMask3) else $error("Assertion failed. mem_wen = %d, expected = %d", mem_wen, 4'b1100);

property shMask4;
    @(posedge clk) (inst_1[6:0] == `OPC_STORE && inst_1[14:12] == `FNC_SH && alu_out[1:0] == 2'b11) |-> (mem_wen == 4'b0000);
endproperty
shMask4_check: assert property(shMask4) else $error("Assertion failed. mem_wen = %d, expected = %d", mem_wen, 4'b0000);

// SB
property sbMask1;
    @(posedge clk) (inst_1[6:0] == `OPC_STORE && inst_1[14:12] == `FNC_SB && alu_out[1:0] == 2'b00) |-> (mem_wen == 4'b0001);
endproperty
sbMask1_check: assert property(sbMask1) else $error("Assertion failed. mem_wen = %d, expected = %d", mem_wen, 4'b0001);

property sbMask2;
    @(posedge clk) (inst_1[6:0] == `OPC_STORE && inst_1[14:12] == `FNC_SB && alu_out[1:0] == 2'b01) |-> (mem_wen == 4'b0010);
endproperty
sbMask2_check: assert property(sbMask2) else $error("Assertion failed. mem_wen = %d, expected = %d", mem_wen, 4'b0010);

property sbMask3;
    @(posedge clk) (inst_1[6:0] == `OPC_STORE && inst_1[14:12] == `FNC_SB && alu_out[1:0] == 2'b10) |-> (mem_wen == 4'b0100);
endproperty
sbMask3_check: assert property(sbMask3) else $error("Assertion failed. mem_wen = %d, expected = %d", mem_wen, 4'b0100);

property sbMask4;
    @(posedge clk) (inst_1[6:0] == `OPC_STORE && inst_1[14:12] == `FNC_SB && alu_out[1:0] == 2'b11) |-> (mem_wen == 4'b1000);
endproperty
sbMask4_check: assert property(sbMask4) else $error("Assertion failed. mem_wen = %d, expected = %d", mem_wen, 4'b1000);

// LH & LHU
property lh1;
    @(posedge clk) (inst_0[6:0] == `OPC_LOAD && inst_0[14:12] == `FNC_LH) |-> ##2 (mem_mux_wb[31:16] == 16'b0) || (mem_mux_wb[31:16] == 16'b1111111111111111);
endproperty
lh1_check: assert property(lh1) else $error("Assertion failed. mem_mux_wb[31:16] = %#010x, expected = 0 or 1. Equal: %d", mem_mux_wb[31:16], mem_mux_wb[31:16] == 8'b0 || mem_mux_wb[31:16] == 8'b1);

property lh2;
    @(posedge clk) (inst_0[6:0] == `OPC_LOAD && inst_0[14:12] == `FNC_LHU) |-> ##2 (mem_mux_wb[31:16] == 8'b0);
endproperty
lh2_check: assert property(lh2) else $error("Assertion failed. mem_mux_wb[31:16] = %#010x, expected = 0", mem_mux_wb[31:16]);

// LB & LBU
property lb1;
    @(posedge clk) (inst_0[6:0] == `OPC_LOAD && inst_0[14:12] == `FNC_LB) |-> ##2 (mem_mux_wb[31:8] == 24'b0) || (mem_mux_wb[31:8] == {24{1'b1}});
endproperty
lb1_check: assume property(lb1);

property lb2;
    @(posedge clk) (inst_0[6:0] == `OPC_LOAD && inst_0[14:12] == `FNC_LBU) |-> ##2 (mem_mux_wb[31:8] == 24'b0);
endproperty
lb2_check: assume property(lb2);

// x0 REGISTER
property zero;
    @(posedge clk) (inst_0[6:0] == `OPC_JALR || inst_0[6:0] == `OPC_ARI_ITYPE ||
                    inst_0[6:0] == `OPC_ARI_RTYPE || inst_0[6:0] == `OPC_JAL || 
                    inst_0[6:0] == `OPC_STORE || inst_0[6:0] == `OPC_LOAD)
    |-> (rf.mem[0] == 0);
endproperty
zero_check: assume property(zero);
endmodule