module uart_transmitter #(
    parameter CLOCK_FREQ = 125_000_000,
    parameter BAUD_RATE = 115_200)
(
    input clk,
    input reset,

    input [7:0] data_in,
    input data_in_valid,
    output data_in_ready,

    output serial_out
);
    // See diagram in the lab guide
    localparam  SYMBOL_EDGE_TIME    =   CLOCK_FREQ / BAUD_RATE;
    localparam  CLOCK_COUNTER_WIDTH =   $clog2(SYMBOL_EDGE_TIME);
    reg [9:0] tot_data_in = 10'b0;
    reg valid_data_present = 0;
    reg [CLOCK_COUNTER_WIDTH -1 : 0] set_cnt = 0;
    reg [4:0] data_ind = 0;

    assign data_in_ready = !valid_data_present;
    assign serial_out = !valid_data_present ? 1 : tot_data_in[data_ind];
    

    always @(posedge clk) begin
        if (reset) begin
            valid_data_present <= 0;
            set_cnt <= 0;
            data_ind <= 0;
            tot_data_in <= 0;
        end
        else if (data_in_ready && data_in_valid) begin
            valid_data_present <= 1;
            tot_data_in <= {1'b1, data_in, 1'b0};
        end
        else if (valid_data_present) begin
            set_cnt <= set_cnt + 1;
            if (set_cnt == SYMBOL_EDGE_TIME - 1) begin
                set_cnt <= 0;
                if (data_ind == 9) begin
                    valid_data_present <= 0;
                    data_ind <= 0;
                end else begin
                    data_ind <= data_ind + 1;
                end
            end 
        end
    end

    // beginning: serial_out & ready should stay 1, and don't increment counter since there is no data
    // once data is received: start incrementing counter, and start w bit 0, then data bits, then bit 1
    // once data is done iteration: set counter to 0, and set ready to 1
endmodule
