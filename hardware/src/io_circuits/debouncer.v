module debouncer #(
    parameter WIDTH              = 1,
    parameter SAMPLE_CNT_MAX     = 62500,
    parameter PULSE_CNT_MAX      = 200,
    parameter WRAPPING_CNT_WIDTH = $clog2(SAMPLE_CNT_MAX),
    parameter SAT_CNT_WIDTH      = $clog2(PULSE_CNT_MAX) + 1
) (
    input clk,
    input [WIDTH-1:0] glitchy_signal,
    output [WIDTH-1:0] debounced_signal
);
    // TODO: fill in neccesary logic to implement the wrapping counter and the saturating counters
    // Some initial code has been provided to you, but feel free to change it however you like
    // One wrapping counter is required, one saturating counter is needed for each bit of glitchy_signal
    // You need to think of the conditions for reseting, clock enable, etc. those registers
    // Refer to the block diagram in the spec
    reg [WRAPPING_CNT_WIDTH-1: 0] wrapping_cnt;
    reg [SAT_CNT_WIDTH-1:0] saturating_counter [WIDTH-1:0];
    integer i;
    initial begin
        wrapping_cnt = 0;
        for(i = 0; i < WIDTH; i = i + 1) begin
            saturating_counter[i] = 0;
        end
    end
    always @(posedge clk) begin
        wrapping_cnt <= wrapping_cnt + 1;
        if (wrapping_cnt == SAMPLE_CNT_MAX) begin
            for (i = 0; i < WIDTH; i = i + 1) begin
                if (glitchy_signal[i] == 1 && saturating_counter[i] < PULSE_CNT_MAX) begin
                    saturating_counter[i] <= saturating_counter[i] + 1;
                end
                if (glitchy_signal[i] == 0) begin
                    saturating_counter[i] <= 0;
                end
            end
            wrapping_cnt <= 0;
        end
        
    end
    genvar j;
    for (j = 0; j < WIDTH; j = j + 1) begin
        assign debounced_signal[j] = saturating_counter[j] == PULSE_CNT_MAX ? 1'b1 : 1'b0;
    end

    // Remove this line once you have created your debouncer
    //assign debounced_signal = 0;
    
endmodule
