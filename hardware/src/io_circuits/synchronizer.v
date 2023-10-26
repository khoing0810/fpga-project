module synchronizer #(parameter WIDTH = 1) (
    input [WIDTH-1:0] async_signal,
    input clk,
    output [WIDTH-1:0] sync_signal
);
    // TODO: Create your 2 flip-flop synchronizer here
    // This module takes in a vector of WIDTH-bit asynchronous
    // (from different clock domain or not clocked, such as button press) signals
    // and should output a vector of WIDTH-bit synchronous signals
    // that are synchronized to the input clk
    reg [WIDTH-1:0] flip_flop_bits;
    reg [WIDTH-1:0] sync_signal_temp;
    always @(posedge clk) begin
        flip_flop_bits <= async_signal;
    end
    always @(posedge clk) begin
        sync_signal_temp <= flip_flop_bits;
    end
    assign sync_signal = sync_signal_temp;
endmodule
