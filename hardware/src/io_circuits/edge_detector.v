module edge_detector #(
    parameter WIDTH = 1
)(
    input clk,
    input [WIDTH-1:0] signal_in,
    output [WIDTH-1:0] edge_detect_pulse
);
    // TODO: implement a multi-bit edge detector that detects a rising edge of 'signal_in[x]'
    // and outputs a one-cycle pulse 'edge_detect_pulse[x]' at the next clock edge
    reg [WIDTH-1:0] signal_prev;
    assign edge_detect_pulse = ~signal_prev & signal_in;   
    always @(posedge clk) begin
        signal_prev <= signal_in;
    end
    

    // Remove this line once you create your edge detector
    //assign edge_detect_pulse = 0;
endmodule
