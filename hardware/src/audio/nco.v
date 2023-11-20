module nco(
    input clk,
    input rst,
    input [23:0] fcw,
    input next_sample,
    output [13:0] code
);
    reg [23:0] pa;
    always @(posedge clk) begin
        if (rst) begin
            pa <= 0;
        end 
        else begin
            if (next_sample) begin
                pa <= pa + fcw;
            end
        end  
    end
    sine_lut sine_lut_inst(
        .address(pa[23:16]),
        .data(code)
    );
endmodule
