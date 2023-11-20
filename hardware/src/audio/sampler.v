module sampler (
    input clk,
    input rst,
    input synth_valid,
    input [9:0] scaled_synth_code,
    output synth_ready,
    output pwm_out
);
    // Remove these lines once you have implemented this module
    reg [11:0] counter = 0;
    wire [9:0] dac_code;
    always @(posedge clk) begin
        if (rst) begin
            counter <= 0;
        end 
        else begin
            counter <= counter + 1;
            if (counter == 12'd2500) begin
                counter <= 0;
            end
        end  
    end
    
    assign synth_ready = counter == 12'd2499;
    assign dac_code = synth_valid ? scaled_synth_code : 10'd0;
    sigma_delta_dac sigma_delta_dac_inst (
        .clk(clk),
        .rst(rst),
        .code(dac_code),
        .pwm(pwm_out)
    );
endmodule