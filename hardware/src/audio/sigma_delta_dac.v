module sigma_delta_dac #(
    parameter CODE_WIDTH = 10
)(
    input clk,
    input rst,
    input [CODE_WIDTH-1:0] code,
    output pwm
);
    reg [CODE_WIDTH:0] acc_prev = 0;
    reg [CODE_WIDTH:0] acc = 0;
    reg pwm_reg = 0;
    
    always @(*) begin
        if (rst) begin
            acc_prev = 0;
        end else begin
            acc_prev = acc + code;
        end
    end
    always @(posedge clk) begin
        if (rst) begin
            acc <= 0;
            pwm_reg <= 0;
        end
        else begin
            acc <= acc + code;
            pwm_reg <= acc[CODE_WIDTH] ^ acc_prev[CODE_WIDTH];
        end
    end

    assign pwm = pwm_reg;
endmodule
