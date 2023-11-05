// branch comparator
module bcmp (
    input [31:0] a_val,
    input [31:0] b_val,
    input br_un,

    output br_eq,
    output br_lt
);

reg signed [31:0] a_val_signed;
reg signed [31:0] b_val_signed;
reg unsigned [31:0] a_val_unsigned;
reg unsigned [31:0] b_val_unsigned;

reg br_eq_reg = 0;
reg br_lt_reg = 0;

assign br_eq = br_eq_reg;
assign br_lt = br_lt_reg;

always @(*) begin
    a_val_signed = a_val;
    a_val_unsigned = a_val;
    b_val_signed = b_val;
    b_val_unsigned = b_val;
       
    if (br_un) begin
        br_eq_reg = (a_val_unsigned == b_val_unsigned) ? 1 : 0;
        br_lt_reg = (a_val_unsigned < b_val_unsigned) ? 1 : 0;
    end
    else begin
        br_eq_reg = (a_val_signed == b_val_signed) ? 1 : 0;
        br_lt_reg = (a_val_signed < b_val_signed) ? 1 : 0;
    end
end
endmodule