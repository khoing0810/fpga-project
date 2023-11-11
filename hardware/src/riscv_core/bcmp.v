// branch comparator
module bcmp (
    input [31:0] a_val,
    input [31:0] b_val,
    input br_un,

    output reg br_eq,
    output reg br_lt
);

reg signed [31:0] a_val_signed;
reg signed [31:0] b_val_signed;
reg unsigned [31:0] a_val_unsigned;
reg unsigned [31:0] b_val_unsigned;



always @(*) begin
    a_val_signed = a_val;
    a_val_unsigned = a_val;
    b_val_signed = b_val;
    b_val_unsigned = b_val;

    br_eq = a_val == b_val;
    br_lt = br_un ? a_val_unsigned < b_val_unsigned :
                    a_val_signed < b_val_signed;
end
endmodule