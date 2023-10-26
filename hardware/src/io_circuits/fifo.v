module fifo #(
    parameter WIDTH = 8,
    parameter DEPTH = 32,
    parameter POINTER_WIDTH = $clog2(DEPTH)
) (
    input clk, rst,

    // Write side
    input wr_en,
    input [WIDTH-1:0] din,
    output full,

    // Read side
    input rd_en,
    output [WIDTH-1:0] dout,
    output empty
);
    reg [WIDTH - 1: 0] buffer [DEPTH-1:0];
    // Read and write addresses are initially both at the first memory location and the FIFO queue is empty.
    reg [POINTER_WIDTH - 1: 0] rd_ptr = 0;
    reg [POINTER_WIDTH - 1: 0] wr_ptr = 0;

    reg full_reg;
    reg empty_reg;
    reg [WIDTH - 1: 0] dout_reg;
    integer i;

    assign full = full_reg; // A FIFO is full when the write address register reaches the read address register. 
    assign empty = empty_reg; // A FIFO is empty when the read address register reaches the write address register. 
    assign dout = dout_reg;
    
    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < DEPTH; i = i + 1) begin
                buffer[i] = 0;
            end
            wr_ptr <= 0;
            rd_ptr <= 0;
            full_reg <= 0;
            empty_reg <= 1;
            dout_reg <= 0;
        end 
        else begin
            if (wr_en && !full_reg) begin
                buffer[wr_ptr] <= din;
                wr_ptr = (wr_ptr + 1) % DEPTH;
                empty_reg <= 0;
                full_reg <= wr_ptr == rd_ptr;
            end 
            if (rd_en && !empty_reg) begin
                dout_reg <= buffer[rd_ptr];
                rd_ptr = (rd_ptr + 1) % DEPTH;
                full_reg <= 0;
                empty_reg <= wr_ptr == rd_ptr;
            end
        end
    end 
    // property wr_ptr_change;
    //     @(posedge clk) 
    //     disable iff (rst) 
    //     (if($past(full) && wr_en) $past(wr_ptr) == wr_ptr); //checks that if wr_en is 1 and was already full, then wr_ptr should stay the same
    // endproperty

    // property rd_ptr_change;
    //     @(posedge clk) 
    //     disable iff (rst) 
    //     (if($past(empty) && rd_en) $past(rd_ptr) == rd_ptr); //checks that if rd_en is 1 and was already empty, then rd_ptr should stay the same
    // endproperty

    // property rst_ptrs;
    //     @(posedge clk) 
    //     disable iff (!rst) 
    //     (wr_ptr == 0 && rd_ptr == 0); // checks that if rst, then wr_ptr and rd_ptr are set to 0
    // endproperty

    // assert property (wr_ptr_change) else $display("Assertion failed: Write pointer should not change if full %d %d %d", full, $past(wr_ptr), wr_ptr);
    // assert property (rd_ptr_change) else $display("Assertion failed: Read pointer should not change when empty");
    // assert property (rst_ptrs) else $display("Assertion failed: Write and read pointers are not reset to 0 w/ reset signal");
endmodule
