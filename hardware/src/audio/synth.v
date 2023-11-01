module synth #(
    parameter N_VOICES = 1
)(
    input clk,
    input rst,
    input [N_VOICES-1:0] [23:0] carrier_fcws,
    input [23:0] mod_fcw,
    input [4:0] mod_shift,
    input [N_VOICES-1:0] note_en,

    output reg [13:0] sample,
    output sample_valid,
    input sample_ready
);
    // Remove these lines once you have implemented this module

    localparam STATE_FETCH_MOD = 0; // fetch modulator samples
    localparam STATE_SET_FCW = 1;  // compute fcw of carrier
    localparam STATE_FETCH_CAR = 2; // fetch carrier samples
    localparam STATE_SUM = 3; // sum carrier samples
    localparam STATE_WAITING = 4; // wait for ready to go high

    // State
    reg [3:0] state;
    //// Modulator NCOs
    wire mod_next_sample;
    wire [13:0] mod_samples;
    //// Modulator samples
    reg [13:0] mod_samples_ff;
    //// Sign extended and shifted modulator samples
    reg [23:0] mod_samples_shifted;
    wire [23:0] carrier_fcws_modulated;
    //// Carrier NCOs
    reg carrier_next_sample;
    wire [13:0] carrier_samples;
    //// Carrier samples
    reg [13:0] carrier_samples_ff;
    //// Summer
    /* verilator lint_off UNOPTFLAT */
    wire [13:0] partial_sums;
    /* lint_on */

    nco mod_nco (
        .clk(clk),
        .rst(rst),
        .fcw(mod_fcw),
        .next_sample(mod_next_sample),
        .code(mod_samples)
    );

    nco carrier_nco (
        .clk(clk),
        .rst(rst),
        .fcw(carrier_fcws_modulated),
        .next_sample(carrier_next_sample),
        .code(carrier_samples)
    );

    // Linear state machine
    assign sample_valid = state == STATE_WAITING;
    always @(posedge clk) begin
        if (rst) begin
            state <= STATE_FETCH_MOD;
        end else begin
            case (state)
                STATE_FETCH_MOD: state <= STATE_SET_FCW;
                STATE_SET_FCW: state <= STATE_FETCH_CAR;
                STATE_FETCH_CAR: state <= STATE_SUM;
                STATE_SUM: state <= STATE_WAITING;
                STATE_WAITING: state <= (sample_valid && sample_ready) ? STATE_FETCH_MOD : state;
            endcase
        end
    end

    // Fetch and save modulator samples
    always @(posedge clk) begin
        if (state == STATE_FETCH_MOD) begin
            mod_samples_ff <= mod_samples;
        end
    end
    
    assign mod_next_sample = (state == STATE_FETCH_MOD) && note_en;

    // Sign extend and shift modulator samples to FCW size
    always @(posedge clk) begin
        if (state == STATE_SET_FCW) begin
            mod_samples_shifted <= ({{10{mod_samples_ff[13]}}, mod_samples_ff}) << mod_shift;
        end
    end
    assign carrier_fcws_modulated = carrier_fcws + mod_samples_shifted;

    // Fetch and save carrier samples
    always @(posedge clk) begin
        if (state == STATE_FETCH_CAR) begin
            carrier_samples_ff <= carrier_samples;
        end
    end

    assign carrier_next_sample = (state == STATE_FETCH_CAR) && note_en;
    assign sample = carrier_samples;

    // TODO: 251B -- Sum samples

endmodule
