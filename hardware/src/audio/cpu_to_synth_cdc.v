module cpu_to_synth_cdc #(
    parameter N_VOICES = 1
)(
    input cpu_clk,
    input [N_VOICES-1:0] [23:0] cpu_carrier_fcws,
    input [23:0] cpu_mod_fcw,
    input [4:0] cpu_mod_shift,
    input [N_VOICES-1:0] cpu_note_en,
    input [4:0] cpu_synth_shift,
    input cpu_req,
    output cpu_ack, //could use reg instead of wire

    input synth_clk,
    output reg [N_VOICES-1:0] [23:0] synth_carrier_fcws,
    output reg [23:0] synth_mod_fcw,
    output reg [4:0] synth_mod_shift,
    output reg [N_VOICES-1:0] synth_note_en,
    output reg [4:0] synth_synth_shift
);
    // Remove these lines once you have implemented this module
    wire cpu_req_sync;

    always @(posedge synth_clk) begin
        if (cpu_req_sync) begin
            synth_carrier_fcws <= cpu_carrier_fcws;
            synth_mod_fcw <= cpu_mod_fcw;
            synth_mod_shift <= cpu_mod_shift;
            synth_note_en <= cpu_note_en;
            synth_synth_shift <= cpu_synth_shift;
        end  
    end

    synchronizer #(N_VOICES) cpu_req_syn(
        .async_signal(cpu_req),
        .clk(synth_clk),
        .sync_signal(cpu_req_sync)
    );

    synchronizer #(N_VOICES) syn_req_cpu(
        .async_signal(cpu_req_sync),
        .clk(cpu_clk),
        .sync_signal(cpu_ack)
    );
endmodule