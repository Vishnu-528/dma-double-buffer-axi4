`timescale 1ns/1ps


// One half of the split per-word compute. Instantiated twice:
//   STAGE = 1 : on the fill side  (AXI read  -> stage 1 -> buffer)
//   STAGE = 2 : on the drain side (buffer -> stage 2 -> AXI write)
//
// Default functions:  stage 1: y = x * 2     stage 2: out = y + 5
// so the full transfer computes  out = 2x + 5  (wraps around on overflow).
//
// Handshake: one pipeline register with valid/ready on both sides.
//   Throughput 1 word/cycle, latency 1 cycle, stalls when out_ready = 0.
//   'last' is a sideband flag that travels with the data.
//   bypass = 1 passes data through unchanged (same 1-cycle latency).

module compute_stage #(
    parameter STAGE      = 1,
    parameter DATA_WIDTH = 32
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  bypass,

    // input stream
    input  wire [DATA_WIDTH-1:0] in_data,
    input  wire                  in_valid,
    input  wire                  in_last,
    output wire                  in_ready,

    // output stream
    output reg  [DATA_WIDTH-1:0] out_data,
    output reg                   out_valid,
    output reg                   out_last,
    input  wire                  out_ready
);

    // compute functions
    function [DATA_WIDTH-1:0] stage1_fn(input [DATA_WIDTH-1:0] x);
        stage1_fn = x << 1;                 // x * 2
    endfunction

    function [DATA_WIDTH-1:0] stage2_fn(input [DATA_WIDTH-1:0] x);
        stage2_fn = x + 5;                  // + 5
    endfunction


    wire [DATA_WIDTH-1:0] result = bypass       ? in_data            :
                                   (STAGE == 1) ? stage1_fn(in_data) :
                                                  stage2_fn(in_data);

    assign in_ready = !out_valid || out_ready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid <= 1'b0;
            out_last  <= 1'b0;
            out_data  <= {DATA_WIDTH{1'b0}};
        end else if (in_ready) begin
            out_valid <= in_valid;
            if (in_valid) begin
                out_data <= result;
                out_last <= in_last;
            end
        end
    end

endmodule
