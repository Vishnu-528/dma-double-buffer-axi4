`timescale 1ns/1ps

// Splits a transfer (base address + number of beats) into AXI INCR bursts of
// up to BURST_LEN beats. The last burst may be shorter.
//
//   load : capture base_addr and total_beats
//   next : the current burst has been issued, move on to the next one
//   addr / len : the current burst (len is AXI style: beats - 1)
//   empty      : no bursts left
//
//  assumption: base addresses are aligned to BURST_LEN * BYTES_PER_BEAT
// (64 bytes by default), so no burst can cross a 4KB boundary.

module axi_address_generator #(
    parameter ADDR_WIDTH     = 32,
    parameter BURST_LEN      = 16,
    parameter BYTES_PER_BEAT = 4
)(
    input  wire                  clk,
    input  wire                  rst_n,

    input  wire                  load,
    input  wire [ADDR_WIDTH-1:0] base_addr,
    input  wire [ADDR_WIDTH-1:0] total_beats,
    input  wire                  next,

    output reg  [ADDR_WIDTH-1:0] addr,
    output wire [7:0]            len,
    output wire                  empty
);

    reg  [ADDR_WIDTH-1:0] beats_left;
    wire [ADDR_WIDTH-1:0] burst_beats = (beats_left > BURST_LEN) ? BURST_LEN : beats_left;

    assign len   = burst_beats - 1'b1;
    assign empty = (beats_left == 0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            addr       <= 0;
            beats_left <= 0;
        end else if (load) begin
            addr       <= base_addr;
            beats_left <= total_beats;
        end else if (next && !empty) begin
            addr       <= addr + burst_beats * BYTES_PER_BEAT;
            beats_left <= beats_left - burst_beats;
        end
    end

endmodule
