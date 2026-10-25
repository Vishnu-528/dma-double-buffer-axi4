`timescale 1ns/1ps


// Two SRAM banks (buffer0, buffer1) with a stream interface on each side.
//
// Fill side  : words from compute_stage1 are written into bank 'fill_bank'
//              at consecutive addresses. 'fill_last' marks the final word of
//              a burst; fill_done pulses when that word is written.
//              Always ready (an SRAM write takes one cycle).
//
// Drain side : drain_start loads a bank number and a length (beats - 1);
//              words then stream out to compute_stage2 with valid/ready.
//              The SRAM's registered read output doubles as the pipeline
//              register, so it runs at 1 word/cycle and holds on stalls.
//
// dma_fsm guarantees the two sides never use the same bank at the same time.
// DEPTH must be >= the AXI burst length.

module double_buffer #(
    parameter DATA_WIDTH = 32,
    parameter DEPTH      = 16,
    parameter AW         = $clog2(DEPTH)
)(
    input  wire                  clk,
    input  wire                  rst_n,

    // fill side
    input  wire                  fill_bank,
    input  wire [DATA_WIDTH-1:0] fill_data,
    input  wire                  fill_valid,
    input  wire                  fill_last,
    output wire                  fill_ready,
    output wire                  fill_done,

    // drain side
    input  wire                  drain_start,
    input  wire                  drain_bank,
    input  wire [7:0]            drain_len,      // beats - 1
    output reg  [DATA_WIDTH-1:0] drain_data,
    output reg                   drain_valid,
    input  wire                  drain_ready
);

    reg [DATA_WIDTH-1:0] buffer0 [0:DEPTH-1];
    reg [DATA_WIDTH-1:0] buffer1 [0:DEPTH-1];

    // ---------------- fill side ----------------------------------------------
    reg [AW-1:0] wr_ptr;

    assign fill_ready = 1'b1;
    assign fill_done  = fill_valid && fill_last;

    always @(posedge clk) begin
        if (fill_valid) begin
            if (fill_bank == 1'b0) buffer0[wr_ptr] <= fill_data;
            else                   buffer1[wr_ptr] <= fill_data;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)          wr_ptr <= {AW{1'b0}};
        else if (fill_valid) wr_ptr <= fill_last ? {AW{1'b0}} : wr_ptr + 1'b1;
    end

    // ---------------- drain side ---------------------------------------------
    reg [AW:0] rd_ptr;        // next word to read
    reg [AW:0] rd_count;      // words in this burst
    reg        rd_bank;

    // Read the next word when words remain and the output register is free.
    wire rd_en = (rd_ptr != rd_count) && (!drain_valid || drain_ready);

    always @(posedge clk) begin
        if (rd_en) begin
            if (rd_bank == 1'b0) drain_data <= buffer0[rd_ptr[AW-1:0]];
            else                 drain_data <= buffer1[rd_ptr[AW-1:0]];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_ptr      <= 0;
            rd_count    <= 0;
            rd_bank     <= 1'b0;
            drain_valid <= 1'b0;
        end else begin
            if (drain_start) begin
                rd_ptr   <= 0;
                rd_count <= drain_len + 1'b1;
                rd_bank  <= drain_bank;
            end else if (rd_en) begin
                rd_ptr   <= rd_ptr + 1'b1;
            end

            if (rd_en)            drain_valid <= 1'b1;
            else if (drain_ready) drain_valid <= 1'b0;
        end
    end

endmodule
