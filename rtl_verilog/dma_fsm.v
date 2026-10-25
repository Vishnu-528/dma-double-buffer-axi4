`timescale 1ns/1ps

// Controls the transfer and the ping-pong between the two banks.
//
// Top level: IDLE --start--> RUN --everything finished--> IDLE (done pulse)
//
// Inside RUN two independent sides share the bank 'full' flags:
//   Read side : when idle, bursts remain and bank[rd_bank] is empty
//               -> rd_start. When the last word lands in the bank
//               (fill_done) -> mark it full, switch rd_bank.
//   Write side: when idle and bank[wr_bank] is full
//               -> wr_start. When the B response arrives (wr_done)
//               -> mark it empty, switch wr_bank.
// There is no explicit "swap" step: the banks swap as the flags change.
//
// On an error the FSM stops issuing new bursts, lets the current ones finish,
// then ends with done + error.

module dma_fsm (
    input  wire clk,
    input  wire rst_n,

    // from / to axi_lite_regs
    input  wire start,
    output wire busy,
    output reg  done_pulse,
    output reg  error_pulse,

    // to both address generators
    output wire gen_load,
    input  wire rd_gen_empty,
    input  wire wr_gen_empty,

    // read side
    output wire rd_start,       // also advances the read address generator
    input  wire fill_done,      // last word of the burst written to the bank
    input  wire rd_error,
    output reg  rd_bank,        // bank being filled

    // write side
    output wire wr_start,       // also advances the write address generator
    input  wire wr_done,        // B response received
    input  wire wr_error,
    output reg  wr_bank         // bank being drained
);

    localparam IDLE = 1'b0,
               RUN  = 1'b1;

    reg       state;
    reg [1:0] full;             // full[i] = bank i holds data not yet written out
    reg       rd_busy, wr_busy;
    reg       err;

    assign busy     = (state == RUN);
    assign gen_load = (state == IDLE) && start;

    assign rd_start = (state == RUN) && !err && !rd_busy && !rd_gen_empty && !full[rd_bank];
    assign wr_start = (state == RUN) && !err && !wr_busy && full[wr_bank];

    wire finished = (state == RUN) && !rd_busy && !wr_busy &&
                    (err || (rd_gen_empty && wr_gen_empty));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= IDLE;
            full        <= 2'b00;
            rd_bank     <= 1'b0;
            wr_bank     <= 1'b0;
            rd_busy     <= 1'b0;
            wr_busy     <= 1'b0;
            err         <= 1'b0;
            done_pulse  <= 1'b0;
            error_pulse <= 1'b0;
        end else begin
            done_pulse  <= 1'b0;
            error_pulse <= 1'b0;

            case (state)
                IDLE: if (start) begin
                    full    <= 2'b00;
                    rd_bank <= 1'b0;
                    wr_bank <= 1'b0;
                    rd_busy <= 1'b0;
                    wr_busy <= 1'b0;
                    err     <= 1'b0;
                    state   <= RUN;
                end

                RUN: begin
                    // read side
                    if (rd_start) rd_busy <= 1'b1;
                    if (fill_done) begin
                        rd_busy       <= 1'b0;
                        full[rd_bank] <= 1'b1;
                        rd_bank       <= ~rd_bank;
                        if (rd_error) err <= 1'b1;
                    end

                    // write side
                    if (wr_start) wr_busy <= 1'b1;
                    if (wr_done) begin
                        wr_busy       <= 1'b0;
                        full[wr_bank] <= 1'b0;
                        wr_bank       <= ~wr_bank;
                        if (wr_error) err <= 1'b1;
                    end

                    if (finished) begin
                        done_pulse  <= 1'b1;
                        error_pulse <= err;
                        state       <= IDLE;
                    end
                end
            endcase
        end
    end

endmodule
