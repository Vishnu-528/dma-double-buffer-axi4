`timescale 1ns/1ps

// Issues one AXI4 write burst per wr_start. Data comes from compute_stage2.
//
//   IDLE --wr_start--> SEND --AW and all W beats done--> RESP --BVALID--> IDLE
//
// - In SEND the address (AW) and data (W) are sent in parallel, as AXI
//   recommends (the master must not wait for AWREADY before sending W).
// - WLAST is generated from a beat counter.
// - wr_done pulses after the B response; wr_error is valid at the same time
//   and stays until the next wr_start.

module axi_master_write #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
)(
    input  wire                    clk,
    input  wire                    rst_n,

    // control from dma_fsm / address generator
    input  wire                    wr_start,
    input  wire [ADDR_WIDTH-1:0]   wr_addr,
    input  wire [7:0]              wr_len,         // beats - 1
    output reg                     wr_done,
    output reg                     wr_error,

    // AXI4 write address channel
    output reg  [ADDR_WIDTH-1:0]   m_axi_awaddr,
    output reg  [7:0]              m_axi_awlen,
    output wire [2:0]              m_axi_awsize,
    output wire [1:0]              m_axi_awburst,
    output reg                     m_axi_awvalid,
    input  wire                    m_axi_awready,

    // AXI4 write data channel
    output wire [DATA_WIDTH-1:0]   m_axi_wdata,
    output wire [DATA_WIDTH/8-1:0] m_axi_wstrb,
    output wire                    m_axi_wlast,
    output wire                    m_axi_wvalid,
    input  wire                    m_axi_wready,

    // AXI4 write response channel
    input  wire [1:0]              m_axi_bresp,
    input  wire                    m_axi_bvalid,
    output wire                    m_axi_bready,

    // data stream in (from compute_stage2)
    input  wire [DATA_WIDTH-1:0]   in_data,
    input  wire                    in_valid,
    output wire                    in_ready
);

    localparam IDLE = 2'd0,
               SEND = 2'd1,
               RESP = 2'd2;

    reg [1:0] state;
    reg [7:0] beat_cnt;
    reg       w_done;          // all W beats of this burst sent

    assign m_axi_awsize  = $clog2(DATA_WIDTH / 8);
    assign m_axi_awburst = 2'b01;                    // INCR

    assign m_axi_wdata   = in_data;
    assign m_axi_wstrb   = {(DATA_WIDTH/8){1'b1}};   // full words only
    assign m_axi_wlast   = (beat_cnt == m_axi_awlen);
    assign m_axi_wvalid  = (state == SEND) && !w_done && in_valid;
    assign in_ready      = (state == SEND) && !w_done && m_axi_wready;
    assign m_axi_bready  = (state == RESP);

    wire aw_fire  = m_axi_awvalid && m_axi_awready;
    wire w_fire   = m_axi_wvalid  && m_axi_wready;
    wire aw_sent  = !m_axi_awvalid || aw_fire;
    wire all_sent = w_done || (w_fire && m_axi_wlast);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= IDLE;
            m_axi_awaddr  <= 0;
            m_axi_awlen   <= 0;
            m_axi_awvalid <= 1'b0;
            beat_cnt      <= 0;
            w_done        <= 1'b0;
            wr_done       <= 1'b0;
            wr_error      <= 1'b0;
        end else begin
            wr_done <= 1'b0;

            case (state)
                IDLE: if (wr_start) begin
                    m_axi_awaddr  <= wr_addr;
                    m_axi_awlen   <= wr_len;
                    m_axi_awvalid <= 1'b1;
                    beat_cnt      <= 0;
                    w_done        <= 1'b0;
                    wr_error      <= 1'b0;
                    state         <= SEND;
                end

                SEND: begin
                    if (aw_fire) m_axi_awvalid <= 1'b0;
                    if (w_fire) begin
                        beat_cnt <= beat_cnt + 1'b1;
                        if (m_axi_wlast) w_done <= 1'b1;
                    end
                    if (aw_sent && all_sent) state <= RESP;
                end

                RESP: if (m_axi_bvalid) begin
                    wr_error <= (m_axi_bresp != 2'b00);
                    wr_done  <= 1'b1;
                    state    <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
