`timescale 1ns/1ps

// axi_master_read.v
// Issues one AXI4 read burst per rd_start (AR channel) and streams the
// returned data (R channel) out to compute_stage1.
//
//   IDLE --rd_start--> ADDR --ARREADY--> DATA --last beat--> IDLE
//
// - RREADY follows out_ready, so a stall downstream stalls the R channel.
// - out_last carries RLAST along with the data.
// - rd_error is set if any beat returns a non-OKAY RRESP; it stays valid
//   until the next rd_start.

module axi_master_read #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32
)(
    input  wire                  clk,
    input  wire                  rst_n,

    // control from dma_fsm / address generator
    input  wire                  rd_start,
    input  wire [ADDR_WIDTH-1:0] rd_addr,
    input  wire [7:0]            rd_len,         // beats - 1
    output reg                   rd_error,

    // AXI4 read address channel
    output reg  [ADDR_WIDTH-1:0] m_axi_araddr,
    output reg  [7:0]            m_axi_arlen,
    output wire [2:0]            m_axi_arsize,
    output wire [1:0]            m_axi_arburst,
    output reg                   m_axi_arvalid,
    input  wire                  m_axi_arready,

    // AXI4 read data channel
    input  wire [DATA_WIDTH-1:0] m_axi_rdata,
    input  wire [1:0]            m_axi_rresp,
    input  wire                  m_axi_rlast,
    input  wire                  m_axi_rvalid,
    output wire                  m_axi_rready,

    // data stream out (to compute_stage1)
    output wire [DATA_WIDTH-1:0] out_data,
    output wire                  out_valid,
    output wire                  out_last,
    input  wire                  out_ready
);

    localparam IDLE = 2'd0,
               ADDR = 2'd1,
               DATA = 2'd2;

    reg [1:0] state;

    assign m_axi_arsize  = $clog2(DATA_WIDTH / 8);   // bytes per beat
    assign m_axi_arburst = 2'b01;                    // INCR

    assign m_axi_rready  = (state == DATA) && out_ready;
    assign out_valid     = (state == DATA) && m_axi_rvalid;
    assign out_data      = m_axi_rdata;
    assign out_last      = m_axi_rlast;

    wire r_fire = m_axi_rvalid && m_axi_rready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= IDLE;
            m_axi_araddr  <= 0;
            m_axi_arlen   <= 0;
            m_axi_arvalid <= 1'b0;
            rd_error      <= 1'b0;
        end else begin
            case (state)
                IDLE: if (rd_start) begin
                    m_axi_araddr  <= rd_addr;
                    m_axi_arlen   <= rd_len;
                    m_axi_arvalid <= 1'b1;
                    rd_error      <= 1'b0;
                    state         <= ADDR;
                end

                ADDR: if (m_axi_arready) begin
                    m_axi_arvalid <= 1'b0;
                    state         <= DATA;
                end

                DATA: if (r_fire) begin
                    if (m_axi_rresp != 2'b00) rd_error <= 1'b1;
                    if (m_axi_rlast)          state    <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
