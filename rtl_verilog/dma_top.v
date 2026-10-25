`timescale 1ns/1ps

// dma_top.v
// Top-level wrapper: wiring only, no logic.
//
// Data path:
//   AXI R -> axi_master_read -> compute_stage1 -> double_buffer
//         -> compute_stage2 -> axi_master_write -> AXI W
// Control:
//   CPU --AXI4-Lite--> axi_lite_regs <-> dma_fsm -> masters, generators, buffer
//
// Simplifications (v1): single clock, no AXI IDs/cache/prot signals,
// SRC/DST 64-byte aligned, LENGTH a multiple of 4 bytes.

module dma_top #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32,      // register block assumes 32-bit
    parameter BURST_LEN  = 16       // beats per burst = words per bank
)(
    input  wire                    clk,
    input  wire                    rst_n,

    // ---------------- AXI4-Lite slave (from CPU) ----------------
    input  wire [ADDR_WIDTH-1:0]   s_axil_awaddr,
    input  wire                    s_axil_awvalid,
    output wire                    s_axil_awready,
    input  wire [31:0]             s_axil_wdata,
    input  wire [3:0]              s_axil_wstrb,
    input  wire                    s_axil_wvalid,
    output wire                    s_axil_wready,
    output wire [1:0]              s_axil_bresp,
    output wire                    s_axil_bvalid,
    input  wire                    s_axil_bready,
    input  wire [ADDR_WIDTH-1:0]   s_axil_araddr,
    input  wire                    s_axil_arvalid,
    output wire                    s_axil_arready,
    output wire [31:0]             s_axil_rdata,
    output wire [1:0]              s_axil_rresp,
    output wire                    s_axil_rvalid,
    input  wire                    s_axil_rready,

    // ---------------- AXI4 master (to memory) ----------------
    output wire [ADDR_WIDTH-1:0]   m_axi_awaddr,
    output wire [7:0]              m_axi_awlen,
    output wire [2:0]              m_axi_awsize,
    output wire [1:0]              m_axi_awburst,
    output wire                    m_axi_awvalid,
    input  wire                    m_axi_awready,
    output wire [DATA_WIDTH-1:0]   m_axi_wdata,
    output wire [DATA_WIDTH/8-1:0] m_axi_wstrb,
    output wire                    m_axi_wlast,
    output wire                    m_axi_wvalid,
    input  wire                    m_axi_wready,
    input  wire [1:0]              m_axi_bresp,
    input  wire                    m_axi_bvalid,
    output wire                    m_axi_bready,
    output wire [ADDR_WIDTH-1:0]   m_axi_araddr,
    output wire [7:0]              m_axi_arlen,
    output wire [2:0]              m_axi_arsize,
    output wire [1:0]              m_axi_arburst,
    output wire                    m_axi_arvalid,
    input  wire                    m_axi_arready,
    input  wire [DATA_WIDTH-1:0]   m_axi_rdata,
    input  wire [1:0]              m_axi_rresp,
    input  wire                    m_axi_rlast,
    input  wire                    m_axi_rvalid,
    output wire                    m_axi_rready,

    output wire                    irq
);

    // ---------------- internal wires ----------------
    // registers <-> fsm
    wire                  start, busy, done_pulse, error_pulse;
    wire [ADDR_WIDTH-1:0] src_addr, dst_addr;
    wire [31:0]           length;
    wire                  bypass1, bypass2;

    // fsm <-> generators
    wire                  gen_load, rd_gen_empty, wr_gen_empty;
    wire [ADDR_WIDTH-1:0] rd_gen_addr, wr_gen_addr;
    wire [7:0]            rd_gen_len,  wr_gen_len;

    // fsm <-> masters / buffer
    wire                  rd_start, rd_error, fill_done, rd_bank;
    wire                  wr_start, wr_done, wr_error, wr_bank;

    // fill path: read master -> stage 1 -> buffer
    wire [DATA_WIDTH-1:0] rd_data,  s1_data;
    wire                  rd_valid, rd_last, rd_ready;
    wire                  s1_valid, s1_last, s1_ready;

    // drain path: buffer -> stage 2 -> write master
    wire [DATA_WIDTH-1:0] buf_data, s2_data;
    wire                  buf_valid, buf_ready;
    wire                  s2_valid, s2_ready;

    wire [ADDR_WIDTH-1:0] total_beats = length >> 2;   // bytes -> 32-bit words

    // ---------------- register block ----------------
    axi_lite_regs #(.ADDR_WIDTH(ADDR_WIDTH)) u_regs (
        .clk(clk), .rst_n(rst_n),
        .s_axil_awaddr(s_axil_awaddr), .s_axil_awvalid(s_axil_awvalid), .s_axil_awready(s_axil_awready),
        .s_axil_wdata(s_axil_wdata),   .s_axil_wstrb(s_axil_wstrb),
        .s_axil_wvalid(s_axil_wvalid), .s_axil_wready(s_axil_wready),
        .s_axil_bresp(s_axil_bresp),   .s_axil_bvalid(s_axil_bvalid), .s_axil_bready(s_axil_bready),
        .s_axil_araddr(s_axil_araddr), .s_axil_arvalid(s_axil_arvalid), .s_axil_arready(s_axil_arready),
        .s_axil_rdata(s_axil_rdata),   .s_axil_rresp(s_axil_rresp),
        .s_axil_rvalid(s_axil_rvalid), .s_axil_rready(s_axil_rready),
        .start(start), .src_addr(src_addr), .dst_addr(dst_addr), .length(length),
        .bypass1(bypass1), .bypass2(bypass2),
        .busy(busy), .done_pulse(done_pulse), .error_pulse(error_pulse),
        .irq(irq));

    // ---------------- controller ----------------
    dma_fsm u_fsm (
        .clk(clk), .rst_n(rst_n),
        .start(start), .busy(busy), .done_pulse(done_pulse), .error_pulse(error_pulse),
        .gen_load(gen_load), .rd_gen_empty(rd_gen_empty), .wr_gen_empty(wr_gen_empty),
        .rd_start(rd_start), .fill_done(fill_done), .rd_error(rd_error), .rd_bank(rd_bank),
        .wr_start(wr_start), .wr_done(wr_done),     .wr_error(wr_error), .wr_bank(wr_bank));

    // ---------------- address generators ----------------
    axi_address_generator #(.ADDR_WIDTH(ADDR_WIDTH), .BURST_LEN(BURST_LEN),
                            .BYTES_PER_BEAT(DATA_WIDTH/8)) u_rd_gen (
        .clk(clk), .rst_n(rst_n),
        .load(gen_load), .base_addr(src_addr), .total_beats(total_beats), .next(rd_start),
        .addr(rd_gen_addr), .len(rd_gen_len), .empty(rd_gen_empty));

    axi_address_generator #(.ADDR_WIDTH(ADDR_WIDTH), .BURST_LEN(BURST_LEN),
                            .BYTES_PER_BEAT(DATA_WIDTH/8)) u_wr_gen (
        .clk(clk), .rst_n(rst_n),
        .load(gen_load), .base_addr(dst_addr), .total_beats(total_beats), .next(wr_start),
        .addr(wr_gen_addr), .len(wr_gen_len), .empty(wr_gen_empty));

    // ---------------- fill path ----------------
    axi_master_read #(.ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH)) u_rd (
        .clk(clk), .rst_n(rst_n),
        .rd_start(rd_start), .rd_addr(rd_gen_addr), .rd_len(rd_gen_len), .rd_error(rd_error),
        .m_axi_araddr(m_axi_araddr), .m_axi_arlen(m_axi_arlen), .m_axi_arsize(m_axi_arsize),
        .m_axi_arburst(m_axi_arburst), .m_axi_arvalid(m_axi_arvalid), .m_axi_arready(m_axi_arready),
        .m_axi_rdata(m_axi_rdata), .m_axi_rresp(m_axi_rresp), .m_axi_rlast(m_axi_rlast),
        .m_axi_rvalid(m_axi_rvalid), .m_axi_rready(m_axi_rready),
        .out_data(rd_data), .out_valid(rd_valid), .out_last(rd_last), .out_ready(rd_ready));

    compute_stage #(.STAGE(1), .DATA_WIDTH(DATA_WIDTH)) u_stage1 (
        .clk(clk), .rst_n(rst_n), .bypass(bypass1),
        .in_data(rd_data),  .in_valid(rd_valid), .in_last(rd_last), .in_ready(rd_ready),
        .out_data(s1_data), .out_valid(s1_valid), .out_last(s1_last), .out_ready(s1_ready));

    // ---------------- double buffer ----------------
    double_buffer #(.DATA_WIDTH(DATA_WIDTH), .DEPTH(BURST_LEN)) u_buf (
        .clk(clk), .rst_n(rst_n),
        .fill_bank(rd_bank), .fill_data(s1_data), .fill_valid(s1_valid), .fill_last(s1_last),
        .fill_ready(s1_ready), .fill_done(fill_done),
        .drain_start(wr_start), .drain_bank(wr_bank), .drain_len(wr_gen_len),
        .drain_data(buf_data), .drain_valid(buf_valid), .drain_ready(buf_ready));

    // ---------------- drain path ----------------
    compute_stage #(.STAGE(2), .DATA_WIDTH(DATA_WIDTH)) u_stage2 (
        .clk(clk), .rst_n(rst_n), .bypass(bypass2),
        .in_data(buf_data), .in_valid(buf_valid), .in_last(1'b0), .in_ready(buf_ready),
        .out_data(s2_data), .out_valid(s2_valid), .out_last(), .out_ready(s2_ready));

    axi_master_write #(.ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH)) u_wr (
        .clk(clk), .rst_n(rst_n),
        .wr_start(wr_start), .wr_addr(wr_gen_addr), .wr_len(wr_gen_len),
        .wr_done(wr_done), .wr_error(wr_error),
        .m_axi_awaddr(m_axi_awaddr), .m_axi_awlen(m_axi_awlen), .m_axi_awsize(m_axi_awsize),
        .m_axi_awburst(m_axi_awburst), .m_axi_awvalid(m_axi_awvalid), .m_axi_awready(m_axi_awready),
        .m_axi_wdata(m_axi_wdata), .m_axi_wstrb(m_axi_wstrb), .m_axi_wlast(m_axi_wlast),
        .m_axi_wvalid(m_axi_wvalid), .m_axi_wready(m_axi_wready),
        .m_axi_bresp(m_axi_bresp), .m_axi_bvalid(m_axi_bvalid), .m_axi_bready(m_axi_bready),
        .in_data(s2_data), .in_valid(s2_valid), .in_ready(s2_ready));

endmodule
