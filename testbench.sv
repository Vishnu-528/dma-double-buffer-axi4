// testbench.sv : memory model + system testbench for EDA Playground (generated from tb/)

// -----------------------------------------------------------------------------
// tb_dma_top.v
// System testbench: CPU (AXI4-Lite tasks) -> dma_top -> axi_slave_mem.
//
// Each test fills the source region with random data, programs the DMA over
// AXI4-Lite, waits for the interrupt, then checks:
//   - every destination word equals ref_out(source word)
//   - the word just past the destination region is untouched
//   - STATUS.done / STATUS.error and irq behave correctly
//   - no AXI protocol errors were reported by the memory model
// -----------------------------------------------------------------------------
`timescale 1ns/1ps
module tb_dma_top;

    // ---------------- register map ----------------
    localparam CTRL   = 32'h00,
               STATUS = 32'h04,
               SRC    = 32'h08,
               DST    = 32'h0C,
               LEN    = 32'h10;

    // ---------------- reference model ----------------
    // Must match the functions in rtl/compute_stage.v
    function [31:0] ref_stage1(input [31:0] x);
        ref_stage1 = x << 1;
    endfunction
    function [31:0] ref_stage2(input [31:0] x);
        ref_stage2 = x + 5;
    endfunction
    function [31:0] ref_out(input [31:0] x, input b1, input b2);
        reg [31:0] mid;
        begin
            mid     = b1 ? x   : ref_stage1(x);
            ref_out = b2 ? mid : ref_stage2(mid);
        end
    endfunction

    // ---------------- clock / reset ----------------
    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    // ---------------- AXI4-Lite (testbench is the CPU) ----------------
    reg  [31:0] axil_awaddr = 0, axil_wdata = 0, axil_araddr = 0;
    reg         axil_awvalid = 0, axil_wvalid = 0, axil_arvalid = 0;
    wire        axil_awready, axil_wready, axil_bvalid, axil_arready, axil_rvalid;
    wire [1:0]  axil_bresp, axil_rresp;
    wire [31:0] axil_rdata;

    // ---------------- AXI4 (dma <-> memory) ----------------
    wire [31:0] awaddr, wdata, araddr, rdata;
    wire [7:0]  awlen, arlen;
    wire [2:0]  awsize, arsize;
    wire [1:0]  awburst, arburst, bresp, rresp;
    wire [3:0]  wstrb;
    wire        awvalid, awready, wlast, wvalid, wready, bvalid, bready;
    wire        arvalid, arready, rlast, rvalid, rready;
    wire        irq;

    dma_top dut (
        .clk(clk), .rst_n(rst_n),
        .s_axil_awaddr(axil_awaddr), .s_axil_awvalid(axil_awvalid), .s_axil_awready(axil_awready),
        .s_axil_wdata(axil_wdata), .s_axil_wstrb(4'hF),
        .s_axil_wvalid(axil_wvalid), .s_axil_wready(axil_wready),
        .s_axil_bresp(axil_bresp), .s_axil_bvalid(axil_bvalid), .s_axil_bready(1'b1),
        .s_axil_araddr(axil_araddr), .s_axil_arvalid(axil_arvalid), .s_axil_arready(axil_arready),
        .s_axil_rdata(axil_rdata), .s_axil_rresp(axil_rresp),
        .s_axil_rvalid(axil_rvalid), .s_axil_rready(1'b1),
        .m_axi_awaddr(awaddr), .m_axi_awlen(awlen), .m_axi_awsize(awsize), .m_axi_awburst(awburst),
        .m_axi_awvalid(awvalid), .m_axi_awready(awready),
        .m_axi_wdata(wdata), .m_axi_wstrb(wstrb), .m_axi_wlast(wlast),
        .m_axi_wvalid(wvalid), .m_axi_wready(wready),
        .m_axi_bresp(bresp), .m_axi_bvalid(bvalid), .m_axi_bready(bready),
        .m_axi_araddr(araddr), .m_axi_arlen(arlen), .m_axi_arsize(arsize), .m_axi_arburst(arburst),
        .m_axi_arvalid(arvalid), .m_axi_arready(arready),
        .m_axi_rdata(rdata), .m_axi_rresp(rresp), .m_axi_rlast(rlast),
        .m_axi_rvalid(rvalid), .m_axi_rready(rready),
        .irq(irq));

    axi_slave_mem u_mem (
        .clk(clk), .rst_n(rst_n),
        .awaddr(awaddr), .awlen(awlen), .awsize(awsize), .awburst(awburst),
        .awvalid(awvalid), .awready(awready),
        .wdata(wdata), .wstrb(wstrb), .wlast(wlast), .wvalid(wvalid), .wready(wready),
        .bresp(bresp), .bvalid(bvalid), .bready(bready),
        .araddr(araddr), .arlen(arlen), .arsize(arsize), .arburst(arburst),
        .arvalid(arvalid), .arready(arready),
        .rdata(rdata), .rresp(rresp), .rlast(rlast), .rvalid(rvalid), .rready(rready));

    // ---------------- AXI4-Lite CPU tasks ----------------
    task axil_write(input [31:0] addr, input [31:0] data);
        begin
            axil_awaddr  <= addr;
            axil_wdata   <= data;
            axil_awvalid <= 1'b1;
            axil_wvalid  <= 1'b1;
            @(posedge clk);
            while (!axil_awready) @(posedge clk);
            axil_awvalid <= 1'b0;
            axil_wvalid  <= 1'b0;
            @(posedge clk);
            while (!axil_bvalid) @(posedge clk);
        end
    endtask

    task axil_read(input [31:0] addr, output [31:0] data);
        begin
            axil_araddr  <= addr;
            axil_arvalid <= 1'b1;
            @(posedge clk);
            while (!axil_arready) @(posedge clk);
            axil_arvalid <= 1'b0;
            @(posedge clk);
            while (!axil_rvalid) @(posedge clk);
            data = axil_rdata;
        end
    endtask

    // ---------------- bookkeeping ----------------
    integer errors = 0, tests = 0;
    integer busy_cycles;
    always @(posedge clk) if (dut.busy) busy_cycles = busy_cycles + 1;

    // cycles where a read burst and a write burst are in flight together
    // (proof that the double buffer overlaps the two sides)
    integer overlap_cycles = 0;
    always @(posedge clk)
        if (dut.u_rd.state != 0 && dut.u_wr.state != 0) overlap_cycles = overlap_cycles + 1;

    // ---------------- one complete DMA test ----------------
    task run_dma(input [8*40-1:0] name,
                 input [31:0] src, input [31:0] dst, input [31:0] len,
                 input b1, input b2, input expect_err);
        integer    i, words, t_err;
        reg [31:0] status, exp;
        begin
            tests = tests + 1;
            t_err = 0;
            words = len >> 2;

            // prepare memory: random source, marker pattern in destination + 1 guard word
            for (i = 0; i < words; i = i + 1) u_mem.mem[(src >> 2) + i] = $random;
            for (i = 0; i <= words; i = i + 1) u_mem.mem[(dst >> 2) + i] = 32'hDEAD_BEEF;

            // program and start
            axil_write(SRC, src);
            axil_write(DST, dst);
            axil_write(LEN, len);
            busy_cycles = 0;
            axil_write(CTRL, {28'd0, b2, b1, 1'b1, 1'b1});   // bypass2, bypass1, irq_en, start

            // wait for the interrupt
            @(posedge clk);
            while (!irq) @(posedge clk);
            axil_read(STATUS, status);

            if (status[1] !== 1'b1 || status[0] !== 1'b0) begin
                $display("    ERROR: STATUS=%h, expected done=1 busy=0", status);
                t_err = t_err + 1;
            end
            if (status[2] !== expect_err) begin
                $display("    ERROR: STATUS.error=%b, expected %b", status[2], expect_err);
                t_err = t_err + 1;
            end

            // check data
            if (!expect_err) begin
                for (i = 0; i < words; i = i + 1) begin
                    exp = ref_out(u_mem.mem[(src >> 2) + i], b1, b2);
                    if (u_mem.mem[(dst >> 2) + i] !== exp) begin
                        if (t_err < 5)
                            $display("    ERROR: word %0d: got %h expected %h",
                                     i, u_mem.mem[(dst >> 2) + i], exp);
                        t_err = t_err + 1;
                    end
                end
                if (u_mem.mem[(dst >> 2) + words] !== 32'hDEAD_BEEF) begin
                    $display("    ERROR: wrote past the end of the destination");
                    t_err = t_err + 1;
                end
            end

            // clear status, irq must drop
            axil_write(STATUS, 32'h6);
            @(posedge clk);
            if (irq !== 1'b0) begin
                $display("    ERROR: irq still high after clearing STATUS");
                t_err = t_err + 1;
            end

            errors = errors + t_err;
            if (words > 0 && !expect_err)
                $display("%s %-36s %5d words %6d cycles  %0.2f words/cycle",
                         (t_err == 0) ? "PASS" : "FAIL", name, words, busy_cycles,
                         words * 1.0 / busy_cycles);
            else
                $display("%s %-36s %5d words %6d cycles",
                         (t_err == 0) ? "PASS" : "FAIL", name, words, busy_cycles);
        end
    endtask

    // ---------------- test sequence ----------------
    initial begin
        $dumpfile("dump.vcd");
        $dumpvars(0, tb_dma_top);

        repeat (5) @(posedge clk);
        rst_n <= 1'b1;
        repeat (2) @(posedge clk);

        //       name                                  src           dst           len   b1 b2 err
        run_dma("plain copy (both stages bypassed)",   32'h0000_1000, 32'h0000_8000,  256, 1, 1, 0);
        run_dma("stage 1 only (x*2)",                  32'h0000_1000, 32'h0000_8000, 1024, 0, 1, 0);
        run_dma("stage 2 only (x+5)",                  32'h0000_1000, 32'h0000_8000, 1024, 1, 0, 0);
        run_dma("both stages (2x+5)",                  32'h0000_1000, 32'h0000_8000, 4096, 0, 0, 0);
        run_dma("short last burst (250 words)",        32'h0000_1000, 32'h0000_8000, 1000, 0, 0, 0);
        run_dma("single word",                         32'h0000_1000, 32'h0000_8000,    4, 0, 0, 0);
        run_dma("zero length",                         32'h0000_1000, 32'h0000_8000,    0, 0, 0, 0);

        u_mem.stall_en = 1'b1;
        run_dma("random stalls, both stages",          32'h0000_1000, 32'h0000_8000, 4096, 0, 0, 0);
        run_dma("random stalls, short last burst",     32'h0000_2040, 32'h0000_9000,  700, 0, 0, 0);
        u_mem.stall_en = 1'b0;

        run_dma("read error (SLVERR source)",          32'h0000_F000, 32'h0000_8000,  256, 0, 0, 1);
        run_dma("write error (SLVERR destination)",    32'h0000_1000, 32'h0000_F000,  256, 0, 0, 1);
        run_dma("normal transfer after errors",        32'h0000_1000, 32'h0000_8000,  512, 0, 0, 0);

        errors = errors + u_mem.proto_errors;
        $display("--------------------------------------------------------------");
        $display("Cycles with a read and a write burst in flight together: %0d", overlap_cycles);
        if (errors == 0) $display("ALL %0d TESTS PASSED", tests);
        else             $display("FAILED: %0d errors (%0d AXI protocol errors)", errors, u_mem.proto_errors);
        $finish;
    end

    initial begin
        #5_000_000;
        $display("FAIL: timeout");
        $finish;
    end

endmodule

`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// axi_slave_mem.v  (testbench only, not synthesizable)
// AXI4 slave memory model for the DMA's master port.
//
// - 64KB of word memory, accessed directly by the testbench as u_mem.mem[i].
// - Read and write channels work independently (one burst at a time each).
// - stall_en = 1 : random READY/VALID gaps and random read latency (0-7 cycles).
// - Bursts starting at or above ERR_BASE get a SLVERR response
//   (and writes there are dropped).
// - Protocol checks: WLAST position, burst type/size. Count in proto_errors.
// -----------------------------------------------------------------------------
module axi_slave_mem #(
    parameter MEM_WORDS = 16384,             // 64KB
    parameter ERR_BASE  = 32'h0000_F000
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire [31:0] awaddr,
    input  wire [7:0]  awlen,
    input  wire [2:0]  awsize,
    input  wire [1:0]  awburst,
    input  wire        awvalid,
    output wire        awready,
    input  wire [31:0] wdata,
    input  wire [3:0]  wstrb,
    input  wire        wlast,
    input  wire        wvalid,
    output wire        wready,
    output wire [1:0]  bresp,
    output reg         bvalid,
    input  wire        bready,

    input  wire [31:0] araddr,
    input  wire [7:0]  arlen,
    input  wire [2:0]  arsize,
    input  wire [1:0]  arburst,
    input  wire        arvalid,
    output wire        arready,
    output reg  [31:0] rdata,
    output wire [1:0]  rresp,
    output reg         rlast,
    output reg         rvalid,
    input  wire        rready
);

    reg [31:0] mem [0:MEM_WORDS-1];

    reg     stall_en     = 1'b0;
    integer proto_errors = 0;

    // random "willing this cycle" flags, all 1 when stalls are off
    reg rnd_aw = 1, rnd_w = 1, rnd_ar = 1, rnd_r = 1, rnd_b = 1;
    always @(posedge clk) begin
        rnd_aw <= !stall_en || ($random & 1);
        rnd_w  <= !stall_en || ($random & 1);
        rnd_ar <= !stall_en || ($random & 1);
        rnd_r  <= !stall_en || ($random & 1);
        rnd_b  <= !stall_en || ($random & 1);
    end

    // ---------------- write side ---------------------------------------------
    reg        w_active, b_pending, w_err;
    reg [31:0] w_addr;
    reg [7:0]  w_len;
    reg [8:0]  w_cnt;

    assign awready = !w_active && !b_pending && rnd_aw;
    assign wready  = w_active && rnd_w;
    assign bresp   = w_err ? 2'b10 : 2'b00;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w_active <= 0; b_pending <= 0; bvalid <= 0; w_err <= 0;
        end else begin
            if (awvalid && awready) begin
                w_active <= 1'b1;
                w_addr   <= awaddr;
                w_len    <= awlen;
                w_cnt    <= 0;
                w_err    <= (awaddr >= ERR_BASE);
                if (awburst != 2'b01 || awsize != 3'd2) begin
                    $display("[%0t] PROTOCOL: bad AWBURST/AWSIZE", $time);
                    proto_errors = proto_errors + 1;
                end
            end
            if (wvalid && wready) begin
                if (wlast != (w_cnt == w_len)) begin
                    $display("[%0t] PROTOCOL: WLAST wrong at beat %0d of %0d", $time, w_cnt, w_len + 1);
                    proto_errors = proto_errors + 1;
                end
                if (!w_err && wstrb == 4'hF) mem[w_addr >> 2] <= wdata;
                w_addr <= w_addr + 4;
                w_cnt  <= w_cnt + 1;
                if (wlast) begin
                    w_active  <= 1'b0;
                    b_pending <= 1'b1;
                end
            end
            if (b_pending && !bvalid && rnd_b) bvalid <= 1'b1;
            if (bvalid && bready) begin
                bvalid    <= 1'b0;
                b_pending <= 1'b0;
            end
        end
    end

    // ---------------- read side ----------------------------------------------
    reg        r_active, r_err;
    reg [31:0] r_addr;
    reg [7:0]  r_len;
    reg [8:0]  r_sent;
    reg [3:0]  r_wait;

    assign arready = !r_active && rnd_ar;
    assign rresp   = r_err ? 2'b10 : 2'b00;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_active <= 0; rvalid <= 0; rlast <= 0; r_err <= 0; r_wait <= 0;
        end else begin
            if (arvalid && arready) begin
                r_active <= 1'b1;
                r_addr   <= araddr;
                r_len    <= arlen;
                r_sent   <= 0;
                r_err    <= (araddr >= ERR_BASE);
                r_wait   <= stall_en ? ($random & 7) : 4'd2;
                if (arburst != 2'b01 || arsize != 3'd2) begin
                    $display("[%0t] PROTOCOL: bad ARBURST/ARSIZE", $time);
                    proto_errors = proto_errors + 1;
                end
            end else if (r_active && r_wait != 0) begin
                r_wait <= r_wait - 1;
            end

            // current beat accepted
            if (rvalid && rready) begin
                rvalid <= 1'b0;
                if (rlast) r_active <= 1'b0;
            end

            // present the next beat (overrides the clear above)
            if (r_active && r_wait == 0 && r_sent <= r_len &&
                (!rvalid || rready) && rnd_r) begin
                rvalid <= 1'b1;
                rdata  <= mem[r_addr >> 2];
                rlast  <= (r_sent == r_len);
                r_sent <= r_sent + 1;
                r_addr <= r_addr + 4;
            end
        end
    end

endmodule
