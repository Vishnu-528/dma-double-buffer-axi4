`timescale 1ns/1ps

// AXI4-Lite slave register block (CPU <-> DMA).

//   0x00 CTRL     [0] start (write 1, self-clearing)  [1] irq enable
//                 [2] bypass stage 1                  [3] bypass stage 2
//   0x04 STATUS   [0] busy (read only)  [1] done (W1C)  [2] error (W1C)
//   0x08 SRC_ADDR source address      (64-byte aligned in v1)
//   0x0C DST_ADDR destination address (64-byte aligned in v1)
//   0x10 LENGTH   transfer size in bytes (multiple of 4)

// Writing start also clears done and error. irq = irq_en & (done | error).
// Simplifications: full 32-bit writes only (WSTRB ignored), responses always OKAY.

module axi_lite_regs #(
    parameter ADDR_WIDTH = 32
)(
    input  wire                  clk,
    input  wire                  rst_n,

    // AXI4-Lite slave: write address / data / response
    input  wire [ADDR_WIDTH-1:0] s_axil_awaddr,
    input  wire                  s_axil_awvalid,
    output wire                  s_axil_awready,
    input  wire [31:0]           s_axil_wdata,
    input  wire [3:0]            s_axil_wstrb,
    input  wire                  s_axil_wvalid,
    output wire                  s_axil_wready,
    output wire [1:0]            s_axil_bresp,
    output reg                   s_axil_bvalid,
    input  wire                  s_axil_bready,

    // AXI4-Lite slave: read address / data
    input  wire [ADDR_WIDTH-1:0] s_axil_araddr,
    input  wire                  s_axil_arvalid,
    output wire                  s_axil_arready,
    output reg  [31:0]           s_axil_rdata,
    output wire [1:0]            s_axil_rresp,
    output reg                   s_axil_rvalid,
    input  wire                  s_axil_rready,

    // to the DMA
    output reg                   start,
    output reg  [ADDR_WIDTH-1:0] src_addr,
    output reg  [ADDR_WIDTH-1:0] dst_addr,
    output reg  [31:0]           length,
    output reg                   bypass1,
    output reg                   bypass2,

    // from the DMA
    input  wire                  busy,
    input  wire                  done_pulse,
    input  wire                  error_pulse,

    output wire                  irq
);

    localparam REG_CTRL   = 3'd0,
               REG_STATUS = 3'd1,
               REG_SRC    = 3'd2,
               REG_DST    = 3'd3,
               REG_LEN    = 3'd4;

    reg irq_en, done, error;

    // write channel
    // Accept a write when address and data are both present.

    wire wr_fire = s_axil_awvalid && s_axil_wvalid && !s_axil_bvalid;

    assign s_axil_awready = wr_fire;
    assign s_axil_wready  = wr_fire;
    assign s_axil_bresp   = 2'b00;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            start         <= 1'b0;
            src_addr      <= 0;
            dst_addr      <= 0;
            length        <= 0;
            irq_en        <= 1'b0;
            bypass1       <= 1'b0;
            bypass2       <= 1'b0;
            done          <= 1'b0;
            error         <= 1'b0;
            s_axil_bvalid <= 1'b0;
        end else begin
            start <= 1'b0;

            if (wr_fire) begin
                case (s_axil_awaddr[4:2])
                    REG_CTRL: begin
                        irq_en  <= s_axil_wdata[1];
                        bypass1 <= s_axil_wdata[2];
                        bypass2 <= s_axil_wdata[3];
                        if (s_axil_wdata[0] && !busy) begin
                            start <= 1'b1;
                            done  <= 1'b0;
                            error <= 1'b0;
                        end
                    end
                    REG_STATUS: begin
                        if (s_axil_wdata[1]) done  <= 1'b0;
                        if (s_axil_wdata[2]) error <= 1'b0;
                    end
                    REG_SRC: src_addr <= s_axil_wdata;
                    REG_DST: dst_addr <= s_axil_wdata;
                    REG_LEN: length   <= s_axil_wdata;
                    default: ;
                endcase
            end

            // events from the DMA win over a same-cycle clear
            if (done_pulse)  done  <= 1'b1;
            if (error_pulse) error <= 1'b1;

            if (wr_fire)            s_axil_bvalid <= 1'b1;
            else if (s_axil_bready) s_axil_bvalid <= 1'b0;
        end
    end

    // read channel
    assign s_axil_arready = !s_axil_rvalid;
    assign s_axil_rresp   = 2'b00;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axil_rvalid <= 1'b0;
            s_axil_rdata  <= 32'd0;
        end else if (s_axil_arvalid && s_axil_arready) begin
            s_axil_rvalid <= 1'b1;
            case (s_axil_araddr[4:2])
                REG_CTRL:   s_axil_rdata <= {28'd0, bypass2, bypass1, irq_en, 1'b0};
                REG_STATUS: s_axil_rdata <= {29'd0, error, done, busy};
                REG_SRC:    s_axil_rdata <= src_addr;
                REG_DST:    s_axil_rdata <= dst_addr;
                REG_LEN:    s_axil_rdata <= length;
                default:    s_axil_rdata <= 32'd0;
            endcase
        end else if (s_axil_rready) begin
            s_axil_rvalid <= 1'b0;
        end
    end

    assign irq = irq_en && (done || error);

endmodule
