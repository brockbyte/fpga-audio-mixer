// SPDX-License-Identifier: MIT
// Copyright 2025-2026 Sascha Muehlbach
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// ============================================================================
// UDP Port Demultiplexer Top-Level Wrapper
// Instantiates UDP port demultiplexer for 4 destination UDP ports
// Features: Port-based packet routing to separate streams for PTP/RTP/control
// Supports: Port 5004 (RTP audio), Port 319/320 (PTP timing), Port 7880 (control)
// Clock Domain: 125MHz Ethernet
// ============================================================================
module udp_port_demux_top (
    input wire clk_125,
    input wire rst_125,
    input wire clk_50,
    input wire rst_50,

    input wire enable_audio,

    // UDP-Interface von verilog-ethernet
    input  wire        s_udp_hdr_valid,
    output wire        s_udp_hdr_ready,
    input  wire [15:0] s_udp_src_port,
    input  wire [15:0] s_udp_dst_port,
    input  wire [15:0] s_udp_length,
    input  wire [15:0] s_udp_checksum,

    input  wire [7:0] s_udp_payload_tdata,
    input  wire       s_udp_payload_tvalid,
    output wire       s_udp_payload_tready,
    input  wire       s_udp_payload_tlast,

    output wire [7:0] ptp319_data,
    output wire       ptp319_start,
    output wire       ptp319_end,
    output wire       ptp319_valid,
    input  wire       ptp319_ready,

    output wire [7:0] ptp320_data,
    output wire       ptp320_start,
    output wire       ptp320_end,
    output wire       ptp320_valid,
    input  wire       ptp320_ready,

    output wire [7:0] rtp5004_data,
    output wire       rtp5004_start,
    output wire       rtp5004_end,
    output wire       rtp5004_valid,
    input  wire       rtp5004_ready,

    output wire [7:0] ctrl7880_data,
    output wire       ctrl7880_start,
    output wire       ctrl7880_end,
    output wire       ctrl7880_valid,
    input  wire       ctrl7880_ready
);

  // AXI-Streams zwischen Demux und den 4 FIFOs
  wire [7:0] m0_tdata, m1_tdata, m2_tdata, m3_tdata;
  wire m0_tvalid, m1_tvalid, m2_tvalid, m3_tvalid;
  wire m0_tready, m1_tready, m2_tready, m3_tready;
  wire m0_tlast, m1_tlast, m2_tlast, m3_tlast;

  // Demux nach Ports 319 / 320 / 5004 / 7880
  udp_port_demux_4 #(
      .DATA_WIDTH(8),
      .PORT0(16'd5004),
      .PORT1(16'd320),
      .PORT2(16'd319),
      .PORT3(16'd7880)
  ) udp_demux_inst (
      .clk(clk_125),
      .rst(rst_125),

      .s_udp_hdr_valid(s_udp_hdr_valid),
      .s_udp_hdr_ready(s_udp_hdr_ready),
      .s_udp_src_port (s_udp_src_port),
      .s_udp_dst_port (s_udp_dst_port),
      .s_udp_length   (s_udp_length),
      .s_udp_checksum (s_udp_checksum),

      .s_axis_tdata (s_udp_payload_tdata),
      .s_axis_tvalid(s_udp_payload_tvalid),
      .s_axis_tready(s_udp_payload_tready),
      .s_axis_tlast (s_udp_payload_tlast),

      .m0_axis_tdata (m0_tdata),
      .m0_axis_tvalid(m0_tvalid),
      .m0_axis_tready(m0_tready),
      .m0_axis_tlast (m0_tlast),

      .m1_axis_tdata (m1_tdata),
      .m1_axis_tvalid(m1_tvalid),
      .m1_axis_tready(m1_tready),
      .m1_axis_tlast (m1_tlast),

      .m2_axis_tdata (m2_tdata),
      .m2_axis_tvalid(m2_tvalid),
      .m2_axis_tready(m2_tready),
      .m2_axis_tlast (m2_tlast),

      .m3_axis_tdata (m3_tdata),
      .m3_axis_tvalid(m3_tvalid),
      .m3_axis_tready(m3_tready),
      .m3_axis_tlast (m3_tlast)
  );

   axis_udp_payload_async_fifo #(
      .DATA_WIDTH(8),
      .ADDR_WIDTH(12)
  ) fifo_ptp319 (
      .s_clk        (clk_125),
      .s_rst        (rst_125),
      .s_axis_tdata (m2_tdata),
      .s_axis_tvalid(m2_tvalid),
      .s_axis_tready(m2_tready),
      .s_axis_tlast (m2_tlast),

      .m_clk  (clk_50),
      .m_rst  (rst_50),
      .m_data (ptp319_data),
      .m_start(ptp319_start),
      .m_end  (ptp319_end),
      .m_valid(ptp319_valid),
      .m_ready(ptp319_ready)
  );

  axis_udp_payload_async_fifo #(
      .DATA_WIDTH(8),
      .ADDR_WIDTH(12)
  ) fifo_ptp320 (
      .s_clk        (clk_125),
      .s_rst        (rst_125),
      .s_axis_tdata (m1_tdata),
      .s_axis_tvalid(m1_tvalid),
      .s_axis_tready(m1_tready),
      .s_axis_tlast (m1_tlast),

      .m_clk  (clk_50),
      .m_rst  (rst_50),
      .m_data (ptp320_data),
      .m_start(ptp320_start),
      .m_end  (ptp320_end),
      .m_valid(ptp320_valid),
      .m_ready(ptp320_ready)
  );

  axis_udp_payload_async_fifo #(
      .DATA_WIDTH(8),
      .ADDR_WIDTH(12)
  ) fifo_rtp5004 (
      .s_clk        (clk_125),
      .s_rst        (rst_125),
      .s_axis_tdata (m0_tdata),
      .s_axis_tvalid(m0_tvalid),
      .s_axis_tready(m0_tready),
      .s_axis_tlast (m0_tlast),

      .m_clk  (clk_50),
      .m_rst  (rst_50),
      .m_data (rtp5004_data),
      .m_start(rtp5004_start),
      .m_end  (rtp5004_end),
      .m_valid(rtp5004_valid),
      .m_ready(rtp5004_ready)
  );

  axis_udp_payload_async_fifo #(
      .DATA_WIDTH(8),
      .ADDR_WIDTH(12)
  ) fifo_ctrl7880 (
      .s_clk        (clk_125),
      .s_rst        (rst_125),
      .s_axis_tdata (m3_tdata),
      .s_axis_tvalid(m3_tvalid),
      .s_axis_tready(m3_tready),
      .s_axis_tlast (m3_tlast),

      .m_clk  (clk_50),
      .m_rst  (rst_50),
      .m_data (ctrl7880_data),
      .m_start(ctrl7880_start),
      .m_end  (ctrl7880_end),
      .m_valid(ctrl7880_valid),
      .m_ready(ctrl7880_ready)
  );

endmodule
