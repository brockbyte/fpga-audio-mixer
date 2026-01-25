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

// Language: Verilog 2001

`resetall
`timescale 1ns / 1ps
`default_nettype none

// ============================================================================
// FPGA Core Logic - Top-Level Mixer Integration
// Instantiates all major subsystems: PTP, RTP, UDP, display, motor control, ADC
// Features:
//   - Ethernet 1000BASE-T RGMII interface with MAC/IP/UDP stacks
//   - PTP v2 timing synchronization (port 319/320) for AES67 audio
//   - AES67 RTP audio receive/transmit (port 5004) with jitter buffer/resampler
//   - 8-channel signed audio mixer with gains, pan, mute, solo controls
//   - SPDIF output for monitoring
//   - 8x 10-bit fader inputs via SPI MCP3008 ADC with PI servo motor control
//   - 16 buttons (mute/solo) via I2C MCP23017 expander
//   - 8 capacitive touch channels via I2C MPR121 sensor
//   - 8 rotary encoders with debouncing for parameter adjustment
//   - 8-channel OLED display with TCA9548 I2C mux for status/metering
// Clock Domains: 125MHz Ethernet, 50MHz primary, 12.288MHz SPDIF, 48kHz audio
// ============================================================================
module fpgamixer_core #(
    parameter TARGET = "GENERIC"

) (
    /*
     * Clock: 125MHz Ethernet
     * Synchronous reset
     */
    input wire clk,
    input wire clk90,
    input wire clk_50m,
    input wire clk_12m288,
    input wire rst,
    input wire rst_audio,

    output wire spdif_out,

    /*
     * GPIO: User buttons, DIP switches, LEDs
     */
    input  wire [3:0] btn,
    input  wire [2:0] sw,
    output wire [3:0] led,

    // I2C Bus (general)
    inout wire i2c_sda,
    inout wire i2c_scl,

    // I2C Bus for Buttons/LEDs (MCP23017)
    inout wire i2c_sda_t,
    inout wire i2c_scl_t,

    // I2C Bus for Touch Sensors (MPR121)
    inout wire i2c_sda_f,
    inout wire i2c_scl_f,

    // Rotary Encoders (8 channels, phase A and B)
    input wire [7:0] enc_a,
    input wire [7:0] enc_b,

    // MCP3008 SPI Interface for 8 Fader ADCs
    output wire adc_cs_n,
    output wire adc_sclk,
    output wire adc_mosi,
    input  wire adc_miso,

    // TB6612 H-Bridge Motor Driver Outputs
    output wire [7:0] m_in1,
    output wire [7:0] m_in2,
    output wire [7:0] m_pwm,
    output wire       tb_stby,

    /*
     * Ethernet: 1000BASE-T RGMII Physical Interface
     */
    input  wire       phy_rx_clk,
    input  wire [3:0] phy_rxd,
    input  wire       phy_rx_ctl,
    output wire       phy_tx_clk,
    output wire [3:0] phy_txd,
    output wire       phy_tx_ctl,
    output wire       phy_reset_n,
    input  wire       phy_int_n
);

  // Important Configuration
  wire [47:0] local_mac   = 48'h02_00_00_00_00_01;
  wire [31:0] local_ip    = {8'd192, 8'd168, 8'd1,   8'd128};
  wire [31:0] gateway_ip  = {8'd192, 8'd168, 8'd1,   8'd1};
  wire [31:0] subnet_mask = {8'd255, 8'd255, 8'd255, 8'd0};

  // ============================================================================
  // ETHERNET/UDP INTERFACE & PTP STACK
  // ============================================================================
  wire [7:0] rx_axis_tdata;
  wire rx_axis_tvalid;
  wire rx_axis_tready;
  wire rx_axis_tlast;
  wire rx_axis_tuser;

  wire [7:0] tx_axis_tdata;
  wire tx_axis_tvalid;
  wire tx_axis_tready;
  wire tx_axis_tlast;
  wire tx_axis_tuser;

  // Ethernet frame between Ethernet modules and UDP stack
  wire rx_eth_hdr_ready;
  wire rx_eth_hdr_valid;
  wire [47:0] rx_eth_dest_mac;
  wire [47:0] rx_eth_src_mac;
  wire [15:0] rx_eth_type;
  wire [7:0] rx_eth_payload_axis_tdata;
  wire rx_eth_payload_axis_tvalid;
  wire rx_eth_payload_axis_tready;
  wire rx_eth_payload_axis_tlast;
  wire rx_eth_payload_axis_tuser;

  wire tx_eth_hdr_ready;
  wire tx_eth_hdr_valid;
  wire [47:0] tx_eth_dest_mac;
  wire [47:0] tx_eth_src_mac;
  wire [15:0] tx_eth_type;
  wire [7:0] tx_eth_payload_axis_tdata;
  wire tx_eth_payload_axis_tvalid;
  wire tx_eth_payload_axis_tready;
  wire tx_eth_payload_axis_tlast;
  wire tx_eth_payload_axis_tuser;

  // IP frame connections
  wire rx_ip_hdr_valid;
  wire rx_ip_hdr_ready;
  wire [47:0] rx_ip_eth_dest_mac;
  wire [47:0] rx_ip_eth_src_mac;
  wire [15:0] rx_ip_eth_type;
  wire [3:0] rx_ip_version;
  wire [3:0] rx_ip_ihl;
  wire [5:0] rx_ip_dscp;
  wire [1:0] rx_ip_ecn;
  wire [15:0] rx_ip_length;
  wire [15:0] rx_ip_identification;
  wire [2:0] rx_ip_flags;
  wire [12:0] rx_ip_fragment_offset;
  wire [7:0] rx_ip_ttl;
  wire [7:0] rx_ip_protocol;
  wire [15:0] rx_ip_header_checksum;
  wire [31:0] rx_ip_source_ip;
  wire [31:0] rx_ip_dest_ip;
  wire [7:0] rx_ip_payload_axis_tdata;
  wire rx_ip_payload_axis_tvalid;
  wire rx_ip_payload_axis_tready;
  wire rx_ip_payload_axis_tlast;
  wire rx_ip_payload_axis_tuser;

  wire tx_ip_hdr_valid;
  wire tx_ip_hdr_ready;
  wire [5:0] tx_ip_dscp;
  wire [1:0] tx_ip_ecn;
  wire [15:0] tx_ip_length;
  wire [7:0] tx_ip_ttl;
  wire [7:0] tx_ip_protocol;
  wire [31:0] tx_ip_source_ip;
  wire [31:0] tx_ip_dest_ip;
  wire [7:0] tx_ip_payload_axis_tdata;
  wire tx_ip_payload_axis_tvalid;
  wire tx_ip_payload_axis_tready;
  wire tx_ip_payload_axis_tlast;
  wire tx_ip_payload_axis_tuser;

  // UDP frame connections
  wire rx_udp_hdr_valid;
  wire rx_udp_hdr_ready;
  wire [47:0] rx_udp_eth_dest_mac;
  wire [47:0] rx_udp_eth_src_mac;
  wire [15:0] rx_udp_eth_type;
  wire [3:0] rx_udp_ip_version;
  wire [3:0] rx_udp_ip_ihl;
  wire [5:0] rx_udp_ip_dscp;
  wire [1:0] rx_udp_ip_ecn;
  wire [15:0] rx_udp_ip_length;
  wire [15:0] rx_udp_ip_identification;
  wire [2:0] rx_udp_ip_flags;
  wire [12:0] rx_udp_ip_fragment_offset;
  wire [7:0] rx_udp_ip_ttl;
  wire [7:0] rx_udp_ip_protocol;
  wire [15:0] rx_udp_ip_header_checksum;
  wire [31:0] rx_udp_ip_source_ip;
  wire [31:0] rx_udp_ip_dest_ip;
  wire [15:0] rx_udp_source_port;
  wire [15:0] rx_udp_dest_port;
  wire [15:0] rx_udp_length;
  wire [15:0] rx_udp_checksum;
  wire [7:0] rx_udp_payload_axis_tdata;
  wire rx_udp_payload_axis_tvalid;
  wire rx_udp_payload_axis_tready;
  wire rx_udp_payload_axis_tlast;
  wire rx_udp_payload_axis_tuser;

  wire tx_udp_hdr_valid;
  wire tx_udp_hdr_ready;
  wire [5:0] tx_udp_ip_dscp;
  wire [1:0] tx_udp_ip_ecn;
  wire [7:0] tx_udp_ip_ttl;
  wire [31:0] tx_udp_ip_source_ip;
  wire [31:0] tx_udp_ip_dest_ip;
  wire [15:0] tx_udp_source_port;
  wire [15:0] tx_udp_dest_port;
  wire [15:0] tx_udp_length;
  wire [15:0] tx_udp_checksum;
  wire [7:0] tx_udp_payload_axis_tdata;
  wire tx_udp_payload_axis_tvalid;
  wire tx_udp_payload_axis_tready;
  wire tx_udp_payload_axis_tlast;
  wire tx_udp_payload_axis_tuser;

  wire [7:0] rx_fifo_udp_payload_axis_tdata;
  wire rx_fifo_udp_payload_axis_tvalid;
  wire rx_fifo_udp_payload_axis_tready;
  wire rx_fifo_udp_payload_axis_tlast;
  wire rx_fifo_udp_payload_axis_tuser;

  wire [7:0] tx_fifo_udp_payload_axis_tdata;
  wire tx_fifo_udp_payload_axis_tvalid;
  wire tx_fifo_udp_payload_axis_tready;
  wire tx_fifo_udp_payload_axis_tlast;
  wire tx_fifo_udp_payload_axis_tuser;

  // IP ports not used
  assign rx_ip_hdr_ready = 1;
  assign rx_ip_payload_axis_tready = 1;

  assign tx_ip_hdr_valid = 0;
  assign tx_ip_dscp = 0;
  assign tx_ip_ecn = 0;
  assign tx_ip_length = 0;
  assign tx_ip_ttl = 0;
  assign tx_ip_protocol = 0;
  assign tx_ip_source_ip = 0;
  assign tx_ip_dest_ip = 0;
  assign tx_ip_payload_axis_tdata = 0;
  assign tx_ip_payload_axis_tvalid = 0;
  assign tx_ip_payload_axis_tlast = 0;
  assign tx_ip_payload_axis_tuser = 0;
  assign tx_udp_payload_axis_tuser = 0;

  // Place first payload byte onto LEDs
  reg valid_last = 0;
  reg [7:0] led_reg = 0;
  assign led = led_reg;

  assign phy_reset_n = ~rst;

  eth_mac_1g_rgmii_fifo #(
      .TARGET(TARGET),
      .USE_CLK90("TRUE"),
      .ENABLE_PADDING(1),
      .MIN_FRAME_LENGTH(64),
      .TX_FIFO_DEPTH(4096),
      .TX_FRAME_FIFO(1),
      .RX_FIFO_DEPTH(4096),
      .RX_FRAME_FIFO(1)
  ) eth_mac_inst (
      .gtx_clk  (clk),
      .gtx_clk90(clk90),
      .gtx_rst  (rst),
      .logic_clk(clk),
      .logic_rst(rst),

      .tx_axis_tdata (tx_axis_tdata),
      .tx_axis_tvalid(tx_axis_tvalid),
      .tx_axis_tready(tx_axis_tready),
      .tx_axis_tlast (tx_axis_tlast),
      .tx_axis_tuser (tx_axis_tuser),

      .rx_axis_tdata (rx_axis_tdata),
      .rx_axis_tvalid(rx_axis_tvalid),
      .rx_axis_tready(rx_axis_tready),
      .rx_axis_tlast (rx_axis_tlast),
      .rx_axis_tuser (rx_axis_tuser),

      .rgmii_rx_clk(phy_rx_clk),
      .rgmii_rxd(phy_rxd),
      .rgmii_rx_ctl(phy_rx_ctl),
      .rgmii_tx_clk(phy_tx_clk),
      .rgmii_txd(phy_txd),
      .rgmii_tx_ctl(phy_tx_ctl),

      .tx_fifo_overflow(),
      .tx_fifo_bad_frame(),
      .tx_fifo_good_frame(),
      .rx_error_bad_frame(),
      .rx_error_bad_fcs(),
      .rx_fifo_overflow(),
      .rx_fifo_bad_frame(),
      .rx_fifo_good_frame(),
      .speed(),

      .cfg_ifg(8'd12),
      .cfg_tx_enable(1'b1),
      .cfg_rx_enable(1'b1)
  );

  eth_axis_rx eth_axis_rx_inst (
      .clk(clk),
      .rst(rst),
      // AXI input
      .s_axis_tdata(rx_axis_tdata),
      .s_axis_tvalid(rx_axis_tvalid),
      .s_axis_tready(rx_axis_tready),
      .s_axis_tlast(rx_axis_tlast),
      .s_axis_tuser(rx_axis_tuser),
      // Ethernet frame output
      .m_eth_hdr_valid(rx_eth_hdr_valid),
      .m_eth_hdr_ready(rx_eth_hdr_ready),
      .m_eth_dest_mac(rx_eth_dest_mac),
      .m_eth_src_mac(rx_eth_src_mac),
      .m_eth_type(rx_eth_type),
      .m_eth_payload_axis_tdata(rx_eth_payload_axis_tdata),
      .m_eth_payload_axis_tvalid(rx_eth_payload_axis_tvalid),
      .m_eth_payload_axis_tready(rx_eth_payload_axis_tready),
      .m_eth_payload_axis_tlast(rx_eth_payload_axis_tlast),
      .m_eth_payload_axis_tuser(rx_eth_payload_axis_tuser),
      // Status signals
      .busy(),
      .error_header_early_termination()
  );

  eth_axis_tx eth_axis_tx_inst (
      .clk(clk),
      .rst(rst),
      // Ethernet frame input
      .s_eth_hdr_valid(tx_eth_hdr_valid),
      .s_eth_hdr_ready(tx_eth_hdr_ready),
      .s_eth_dest_mac(tx_eth_dest_mac),
      .s_eth_src_mac(tx_eth_src_mac),
      .s_eth_type(tx_eth_type),
      .s_eth_payload_axis_tdata(tx_eth_payload_axis_tdata),
      .s_eth_payload_axis_tvalid(tx_eth_payload_axis_tvalid),
      .s_eth_payload_axis_tready(tx_eth_payload_axis_tready),
      .s_eth_payload_axis_tlast(tx_eth_payload_axis_tlast),
      .s_eth_payload_axis_tuser(tx_eth_payload_axis_tuser),
      // AXI output
      .m_axis_tdata(tx_axis_tdata),
      .m_axis_tvalid(tx_axis_tvalid),
      .m_axis_tready(tx_axis_tready),
      .m_axis_tlast(tx_axis_tlast),
      .m_axis_tuser(tx_axis_tuser),
      // Status signals
      .busy()
  );

  udp_complete #(
      .UDP_CHECKSUM_GEN_ENABLE(0)
  ) udp_complete_inst (
      .clk(clk),
      .rst(rst),
      // Ethernet frame input
      .s_eth_hdr_valid(rx_eth_hdr_valid),
      .s_eth_hdr_ready(rx_eth_hdr_ready),
      .s_eth_dest_mac(rx_eth_dest_mac),
      .s_eth_src_mac(rx_eth_src_mac),
      .s_eth_type(rx_eth_type),
      .s_eth_payload_axis_tdata(rx_eth_payload_axis_tdata),
      .s_eth_payload_axis_tvalid(rx_eth_payload_axis_tvalid),
      .s_eth_payload_axis_tready(rx_eth_payload_axis_tready),
      .s_eth_payload_axis_tlast(rx_eth_payload_axis_tlast),
      .s_eth_payload_axis_tuser(rx_eth_payload_axis_tuser),
      // Ethernet frame output
      .m_eth_hdr_valid(tx_eth_hdr_valid),
      .m_eth_hdr_ready(tx_eth_hdr_ready),
      .m_eth_dest_mac(tx_eth_dest_mac),
      .m_eth_src_mac(tx_eth_src_mac),
      .m_eth_type(tx_eth_type),
      .m_eth_payload_axis_tdata(tx_eth_payload_axis_tdata),
      .m_eth_payload_axis_tvalid(tx_eth_payload_axis_tvalid),
      .m_eth_payload_axis_tready(tx_eth_payload_axis_tready),
      .m_eth_payload_axis_tlast(tx_eth_payload_axis_tlast),
      .m_eth_payload_axis_tuser(tx_eth_payload_axis_tuser),
      // IP frame input
      .s_ip_hdr_valid(tx_ip_hdr_valid),
      .s_ip_hdr_ready(tx_ip_hdr_ready),
      .s_ip_dscp(tx_ip_dscp),
      .s_ip_ecn(tx_ip_ecn),
      .s_ip_length(tx_ip_length),
      .s_ip_ttl(tx_ip_ttl),
      .s_ip_protocol(tx_ip_protocol),
      .s_ip_source_ip(tx_ip_source_ip),
      .s_ip_dest_ip(tx_ip_dest_ip),
      .s_ip_payload_axis_tdata(tx_ip_payload_axis_tdata),
      .s_ip_payload_axis_tvalid(tx_ip_payload_axis_tvalid),
      .s_ip_payload_axis_tready(tx_ip_payload_axis_tready),
      .s_ip_payload_axis_tlast(tx_ip_payload_axis_tlast),
      .s_ip_payload_axis_tuser(tx_ip_payload_axis_tuser),
      // IP frame output
      .m_ip_hdr_valid(rx_ip_hdr_valid),
      .m_ip_hdr_ready(rx_ip_hdr_ready),
      .m_ip_eth_dest_mac(rx_ip_eth_dest_mac),
      .m_ip_eth_src_mac(rx_ip_eth_src_mac),
      .m_ip_eth_type(rx_ip_eth_type),
      .m_ip_version(rx_ip_version),
      .m_ip_ihl(rx_ip_ihl),
      .m_ip_dscp(rx_ip_dscp),
      .m_ip_ecn(rx_ip_ecn),
      .m_ip_length(rx_ip_length),
      .m_ip_identification(rx_ip_identification),
      .m_ip_flags(rx_ip_flags),
      .m_ip_fragment_offset(rx_ip_fragment_offset),
      .m_ip_ttl(rx_ip_ttl),
      .m_ip_protocol(rx_ip_protocol),
      .m_ip_header_checksum(rx_ip_header_checksum),
      .m_ip_source_ip(rx_ip_source_ip),
      .m_ip_dest_ip(rx_ip_dest_ip),
      .m_ip_payload_axis_tdata(rx_ip_payload_axis_tdata),
      .m_ip_payload_axis_tvalid(rx_ip_payload_axis_tvalid),
      .m_ip_payload_axis_tready(rx_ip_payload_axis_tready),
      .m_ip_payload_axis_tlast(rx_ip_payload_axis_tlast),
      .m_ip_payload_axis_tuser(rx_ip_payload_axis_tuser),
      // UDP frame input
      .s_udp_hdr_valid(tx_udp_hdr_valid),
      .s_udp_hdr_ready(tx_udp_hdr_ready),
      .s_udp_ip_dscp(tx_udp_ip_dscp),
      .s_udp_ip_ecn(tx_udp_ip_ecn),
      .s_udp_ip_ttl(tx_udp_ip_ttl),
      .s_udp_ip_source_ip(tx_udp_ip_source_ip),
      .s_udp_ip_dest_ip(tx_udp_ip_dest_ip),
      .s_udp_source_port(tx_udp_source_port),
      .s_udp_dest_port(tx_udp_dest_port),
      .s_udp_length(tx_udp_length),
      .s_udp_checksum(tx_udp_checksum),
      .s_udp_payload_axis_tdata(tx_udp_payload_axis_tdata),
      .s_udp_payload_axis_tvalid(tx_udp_payload_axis_tvalid),
      .s_udp_payload_axis_tready(tx_udp_payload_axis_tready),
      .s_udp_payload_axis_tlast(tx_udp_payload_axis_tlast),
      .s_udp_payload_axis_tuser(tx_udp_payload_axis_tuser),
      // UDP frame output
      .m_udp_hdr_valid(rx_udp_hdr_valid),
      .m_udp_hdr_ready(rx_udp_hdr_ready),
      .m_udp_eth_dest_mac(rx_udp_eth_dest_mac),
      .m_udp_eth_src_mac(rx_udp_eth_src_mac),
      .m_udp_eth_type(rx_udp_eth_type),
      .m_udp_ip_version(rx_udp_ip_version),
      .m_udp_ip_ihl(rx_udp_ip_ihl),
      .m_udp_ip_dscp(rx_udp_ip_dscp),
      .m_udp_ip_ecn(rx_udp_ip_ecn),
      .m_udp_ip_length(rx_udp_ip_length),
      .m_udp_ip_identification(rx_udp_ip_identification),
      .m_udp_ip_flags(rx_udp_ip_flags),
      .m_udp_ip_fragment_offset(rx_udp_ip_fragment_offset),
      .m_udp_ip_ttl(rx_udp_ip_ttl),
      .m_udp_ip_protocol(rx_udp_ip_protocol),
      .m_udp_ip_header_checksum(rx_udp_ip_header_checksum),
      .m_udp_ip_source_ip(rx_udp_ip_source_ip),
      .m_udp_ip_dest_ip(rx_udp_ip_dest_ip),
      .m_udp_source_port(rx_udp_source_port),
      .m_udp_dest_port(rx_udp_dest_port),
      .m_udp_length(rx_udp_length),
      .m_udp_checksum(rx_udp_checksum),
      .m_udp_payload_axis_tdata(rx_udp_payload_axis_tdata),
      .m_udp_payload_axis_tvalid(rx_udp_payload_axis_tvalid),
      .m_udp_payload_axis_tready(rx_udp_payload_axis_tready),
      .m_udp_payload_axis_tlast(rx_udp_payload_axis_tlast),
      .m_udp_payload_axis_tuser(rx_udp_payload_axis_tuser),
      // Status signals
      .ip_rx_busy(),
      .ip_tx_busy(),
      .udp_rx_busy(),
      .udp_tx_busy(),
      .ip_rx_error_header_early_termination(),
      .ip_rx_error_payload_early_termination(),
      .ip_rx_error_invalid_header(),
      .ip_rx_error_invalid_checksum(),
      .ip_tx_error_payload_early_termination(),
      .ip_tx_error_arp_failed(),
      .udp_rx_error_header_early_termination(),
      .udp_rx_error_payload_early_termination(),
      .udp_tx_error_payload_early_termination(),
      // Configuration
      .local_mac(local_mac),
      .local_ip(local_ip),
      .gateway_ip(gateway_ip),
      .subnet_mask(subnet_mask),
      .clear_arp_cache(0)
  );

  // ============================================================================
  // PTP TIME SYNCHRONIZATION & PORT DEMULTIPLEXING
  // ============================================================================

  // ------------------------------------------------------------------
  // udp_complete: UDP protocol stack with relevant RX/TX signals
  // ------------------------------------------------------------------
  wire        s_udp_rx_hdr_valid;
  wire        s_udp_rx_hdr_ready;
  wire [31:0] s_udp_rx_ip_source_ip;
  wire [31:0] s_udp_rx_ip_dest_ip;
  wire [15:0] s_udp_rx_source_port;
  wire [15:0] s_udp_rx_dest_port;
  wire [15:0] s_udp_rx_length;

  wire [ 7:0] s_udp_rx_payload_axis_tdata;
  wire        s_udp_rx_payload_axis_tvalid;
  wire        s_udp_rx_payload_axis_tready;
  wire        s_udp_rx_payload_axis_tlast;
  wire        s_udp_rx_payload_axis_tuser;

  // TX (if used for PTP Delay_Req/Status)
  wire        m_udp_tx_hdr_valid;
  wire        m_udp_tx_hdr_ready;
  wire [31:0] m_udp_tx_ip_dest_ip;
  wire [15:0] m_udp_tx_source_port;
  wire [15:0] m_udp_tx_dest_port;
  wire [15:0] m_udp_tx_length;
  wire [ 7:0] m_udp_tx_payload_axis_tdata;
  wire        m_udp_tx_payload_axis_tvalid;
  wire        m_udp_tx_payload_axis_tready;
  wire        m_udp_tx_payload_axis_tlast;
  wire        m_udp_tx_payload_axis_tuser;


  // ------------------------------------------------------------------
  // 1) UDP RX Port Demux: 319 / 320 / 5004
  // ------------------------------------------------------------------
  // PTP Event (319)
  wire ptp_evt_hdr_valid, ptp_evt_hdr_ready;
  wire [31:0] ptp_evt_ip_src, ptp_evt_ip_dst;
  wire [15:0] ptp_evt_src_port, ptp_evt_dst_port, ptp_evt_len;
  wire [7:0] ptp_evt_tdata;
  wire ptp_evt_tvalid, ptp_evt_tready, ptp_evt_tlast, ptp_evt_tuser;
  // PTP General (320)
  wire ptp_gen_hdr_valid, ptp_gen_hdr_ready;
  wire [31:0] ptp_gen_ip_src, ptp_gen_ip_dst;
  wire [15:0] ptp_gen_src_port, ptp_gen_dst_port, ptp_gen_len;
  wire [7:0] ptp_gen_tdata;
  wire ptp_gen_tvalid, ptp_gen_tready, ptp_gen_tlast, ptp_gen_tuser;
  // AES67 (5004)
  wire aes_hdr_valid, aes_hdr_ready;
  wire [31:0] aes_ip_src, aes_ip_dst;
  wire [15:0] aes_src_port, aes_dst_port, aes_len;
  wire [7:0] aes_tdata;
  wire aes_tvalid, aes_tready, aes_tlast, aes_tuser;

  // AES67 (5004)
  wire rx_ctrl_hdr_valid, rx_ctrl_hdr_ready;
  wire [15:0] rx_ctrl_dst_port;
  wire [ 7:0] rx_ctrl_payload_tdata;
  wire rx_ctrl_payload_tvalid, rx_ctrl_payload_tready, rx_ctrl_payload_tlast, rx_ctrl_payload_tuser;


  // Port 319 (PTP Event)
  wire [7:0] ptp319_data;
  wire       ptp319_start;
  wire       ptp319_end;
  wire       ptp319_valid;
  wire       ptp319_ready;

  // Port 320 (PTP General)
  wire [7:0] ptp320_data;
  wire       ptp320_start;
  wire       ptp320_end;
  wire       ptp320_valid;
  wire       ptp320_ready;

  // Port 5004 (RTP AES67)
  wire [7:0] rtp5004_data;
  wire       rtp5004_start;
  wire       rtp5004_end;
  wire       rtp5004_valid;
  wire       rtp5004_ready;

  // Port 7880 (Control)
  wire [7:0] ctrl7880_data;
  wire       ctrl7880_start;
  wire       ctrl7880_end;
  wire       ctrl7880_valid;
  wire       ctrl7880_ready;

  wire       recv_tick;

  // ---------------------------------------------------------
  // UDP Port Demux + 4 asynchrone FIFOs
  // ---------------------------------------------------------
  udp_port_demux_top udp_port_demux_top_inst (
      // Clocks / Reset
      .clk_125(clk),
      .rst_125(rst),
      .clk_50 (clk_50m),
      .rst_50 (rst_audio),

      // UDP RX Interface von verilog-ethernet
      .s_udp_hdr_valid(rx_udp_hdr_valid),
      .s_udp_hdr_ready(rx_udp_hdr_ready),
      .s_udp_src_port (rx_udp_source_port),
      .s_udp_dst_port (rx_udp_dest_port),
      .s_udp_length   (rx_udp_length),
      .s_udp_checksum (rx_udp_checksum),

      .s_udp_payload_tdata (rx_udp_payload_axis_tdata),
      .s_udp_payload_tvalid(rx_udp_payload_axis_tvalid),
      .s_udp_payload_tready(rx_udp_payload_axis_tready),
      .s_udp_payload_tlast (rx_udp_payload_axis_tlast),

      // Port 319 (PTP Event)
      .ptp319_data (ptp319_data),
      .ptp319_start(ptp319_start),
      .ptp319_end  (ptp319_end),
      .ptp319_valid(ptp319_valid),
      .ptp319_ready(ptp319_ready),

      // Port 320 (PTP General)
      .ptp320_data (ptp320_data),
      .ptp320_start(ptp320_start),
      .ptp320_end  (ptp320_end),
      .ptp320_valid(ptp320_valid),
      .ptp320_ready(ptp320_ready),

      // Port 5004 (RTP AES67)
      .rtp5004_data (rtp5004_data),
      .rtp5004_start(rtp5004_start),
      .rtp5004_end  (rtp5004_end),
      .rtp5004_valid(rtp5004_valid),
      .rtp5004_ready(rtp5004_ready),

      // Port 7880 (Control)
      .ctrl7880_data (ctrl7880_data),
      .ctrl7880_start(ctrl7880_start),
      .ctrl7880_end  (ctrl7880_end),
      .ctrl7880_valid(ctrl7880_valid),
      .ctrl7880_ready(ctrl7880_ready)
  );

  wire [47:0] ptp_sec;
  wire [31:0] ptp_ns;
  wire        perout_48k;

  assign ptp_time = {ptp_sec, ptp_ns};

  ptp_client_stream_50m u_ptp (
      .clk    (clk_50m),
      .rst    (rst_audio),
      .s_data (ptp319_data),
      .s_valid(ptp319_valid),
      .s_ready(ptp319_ready),
      .s_start(ptp319_start),
      .s_end  (ptp319_end),
      .ptp_sec(ptp_sec),
      .ptp_ns (ptp_ns)
  );

  ptp_perout_48k u_perout (
      .clk       (clk_50m),
      .rst       (rst_audio),
      .ptp_sec   (ptp_sec),
      .ptp_ns    (ptp_ns),
      .perout_48k(perout_48k)
  );

  // Central registers for the mixer:

  // Raw controls (preferred)
  reg        [ 9:0] gain0;
  reg        [ 9:0] gain1;
  reg        [ 9:0] gain2;
  reg        [ 9:0] gain3;
  reg        [ 9:0] gain4;
  reg        [ 9:0] gain5;
  reg        [ 9:0] gain6;
  reg        [ 9:0] gain7;
  reg        [ 9:0] gain8;
  reg        [ 9:0] gain9;
  reg        [ 9:0] gain10;
  reg        [ 9:0] gain11;
  reg        [ 9:0] gain12;
  reg        [ 9:0] gain13;
  reg        [ 9:0] gain14;
  reg        [ 9:0] gain15;

  reg signed [ 7:0] pan0;
  reg signed [ 7:0] pan1;
  reg signed [ 7:0] pan2;
  reg signed [ 7:0] pan3;
  reg signed [ 7:0] pan4;
  reg signed [ 7:0] pan5;
  reg signed [ 7:0] pan6;
  reg signed [ 7:0] pan7;
  reg signed [ 7:0] pan8;
  reg signed [ 7:0] pan9;
  reg signed [ 7:0] pan10;
  reg signed [ 7:0] pan11;
  reg signed [ 7:0] pan12;
  reg signed [ 7:0] pan13;
  reg signed [ 7:0] pan14;
  reg signed [ 7:0] pan15;

  reg        [15:0] mute_button;
  reg        [15:0] solo_button;

  // Raw controls (preferred)
  wire [9:0] c_gain0, r_gain0;
  wire [9:0] c_gain1, r_gain1;
  wire [9:0] c_gain2, r_gain2;
  wire [9:0] c_gain3, r_gain3;
  wire [9:0] c_gain4, r_gain4;
  wire [9:0] c_gain5, r_gain5;
  wire [9:0] c_gain6, r_gain6;
  wire [9:0] c_gain7, r_gain7;
  wire [9:0] c_gain8, r_gain8;
  wire [9:0] c_gain9, r_gain9;
  wire [9:0] c_gain10, r_gain10;
  wire [9:0] c_gain11, r_gain11;
  wire [9:0] c_gain12, r_gain12;
  wire [9:0] c_gain13, r_gain13;
  wire [9:0] c_gain14, r_gain14;
  wire [9:0] c_gain15, r_gain15;

  wire signed [7:0] c_pan0, r_pan0;
  wire signed [7:0] c_pan1, r_pan1;
  wire signed [7:0] c_pan2, r_pan2;
  wire signed [7:0] c_pan3, r_pan3;
  wire signed [7:0] c_pan4, r_pan4;
  wire signed [7:0] c_pan5, r_pan5;
  wire signed [7:0] c_pan6, r_pan6;
  wire signed [7:0] c_pan7, r_pan7;
  wire signed [7:0] c_pan8, r_pan8;
  wire signed [7:0] c_pan9, r_pan9;
  wire signed [7:0] c_pan10, r_pan10;
  wire signed [7:0] c_pan11, r_pan11;
  wire signed [7:0] c_pan12, r_pan12;
  wire signed [7:0] c_pan13, r_pan13;
  wire signed [7:0] c_pan14, r_pan14;
  wire signed [7:0] c_pan15, r_pan15;

  wire [15:0] c_mute_button, r_mute_button;
  wire [15:0] c_solo_button, r_solo_button;

  wire c_set_mute, c_set_solo;
  wire r_set_mute, r_set_gain, r_set_pan, r_set_solo;


  // 50 MHz, 200 ms lockout => 10,000,000 cycles
  localparam integer CLK_HZ = 50000000;
  localparam integer LOCK_MS = 200;
  localparam integer LOCK_TICKS = (CLK_HZ / 1000) * LOCK_MS;  // 50_000 * 200 = 10_000_000

  // 10,000,000 < 2^24 (16,777,216) => 24-bit reichen
  reg [23:0] block_c_cnt;  // blocks C-updates after accepted R-update
  reg [23:0] block_r_cnt;  // blocks R-updates after accepted C-update

  wire c_blocked = (block_c_cnt != 24'd0);
  wire r_blocked = (block_r_cnt != 24'd0);

  wire c_set_any = c_set_gain | c_set_pan | c_set_mute | c_set_solo;
  wire r_set_any = r_set_gain | r_set_pan | r_set_mute | r_set_solo;

  // "akzeptiert" bedeutet: set-puls kommt UND nicht geblockt
  wire c_any_accept = c_set_any & ~c_blocked;
  wire r_any_accept = r_set_any & ~r_blocked;

  wire c_gain_we = c_set_gain & ~c_blocked;
  wire r_gain_we = r_set_gain & ~r_blocked;

  wire c_pan_we = c_set_pan & ~c_blocked;
  wire r_pan_we = r_set_pan & ~r_blocked;

  wire c_mute_we = c_set_mute & ~c_blocked;
  wire r_mute_we = r_set_mute & ~r_blocked;

  wire c_solo_we = c_set_solo & ~c_blocked;
  wire r_solo_we = r_set_solo & ~r_blocked;

  always @(posedge clk_50m) begin
    if (rst_audio) begin
      block_c_cnt <= 24'd0;
      block_r_cnt <= 24'd0;

      // optional init defaults:
      {gain15,gain14,gain13,gain12,gain11,gain10,gain9,gain8,gain7,gain6,gain5,gain4,gain3,gain2,gain1,gain0} <= {16{10'd0}};
      {pan15,pan14,pan13,pan12,pan11,pan10,pan9,pan8,pan7,pan6,pan5,pan4,pan3,pan2,pan1,pan0} <= {16{8'sd0}};
      mute_button <= 16'h0000;
      solo_button <= 16'h0000;
    end else begin
      // Countdown counter
      if (block_c_cnt != 24'd0) block_c_cnt <= block_c_cnt - 24'd1;
      if (block_r_cnt != 24'd0) block_r_cnt <= block_r_cnt - 24'd1;

      // Lockout-Start/Restart:
      // If C was accepted => block R for 200ms (restart)
      if (c_any_accept) begin
        block_r_cnt <= LOCK_TICKS[23:0];
      end
      // If R was accepted => block C for 200ms (restart)
      if (r_any_accept) begin
        block_c_cnt <= LOCK_TICKS[23:0];
      end

      // Register-Updates (nur wenn jeweilige Seite nicht geblockt ist)
      if (c_gain_we) begin
        if (touch[0]) gain0 <= c_gain0;
        if (touch[1]) gain1 <= c_gain1;
        if (touch[2]) gain2 <= c_gain2;
        if (touch[3]) gain3 <= c_gain3;
        if (touch[4]) gain4 <= c_gain4;
        if (touch[5]) gain5 <= c_gain5;
        if (touch[6]) gain6 <= c_gain6;
        if (touch[7]) gain7 <= c_gain7;
      end else if (r_gain_we) begin
        {gain15,gain14,gain13,gain12,gain11,gain10,gain9,gain8,gain7,gain6,gain5,gain4,gain3,gain2,gain1,gain0} <=
            {
          r_gain15,
          r_gain14,
          r_gain13,
          r_gain12,
          r_gain11,
          r_gain10,
          r_gain9,
          r_gain8,
          r_gain7,
          r_gain6,
          r_gain5,
          r_gain4,
          r_gain3,
          r_gain2,
          r_gain1,
          r_gain0
        };
      end

      if (c_pan_we) begin
        {pan15,pan14,pan13,pan12,pan11,pan10,pan9,pan8,pan7,pan6,pan5,pan4,pan3,pan2,pan1,pan0} <=
            {
          c_pan15,
          c_pan14,
          c_pan13,
          c_pan12,
          c_pan11,
          c_pan10,
          c_pan9,
          c_pan8,
          c_pan7,
          c_pan6,
          c_pan5,
          c_pan4,
          c_pan3,
          c_pan2,
          c_pan1,
          c_pan0
        };
      end else if (r_pan_we) begin
        {pan15,pan14,pan13,pan12,pan11,pan10,pan9,pan8,pan7,pan6,pan5,pan4,pan3,pan2,pan1,pan0} <=
            {
          r_pan15,
          r_pan14,
          r_pan13,
          r_pan12,
          r_pan11,
          r_pan10,
          r_pan9,
          r_pan8,
          r_pan7,
          r_pan6,
          r_pan5,
          r_pan4,
          r_pan3,
          r_pan2,
          r_pan1,
          r_pan0
        };
      end

      if (c_mute_we) begin
        mute_button <= mute_button ^ c_mute_button;
      end else if (r_mute_we) begin
        mute_button <= r_mute_button;
      end

      if (c_solo_we) begin
        solo_button <= c_solo_button;
      end else if (r_solo_we) begin
        solo_button <= r_solo_button;
      end
    end
  end


  wire c_set_pan = (c_pan_dirty > 0);
  wire c_set_gain = (touch > 0);

  // Flattened results from batch module
  wire [16*16-1:0] gain_l_flat;
  wire [16*16-1:0] gain_r_flat;

  // Instantiate batch module (FIFO/ready handshake)
  pan_gain_coeff_calc16_rom u_pan_gain (
      .clk  (clk_50m),
      .rst  (rst_audio),
      .start(perout_48k),

      .gain0 (gain0),
      .pan0  (pan0),
      .gain1 (gain1),
      .pan1  (pan1),
      .gain2 (gain2),
      .pan2  (pan2),
      .gain3 (gain3),
      .pan3  (pan3),
      .gain4 (gain4),
      .pan4  (pan4),
      .gain5 (gain5),
      .pan5  (pan5),
      .gain6 (gain6),
      .pan6  (pan6),
      .gain7 (gain7),
      .pan7  (pan7),
      .gain8 (gain8),
      .pan8  (pan8),
      .gain9 (gain9),
      .pan9  (pan9),
      .gain10(gain10),
      .pan10 (pan10),
      .gain11(gain11),
      .pan11 (pan11),
      .gain12(gain12),
      .pan12 (pan12),
      .gain13(gain13),
      .pan13 (pan13),
      .gain14(gain14),
      .pan14 (pan14),
      .gain15(gain15),
      .pan15 (pan15),

      .g0L (g0L),
      .g1L (g1L),
      .g2L (g2L),
      .g3L (g3L),
      .g4L (g4L),
      .g5L (g5L),
      .g6L (g6L),
      .g7L (g7L),
      .g8L (g8L),
      .g9L (g9L),
      .g10L(g10L),
      .g11L(g11L),
      .g12L(g12L),
      .g13L(g13L),
      .g14L(g14L),
      .g15L(g15L),
      .g0R (g0R),
      .g1R (g1R),
      .g2R (g2R),
      .g3R (g3R),
      .g4R (g4R),
      .g5R (g5R),
      .g6R (g6R),
      .g7R (g7R),
      .g8R (g8R),
      .g9R (g9R),
      .g10R(g10R),
      .g11R(g11R),
      .g12R(g12R),
      .g13R(g13R),
      .g14R(g14R),
      .g15R(g15R),

      .busy(),
      .done()
  );


  // ------------------------------------------------------------------
  // 3) AES67 RTP RX -> Jitter Buffer -> Mixer -> SPDIF
  // ------------------------------------------------------------------
  // AES67 RTP RX
  wire [31:0] a_rtp_ts;
  wire [15:0] a_rtp_seq;
  wire signed [23:0] a_ch0,a_ch1,a_ch2,a_ch3,a_ch4,a_ch5,a_ch6,a_ch7,
a_ch8,a_ch9,a_ch10,a_ch11,a_ch12,a_ch13,a_ch14,a_ch15;
  wire        a_samples_valid;

  wire [31:0] f_rtp_ts;
  wire [15:0] f_rtp_seq;
  wire signed [23:0] f_ch0,f_ch1,f_ch2,f_ch3,f_ch4,f_ch5,f_ch6,f_ch7,
f_ch8,f_ch9,f_ch10,f_ch11,f_ch12,f_ch13,f_ch14,f_ch15;
  wire f_samples_valid;

  wire debug_tx_tick;

  wire signed [23:0] jb_ch0,jb_ch1,jb_ch2,jb_ch3,jb_ch4,jb_ch5,jb_ch6,jb_ch7,
jb_ch8,jb_ch9,jb_ch10,jb_ch11,jb_ch12,jb_ch13,jb_ch14,jb_ch15;
  wire jb_valid;

  wire signed [15:0] g0L,g1L,g2L,g3L,
g4L,g5L,g6L,g7L,
g8L,g9L,g10L,g11L,
g12L,g13L,g14L,g15L;
  wire signed [15:0] g0R,g1R,g2R,g3R,
g4R,g5R,g6R,g7R,
g8R,g9R,g10R,g11R,
g12R,g13R,g14R,g15R;


  wire signed [23:0] mix_l, mix_r;
  wire mix_valid;

  aes67_rtp_rx_fifo u_aes67_rx (
      .clk(clk_50m),
      .rst(rst_audio),
      // FIFO-Interface
      .fifo_dout  (rtp5004_data),
      .fifo_start  (rtp5004_start),
      .fifo_end   (rtp5004_end),
      .fifo_valid (rtp5004_valid),
      .fifo_ready (rtp5004_ready),
      .ptp_time(ptp_time),
      .recv_tick(recv_tick),
      .ch0(a_ch0),
      .ch1(a_ch1),
      .ch2(a_ch2),
      .ch3(a_ch3),
      .ch4(a_ch4),
      .ch5(a_ch5),
      .ch6(a_ch6),
      .ch7(a_ch7),
      .ch8(a_ch8),
      .ch9(a_ch9),
      .ch10(a_ch10),
      .ch11(a_ch11),
      .ch12(a_ch12),
      .ch13(a_ch13),
      .ch14(a_ch14),
      .ch15(a_ch15),
      .samples_valid(a_samples_valid),
      .rtp_ts(),
      .rx_ts(a_rtp_ts),
      .rtp_seq(a_rtp_seq)
  );

  mixer_16x2_q14_pipelined u_mix (
      .clk(clk_50m),
      .rst(rst_audio),
      .tick(),
      .ch_valid(a_samples_valid),
      .ch0(a_ch0),
      .ch1(a_ch1),
      .ch2(a_ch2),
      .ch3(a_ch3),
      .ch4(a_ch4),
      .ch5(a_ch5),
      .ch6(a_ch6),
      .ch7(a_ch7),
      .ch8(a_ch8),
      .ch9(a_ch9),
      .ch10(a_ch10),
      .ch11(a_ch11),
      .ch12(a_ch12),
      .ch13(a_ch13),
      .ch14(a_ch14),
      .ch15(a_ch15),
      .g0L(g0L),
      .g1L(g1L),
      .g2L(g2L),
      .g3L(g3L),
      .g4L(g4L),
      .g5L(g5L),
      .g6L(g6L),
      .g7L(g7L),
      .g8L(g8L),
      .g9L(g9L),
      .g10L(g10L),
      .g11L(g11L),
      .g12L(g12L),
      .g13L(g13L),
      .g14L(g14L),
      .g15L(g15L),
      .g0R(g0R),
      .g1R(g1R),
      .g2R(g2R),
      .g3R(g3R),
      .g4R(g4R),
      .g5R(g5R),
      .g6R(g6R),
      .g7R(g7R),
      .g8R(g8R),
      .g9R(g9R),
      .g10R(g10R),
      .g11R(g11R),
      .g12R(g12R),
      .g13R(g13R),
      .g14R(g14R),
      .g15R(g15R),
      .solo_button(solo_button),
      .mute_button(mute_button),
      .out_l(mix_r),
      .out_r(mix_l),
      .out_valid(mix_valid)
  );


  AudioJitterBuffer jitter_inst (
      // Netz-Domain (AES67)
      .clk_in(clk_50m),
      .rst_n (~rst_audio),
      .din_l (mix_l),
      .din_r (mix_r),
      .wr_en (mix_valid),

      // Audio-Domain (SPDIF)
      .clk_out(clk_12m288),
      .rd_en(tick_48k),
      .out_valid(spdif_valid),

      .dout_l(spdif_l),
      .dout_r(spdif_r),

      .fill_count(xfade_level),
      .cnt_fifo_reads(dbg_xfade_read),
      .cnt_out_samples(spdif_samples),
      .cnt_duplicated(dbg_xfade_dup),
      .cnt_skipped(dbg_xfade_skip)
  );

  // SPDIF-Transmitter
  spdif_transmitter u_spdif (
      .bit_clock   (clk_12m288),
      .sample_tick (tick_48k),
      .sample_valid(spdif_valid),
      .sample_l    (spdif_l),
      .sample_r    (spdif_r),
      .spdif_out   (spdif_out)
  );

  wire spdif_valid, dac_valid;
  wire [23:0] spdif_l, dac_l;
  wire [23:0] spdif_r, dac_r;

  wire tick_48k;
  wire playback_active;


  aes67_ctrl_name_rx ctrl_name_rx_inst (
      .clk(clk_50m),
      .rst(rst_audio),

      .udp_rx_payload_tdata (ctrl7880_data),
      .udp_rx_payload_tvalid(ctrl7880_valid),
      .udp_rx_payload_tready(ctrl7880_ready),
      .udp_rx_payload_start (ctrl7880_start),
      .udp_rx_payload_end   (ctrl7880_end),

      .gain0 (r_gain0),
      .pan0  (r_pan0),
      .gain1 (r_gain1),
      .pan1  (r_pan1),
      .gain2 (r_gain2),
      .pan2  (r_pan2),
      .gain3 (r_gain3),
      .pan3  (r_pan3),
      .gain4 (r_gain4),
      .pan4  (r_pan4),
      .gain5 (r_gain5),
      .pan5  (r_pan5),
      .gain6 (r_gain6),
      .pan6  (r_pan6),
      .gain7 (r_gain7),
      .pan7  (r_pan7),
      .gain8 (r_gain8),
      .pan8  (r_pan8),
      .gain9 (r_gain9),
      .pan9  (r_pan9),
      .gain10(r_gain10),
      .pan10 (r_pan10),
      .gain11(r_gain11),
      .pan11 (r_pan11),
      .gain12(r_gain12),
      .pan12 (r_pan12),
      .gain13(r_gain13),
      .pan13 (r_pan13),
      .gain14(r_gain14),
      .pan14 (r_pan14),
      .gain15(r_gain15),
      .pan15 (r_pan15),

      .name_rd0_addr(name_bram_addr),
      .name_rd0_data(name_bram_dout),

      .name_rd1_addr(name_rd1_addr),
      .name_rd1_data(name_rd1_data),

      .mute_button(r_mute_button),
      .solo_button(r_solo_button),

      .set_gain(r_set_gain),
      .set_pan (r_set_pan),
      .set_mute(r_set_mute),
      .set_solo(r_set_solo),

      .set_name(),

      .name_dirty(name_dirty_set),
      .pan_dirty (pan_dirty_set)
  );



  // --------------------------------------------------------------------
  // Status-Telemetrie (UDP) + PTP-DelayReq (optional) → TX-Mux → udp_complete
  // --------------------------------------------------------------------
  // Status-TX
  wire st_hdr_valid, st_hdr_ready;
  wire [31:0] st_ip_dest;
  wire [15:0] st_src_port, st_dst_port, st_length;
  wire [7:0] st_tdata;
  wire st_tvalid, st_tready, st_end, st_start;
  // JB-Statistiken
  wire [31:0] jb_und = 32'd0;  // underrun
  wire [31:0] jb_ovw = 32'd0;  // overwrite
  wire [31:0] jb_lat = 32'd0;  // late
  wire [31:0] jb_ear = 32'd0;  // early

  wire [31:0] ptp_debug_time;
  assign ptp_debug_time = ptp_time[31:0];

  wire [63:0] dbg_ptp_time_ns;  // absolute PTP-Zeit in ns
  wire [31:0] dbg_rtp_read_ts;  // aktueller Read-TS im Jitter-Buffer
  wire [31:0] dbg_rtp_write_ts;  // letzter Write-TS
  wire [31:0] dbg_samples_read = 32'd0;  // Anzahl gelesener Samples
  wire [31:0] dbg_samples_written = 32'd0;  // Anzahl geschriebener Sample
  wire [31:0] spdif_samples;  // Anzahl geschriebener Sample
  wire [31:0] dbg_xfade_skip;  // Anzahl geschriebener Sample
  wire [31:0] dbg_xfade_dup;  // Anzahl geschriebener Sample
  wire [31:0] dbg_xfade_read;  // Anzahl geschriebener Sample
  wire [31:0] xfade_level;  // Anzahl geschriebener Sample

  wire        sample_debug_rd;
  wire [47:0] sample_debug_out;
  wire        sample_debug_empty;
  wire [ 8:0] sample_debug_level;

  wire [95:0] ptp_time;

  wire [31:0] zeros;
  assign zeros = 32'd0;

  assign dbg_ptp_time_ns = ptp_time[63:0];
  wire signed [31:0] ptp_rate_ppb;
  assign ptp_rate_ppb = 32'd0;

  wire [16*16-1:0] gain_reg_flat;
  wire [16*16-1:0] pan_reg_flat;

  // interne "Arrays"
  wire signed [15:0] gain_reg[0:15];
  wire signed [15:0] pan_reg[0:15];

  genvar i;
  generate
    for (i = 0; i < 16; i = i + 1) begin : GEN_UNPACK
      assign gain_reg[i] = gain_reg_flat[i*16+:16];
      assign pan_reg[i]  = pan_reg_flat[i*16+:16];
    end
  endgenerate

  wire [7:0] name_rd1_addr;
  wire [7:0] name_rd1_data;

  udp_status_tx_bram #(
      .INTERVAL_MS(50)
  ) u_status (
      .clk(clk_50m),
      .rst(rst_audio),
      .sample_tick(a_samples_valid),
      .tick_48k(perout_48k),
      .stat_underrun(jb_und),
      .stat_overwrite(jb_ovw),
      .stat_late(jb_lat),
      .stat_early(jb_ear),
      .ptp_time_sec(ptp_sec),
      .ptp_time_ns(ptp_ns),
      .rtp_read_ts(dbg_rtp_read_ts),
      .rtp_write_ts(dbg_rtp_write_ts),
      .samples_read(dbg_samples_read),
      .samples_written(dbg_samples_written),

      .buffer_fill    (xfade_level),
      .spdif_samples  (spdif_samples),
      .xfade_main_cnt (dbg_xfade_read),
      .xfade_extra_cnt(dbg_xfade_skip),
      .xfade_dup_cnt  (dbg_xfade_dup),
      .xfade_zero_cnt (zeros),

      .ch0(a_ch0),
      .ch1(a_ch1),
      .ch2(a_ch2),
      .ch3(a_ch3),
      .ch4(a_ch4),
      .ch5(a_ch5),
      .ch6(a_ch6),
      .ch7(a_ch7),
      .ch8(a_ch8),
      .ch9(a_ch9),
      .ch10(a_ch10),
      .ch11(a_ch11),
      .ch12(a_ch12),
      .ch13(a_ch13),
      .ch14(a_ch14),
      .ch15(a_ch15),
      .peak_ch0(peak0_q15),
      .peak_ch1(peak1_q15),
      .peak_ch2(peak2_q15),
      .peak_ch3(peak3_q15),
      .peak_ch4(peak4_q15),
      .peak_ch5(peak5_q15),
      .peak_ch6(peak6_q15),
      .peak_ch7(peak7_q15),
      .peak_ch8(peak8_q15),
      .peak_ch9(peak9_q15),
      .peak_ch10(peak10_q15),
      .peak_ch11(peak11_q15),
      .peak_ch12(peak12_q15),
      .peak_ch13(peak13_q15),
      .peak_ch14(peak14_q15),
      .peak_ch15(peak15_q15),

      .mute_mask(mute_button),
      .solo_mask(solo_button),

      // gain/pan direkt aus CTRL-Arrays
      .gain_u10_ch0 (gain0),
      .gain_u10_ch1 (gain1),
      .gain_u10_ch2 (gain2),
      .gain_u10_ch3 (gain3),
      .gain_u10_ch4 (gain4),
      .gain_u10_ch5 (gain5),
      .gain_u10_ch6 (gain6),
      .gain_u10_ch7 (gain7),
      .gain_u10_ch8 (gain8),
      .gain_u10_ch9 (gain9),
      .gain_u10_ch10(gain10),
      .gain_u10_ch11(gain11),
      .gain_u10_ch12(gain12),
      .gain_u10_ch13(gain13),
      .gain_u10_ch14(gain14),
      .gain_u10_ch15(gain15),

      .pan_s8_ch0 (pan0),
      .pan_s8_ch1 (pan1),
      .pan_s8_ch2 (pan2),
      .pan_s8_ch3 (pan3),
      .pan_s8_ch4 (pan4),
      .pan_s8_ch5 (pan5),
      .pan_s8_ch6 (pan6),
      .pan_s8_ch7 (pan7),
      .pan_s8_ch8 (pan8),
      .pan_s8_ch9 (pan9),
      .pan_s8_ch10(pan10),
      .pan_s8_ch11(pan11),
      .pan_s8_ch12(pan12),
      .pan_s8_ch13(pan13),
      .pan_s8_ch14(pan14),
      .pan_s8_ch15(pan15),

      .name_rd_addr(name_rd1_addr),
      .name_rd_data(name_rd1_data),

      .m_tdata(st_tdata),
      .m_tvalid(st_tvalid),
      .m_tready(st_tready),
      .m_end(st_end),
      .m_start(st_start)
  );

  // Debug-TX
  wire dbg_hdr_valid, dbg_hdr_ready;
  wire [31:0] dbg_ip_dest;
  wire [15:0] dbg_src_port, dbg_dst_port, dbg_length;
  wire [7:0] dbg_tdata;
  wire dbg_tvalid, dbg_tready, dbg_end, dbg_start;
  wire [7:0] rtpout_tdata;
  wire rtpout_tvalid, rtpout_tready, rtpout_end, rtpout_start;
  wire [7:0] sdp_tdata;
  wire sdp_tvalid, sdp_tready, sdp_end, sdp_start;

  // udp_stats_sender instanzieren
  udp_debug_tx udp_debug_tx_inst (
      .clk(clk_50m),
      .rst(rst_audio),

      // Trigger: nur Paket senden, wenn Snapshot bereit ist
      .tick(recv_tick),

      .ptp_time       (ptp_time),
      .rtp_seq        (a_rtp_seq),
      .rtp_ts         (a_rtp_ts),
      .written_samples(dbg_samples_written),
      .read_samples   (dbg_samples_read),
      .buffer_fill    (xfade_level),
      .first_sample   (a_ch0),
      .spdif_samples  (spdif_samples),
      .xfade_main_cnt (dbg_xfade_read),
      .xfade_extra_cnt(dbg_xfade_skip),
      .xfade_dup_cnt  (dbg_xfade_dup),
      .xfade_zero_cnt (zeros),

      .sample_debug_rd(sample_debug_rd),
      .sample_debug_out(sample_debug_out),
      .sample_debug_empty(sample_debug_empty),
      .sample_debug_level(sample_debug_level),


      .m_tdata (dbg_tdata),
      .m_tvalid(dbg_tvalid),
      .m_tready(dbg_tready),
      .m_end   (dbg_end),
      .m_start (dbg_start)

  );

  wire sample_debug_wr;
  wire sample_debug_full;

  assign sample_debug_wr = spdif_valid && !sample_debug_full;

  sdebug_async_fifo sample_debug_fifo (
      .aclr(rst_audio),
      .data({spdif_r, spdif_l}),
      .rdclk(clk_50m),
      .rdreq(sample_debug_rd),
      .wrclk(clk_12m288),
      .wrreq(sample_debug_wr),
      .q(sample_debug_out),
      .rdempty(sample_debug_empty),
      .rdusedw(sample_debug_level),
      .wrfull(sample_debug_full)
  );

  wire [49:0] aes67_sfifo_in_samples = {2'd0, mix_r, mix_l};
  wire [49:0] aes67_sfifo_out_samples;
  wire aes67_sfifo_empty, aes67_sfifo_full, aes67_sfifo_read, aes67_sfifo_write;
  wire [7:0] aes67_sfifo_level;
  wire aes67_sfifo_almfull;

  aes67sendfifo aes67sendfifo_inst (
      .aclr(rst_audio),
      .data(aes67_sfifo_in_samples),
      .clock(clk_50m),
      .rdreq(aes67_sfifo_read),
      .wrreq(mix_valid),
      .q(aes67_sfifo_out_samples),
      .empty(aes67_sfifo_empty),
      .usedw(aes67_sfifo_level),
      .almost_full(aes67_sfifo_almfull),
      .full(aes67_sfifo_full)
  );

  wire p0_hdr_valid = 1'b0;
  wire p0_hdr_ready;
  wire [31:0] p0_ip_dest = 32'd0;
  wire [15:0] p0_src_port = 16'd0, p0_dst_port = 16'd0, p0_length = 16'd0;
  wire [7:0] p0_tdata = 8'd0;
  wire p0_tvalid = 1'b0;
  wire p0_tready;
  wire p0_tlast = 1'b0;
  wire p0_tuser = 1'b0;

  sdp_udp_stream_from_rom #(
      .ADDR_WIDTH(10),
      .ROM_LEN   (249),       
      .PERIOD_MS (100000000)
  ) sdp_tx_i (
      .clk(clk_50m),
      .rst(rst_audio),
      .tick_1ms(tick_1ms),
      .rom_addr(sdp_rom_addr),
      .rom_data(sdp_rom_data),
      .s_data(sdp_tdata),
      .s_start(sdp_start),
      .s_end(sdp_end),
      .s_valid(sdp_tvalid),
      .s_avail(),
      .s_ready(sdp_tready)
  );

  wire [7:0] sdp_rom_data;
  wire [9:0] sdp_rom_addr;

  sdp_rom #(
      .ADDR_WIDTH(10),
      .ROM_LEN(249),
      .INIT_FILE("sdp4_rom.hex")
  ) sdp_rom_i (
      .clk (clk),
      .addr(sdp_rom_addr),
      .data(sdp_rom_data)
  );


  wire        udp_tx_hdr_valid;
  wire        udp_tx_hdr_ready;
  wire [31:0] udp_tx_ip_dest_ip;
  wire [15:0] udp_tx_udp_src_port;
  wire [15:0] udp_tx_udp_dst_port;
  wire [15:0] udp_tx_udp_length;

  wire [ 7:0] udp_tx_axis_tdata;
  wire        udp_tx_axis_tvalid;
  wire        udp_tx_axis_tready;
  wire        udp_tx_axis_tlast;

  wire        tick_1ms = 1'b1;

  aes67_tx_stream2_from_fifo48_lr #(
      .SSRC  (32'h12345678),
      .RTP_PT(8'd96)
  ) aes_src (
      .clk(clk_50m),
      .rst(rst_audio),
      .tick_1ms(recv_tick),
      .samp_fifo_dout(aes67_sfifo_out_samples[47:0]),
      .samp_fifo_valid(!aes67_sfifo_empty),
      .samp_fifo_ready(aes67_sfifo_read),
      .samp_fifo_level(aes67_sfifo_level),
      .samp_fifo_alm_full(aes67_sfifo_almfull),
      .seq_set(recv_tick),
      .seq_in(a_rtp_seq),
      .ts_set(recv_tick),
      .ts_in(a_rtp_ts),
      .s_data(rtpout_tdata),
      .s_start(rtpout_start),
      .s_end(rtpout_end),
      .s_valid(rtpout_tvalid),
      .s_avail(),
      .s_ready(rtpout_tready)
  );


  // UDP IP-Felder/Checksum Defaults
  assign tx_udp_ip_dscp = 6'd0;
  assign tx_udp_ip_ecn = 2'd0;
  assign tx_udp_ip_ttl = 8'd64;
  assign tx_udp_ip_source_ip = local_ip;
  assign tx_udp_checksum = 16'd0;  // vom Core berechnet

  wire udp_tx_busy = 1'b0;

  // -------------------------
  // Channel -> Mux wires
  // -------------------------
  wire [7:0] s0_data, s1_data, s2_data, s3_data;
  wire s0_start, s1_start, s2_start, s3_start;
  wire s0_end, s1_end, s2_end, s3_end;
  wire s0_valid, s1_valid, s2_valid, s3_valid;
  wire s0_avail, s1_avail, s2_avail, s3_avail;
  wire s0_ready, s1_ready, s2_ready, s3_ready;

  // optional fifo debug
  wire ch0_full, ch1_full, ch2_full, ch3_full;
  wire ch0_empty, ch1_empty, ch2_empty, ch3_empty;
  wire [10:0] ch0_level, ch1_level, ch2_level, ch3_level;

  // -------------------------
  // 4x Dual-clock FIFO wrapper
  // -------------------------

  // CH0: Status
  tx_asyncfifo10b_axi #(
      .PAYLOAD_LEN(16'd420)
  ) ch0 (
      .wrclk  (clk_50m),
      .wrrst  (rst_audio),
      .w_data (st_tdata),
      .w_start(st_start),
      .w_end  (st_end),
      .w_valid(st_tvalid),
      .w_ready(st_tready),

      .rdclk(clk),
      .rdrst(rst),

      .s_data (s0_data),
      .s_start(s0_start),
      .s_end  (s0_end),
      .s_valid(s0_valid),
      .s_avail(s0_avail),
      .s_ready(s0_ready),

      .fifo_full (ch0_full),
      .fifo_empty(ch0_empty),
      .fifo_level(ch0_level)
  );

  // CH1: Debug
  tx_asyncfifo10b_axi #(
      .PAYLOAD_LEN(16'd345)
  ) ch1 (
      .wrclk  (clk_50m),
      .wrrst  (rst_audio),
      .w_data (dbg_tdata),
      .w_start(dbg_start),
      .w_end  (dbg_end),
      .w_valid(dbg_tvalid),
      .w_ready(dbg_tready),

      .rdclk(clk),
      .rdrst(rst),

      .s_data (s1_data),
      .s_start(s1_start),
      .s_end  (s1_end),
      .s_valid(s1_valid),
      .s_avail(s1_avail),
      .s_ready(s1_ready),

      .fifo_full (ch1_full),
      .fifo_empty(ch1_empty),
      .fifo_level(ch1_level)
  );

  // CH2: AES67 RTP
  tx_asyncfifo10b_axi #(
      .PAYLOAD_LEN(16'd300)
  ) ch2 (
      .wrclk  (clk_50m),
      .wrrst  (rst_audio),
      .w_data (rtpout_tdata),
      .w_start(rtpout_start),
      .w_end  (rtpout_end),
      .w_valid(rtpout_tvalid),
      .w_ready(rtpout_tready),

      .rdclk(clk),
      .rdrst(rst),

      .s_data (s2_data),
      .s_start(s2_start),
      .s_end  (s2_end),
      .s_valid(s2_valid),
      .s_avail(s2_avail),
      .s_ready(s2_ready),

      .fifo_full (ch2_full),
      .fifo_empty(ch2_empty),
      .fifo_level(ch2_level)
  );

  // CH3: SDP
  tx_asyncfifo10b_axi #(
      .PAYLOAD_LEN(16'd249)
  ) ch3 (
      .wrclk  (clk_50m),
      .wrrst  (rst_audio),
      .w_data (sdp_tdata),
      .w_start(sdp_start),
      .w_end  (sdp_end),
      .w_valid(sdp_tvalid),
      .w_ready(sdp_tready),

      .rdclk(clk),
      .rdrst(rst),

      .s_data (s3_data),
      .s_start(s3_start),
      .s_end  (s3_end),
      .s_valid(s3_valid),
      .s_avail(s3_avail),
      .s_ready(s3_ready),

      .fifo_full (ch3_full),
      .fifo_empty(ch3_empty),
      .fifo_level(ch3_level)
  );

  // -------------------------
  // Mux -> udp_complete UDP TX
  // -------------------------
  udp_tx_mux4_from_streams_clean #(
      .DATA_WIDTH(8),

      .S0_IP_DEST(32'hC0A80164),
      .S0_PORT_SRC(16'd5005),
      .S0_PORT_DST(16'd7800),
      .S0_PAYLOAD_LEN(16'd420),

      .S1_IP_DEST(32'hC0A80164),
      .S1_PORT_SRC(16'd5005),
      .S1_PORT_DST(16'd7801),
      .S1_PAYLOAD_LEN(16'd345),

      // AES67: ggf. wieder auf Multicast setzen
      //.S2_IP_DEST(32'hC0A80164),
      .S2_IP_DEST(32'hEF450102),
      .S2_PORT_SRC(16'd5004),
      .S2_PORT_DST(16'd5004),
      .S2_PAYLOAD_LEN(16'd300),

      //.S3_IP_DEST(32'hC0A80164),
      .S3_IP_DEST(32'hEFFFFFFF),
      .S3_PORT_SRC(16'd9875),
      .S3_PORT_DST(16'd9875),
      .S3_PAYLOAD_LEN(16'd249)
  ) u_mux (
      .clk(clk),
      .rst(rst),
      .udp_tx_busy(udp_tx_busy),

      .s0_data (s0_data),
      .s0_start(s0_start),
      .s0_end  (s0_end),
      .s0_valid(s0_valid),
      .s0_avail(s0_avail),
      .s0_ready(s0_ready),

      .s1_data (s1_data),
      .s1_start(s1_start),
      .s1_end  (s1_end),
      .s1_valid(s1_valid),
      .s1_avail(s1_avail),
      .s1_ready(s1_ready),

      .s2_data (s2_data),
      .s2_start(s2_start),
      .s2_end  (s2_end),
      .s2_valid(s2_valid),
      .s2_avail(s2_avail),
      .s2_ready(s2_ready),

      .s3_data (s3_data),
      .s3_start(s3_start),
      .s3_end  (s3_end),
      .s3_valid(s3_valid),
      .s3_avail(s3_avail),
      .s3_ready(s3_ready),

      .m_udp_hdr_valid(tx_udp_hdr_valid),
      .m_udp_hdr_ready(tx_udp_hdr_ready),
      .m_ip_dest_ip   (tx_udp_ip_dest_ip),
      .m_udp_src_port (tx_udp_source_port),
      .m_udp_dst_port (tx_udp_dest_port),
      .m_udp_length   (tx_udp_length),

      .m_axis_tdata (tx_udp_payload_axis_tdata),
      .m_axis_tvalid(tx_udp_payload_axis_tvalid),
      .m_axis_tready(tx_udp_payload_axis_tready),
      .m_axis_tlast (tx_udp_payload_axis_tlast)
  );



  wire [7:0] name_bram_addr;
  wire [7:0] name_bram_dout;

  // Set bit i for display i to 1 for 1 cycle when name/pan is new:
  wire [7:0] name_dirty_set;
  wire [7:0] pan_dirty_set, c_pan_dirty, pan_dirty_pulse, c_pan_dirty_plus, c_pan_dirty_minus;

  assign c_pan_dirty = c_pan_dirty_plus | c_pan_dirty_minus;

  wire signed [15:0] peak0_q15;
  wire signed [15:0] peak1_q15;
  wire signed [15:0] peak2_q15;
  wire signed [15:0] peak3_q15;
  wire signed [15:0] peak4_q15;
  wire signed [15:0] peak5_q15;
  wire signed [15:0] peak6_q15;
  wire signed [15:0] peak7_q15;

  wire signed [15:0] peak8_q15;
  wire signed [15:0] peak9_q15;
  wire signed [15:0] peak10_q15;
  wire signed [15:0] peak11_q15;
  wire signed [15:0] peak12_q15;
  wire signed [15:0] peak13_q15;
  wire signed [15:0] peak14_q15;
  wire signed [15:0] peak15_q15;


  wire [63:0] pan_s8_flat, cur_pan_s8_flat;
  wire [15:0] enc_ab;
  wire        enc_strobe;


  rotary8_stec12e08_debounced #(
      .DIR_INV (8'h00),
      .DEB_BITS(11)
  ) u_rotary8 (
      .clk(clk_50m),
      .rst(rst_audio),

      .enc_a({enc_b[3:0], enc_b[4], enc_b[5], enc_b[6], enc_a[7]}),
      .enc_b({enc_a[3:0], enc_a[4], enc_a[5], enc_a[6], enc_b[7]}),

      .ext_we  (1'b0),
      .ext_val0(pan0),
      .ext_val1(pan1),
      .ext_val2(pan2),
      .ext_val3(pan3),
      .ext_val4(pan4),
      .ext_val5(pan5),
      .ext_val6(pan6),
      .ext_val7(pan7),

      .val0(c_pan0),
      .val1(c_pan1),
      .val2(c_pan2),
      .val3(c_pan3),
      .val4(c_pan4),
      .val5(c_pan5),
      .val6(c_pan6),
      .val7(c_pan7),

      .step_plus (c_pan_dirty_plus),
      .step_minus(c_pan_dirty_minus)
  );

  mcp23017_dual_ctrl u_buttons (
      .clk(clk_50m),
      .rst(rst_audio),

      .i2c_scl(i2c_scl_t),
      .i2c_sda(i2c_sda_t),

      .cur_status0(mute_button),
      .cur_status1(solo_button),

      .status0(c_mute_button),
      .status1(c_solo_button),

      .set_mute(c_set_mute),
      .set_solo(c_set_solo)
  );


  //dirty_shift_pulse_debounce u_dirty_pulse (
  //  .clk           (clk_50m),
  //  .rst           (rst_audio),
  //
  //  // Raw button signal, ACTIVE-LOW: 0 = pressed
  //  .key_n (btn[0]),
  //
  //  .touch(touch)
  //
  //  //.name_dirty_set (name_dirty_set),
  //  //.pan_dirty_set (pan_dirty_set)
  //);

  assign pan_dirty_pulse = pan_dirty_set | c_pan_dirty;

  oled_mixer8_ssd1306_tca9548_axis #(
      .CLK_HZ      (50000000),
      .I2C_HZ      (400000),
      .TCA_ADDR    (7'h70),
      .SSD1306_ADDR(7'h3C),
      .METER_MS    (20)
  ) u_oled8 (
      .clk(clk_50m),
      .rst(rst_audio),

      // I2C
      .i2c_scl_i(scl_i),
      .i2c_scl_o(scl_o),
      .i2c_scl_t(scl_t),
      .i2c_sda_i(sda_i),  // SDA Pin als Input lesen
      .i2c_sda_o(sda_o),
      .i2c_sda_t(sda_t),

      //  .start        (btn[0]),

      // Dirty Set (1-Takt Pulse)
      .name_dirty_set(name_dirty_set),
      .pan_dirty_set (pan_dirty_pulse),

      // Peaks (Q1.15 signed)
      .peak0_q15(peak0_q15),
      .peak1_q15(peak1_q15),
      .peak2_q15(peak2_q15),
      .peak3_q15(peak3_q15),
      .peak4_q15(peak4_q15),
      .peak5_q15(peak5_q15),
      .peak6_q15(peak6_q15),
      .peak7_q15(peak7_q15),

      // Pan (Q1.15 signed)
      .pan0_s8(pan0),
      .pan1_s8(pan1),
      .pan2_s8(pan2),
      .pan3_s8(pan3),
      .pan4_s8(pan4),
      .pan5_s8(pan5),
      .pan6_s8(pan6),
      .pan7_s8(pan7),

      // Name BRAM
      .name_bram_addr(name_bram_addr),
      .name_bram_dout(name_bram_dout)
  );

  wire [6:0] s_axis_cmd_address;
  wire s_axis_cmd_start;
  wire s_axis_cmd_read;
  wire s_axis_cmd_write;
  wire s_axis_cmd_write_multiple;
  wire s_axis_cmd_stop;
  wire s_axis_cmd_valid;
  wire [7:0] s_axis_data_tdata;
  wire s_axis_data_tvalid;
  wire s_axis_data_tlast;
  wire m_axis_data_tready;
  wire stop_on_idle;

  // Outputs
  wire s_axis_cmd_ready;
  wire s_axis_data_tready;
  wire [7:0] m_axis_data_tdata;
  wire m_axis_data_tvalid;
  wire m_axis_data_tlast;
  wire busy_1, busy_2;
  wire bus_control;
  wire bus_active;
  wire missed_ack;
  wire start;

  reg [15:0] prescale = 16'd125;

  wire scl_o, scl_t, sda_o, sda_t;
  wire scl_i = i2c_scl;
  wire sda_i = i2c_sda;

  //assign i2c_scl = scl_t ? 1'bz : scl_o;
  //assign i2c_sda = sda_t ? 1'bz : sda_o;


  //assign scl_i = scl_pin;
  assign i2c_scl = scl_o ? 1'bz : 1'b0;
  //assign sda_i = sda_pin;
  assign i2c_sda = sda_o ? 1'bz : 1'b0;

  //reg [7:0] touch = 8'hFF;
  wire [7:0] touch;

  reg stby = 1'b1;

  assign tb_stby = stby;

  // 8*10-bit gepackt: ch0 in [9:0], ch1 in [19:10], ...
  wire [79:0] target_pos_flat = {gain7, gain6, gain5, gain4, gain3, gain2, gain1, gain0};

  assign c_gain0 = pos_raw_flat[9:0];
  assign c_gain1 = pos_raw_flat[19:10];
  assign c_gain2 = pos_raw_flat[29:20];
  assign c_gain3 = pos_raw_flat[39:30];
  assign c_gain4 = pos_raw_flat[49:40];
  assign c_gain5 = pos_raw_flat[59:50];
  assign c_gain6 = pos_raw_flat[69:60];
  assign c_gain7 = pos_raw_flat[79:70];

  // Optional: Debug outputs
  wire [79:0] pos_raw_flat;
  wire [79:0] pos_min_flat;
  wire [79:0] pos_max_flat;
  wire [ 7:0] ready;

  mixer_top_8ch u_mixerfader (
      .clk(clk_50m),
      .rst(rst_audio),

      .spi_cs  (adc_cs_n),
      .spi_sck (adc_sclk),
      .spi_mosi(adc_mosi),
      .spi_miso(adc_miso),

      .faders_en(~touch),

      .targets_flat(target_pos_flat),

      .actuals_flat(pos_raw_flat),

      .motor_up  (m_in1),
      .motor_down(m_in2),
      .motor_pwm (m_pwm)
  );

  wire mpr_scl, mpr_sda;
  wire mpr_touch_valid, mpr_init_done;
  wire mpr_missed_ack;

  mpr121_poll8_simplified #(
      .CLK_HZ  (50000000),
      .I2C_HZ  (400000),
      .I2C_ADDR(7'h5A)
  ) mpr121_inst (
      .clk(clk_50m),
      .rst(rst_audio),

      .i2c_scl(i2c_scl_f),
      .i2c_sda(i2c_sda_f),

      .touch8({touch[0], touch[1], touch[2], touch[3], touch[4], touch[5], touch[6], touch[7]}),
      .touch_valid(mpr_touch_valid),
      .init_done(mpr_init_done),

      .busy(),
      .bus_control(),
      .bus_active(),
      .missed_ack(mpr_missed_ack)
  );

endmodule
