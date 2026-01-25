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

// =============================================================================
// 4-Way UDP Stream Multiplexer with Fixed Priority Arbitration
// =============================================================================
// Multiplexes 4 independent UDP payload streams onto a single UDP transmit path.
// Uses fixed priority arbitration (Stream 0 > 1 > 2 > 3).
//
// Features:
// - No cooldown state - starts next packet immediately when UDP TX is idle
// - Independent source/destination IP/port configuration per stream
// - Per-stream payload length parameters
// - AXI-Stream output interface with TLAST support
// - Waits for udp_tx_busy==0 before starting new packet
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

module udp_tx_mux4_from_streams_clean #(
    parameter DATA_WIDTH = 8,

    // Sender 0
    parameter [31:0] S0_IP_DEST     = 32'hC0A80164,
    parameter [15:0] S0_PORT_SRC    = 16'd5005,
    parameter [15:0] S0_PORT_DST    = 16'd7800,
    parameter [15:0] S0_PAYLOAD_LEN = 16'd420,

    // Sender 1
    parameter [31:0] S1_IP_DEST     = 32'hC0A80164,
    parameter [15:0] S1_PORT_SRC    = 16'd5005,
    parameter [15:0] S1_PORT_DST    = 16'd7801,
    parameter [15:0] S1_PAYLOAD_LEN = 16'd105,

    // Stream 2 (AES67 RTP)
    parameter [31:0] S2_IP_DEST     = 32'hC0A80164,
    parameter [15:0] S2_PORT_SRC    = 16'd5004,
    parameter [15:0] S2_PORT_DST    = 16'd5004,
    parameter [15:0] S2_PAYLOAD_LEN = 16'd300,

    // Sender 3 (SDP over UDP)
    parameter [31:0] S3_IP_DEST     = 32'hC0A80164,
    parameter [15:0] S3_PORT_SRC    = 16'd9875,
    parameter [15:0] S3_PORT_DST    = 16'd9875,
    parameter [15:0] S3_PAYLOAD_LEN = 16'd150
) (
    input wire clk,
    input wire rst,

    // ========== UDP TX Status Input ==========
    // Busy signal from udp_complete module
    input wire udp_tx_busy,

    // Sender 0
    input  wire [DATA_WIDTH-1:0] s0_data,
    input  wire                  s0_start,
    input  wire                  s0_end,
    input  wire                  s0_valid,
    input  wire                  s0_avail,
    output reg                   s0_ready,

    // Sender 1
    input  wire [DATA_WIDTH-1:0] s1_data,
    input  wire                  s1_start,
    input  wire                  s1_end,
    input  wire                  s1_valid,
    input  wire                  s1_avail,
    output reg                   s1_ready,

    // Sender 2
    input  wire [DATA_WIDTH-1:0] s2_data,
    input  wire                  s2_start,
    input  wire                  s2_end,
    input  wire                  s2_valid,
    input  wire                  s2_avail,
    output reg                   s2_ready,

    // Sender 3
    input  wire [DATA_WIDTH-1:0] s3_data,
    input  wire                  s3_start,
    input  wire                  s3_end,
    input  wire                  s3_valid,
    input  wire                  s3_avail,
    output reg                   s3_ready,

    // ========== UDP Header Output ==========
    output reg         m_udp_hdr_valid,
    input  wire        m_udp_hdr_ready,
    output reg  [31:0] m_ip_dest_ip,
    output reg  [15:0] m_udp_src_port,
    output reg  [15:0] m_udp_dst_port,
    output reg  [15:0] m_udp_length,     // Including 8-byte UDP header

    // AXI-Stream Payload-Ausgang
    output reg  [DATA_WIDTH-1:0] m_axis_tdata,
    output reg                   m_axis_tvalid,
    input  wire                  m_axis_tready,
    output reg                   m_axis_tlast
);

  localparam [15:0] S0_UDP_LENGTH = S0_PAYLOAD_LEN + 16'd8;
  localparam [15:0] S1_UDP_LENGTH = S1_PAYLOAD_LEN + 16'd8;
  localparam [15:0] S2_UDP_LENGTH = S2_PAYLOAD_LEN + 16'd8;
  localparam [15:0] S3_UDP_LENGTH = S3_PAYLOAD_LEN + 16'd8;

  localparam STATE_IDLE = 2'd0;
  localparam STATE_HDR = 2'd1;
  localparam STATE_PAYLOAD = 2'd2;

  reg [1:0] state_reg;
  reg [1:0] active_ch_reg;  // Active channel: 0..3

  always @(posedge clk or posedge rst) begin
    if (rst) begin
      state_reg       <= STATE_IDLE;
      active_ch_reg   <= 2'd0;

      m_udp_hdr_valid <= 1'b0;
      m_ip_dest_ip    <= 32'd0;
      m_udp_src_port  <= 16'd0;
      m_udp_dst_port  <= 16'd0;
      m_udp_length    <= 16'd0;

      m_axis_tdata    <= {DATA_WIDTH{1'b0}};
      m_axis_tvalid   <= 1'b0;
      m_axis_tlast    <= 1'b0;

      s0_ready        <= 1'b0;
      s1_ready        <= 1'b0;
      s2_ready        <= 1'b0;
      s3_ready        <= 1'b0;
    end else begin
      // Defaults
      s0_ready      <= 1'b0;
      s1_ready      <= 1'b0;
      s2_ready      <= 1'b0;
      s3_ready      <= 1'b0;
      m_axis_tvalid <= 1'b0;
      m_axis_tlast  <= 1'b0;

      case (state_reg)
        STATE_IDLE: begin
          m_udp_hdr_valid <= 1'b0;

          // Only start when UDP transmit path is idle
          if (!udp_tx_busy) begin
            // Fixed priority: 0 > 1 > 2 > 3
            if (s0_avail) begin
              active_ch_reg   <= 2'd0;
              m_ip_dest_ip    <= S0_IP_DEST;
              m_udp_src_port  <= S0_PORT_SRC;
              m_udp_dst_port  <= S0_PORT_DST;
              m_udp_length    <= S0_UDP_LENGTH;
              m_udp_hdr_valid <= 1'b1;
              state_reg       <= STATE_HDR;
            end else if (s1_avail) begin
              active_ch_reg   <= 2'd1;
              m_ip_dest_ip    <= S1_IP_DEST;
              m_udp_src_port  <= S1_PORT_SRC;
              m_udp_dst_port  <= S1_PORT_DST;
              m_udp_length    <= S1_UDP_LENGTH;
              m_udp_hdr_valid <= 1'b1;
              state_reg       <= STATE_HDR;
            end else if (s2_avail) begin
              active_ch_reg   <= 2'd2;
              m_ip_dest_ip    <= S2_IP_DEST;
              m_udp_src_port  <= S2_PORT_SRC;
              m_udp_dst_port  <= S2_PORT_DST;
              m_udp_length    <= S2_UDP_LENGTH;
              m_udp_hdr_valid <= 1'b1;
              state_reg       <= STATE_HDR;
            end else if (s3_avail) begin
              active_ch_reg   <= 2'd3;
              m_ip_dest_ip    <= S3_IP_DEST;
              m_udp_src_port  <= S3_PORT_SRC;
              m_udp_dst_port  <= S3_PORT_DST;
              m_udp_length    <= S3_UDP_LENGTH;
              m_udp_hdr_valid <= 1'b1;
              state_reg       <= STATE_HDR;
            end
          end
        end

        STATE_HDR: begin
          // Header fields remain stable until accepted
          if (m_udp_hdr_valid && m_udp_hdr_ready) begin
            m_udp_hdr_valid <= 1'b0;
            state_reg       <= STATE_PAYLOAD;
          end
        end

        STATE_PAYLOAD: begin
          case (active_ch_reg)
            2'd0: begin
              m_axis_tdata  <= s0_data;
              m_axis_tvalid <= s0_valid;
              s0_ready      <= m_axis_tready;
              m_axis_tlast  <= s0_valid && s0_end;
              if (m_axis_tready && s0_valid && s0_end) state_reg <= STATE_IDLE;
            end
            2'd1: begin
              m_axis_tdata  <= s1_data;
              m_axis_tvalid <= s1_valid;
              s1_ready      <= m_axis_tready;
              m_axis_tlast  <= s1_valid && s1_end;
              if (m_axis_tready && s1_valid && s1_end) state_reg <= STATE_IDLE;
            end
            2'd2: begin
              m_axis_tdata  <= s2_data;
              m_axis_tvalid <= s2_valid;
              s2_ready      <= m_axis_tready;
              m_axis_tlast  <= s2_valid && s2_end;
              if (m_axis_tready && s2_valid && s2_end) state_reg <= STATE_IDLE;
            end
            default: begin
              m_axis_tdata  <= s3_data;
              m_axis_tvalid <= s3_valid;
              s3_ready      <= m_axis_tready;
              m_axis_tlast  <= s3_valid && s3_end;
              if (m_axis_tready && s3_valid && s3_end) state_reg <= STATE_IDLE;
            end
          endcase
        end

        default: state_reg <= STATE_IDLE;
      endcase
    end
  end
endmodule

`default_nettype wire

