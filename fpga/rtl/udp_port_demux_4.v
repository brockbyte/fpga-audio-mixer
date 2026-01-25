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
// UDP Port Demultiplexer (4-Way)
// =============================================================================
// Demultiplexes a UDP stream (header + AXI-Stream payload) to 4 AXI-Stream
// outputs based on UDP destination port.
//
// Port Assignments:
//   PORT0 = 319   (e.g., PTP Event)
//   PORT1 = 320   (e.g., PTP General)
//   PORT2 = 5004  (e.g., AES67/RTP)
//   PORT3 = 7880  (e.g., Control)
//
// Unmatched ports are dropped.
// =============================================================================

`timescale 1ns / 1ps

`default_nettype none

module udp_port_demux_4 #(
    parameter DATA_WIDTH = 8,
    parameter PORT0 = 16'd5004,
    parameter PORT1 = 16'd319,
    parameter PORT2 = 16'd320,
    parameter PORT3 = 16'd7880
) (
    input wire clk,
    input wire rst,

    // ========== UDP Header from verilog-ethernet UDP RX ==========
    input  wire        s_udp_hdr_valid,
    output reg         s_udp_hdr_ready,
    input  wire [15:0] s_udp_src_port,
    input  wire [15:0] s_udp_dst_port,
    input  wire [15:0] s_udp_length,
    input  wire [15:0] s_udp_checksum,

    // ========== UDP Payload AXI-Stream Input ==========
    input  wire [DATA_WIDTH-1:0] s_axis_tdata,
    input  wire                  s_axis_tvalid,
    output wire                  s_axis_tready,
    input  wire                  s_axis_tlast,

    // ========== AXI-Stream Output for PORT0 (e.g., PTP Event 319) ==========
    output wire [DATA_WIDTH-1:0] m0_axis_tdata,
    output wire                  m0_axis_tvalid,
    input  wire                  m0_axis_tready,
    output wire                  m0_axis_tlast,

    // AXI-Stream output for PORT1 (e.g., port 320)
    output wire [DATA_WIDTH-1:0] m1_axis_tdata,
    output wire                  m1_axis_tvalid,
    input  wire                  m1_axis_tready,
    output wire                  m1_axis_tlast,

    // AXI-Stream output for PORT2 (e.g., port 5004)
    output wire [DATA_WIDTH-1:0] m2_axis_tdata,
    output wire                  m2_axis_tvalid,
    input  wire                  m2_axis_tready,
    output wire                  m2_axis_tlast,

    // AXI-Stream output for PORT3 (e.g., port 7880)
    output wire [DATA_WIDTH-1:0] m3_axis_tdata,
    output wire                  m3_axis_tvalid,
    input  wire                  m3_axis_tready,
    output wire                  m3_axis_tlast
);

  // ========== State Machine ==========
  localparam STATE_IDLE = 1'b0;
  localparam STATE_PAYLOAD = 1'b1;

  reg state_reg;
  reg drop_frame_reg;
  reg [1:0] port_sel_reg;

  // ========== Header State Machine ==========
  // Evaluates UDP header and selects output port
  always @(posedge clk or posedge rst) begin
    if (rst) begin
      state_reg       <= STATE_IDLE;
      s_udp_hdr_ready <= 1'b0;
      drop_frame_reg  <= 1'b0;
      port_sel_reg    <= 2'b00;
    end else begin
      case (state_reg)
        STATE_IDLE: begin
          // Wait for new UDP header
          s_udp_hdr_ready <= 1'b1;
          drop_frame_reg  <= 1'b0;

          if (s_udp_hdr_valid && s_udp_hdr_ready) begin
            // Header handshake: evaluate port
            if (s_udp_dst_port == PORT0) begin
              port_sel_reg   <= 2'd0;
              drop_frame_reg <= 1'b0;
            end else if (s_udp_dst_port == PORT1) begin
              port_sel_reg   <= 2'd2;
              drop_frame_reg <= 1'b0;
            end else if (s_udp_dst_port == PORT2) begin
              port_sel_reg   <= 2'd2;
              drop_frame_reg <= 1'b0;
            end else if (s_udp_dst_port == PORT3) begin
              port_sel_reg   <= 2'd3;
              drop_frame_reg <= 1'b0;
            end else begin
              // Unknown port → drop packet
              port_sel_reg   <= 2'd0;
              drop_frame_reg <= 1'b1;
            end

            // After header, payload begins
            state_reg       <= STATE_PAYLOAD;
            s_udp_hdr_ready <= 1'b0;
          end
        end

        STATE_PAYLOAD: begin
          // Don't accept new headers during payload
          s_udp_hdr_ready <= 1'b0;

          // Stay in this state until tlast is seen
          if (s_axis_tvalid && s_axis_tready && s_axis_tlast) begin
            state_reg <= STATE_IDLE;
          end
        end

        default: begin
          state_reg       <= STATE_IDLE;
          s_udp_hdr_ready <= 1'b0;
          drop_frame_reg  <= 1'b0;
          port_sel_reg    <= 2'b00;
        end
      endcase
    end
  end

  // ========== AXI-Stream Demux for Payload ==========
  // Payload is routed to all outputs, but only the selected port
  // gets tvalid=1. Unknown ports: drop_frame_reg=1 → no output active.

  // Data is the same for all outputs (only the one with valid=1 uses it)
  assign m0_axis_tdata = s_axis_tdata;
  assign m1_axis_tdata = s_axis_tdata;
  assign m2_axis_tdata = s_axis_tdata;
  assign m3_axis_tdata = s_axis_tdata;

  // tvalid depends on port selection and state
  assign m0_axis_tvalid = (state_reg == STATE_PAYLOAD) && !drop_frame_reg &&
    (port_sel_reg == 2'd0) && s_axis_tvalid;
  assign m1_axis_tvalid = (state_reg == STATE_PAYLOAD) && !drop_frame_reg &&
    (port_sel_reg == 2'd1) && s_axis_tvalid;
  assign m2_axis_tvalid = (state_reg == STATE_PAYLOAD) && !drop_frame_reg &&
    (port_sel_reg == 2'd2) && s_axis_tvalid;
  assign m3_axis_tvalid = (state_reg == STATE_PAYLOAD) && !drop_frame_reg &&
    (port_sel_reg == 2'd3) && s_axis_tvalid;

  // tlast only on active port
  assign m0_axis_tlast = m0_axis_tvalid && s_axis_tlast;
  assign m1_axis_tlast = m1_axis_tvalid && s_axis_tlast;
  assign m2_axis_tlast = m2_axis_tvalid && s_axis_tlast;
  assign m3_axis_tlast = m3_axis_tvalid && s_axis_tlast;

  // Backpressure: only the selected port can apply backpressure
  wire sel_m0_ready = (port_sel_reg == 2'd0) ? m0_axis_tready : 1'b0;
  wire sel_m1_ready = (port_sel_reg == 2'd1) ? m1_axis_tready : 1'b0;
  wire sel_m2_ready = (port_sel_reg == 2'd2) ? m2_axis_tready : 1'b0;
  wire sel_m3_ready = (port_sel_reg == 2'd3) ? m3_axis_tready : 1'b0;

  assign s_axis_tready = (state_reg != STATE_PAYLOAD) ? 1'b0 :  // Payload not yet expected
      (drop_frame_reg) ? 1'b1 :  // Dropping packet → always ready
      (sel_m0_ready | sel_m1_ready |
    sel_m2_ready | sel_m3_ready);                        // Selected output determines ready

endmodule
