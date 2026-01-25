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
// AES67 RTP Transmitter with 48kHz Stereo Audio from FIFO
// =============================================================================
// Constructs and transmits RTP packets containing 48 stereo audio frames
// (96 samples total) in L24 format. Each packet is triggered by a 1ms tick.
//
// Features:
// - RTP header generation with sequence number and timestamp
// - 48-bit FIFO interface: {L[23:0], R[23:0]} per stereo frame
// - Packet transmission only starts when FIFO has sufficient data (alm_full)
// - 48 frames per packet (1ms @ 48kHz), 300 bytes payload (12+288)
// - Automatic RTP sequence and timestamp increment
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

module aes67_tx_stream2_from_fifo48_lr #(
    parameter [31:0] SSRC = 32'h12345678,
    parameter [7:0] RTP_PT = 8'd96,
    parameter integer PENDING_W = 3  // queued packets (up to 7)
) (
    input wire clk,
    input wire rst,

    // 1ms tick -> queue one RTP packet
    input wire tick_1ms,

    // ========== Sample FIFO Interface ==========
    // 48-bit stereo frame: {L[23:0], R[23:0]}
    // Requires 48 frames per RTP packet
    input  wire [47:0] samp_fifo_dout,
    input  wire        samp_fifo_valid,
    output reg         samp_fifo_ready,
    input  wire [ 7:0] samp_fifo_level,
    input  wire        samp_fifo_alm_full,

    // ========== RTP Sequence/Timestamp Control ==========
    // External override for next packet header values
    input wire        seq_set,
    input wire [15:0] seq_in,
    input wire        ts_set,
    input wire [31:0] ts_in,

    // ========== UDP Payload Stream Output ==========
    // Standard start/end/valid/ready protocol
    output reg  [7:0] s_data,
    output reg        s_start,
    output reg        s_end,
    output reg        s_valid,
    output reg        s_avail,
    input  wire       s_ready,

    // debug (optional)
    output reg [15:0] rtp_seq_cur,
    output reg [31:0] rtp_ts_cur
);

  localparam integer FRAMES_PER_PKT = 48;
  localparam integer RTP_HDR_BYTES = 12;
  localparam [7:0] START_MIN_LEVEL = 8'd6;

  reg [PENDING_W-1:0] pending_cnt;

  reg [15:0] pkt_seq;
  reg [31:0] pkt_ts;

  reg [5:0]  frame_idx;     // Frame counter: 0..47
  reg [2:0]  byte6_idx;     // Byte within frame: 0..5 (3 bytes L + 3 bytes R)
  reg [23:0] l24, r24;  // Stereo sample registers

  reg  started_this_pkt;

  wire out_fire = s_valid;

  // ========== RTP Header Byte Generator ==========
  // Returns byte at index (0..11) for standard RTP v2 header
  function [7:0] rtp_hdr_byte;
    input [3:0] idx;
    input [15:0] seq;
    input [31:0] ts;
    input [31:0] ssrc;
    input [7:0] pt;
    reg [7:0] b;
    begin
      case (idx)
        4'd0:    b = 8'h80;  // V=2, P=0, X=0, CC=0
        4'd1:    b = {1'b0, pt[6:0]};  // M=0, PT
        4'd2:    b = seq[15:8];  // Sequence MSB
        4'd3:    b = seq[7:0];  // Sequence LSB
        4'd4:    b = ts[31:24];  // Timestamp byte 0
        4'd5:    b = ts[23:16];  // Timestamp byte 1
        4'd6:    b = ts[15:8];  // Timestamp byte 2
        4'd7:    b = ts[7:0];  // Timestamp byte 3
        4'd8:    b = ssrc[31:24];  // SSRC byte 0
        4'd9:    b = ssrc[23:16];  // SSRC byte 1
        4'd10:   b = ssrc[15:8];  // SSRC byte 2
        4'd11:   b = ssrc[7:0];  // SSRC byte 3
        default: b = 8'h00;
      endcase
      rtp_hdr_byte = b;
    end
  endfunction

  // ========== State Machine States ==========
  localparam [2:0] ST_IDLE = 3'd0,  // Wait for start condition
  ST_HDR = 3'd1,  // Send RTP header (12 bytes)
  ST_POP = 3'd2,  // (Unused)
  ST_FRAME = 3'd3,  // Send audio payload (48 frames)
  ST_DONE = 3'd4;  // Packet complete

  reg [2:0] st;
  reg [3:0] hdr_idx;

  // Start condition: only begin packet when FIFO has sufficient data
  wire can_start_pkt = samp_fifo_alm_full;

  reg [23:0] l24_next, r24_next;

  always @* begin
    l24_next = l24;
    r24_next = r24;
    if ((byte6_idx == 3'd5) || (hdr_idx == RTP_HDR_BYTES - 1)) begin
      l24_next = samp_fifo_dout[47:24];
      r24_next = samp_fifo_dout[23:0];
    end
  end


  always @(posedge clk) begin
    if (rst) begin
      pending_cnt      <= {PENDING_W{1'b0}};
      s_avail          <= 1'b0;

      rtp_seq_cur      <= 16'd0;
      rtp_ts_cur       <= 32'd0;
      pkt_seq          <= 16'd0;
      pkt_ts           <= 32'd0;

      st               <= ST_IDLE;
      hdr_idx          <= 4'd0;

      frame_idx        <= 6'd0;
      byte6_idx        <= 3'd0;

      l24              <= 24'd0;
      r24              <= 24'd0;

      s_data           <= 8'd0;
      s_valid          <= 1'b0;
      s_start          <= 1'b0;
      s_end            <= 1'b0;
      started_this_pkt <= 1'b0;

      samp_fifo_ready  <= 1'b0;
    end else begin
      // defaults
      s_start <= 1'b0;
      s_end <= 1'b0;
      samp_fifo_ready <= 1'b0;

      // external set of next header values
      if (seq_set) rtp_seq_cur <= seq_in;
      if (ts_set) rtp_ts_cur <= ts_in;

      s_avail <= can_start_pkt;

      case (st)
        ST_IDLE: begin
          s_valid <= 1'b0;
          started_this_pkt <= 1'b0;

          // NEW: only start if fifo_level >= 6
          if (can_start_pkt && s_ready) begin
            pkt_seq <= rtp_seq_cur;
            pkt_ts <= rtp_ts_cur;

            rtp_seq_cur <= rtp_seq_cur + 16'd1;
            rtp_ts_cur <= rtp_ts_cur + 32'd48;

            hdr_idx <= 4'd0;
            frame_idx <= 6'd0;
            byte6_idx <= 3'd0;

            st <= ST_HDR;
          end
        end

        ST_HDR: begin
          //if (s_ready) begin
          s_valid <= 1'b1;
          s_data  <= rtp_hdr_byte(hdr_idx, pkt_seq, pkt_ts, SSRC, RTP_PT);

          if (!started_this_pkt) begin
            s_start <= 1'b1;
            started_this_pkt <= 1'b1;
          end

          if (hdr_idx == RTP_HDR_BYTES - 4) samp_fifo_ready <= 1'b1;

          if (hdr_idx == RTP_HDR_BYTES - 1) begin
            l24 <= l24_next;
            r24 <= r24_next;
            byte6_idx <= 3'd0;
            st <= ST_FRAME;
          end else hdr_idx <= hdr_idx + 4'd1;
          //end
        end

        ST_FRAME: begin
          //if (s_ready) begin

          s_valid <= 1'b1;

          if (byte6_idx == 3'd2) samp_fifo_ready <= 1'b1;

          if (byte6_idx == 3'd5) begin
            l24 <= l24_next;
            r24 <= r24_next;
            byte6_idx <= 3'd0;
          end

          case (byte6_idx)
            3'd0: s_data <= l24[23:16];
            3'd1: s_data <= l24[15:8];
            3'd2: s_data <= l24[7:0];
            3'd3: s_data <= r24[23:16];
            3'd4: s_data <= r24[15:8];
            default: s_data <= r24[7:0];
          endcase


          if (byte6_idx == 3'd5) begin
            if (frame_idx == FRAMES_PER_PKT - 1) begin
              s_end <= 1'b1;
              st <= ST_DONE;
            end else begin
              frame_idx <= frame_idx + 6'd1;
              st <= ST_FRAME;
            end
          end else begin
            byte6_idx <= byte6_idx + 3'd1;
          end
          //end
        end

        ST_DONE: begin
          s_valid <= 1'b0;
          started_this_pkt <= 1'b0;
          st <= ST_IDLE;
        end

        default: st <= ST_IDLE;
      endcase
    end
  end

endmodule

`default_nettype wire
