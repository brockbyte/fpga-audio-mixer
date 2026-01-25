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

/*
 * AES67 RTP Receiver with FIFO Interface
 *
 * Features:
 * - Receives RTP-encapsulated audio from async FIFO
 * - Parses 12-byte RTP header (sequence, timestamp)
 * - Extracts 8 channels of interleaved 24-bit audio (Big-Endian)
 * - Supports variable packet sizes (typically 48 samples per packet)
 * - Outputs per-frame sample-valid pulse for 8-channel groups
 * - Generates 48kHz tick on first sample of each packet
 *
 * FIFO Interface:
 * - Input: 10-bit words {start_flag, end_flag, data[7:0]}
 * - Reads until end_flag encountered
 * - Automatic frame synchronization
 *
 * Audio Format:
 * - 8 channels interleaved: ch0, ch1, ... ch7
 * - 3 bytes per sample (24-bit, Big-Endian)
 * - Output: 16 channel ports (ch8-ch15 unused, set to 0)
 */
`timescale 1ns / 1ps
`default_nettype none

module aes67_rtp_rx_fifo (
    input wire clk,
    input wire rst,

    // FIFO interface (read side from async_fifo_v2)
    input  wire [7:0] fifo_dout,   // Data byte
    input  wire       fifo_start,  // Start of packet flag
    input  wire       fifo_end,    // End of packet flag
    input  wire       fifo_valid,  // Data valid
    output reg        fifo_ready,  // Ready to accept data

    // PTP time (optional, not currently used)
    input wire [95:0] ptp_time,

    // RTP header information
    output reg [31:0] rtp_ts,   // RTP timestamp
    output reg [15:0] rtp_seq,  // RTP sequence number
    output reg [31:0] rx_ts,    // Receive timestamp

    // 48kHz tick (asserted on first sample frame of each packet)
    output reg recv_tick,

    // Audio outputs: 16 channels L24 (8 used, 8 set to 0)
    output reg signed [23:0] ch0,
    output reg signed [23:0] ch1,
    output reg signed [23:0] ch2,
    output reg signed [23:0] ch3,
    output reg signed [23:0] ch4,
    output reg signed [23:0] ch5,
    output reg signed [23:0] ch6,
    output reg signed [23:0] ch7,
    output reg signed [23:0] ch8,
    output reg signed [23:0] ch9,
    output reg signed [23:0] ch10,
    output reg signed [23:0] ch11,
    output reg signed [23:0] ch12,
    output reg signed [23:0] ch13,
    output reg signed [23:0] ch14,
    output reg signed [23:0] ch15,

    output reg samples_valid  // 1-cycle: new 8-channel sample group valid
);

  // ================================================================
  // FSM states
  // ================================================================
  localparam ST_IDLE = 2'd0;
  localparam ST_HEADER = 2'd1;
  localparam ST_PAYLD = 2'd2;

  reg  [ 1:0] state;

  // ================================================================
  // Counters and shift registers
  // ================================================================
  // RTP header byte counter (0..11 for 12 bytes)
  reg  [ 3:0] hdr_cnt;

  // Payload counters
  reg  [ 1:0] sample_byte_cnt;  // 0, 1, 2 for L24 bytes
  reg  [ 2:0] ch_idx;  // 0..7 for channel index
  reg  [ 5:0] smpl_idx;  // Sample index within packet

  // 24-bit sample shift register
  reg  [23:0] smpl_shift;

  // Pipeline flag for 3-byte alignment
  reg         notfirst;

  // FIFO data byte
  wire [ 7:0] byte_in = fifo_dout[7:0];

  // ================================================================
  // Main FSM: parse RTP header and extract audio samples
  // ================================================================
  always @(posedge clk) begin
    if (rst) begin
      state           <= ST_IDLE;

      fifo_ready      <= 1'b0;

      hdr_cnt         <= 4'd0;
      sample_byte_cnt <= 2'd0;
      ch_idx          <= 3'd0;
      smpl_shift      <= 24'd0;
      smpl_idx        <= 6'd0;
      notfirst        <= 1'b0;

      rtp_seq         <= 16'd0;
      rtp_ts          <= 32'd0;
      rx_ts           <= 32'd0;

      recv_tick       <= 1'b0;
      samples_valid   <= 1'b0;

      ch0             <= 24'sd0;
      ch1             <= 24'sd0;
      ch2             <= 24'sd0;
      ch3             <= 24'sd0;
      ch4             <= 24'sd0;
      ch5             <= 24'sd0;
      ch6             <= 24'sd0;
      ch7             <= 24'sd0;
      ch8             <= 24'sd0;
      ch9             <= 24'sd0;
      ch10            <= 24'sd0;
      ch11            <= 24'sd0;
      ch12            <= 24'sd0;
      ch13            <= 24'sd0;
      ch14            <= 24'sd0;
      ch15            <= 24'sd0;

    end else begin
      // Default values
      fifo_ready    <= 1'b1;
      samples_valid <= 1'b0;
      recv_tick     <= 1'b0;
      rx_ts         <= rtp_ts + smpl_idx;

      case (state)
        // --------------------------------------------------------
        // IDLE: Wait for packet start flag
        // --------------------------------------------------------
        ST_IDLE: begin
          // If incomplete frame was received, finalize last sample frame
          if (smpl_idx > 0) begin
            ch7           <= smpl_shift;
            samples_valid <= 1'b1;
          end

          // Reset counters and flags
          hdr_cnt         <= 4'd0;
          sample_byte_cnt <= 2'd0;
          ch_idx          <= 3'd0;
          smpl_idx        <= 6'd0;
          smpl_shift      <= 24'd0;
          notfirst        <= 1'b0;

          // Read bytes until start flag seen
          if (fifo_valid) begin
            if (fifo_start) begin
              // This byte is header byte 0 (not parsed)
              hdr_cnt <= 4'd1;
              state   <= ST_HEADER;
              // Handle unexpected early packet end
              if (fifo_end) begin
                state <= ST_IDLE;
              end
            end
            // Bytes without start_flag are ignored
          end
        end

        // --------------------------------------------------------
        // HEADER: Read 12 RTP header bytes
        // Byte 0 already consumed in IDLE state (hdr_cnt=1)
        // --------------------------------------------------------
        ST_HEADER: begin
          if (fifo_valid) begin
            // Extract relevant header fields
            case (hdr_cnt)
              // Byte 2-3: Sequence Number
              4'd2: rtp_seq[15:8]  <= byte_in;
              4'd3: rtp_seq[7:0]   <= byte_in;
              // Byte 4-7: Timestamp
              4'd4: rtp_ts[31:24]  <= byte_in;
              4'd5: rtp_ts[23:16]  <= byte_in;
              4'd6: rtp_ts[15:8]   <= byte_in;
              4'd7: rtp_ts[7:0]    <= byte_in;
              default: ;
            endcase

            if (hdr_cnt == 4'd11) begin
              // Read all 12 header bytes (0..11)
              hdr_cnt         <= 4'd0;
              sample_byte_cnt <= 2'd0;
              ch_idx          <= 3'd0;
              smpl_shift      <= 24'd0;
              notfirst        <= 1'b0;
              state           <= ST_PAYLD;
            end else begin
              hdr_cnt <= hdr_cnt + 4'd1;
            end

            // Handle early packet end
            if (fifo_end) begin
              state <= ST_IDLE;
            end
          end
        end

        // --------------------------------------------------------
        // PAYLD: Extract L24 samples (8 channels interleaved)
        // --------------------------------------------------------
        ST_PAYLD: begin
          if (fifo_valid) begin
            // 3 bytes per sample (Big-Endian): {B0, B1, B2}
            smpl_shift      <= {smpl_shift[15:0], byte_in};
            sample_byte_cnt <= sample_byte_cnt + 2'd1;

            // Complete 3-byte sample (when counter wraps to 0 with notfirst set)
            if (sample_byte_cnt == 2'd0 && notfirst) begin
              case (ch_idx)
                3'd0:    ch0 <= smpl_shift;
                3'd1:    ch1 <= smpl_shift;
                3'd2:    ch2 <= smpl_shift;
                3'd3:    ch3 <= smpl_shift;
                3'd4:    ch4 <= smpl_shift;
                3'd5:    ch5 <= smpl_shift;
                3'd6:    ch6 <= smpl_shift;
                3'd7: begin
                  ch7           <= smpl_shift;
                  samples_valid <= 1'b1;
                  if (smpl_idx == 6'd0) begin
                    // First sample frame of packet: generate 48kHz tick
                    recv_tick <= 1'b1;
                  end
                  smpl_idx <= smpl_idx + 6'd1;
                end
                default: ;
              endcase

              // Set unused upper 8 channels to 0
              ch8  <= 24'sd0;
              ch9  <= 24'sd0;
              ch10 <= 24'sd0;
              ch11 <= 24'sd0;
              ch12 <= 24'sd0;
              ch13 <= 24'sd0;
              ch14 <= 24'sd0;
              ch15 <= 24'sd0;

              // Advance channel index within frame
              if (ch_idx == 3'd7) begin
                ch_idx <= 3'd0;
              end else begin
                ch_idx <= ch_idx + 3'd1;
              end
            end

            // Wrap byte counter after 3 bytes
            if (sample_byte_cnt == 2'd2) begin
              sample_byte_cnt <= 2'd0;
            end

            notfirst <= 1'b1;

            // End of packet: return to IDLE
            if (fifo_end) begin
              state <= ST_IDLE;
            end
          end
        end

        default: begin
          state <= ST_IDLE;
        end
      endcase
    end
  end

endmodule

`default_nettype wire
