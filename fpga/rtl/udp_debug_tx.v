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
// UDP Debug Telemetry Transmitter
// =============================================================================
// Periodically transmits status telemetry via UDP for debugging and monitoring.
//
// Transmitted Data:
// - PTP time offset and rate
// - Jitter buffer statistics
// - 16x audio level peaks
// - PTP absolute time
// - RTP read/write timestamps and sequence numbers
// - Sample counters
// - SPDIF statistics
// - Crossfade counters
// - Audio sample buffer dump
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

module udp_debug_tx (
    input wire clk,
    input wire rst,

    // ========== 48 kHz Sample Tick ==========
    // Triggers packet transmission
    input wire tick,

    // ========== Values to Transmit ==========
    input wire [95:0] ptp_time,
    input wire [31:0] rtp_seq,
    input wire [31:0] rtp_ts,
    input wire [31:0] written_samples,
    input wire [31:0] read_samples,
    input wire [ 7:0] buffer_fill,
    input wire [23:0] first_sample,
    input wire [31:0] spdif_samples,
    input wire [31:0] xfade_main_cnt,
    input wire [31:0] xfade_extra_cnt,
    input wire [15:0] xfade_dup_cnt,
    input wire [15:0] xfade_zero_cnt,

    input wire [47:0] sample_debug_out,
    input wire [ 8:0] sample_debug_level,
    input wire        sample_debug_empty,

    output reg sample_debug_rd,

    // ========== UDP TX Output (to udp_complete) ==========
    output reg  [7:0] m_tdata,
    output reg        m_tvalid,
    input  wire       m_tready,
    output reg        m_start,
    output reg        m_end
);


  localparam integer LEN = 345;

  reg [ 8:0] idx;
  reg        sending;
  reg        samplesbuf;

  // ========== Latched Values at Transmission Start ==========
  reg [95:0] ptp_time_reg;
  reg [31:0] rtp_seq_reg;
  reg [31:0] written_samples_reg;
  reg [31:0] read_samples_reg;
  reg [ 7:0] buffer_fill_reg;
  reg [23:0] first_sample_reg;

  reg [31:0] spdif_samples_reg;
  reg [31:0] xfade_main_cnt_reg;
  reg [31:0] xfade_extra_cnt_reg;
  reg [15:0] xfade_dup_cnt_reg;
  reg [15:0] xfade_zero_cnt_reg;

  reg [ 5:0] frame_idx;  // 0..47
  reg [ 2:0] byte6_idx;  // 0..5 within frame
  reg [23:0] l24, r24;

  reg samplebuf;

  // -----------------------------
  // UDP Sender FSM (correct handshake)
  // -----------------------------
  always @(posedge clk) begin
    if (rst) begin
      m_tvalid        <= 1'b0;
      m_end           <= 1'b0;
      m_start         <= 1'b0;
      m_tdata         <= 8'b0;

      sending         <= 1'b0;
      idx             <= 9'd0;

      sample_debug_rd <= 1'b0;

      frame_idx       <= 6'd0;
      byte6_idx       <= 3'd0;

      samplebuf       <= 1'b0;

    end else begin
      // Default: signals remain stable, only toggle where needed
      m_end <= 1'b0;
      m_start <= 1'b0;
      m_tvalid <= 1'b0;

      sample_debug_rd <= 1'b0;

      // ---------- Initiate packet start ----------
      if (!sending && tick) begin
        // Messwerte latched (einmalig zu Paketbeginn)
        ptp_time_reg        <= ptp_time;
        rtp_seq_reg         <= rtp_seq;
        written_samples_reg <= written_samples;
        read_samples_reg    <= read_samples;
        buffer_fill_reg     <= buffer_fill;
        first_sample_reg    <= first_sample;

        spdif_samples_reg   <= spdif_samples;
        xfade_main_cnt_reg  <= xfade_main_cnt;
        xfade_extra_cnt_reg <= xfade_extra_cnt;
        xfade_dup_cnt_reg   <= xfade_dup_cnt;
        xfade_zero_cnt_reg  <= xfade_zero_cnt;

        // Header VALID setzen und warten, bis READY kommt
        sending             <= 1'b1;
        idx                 <= 9'd0;  // Payload-Index vorbereiten
      end

      // ---------- Payload senden ----------
      if (sending) begin
        if (m_tready) begin
          // Generate data
          case (idx)
            // 0..11: ptp_time[95:0], MSB zuerst
            8'd0: begin
              m_tdata   <= ptp_time_reg[95:88];
              m_start   <= 1'b1;
              samplebuf <= 1'b0;
            end
            //8'd0:  begin m_tdata <= 8'd182; m_start <= 1'b1; end
            8'd1:  m_tdata <= ptp_time_reg[87:80];
            8'd2:  m_tdata <= ptp_time_reg[79:72];
            8'd3:  m_tdata <= ptp_time_reg[71:64];
            8'd4:  m_tdata <= ptp_time_reg[63:56];
            8'd5:  m_tdata <= ptp_time_reg[55:48];
            8'd6:  m_tdata <= ptp_time_reg[47:40];
            8'd7:  m_tdata <= ptp_time_reg[39:32];
            8'd8:  m_tdata <= ptp_time_reg[31:24];
            8'd9:  m_tdata <= ptp_time_reg[23:16];
            8'd10: m_tdata <= ptp_time_reg[15:8];
            8'd11: m_tdata <= ptp_time_reg[7:0];

            // 12..15: rtp_seq[31:0]
            8'd12: m_tdata <= rtp_seq_reg[31:24];
            8'd13: m_tdata <= rtp_seq_reg[23:16];
            8'd14: m_tdata <= rtp_seq_reg[15:8];
            8'd15: m_tdata <= rtp_seq_reg[7:0];

            // 16..19: written_samples[31:0]
            8'd16: m_tdata <= written_samples_reg[31:24];
            8'd17: m_tdata <= written_samples_reg[23:16];
            8'd18: m_tdata <= written_samples_reg[15:8];
            8'd19: m_tdata <= written_samples_reg[7:0];

            // 20..23: read_samples[31:0]
            8'd20: m_tdata <= read_samples_reg[31:24];
            8'd21: m_tdata <= read_samples_reg[23:16];
            8'd22: m_tdata <= read_samples_reg[15:8];
            8'd23: m_tdata <= read_samples_reg[7:0];

            // 24: buffer_fill[7:0]
            8'd24: m_tdata <= buffer_fill_reg;

            // 25..27: first_sample[23:0]
            8'd25: m_tdata <= first_sample_reg[23:16];
            8'd26: m_tdata <= first_sample_reg[15:8];
            8'd27: m_tdata <= first_sample_reg[7:0];

            // 20..23: read_samples[31:0]
            8'd28: m_tdata <= spdif_samples_reg[31:24];
            8'd29: m_tdata <= spdif_samples_reg[23:16];
            8'd30: m_tdata <= spdif_samples_reg[15:8];
            8'd31: m_tdata <= spdif_samples_reg[7:0];

            // 20..23: read_samples[31:0]
            8'd32: m_tdata <= xfade_main_cnt_reg[31:24];
            8'd33: m_tdata <= xfade_main_cnt_reg[23:16];
            8'd34: m_tdata <= xfade_main_cnt_reg[15:8];
            8'd35: m_tdata <= xfade_main_cnt_reg[7:0];

            // 20..23: read_samples[31:0]
            8'd36: m_tdata <= xfade_extra_cnt_reg[31:24];
            8'd37: m_tdata <= xfade_extra_cnt_reg[23:16];
            8'd38: m_tdata <= xfade_extra_cnt_reg[15:8];
            8'd39: m_tdata <= xfade_extra_cnt_reg[7:0];

            // 20..23: read_samples[31:0]
            8'd40: m_tdata <= xfade_dup_cnt_reg[15:8];
            8'd41: m_tdata <= xfade_dup_cnt_reg[7:0];

            // 20..23: read_samples[31:0]
            8'd42: m_tdata <= xfade_zero_cnt_reg[15:8];
            8'd43: m_tdata <= xfade_zero_cnt_reg[7:0];

            8'd44: m_tdata <= rtp_ts[31:24];
            8'd45: m_tdata <= rtp_ts[23:16];
            8'd46: m_tdata <= rtp_ts[15:8];

            8'd47: m_tdata <= rtp_ts[7:0];

            //8'd59: begin m_tdata <= 8'h00; m_end <= 1'b1; end
            8'd48: begin
              if (sample_debug_level >= 8'd48) begin
                sample_debug_rd <= 1'b1;
                samplebuf <= 1'b1;
                m_tdata <= 8'd1;
              end else begin
                m_tdata <= 8'd0;
              end
            end
            8'd49:  m_tdata <= 8'd128;
            8'd50: begin
              m_tdata <= sample_debug_out[23:16];
              l24 <= sample_debug_out[47:24];
              r24 <= sample_debug_out[23:0];
              frame_idx <= 6'd0;
              byte6_idx <= 3'd1;
            end
            9'd338: m_tdata <= 8'h01;
            9'd339: m_tdata <= 8'h02;
            9'd340: m_tdata <= 8'h03;
            9'd341: m_tdata <= 8'h04;
            9'd342: m_tdata <= 8'h05;
            9'd343: m_tdata <= 8'h06;
            9'd344: m_tdata <= 8'h07;
            9'd345: m_tdata <= 8'h08;
            default: begin

              if (byte6_idx == 3'd3 && !(frame_idx == 47) && samplebuf) sample_debug_rd <= 1'b1;

              if (byte6_idx == 3'd5) begin
                l24 <= sample_debug_out[47:24];
                r24 <= sample_debug_out[23:0];
                byte6_idx <= 3'd0;
              end

              case (byte6_idx)
                3'd0: m_tdata <= l24[23:16];
                3'd1: m_tdata <= l24[15:8];
                3'd2: m_tdata <= l24[7:0];
                3'd3: m_tdata <= r24[23:16];
                3'd4: m_tdata <= r24[15:8];
                default: m_tdata <= r24[7:0];
              endcase

              if (byte6_idx == 3'd5) begin
                if (frame_idx == 47) begin
                  //s_end <= 1'b1;
                end else begin
                  frame_idx <= frame_idx + 6'd1;
                end
              end else begin
                byte6_idx <= byte6_idx + 3'd1;
              end

            end
          endcase

          m_tvalid <= 1'b1;

          // Advance Index/FSM
          if (idx == LEN - 1) begin
            // Last byte just transmitted
            sending <= 1'b0;
            m_end   <= 1'b1;
            //m_tvalid <= 1'b1;  // Payload fertig
            idx     <= 9'd0;
          end else begin
            idx <= idx + 9'd1;
          end
        end
      end  // if (sending)
    end
  end

endmodule

`resetall
`default_nettype wire
