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
// UDP Status Transmitter with Byte RAM Buffer
// =============================================================================
// Captures mixer status (peaks, jitter buffer, gains, pan, mutes) into a
// byte RAM, then streams as UDP payload. Two FSMs: capture and transmit.
//
// Features:
// - Periodically captures snapshot of all status values
// - Peak detection and hold with configurable window
// - Stores entire status message in BRAM (efficient read)
// - Streams status via AXI-Stream interface
// - Supports 16 audio channels with individual peaks and control values
// - Includes PTP time, RTP timestamps, and debug counters
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

module udp_status_tx_bram #(
    parameter integer INTERVAL_MS = 50
) (
    input wire clk,
    input wire rst,

    // 48 kHz Tick
    input wire sample_tick,
    input wire tick_48k,

    // Jitter-Buffer-Stats
    input wire [31:0] stat_underrun,
    input wire [31:0] stat_overwrite,
    input wire [31:0] stat_late,
    input wire [31:0] stat_early,

    // Audio for peak
    input wire signed [23:0] ch0,
    input wire signed [23:0] ch1,
    input wire signed [23:0] ch2,
    input wire signed [23:0] ch3,
    input wire signed [23:0] ch4,
    input wire signed [23:0] ch5,
    input wire signed [23:0] ch6,
    input wire signed [23:0] ch7,
    input wire signed [23:0] ch8,
    input wire signed [23:0] ch9,
    input wire signed [23:0] ch10,
    input wire signed [23:0] ch11,
    input wire signed [23:0] ch12,
    input wire signed [23:0] ch13,
    input wire signed [23:0] ch14,
    input wire signed [23:0] ch15,

    // Additional debug values
    input wire [63:0] ptp_time_sec,
    input wire [31:0] ptp_time_ns,
    input wire [ 7:0] buffer_fill,
    input wire [31:0] spdif_samples,
    input wire [31:0] xfade_main_cnt,
    input wire [31:0] xfade_extra_cnt,
    input wire [15:0] xfade_dup_cnt,
    input wire [15:0] xfade_zero_cnt,

    input wire [31:0] rtp_read_ts,
    input wire [31:0] rtp_write_ts,
    input wire [31:0] samples_read,
    input wire [31:0] samples_written,

    // Neu: Gain/Pan/Mute/Solo aus Control-Registern
    input wire [9:0] gain_u10_ch0,
    input wire [9:0] gain_u10_ch1,
    input wire [9:0] gain_u10_ch2,
    input wire [9:0] gain_u10_ch3,
    input wire [9:0] gain_u10_ch4,
    input wire [9:0] gain_u10_ch5,
    input wire [9:0] gain_u10_ch6,
    input wire [9:0] gain_u10_ch7,
    input wire [9:0] gain_u10_ch8,
    input wire [9:0] gain_u10_ch9,
    input wire [9:0] gain_u10_ch10,
    input wire [9:0] gain_u10_ch11,
    input wire [9:0] gain_u10_ch12,
    input wire [9:0] gain_u10_ch13,
    input wire [9:0] gain_u10_ch14,
    input wire [9:0] gain_u10_ch15,

    input wire signed [7:0] pan_s8_ch0,
    input wire signed [7:0] pan_s8_ch1,
    input wire signed [7:0] pan_s8_ch2,
    input wire signed [7:0] pan_s8_ch3,
    input wire signed [7:0] pan_s8_ch4,
    input wire signed [7:0] pan_s8_ch5,
    input wire signed [7:0] pan_s8_ch6,
    input wire signed [7:0] pan_s8_ch7,
    input wire signed [7:0] pan_s8_ch8,
    input wire signed [7:0] pan_s8_ch9,
    input wire signed [7:0] pan_s8_ch10,
    input wire signed [7:0] pan_s8_ch11,
    input wire signed [7:0] pan_s8_ch12,
    input wire signed [7:0] pan_s8_ch13,
    input wire signed [7:0] pan_s8_ch14,
    input wire signed [7:0] pan_s8_ch15,

    input wire [15:0] mute_mask,
    input wire [15:0] solo_mask,

    // Peak Outputs (live)
    output wire [15:0] peak_ch0,
    output wire [15:0] peak_ch1,
    output wire [15:0] peak_ch2,
    output wire [15:0] peak_ch3,
    output wire [15:0] peak_ch4,
    output wire [15:0] peak_ch5,
    output wire [15:0] peak_ch6,
    output wire [15:0] peak_ch7,
    output wire [15:0] peak_ch8,
    output wire [15:0] peak_ch9,
    output wire [15:0] peak_ch10,
    output wire [15:0] peak_ch11,
    output wire [15:0] peak_ch12,
    output wire [15:0] peak_ch13,
    output wire [15:0] peak_ch14,
    output wire [15:0] peak_ch15,

    // Name BRAM read-port (sync read, 1-cycle latency)
    output reg  [7:0] name_rd_addr,  // 0..255
    input  wire [7:0] name_rd_data,

    // Byte-Stream Output
    output reg  [7:0] m_tdata,
    output reg        m_tvalid,
    input  wire       m_tready,
    output reg        m_start,
    output reg        m_end
);

  function [7:0] peak_hi;
    input [3:0] ch;
    reg [15:0] w;
    begin
      w = sel_peak(ch);
      peak_hi = w[15:8];
    end
  endfunction

  function [7:0] peak_lo;
    input [3:0] ch;
    reg [15:0] w;
    begin
      w = sel_peak(ch);
      peak_lo = w[7:0];
    end
  endfunction

  function [7:0] gain_hi;
    input [3:0] ch;
    reg [15:0] w;
    begin
      w = sel_gain_u16(ch);
      gain_hi = w[15:8];
    end
  endfunction

  function [7:0] gain_lo;
    input [3:0] ch;
    reg [15:0] w;
    begin
      w = sel_gain_u16(ch);
      gain_lo = w[7:0];
    end
  endfunction


  // -----------------------------
  // Peak-Hold window
  // -----------------------------
  localparam integer SPM = (48000 * INTERVAL_MS) / 1000;  // Samples pro Meldung

  function [23:0] abs24;
    input signed [23:0] x;
    begin
      abs24 = x[23] ? (~x + 24'd1) : x;
    end
  endfunction

  reg [15:0]
      peak0,
      peak1,
      peak2,
      peak3,
      peak4,
      peak5,
      peak6,
      peak7,
      peak8,
      peak9,
      peak10,
      peak11,
      peak12,
      peak13,
      peak14,
      peak15;
  reg [15:0]
      cur0,
      cur1,
      cur2,
      cur3,
      cur4,
      cur5,
      cur6,
      cur7,
      cur8,
      cur9,
      cur10,
      cur11,
      cur12,
      cur13,
      cur14,
      cur15;

  reg  [15:0] smp_cnt;
  reg         tick_accum;  // 1-cycle pulse when window complete
  reg         peak_freeze_r;  // nur Capture-FSM treibt das
  reg         peak_reset_req;  // 1-cycle pulse, only Capture FSM drives this
  wire        peak_freeze = peak_freeze_r;

  assign peak_ch0  = peak0;
  assign peak_ch1  = peak1;
  assign peak_ch2  = peak2;
  assign peak_ch3  = peak3;
  assign peak_ch4  = peak4;
  assign peak_ch5  = peak5;
  assign peak_ch6  = peak6;
  assign peak_ch7  = peak7;
  assign peak_ch8  = peak8;
  assign peak_ch9  = peak9;
  assign peak_ch10 = peak10;
  assign peak_ch11 = peak11;
  assign peak_ch12 = peak12;
  assign peak_ch13 = peak13;
  assign peak_ch14 = peak14;
  assign peak_ch15 = peak15;

  task reset_peaks;
    begin
      peak0  <= 0;
      peak1  <= 0;
      peak2  <= 0;
      peak3  <= 0;
      peak4  <= 0;
      peak5  <= 0;
      peak6  <= 0;
      peak7  <= 0;
      peak8  <= 0;
      peak9  <= 0;
      peak10 <= 0;
      peak11 <= 0;
      peak12 <= 0;
      peak13 <= 0;
      peak14 <= 0;
      peak15 <= 0;
    end
  endtask

  always @(posedge clk) begin
    if (rst) begin
      reset_peaks();
      cur0 <= 0;
      cur1 <= 0;
      cur2 <= 0;
      cur3 <= 0;
      cur4 <= 0;
      cur5 <= 0;
      cur6 <= 0;
      cur7 <= 0;
      cur8 <= 0;
      cur9 <= 0;
      cur10 <= 0;
      cur11 <= 0;
      cur12 <= 0;
      cur13 <= 0;
      cur14 <= 0;
      cur15 <= 0;
      smp_cnt <= 16'd0;
      tick_accum <= 1'b0;
      //peak_freeze<=1'b0;
    end else begin
      tick_accum <= 1'b0;

      // Peak-Reset auf Anfrage (vom Capture-FSM)
      if (peak_reset_req) begin
        reset_peaks();
      end


      if (sample_tick) begin
        // Absolutwerte -> 16 Bit
        cur0  <= abs24(ch0) >> 8;
        cur1  <= abs24(ch1) >> 8;
        cur2  <= abs24(ch2) >> 8;
        cur3  <= abs24(ch3) >> 8;
        cur4  <= abs24(ch4) >> 8;
        cur5  <= abs24(ch5) >> 8;
        cur6  <= abs24(ch6) >> 8;
        cur7  <= abs24(ch7) >> 8;
        cur8  <= abs24(ch8) >> 8;
        cur9  <= abs24(ch9) >> 8;
        cur10 <= abs24(ch10) >> 8;
        cur11 <= abs24(ch11) >> 8;
        cur12 <= abs24(ch12) >> 8;
        cur13 <= abs24(ch13) >> 8;
        cur14 <= abs24(ch14) >> 8;
        cur15 <= abs24(ch15) >> 8;

        // Peaks nur wenn nicht eingefroren
        if (!peak_freeze) begin
          if (cur0 > peak0) peak0 <= cur0;
          if (cur1 > peak1) peak1 <= cur1;
          if (cur2 > peak2) peak2 <= cur2;
          if (cur3 > peak3) peak3 <= cur3;
          if (cur4 > peak4) peak4 <= cur4;
          if (cur5 > peak5) peak5 <= cur5;
          if (cur6 > peak6) peak6 <= cur6;
          if (cur7 > peak7) peak7 <= cur7;
          if (cur8 > peak8) peak8 <= cur8;
          if (cur9 > peak9) peak9 <= cur9;
          if (cur10 > peak10) peak10 <= cur10;
          if (cur11 > peak11) peak11 <= cur11;
          if (cur12 > peak12) peak12 <= cur12;
          if (cur13 > peak13) peak13 <= cur13;
          if (cur14 > peak14) peak14 <= cur14;
          if (cur15 > peak15) peak15 <= cur15;
        end
      end

      if (tick_48k) begin
        if (smp_cnt == (SPM - 1)) begin
          smp_cnt    <= 16'd0;
          tick_accum <= 1'b1;
        end else begin
          smp_cnt <= smp_cnt + 16'd1;
        end
      end
    end
  end

  // -----------------------------
  // Payload length
  // -----------------------------
  localparam integer LEN = 420;  // wie bei dir

  // -----------------------------
  // Byte-BRAM: status_mem (inferred)
  // -----------------------------
  // depth: >= LEN
  reg [7:0] status_mem[0:511];

  reg [8:0] mem_waddr;
  reg [7:0] mem_wdata;
  reg       mem_we;

  reg [8:0] mem_raddr;
  reg [7:0] mem_rdata;

  // sync read
  always @(posedge clk) begin
    if (mem_we) begin
      status_mem[mem_waddr] <= mem_wdata;
    end
    mem_rdata <= status_mem[mem_raddr];
  end

  // -----------------------------
  // Helpers: select peaks/gain/pan per channel index
  // -----------------------------
  function [15:0] sel_peak;
    input [3:0] ch;
    begin
      case (ch)
        4'd0: sel_peak = peak0;
        4'd1: sel_peak = peak1;
        4'd2: sel_peak = peak2;
        4'd3: sel_peak = peak3;
        4'd4: sel_peak = peak4;
        4'd5: sel_peak = peak5;
        4'd6: sel_peak = peak6;
        4'd7: sel_peak = peak7;
        4'd8: sel_peak = peak8;
        4'd9: sel_peak = peak9;
        4'd10: sel_peak = peak10;
        4'd11: sel_peak = peak11;
        4'd12: sel_peak = peak12;
        4'd13: sel_peak = peak13;
        4'd14: sel_peak = peak14;
        4'd15: sel_peak = peak15;
        default: sel_peak = 16'd0;
      endcase
    end
  endfunction

  function [15:0] sel_gain_u16;
    input [3:0] ch;
    begin
      case (ch)
        4'd0: sel_gain_u16 = {6'd0, gain_u10_ch0};
        4'd1: sel_gain_u16 = {6'd0, gain_u10_ch1};
        4'd2: sel_gain_u16 = {6'd0, gain_u10_ch2};
        4'd3: sel_gain_u16 = {6'd0, gain_u10_ch3};
        4'd4: sel_gain_u16 = {6'd0, gain_u10_ch4};
        4'd5: sel_gain_u16 = {6'd0, gain_u10_ch5};
        4'd6: sel_gain_u16 = {6'd0, gain_u10_ch6};
        4'd7: sel_gain_u16 = {6'd0, gain_u10_ch7};
        4'd8: sel_gain_u16 = {6'd0, gain_u10_ch8};
        4'd9: sel_gain_u16 = {6'd0, gain_u10_ch9};
        4'd10: sel_gain_u16 = {6'd0, gain_u10_ch10};
        4'd11: sel_gain_u16 = {6'd0, gain_u10_ch11};
        4'd12: sel_gain_u16 = {6'd0, gain_u10_ch12};
        4'd13: sel_gain_u16 = {6'd0, gain_u10_ch13};
        4'd14: sel_gain_u16 = {6'd0, gain_u10_ch14};
        4'd15: sel_gain_u16 = {6'd0, gain_u10_ch15};
        default: sel_gain_u16 = 16'd0;
      endcase
    end
  endfunction

  function [7:0] sel_pan_u8;
    input [3:0] ch;
    begin
      case (ch)
        4'd0: sel_pan_u8 = pan_s8_ch0;
        4'd1: sel_pan_u8 = pan_s8_ch1;
        4'd2: sel_pan_u8 = pan_s8_ch2;
        4'd3: sel_pan_u8 = pan_s8_ch3;
        4'd4: sel_pan_u8 = pan_s8_ch4;
        4'd5: sel_pan_u8 = pan_s8_ch5;
        4'd6: sel_pan_u8 = pan_s8_ch6;
        4'd7: sel_pan_u8 = pan_s8_ch7;
        4'd8: sel_pan_u8 = pan_s8_ch8;
        4'd9: sel_pan_u8 = pan_s8_ch9;
        4'd10: sel_pan_u8 = pan_s8_ch10;
        4'd11: sel_pan_u8 = pan_s8_ch11;
        4'd12: sel_pan_u8 = pan_s8_ch12;
        4'd13: sel_pan_u8 = pan_s8_ch13;
        4'd14: sel_pan_u8 = pan_s8_ch14;
        4'd15: sel_pan_u8 = pan_s8_ch15;
        default: sel_pan_u8 = 8'd0;
      endcase
    end
  endfunction

  // -----------------------------
  // Capture FSM: writes LEN bytes to status_mem
  // -----------------------------
  localparam CAP_IDLE = 3'd0;
  localparam CAP_WRITE = 3'd1;
  localparam CAP_NAME_PREF = 3'd2;
  localparam CAP_NAMES = 3'd3;
  localparam CAP_DONE = 3'd4;

  reg [2:0] cap_st;
  reg [8:0] cap_idx;  // 0..LEN-1
  reg [7:0] name_idx;  // 0..255

  // Signal for stream start when capture is done
  reg       snapshot_ready;

  // Byte generator for capture
  reg [7:0] gen_byte;
  reg       gen_is_name;  // Names werden extra behandelt (sync read)

  always @(*) begin
    gen_byte    = 8'h00;
    gen_is_name = 1'b0;

    // Default: gleiche Payload-Definition wie bisher
    case (cap_idx)
      9'd0: gen_byte = 8'h53;  // 'S'
      9'd1: gen_byte = 8'h54;  // 'T'
      9'd2: gen_byte = 8'h41;  // 'A'
      9'd3: gen_byte = 8'h54;  // 'T'
      9'd4: gen_byte = 8'h05;  // version (0x05)
      9'd5: gen_byte = 8'h00;
      9'd6: gen_byte = 8'd16;
      9'd7: gen_byte = 8'h00;
      9'd8: gen_byte = INTERVAL_MS[15:8];
      9'd9: gen_byte = INTERVAL_MS[7:0];

      // stat_underrun 10..13
      9'd10: gen_byte = stat_underrun[31:24];
      9'd11: gen_byte = stat_underrun[23:16];
      9'd12: gen_byte = stat_underrun[15:8];
      9'd13: gen_byte = stat_underrun[7:0];

      // stat_overwrite 14..17
      9'd14: gen_byte = stat_overwrite[31:24];
      9'd15: gen_byte = stat_overwrite[23:16];
      9'd16: gen_byte = stat_overwrite[15:8];
      9'd17: gen_byte = stat_overwrite[7:0];

      // stat_late 18..21
      9'd18: gen_byte = stat_late[31:24];
      9'd19: gen_byte = stat_late[23:16];
      9'd20: gen_byte = stat_late[15:8];
      9'd21: gen_byte = stat_late[7:0];

      // stat_early 22..25
      9'd22: gen_byte = stat_early[31:24];
      9'd23: gen_byte = stat_early[23:16];
      9'd24: gen_byte = stat_early[15:8];
      9'd25: gen_byte = stat_early[7:0];

      default: begin
        // Peaks 26..57 (16x u16 big-endian)
        if (cap_idx >= 9'd26 && cap_idx <= 9'd57) begin
          if (((cap_idx - 9'd26) & 9'd1) == 9'd0) gen_byte = peak_hi((cap_idx - 9'd26) >> 1);
          else gen_byte = peak_lo((cap_idx - 9'd26) >> 1);

          // ptp_time_sec 58..65
        end else if (cap_idx >= 9'd58 && cap_idx <= 9'd65) begin
          case (cap_idx)
            9'd58:   gen_byte = ptp_time_sec[63:56];
            9'd59:   gen_byte = ptp_time_sec[55:48];
            9'd60:   gen_byte = ptp_time_sec[47:40];
            9'd61:   gen_byte = ptp_time_sec[39:32];
            9'd62:   gen_byte = ptp_time_sec[31:24];
            9'd63:   gen_byte = ptp_time_sec[23:16];
            9'd64:   gen_byte = ptp_time_sec[15:8];
            9'd65:   gen_byte = ptp_time_sec[7:0];
            default: gen_byte = 8'h00;
          endcase

          // ptp_time_ns 66..69
        end else if (cap_idx >= 9'd66 && cap_idx <= 9'd69) begin
          case (cap_idx)
            9'd66:   gen_byte = ptp_time_ns[31:24];
            9'd67:   gen_byte = ptp_time_ns[23:16];
            9'd68:   gen_byte = ptp_time_ns[15:8];
            9'd69:   gen_byte = ptp_time_ns[7:0];
            default: gen_byte = 8'h00;
          endcase

          // buffer_fill + reserved
        end else if (cap_idx == 9'd70) begin
          gen_byte = buffer_fill;
        end else if (cap_idx == 9'd71) begin
          gen_byte = 8'h00;

          // spdif 72..75
        end else if (cap_idx >= 9'd72 && cap_idx <= 9'd75) begin
          case (cap_idx)
            9'd72:   gen_byte = spdif_samples[31:24];
            9'd73:   gen_byte = spdif_samples[23:16];
            9'd74:   gen_byte = spdif_samples[15:8];
            9'd75:   gen_byte = spdif_samples[7:0];
            default: gen_byte = 8'h00;
          endcase

          // xfade_main 76..79
        end else if (cap_idx >= 9'd76 && cap_idx <= 9'd79) begin
          case (cap_idx)
            9'd76:   gen_byte = xfade_main_cnt[31:24];
            9'd77:   gen_byte = xfade_main_cnt[23:16];
            9'd78:   gen_byte = xfade_main_cnt[15:8];
            9'd79:   gen_byte = xfade_main_cnt[7:0];
            default: gen_byte = 8'h00;
          endcase

          // xfade_extra 80..83
        end else if (cap_idx >= 9'd80 && cap_idx <= 9'd83) begin
          case (cap_idx)
            9'd80:   gen_byte = xfade_extra_cnt[31:24];
            9'd81:   gen_byte = xfade_extra_cnt[23:16];
            9'd82:   gen_byte = xfade_extra_cnt[15:8];
            9'd83:   gen_byte = xfade_extra_cnt[7:0];
            default: gen_byte = 8'h00;
          endcase

          // xfade_dup / xfade_zero 84..87
        end else if (cap_idx == 9'd84) gen_byte = xfade_dup_cnt[15:8];
        else if (cap_idx == 9'd85) gen_byte = xfade_dup_cnt[7:0];
        else if (cap_idx == 9'd86) gen_byte = xfade_zero_cnt[15:8];
        else if (cap_idx == 9'd87) gen_byte = xfade_zero_cnt[7:0];

        // samples_read 88..91
        else if (cap_idx >= 9'd88 && cap_idx <= 9'd91) begin
          case (cap_idx)
            9'd88:   gen_byte = samples_read[31:24];
            9'd89:   gen_byte = samples_read[23:16];
            9'd90:   gen_byte = samples_read[15:8];
            9'd91:   gen_byte = samples_read[7:0];
            default: gen_byte = 8'h00;
          endcase

          // samples_written 92..95
        end else if (cap_idx >= 9'd92 && cap_idx <= 9'd95) begin
          case (cap_idx)
            9'd92:   gen_byte = samples_written[31:24];
            9'd93:   gen_byte = samples_written[23:16];
            9'd94:   gen_byte = samples_written[15:8];
            9'd95:   gen_byte = samples_written[7:0];
            default: gen_byte = 8'h00;
          endcase

          // gain 96..127 (16x u16)
        end else if (cap_idx >= 9'd96 && cap_idx <= 9'd127) begin
          if (((cap_idx - 9'd96) & 9'd1) == 9'd0) gen_byte = gain_hi((cap_idx - 9'd96) >> 1);
          else gen_byte = gain_lo((cap_idx - 9'd96) >> 1);

          // pan 128..143 (16x s8)
        end else if (cap_idx >= 9'd128 && cap_idx <= 9'd143) begin
          gen_byte = sel_pan_u8(cap_idx - 9'd128);

          // mute/solo 144..147
        end else if (cap_idx == 9'd144) gen_byte = mute_mask[15:8];
        else if (cap_idx == 9'd145) gen_byte = mute_mask[7:0];
        else if (cap_idx == 9'd146) gen_byte = solo_mask[15:8];
        else if (cap_idx == 9'd147) gen_byte = solo_mask[7:0];

        // names 148..403 (256 bytes, sync from name_rd_data)
        else if (cap_idx >= 9'd148 && cap_idx <= 9'd403) begin
          gen_is_name = 1'b1;
          gen_byte    = 8'h00; // will be replaced in CAP_NAMES by name_rd_data
        end else begin
          // padding
          gen_byte = 8'h00;
        end
      end
    endcase
  end

  always @(posedge clk) begin
    if (rst) begin
      cap_st         <= CAP_IDLE;
      cap_idx        <= 9'd0;
      name_idx       <= 8'd0;
      name_rd_addr   <= 8'd0;
      mem_we         <= 1'b0;
      mem_waddr      <= 9'd0;
      mem_wdata      <= 8'd0;
      snapshot_ready <= 1'b0;
      //peak_freeze    <= 1'b0;
      peak_freeze_r  <= 1'b0;
      peak_reset_req <= 1'b0;
    end else begin
      mem_we         <= 1'b0;
      snapshot_ready <= 1'b0;
      peak_reset_req <= 1'b0;  // default: pulse only 1 cycle

      case (cap_st)
        CAP_IDLE: begin
          if (tick_accum && !sending) begin
            // Freeze peaks during snapshot
            peak_freeze_r <= 1'b1;

            cap_idx <= 9'd0;
            cap_st <= CAP_WRITE;
          end
        end

        CAP_WRITE: begin
          if (gen_is_name) begin
            // Names: erst Address setzen und 1 Zyklus warten
            //name_idx     <= 8'd0;
            name_rd_addr <= 8'd0;  // prefetch addr 0
            cap_st       <= CAP_NAME_PREF;
          end else begin
            mem_we    <= 1'b1;
            mem_waddr <= cap_idx;
            mem_wdata <= gen_byte;

            if (cap_idx == (LEN - 1)) begin
              cap_st <= CAP_DONE;
            end else begin
              cap_idx <= cap_idx + 9'd1;
            end
          end
        end

        CAP_NAME_PREF: begin
          // 1-cycle latency -> name_rd_data is now valid for addr=0
          cap_st <= CAP_NAMES;
          //name_idx <= name_idx + 8'd1;
          name_rd_addr <= name_rd_addr + 8'd1;  // request next
        end

        CAP_NAMES: begin
          // write current name byte
          mem_we    <= 1'b1;
          mem_waddr <= cap_idx;
          mem_wdata <= name_rd_data;

          // next
          if (cap_idx == 9'd403) begin
            cap_idx <= cap_idx + 9'd1;  // -> 404
            cap_st  <= CAP_WRITE;
          end else begin
            cap_idx <= cap_idx + 9'd1;
            //name_idx <= name_idx + 8'd1;
            name_rd_addr <= name_rd_addr + 8'd1;  // request next
          end
        end

        CAP_DONE: begin
          // Snapshot complete -> Release peaks and reset for new window
          peak_freeze_r <= 1'b0;
          peak_reset_req <= 1'b1;  // Reset peaks for next window (in Peak-Always!)

          //reset_peaks();

          snapshot_ready <= 1'b1;
          cap_st <= CAP_IDLE;
        end

        default: cap_st <= CAP_IDLE;
      endcase
    end
  end

  // -----------------------------
  // Stream FSM: liest status_mem und sendet
  // -----------------------------
  localparam STR_IDLE = 2'd0;
  localparam STR_PREFETCH = 2'd1;
  localparam STR_SEND = 2'd2;

  reg [1:0] str_st;
  reg [8:0] str_idx;
  reg       sending;

  always @(posedge clk) begin
    if (rst) begin
      m_tvalid <= 1'b0;
      m_end    <= 1'b0;
      m_start  <= 1'b0;
      m_tdata  <= 8'd0;

      mem_raddr <= 9'd0;

      str_st   <= STR_IDLE;
      str_idx  <= 9'd0;
      sending  <= 1'b0;
    end else begin
      m_tvalid <= 1'b0;
      m_end    <= 1'b0;
      m_start  <= 1'b0;

      case (str_st)
        STR_IDLE: begin
          sending <= 1'b0;
          if (snapshot_ready && m_tready) begin
            // prefetch first byte
            mem_raddr <= 9'd0;
            str_idx   <= 9'd0;
            str_st    <= STR_PREFETCH;
          end
        end

        STR_PREFETCH: begin
          // 1-cycle latency -> mem_rdata valid next cycle
          sending <= 1'b1;
          str_st <= STR_SEND;
          mem_raddr <= 9'd1;
        end

        STR_SEND: begin
          //if (m_tready) begin
          // output byte from previous read
          m_tdata  <= mem_rdata;
          m_tvalid <= 1'b1;

          if (str_idx == 9'd0) m_start <= 1'b1;

          if (str_idx == (LEN - 1)) begin
            m_end   <= 1'b1;
            str_st  <= STR_IDLE;
            sending <= 1'b0;
          end else begin
            // request next address
            str_idx   <= str_idx + 9'd1;
            mem_raddr <= str_idx + 9'd2;
          end
          //end
        end

        default: str_st <= STR_IDLE;
      endcase
    end
  end

endmodule

`resetall
`default_nettype wire
