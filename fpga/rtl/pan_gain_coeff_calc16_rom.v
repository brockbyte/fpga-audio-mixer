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

`timescale 1ns / 1ps
`default_nettype none

module pan_gain_coeff_calc16_rom #(
    parameter integer COEFF_Q = 14,

    // OFF/Hysterese (gain 0..1023)
    parameter [9:0] GAIN_OFF_TH = 10'd41,  // ~4%
    parameter [9:0] GAIN_ON_TH  = 10'd61,  // ~6%

    // Init files (1 hex value per line)
    //parameter       GAIN_INIT_FILE = "gain_lut_1024_q2_14.hex",        // 1024 x 16-bit
    parameter GAIN_INIT_FILE = "gain_q2_14.hex",               // 1024 x 16-bit
    parameter PAN_INIT_FILE  = "pan_lut_256_q2_14_packed.hex"  // 256  x 32-bit packed {L,R}
) (
    input wire clk,
    input wire rst,

    // 1-cycle pulse: recalculate all 16 coefficient pairs
    input wire start,

    // Inputs (pro Kanal)
    input wire        [9:0] gain0,
    input wire signed [7:0] pan0,
    input wire        [9:0] gain1,
    input wire signed [7:0] pan1,
    input wire        [9:0] gain2,
    input wire signed [7:0] pan2,
    input wire        [9:0] gain3,
    input wire signed [7:0] pan3,
    input wire        [9:0] gain4,
    input wire signed [7:0] pan4,
    input wire        [9:0] gain5,
    input wire signed [7:0] pan5,
    input wire        [9:0] gain6,
    input wire signed [7:0] pan6,
    input wire        [9:0] gain7,
    input wire signed [7:0] pan7,
    input wire        [9:0] gain8,
    input wire signed [7:0] pan8,
    input wire        [9:0] gain9,
    input wire signed [7:0] pan9,
    input wire        [9:0] gain10,
    input wire signed [7:0] pan10,
    input wire        [9:0] gain11,
    input wire signed [7:0] pan11,
    input wire        [9:0] gain12,
    input wire signed [7:0] pan12,
    input wire        [9:0] gain13,
    input wire signed [7:0] pan13,
    input wire        [9:0] gain14,
    input wire signed [7:0] pan14,
    input wire        [9:0] gain15,
    input wire signed [7:0] pan15,

    // Status
    output reg busy,
    output reg done,  // 1-cycle pulse at end

    // Outputs: signed 16-bit Q2.14 (for mixer)
    output wire signed [15:0] g0L,
    output wire signed [15:0] g0R,
    output wire signed [15:0] g1L,
    output wire signed [15:0] g1R,
    output wire signed [15:0] g2L,
    output wire signed [15:0] g2R,
    output wire signed [15:0] g3L,
    output wire signed [15:0] g3R,
    output wire signed [15:0] g4L,
    output wire signed [15:0] g4R,
    output wire signed [15:0] g5L,
    output wire signed [15:0] g5R,
    output wire signed [15:0] g6L,
    output wire signed [15:0] g6R,
    output wire signed [15:0] g7L,
    output wire signed [15:0] g7R,
    output wire signed [15:0] g8L,
    output wire signed [15:0] g8R,
    output wire signed [15:0] g9L,
    output wire signed [15:0] g9R,
    output wire signed [15:0] g10L,
    output wire signed [15:0] g10R,
    output wire signed [15:0] g11L,
    output wire signed [15:0] g11R,
    output wire signed [15:0] g12L,
    output wire signed [15:0] g12R,
    output wire signed [15:0] g13L,
    output wire signed [15:0] g13R,
    output wire signed [15:0] g14L,
    output wire signed [15:0] g14R,
    output wire signed [15:0] g15L,
    output wire signed [15:0] g15R
);

  // ------------------------------------------------------------
  // Inferred synchronous ROMs (1-cycle latency)
  // ------------------------------------------------------------
  reg [15:0] gain_mem[0:1023];
  reg [31:0] pan_mem[0:255];

  integer i;
  initial begin
    for (i = 0; i < 1024; i = i + 1) gain_mem[i] = 16'h0000;
    for (i = 0; i < 256; i = i + 1) pan_mem[i] = 32'h00000000;
    if (GAIN_INIT_FILE != "") $readmemh(GAIN_INIT_FILE, gain_mem);
    if (PAN_INIT_FILE != "") $readmemh(PAN_INIT_FILE, pan_mem);
  end

  reg        gain_rom_en;
  reg [ 9:0] gain_rom_addr;
  reg [15:0] gain_rom_dout;

  reg        pan_rom_en;
  reg [ 7:0] pan_rom_addr;
  reg [31:0] pan_rom_dout;

  always @(posedge clk) begin
    if (gain_rom_en) gain_rom_dout <= gain_mem[gain_rom_addr];
    if (pan_rom_en) pan_rom_dout <= pan_mem[pan_rom_addr];
  end

  // ------------------------------------------------------------
  // Output registers (Q2.14)
  // ------------------------------------------------------------
  reg [15:0] gL_mem[0:15];
  reg [15:0] gR_mem[0:15];
  reg gain_on[0:15];

  assign g0L  = $signed(gL_mem[0]);
  assign g0R  = $signed(gR_mem[0]);
  assign g1L  = $signed(gL_mem[1]);
  assign g1R  = $signed(gR_mem[1]);
  assign g2L  = $signed(gL_mem[2]);
  assign g2R  = $signed(gR_mem[2]);
  assign g3L  = $signed(gL_mem[3]);
  assign g3R  = $signed(gR_mem[3]);
  assign g4L  = $signed(gL_mem[4]);
  assign g4R  = $signed(gR_mem[4]);
  assign g5L  = $signed(gL_mem[5]);
  assign g5R  = $signed(gR_mem[5]);
  assign g6L  = $signed(gL_mem[6]);
  assign g6R  = $signed(gR_mem[6]);
  assign g7L  = $signed(gL_mem[7]);
  assign g7R  = $signed(gR_mem[7]);
  assign g8L  = $signed(gL_mem[8]);
  assign g8R  = $signed(gR_mem[8]);
  assign g9L  = $signed(gL_mem[9]);
  assign g9R  = $signed(gR_mem[9]);
  assign g10L = $signed(gL_mem[10]);
  assign g10R = $signed(gR_mem[10]);
  assign g11L = $signed(gL_mem[11]);
  assign g11R = $signed(gR_mem[11]);
  assign g12L = $signed(gL_mem[12]);
  assign g12R = $signed(gR_mem[12]);
  assign g13L = $signed(gL_mem[13]);
  assign g13R = $signed(gR_mem[13]);
  assign g14L = $signed(gL_mem[14]);
  assign g14R = $signed(gR_mem[14]);
  assign g15L = $signed(gL_mem[15]);
  assign g15R = $signed(gR_mem[15]);

  // ------------------------------------------------------------
  // Helpers: select pan/gain by channel index
  // ------------------------------------------------------------
  function [9:0] sel_gain;
    input [3:0] idx;
    begin
      case (idx)
        4'd0: sel_gain = gain0;
        4'd1: sel_gain = gain1;
        4'd2: sel_gain = gain2;
        4'd3: sel_gain = gain3;
        4'd4: sel_gain = gain4;
        4'd5: sel_gain = gain5;
        4'd6: sel_gain = gain6;
        4'd7: sel_gain = gain7;
        4'd8: sel_gain = gain8;
        4'd9: sel_gain = gain9;
        4'd10: sel_gain = gain10;
        4'd11: sel_gain = gain11;
        4'd12: sel_gain = gain12;
        4'd13: sel_gain = gain13;
        4'd14: sel_gain = gain14;
        default: sel_gain = gain15;
      endcase
    end
  endfunction

  function signed [7:0] sel_pan;
    input [3:0] idx;
    begin
      case (idx)
        4'd0: sel_pan = pan0;
        4'd1: sel_pan = pan1;
        4'd2: sel_pan = pan2;
        4'd3: sel_pan = pan3;
        4'd4: sel_pan = pan4;
        4'd5: sel_pan = pan5;
        4'd6: sel_pan = pan6;
        4'd7: sel_pan = pan7;
        4'd8: sel_pan = pan8;
        4'd9: sel_pan = pan9;
        4'd10: sel_pan = pan10;
        4'd11: sel_pan = pan11;
        4'd12: sel_pan = pan12;
        4'd13: sel_pan = pan13;
        4'd14: sel_pan = pan14;
        default: sel_pan = pan15;
      endcase
    end
  endfunction

  // ========== Pan Value to ROM Index Conversion ==========
  // Converts signed pan value (-127..+127) to ROM address (0..255)
  function [7:0] pan_to_idx;
    input signed [7:0] p;  // -127..+127
    reg signed [8:0] tmp;
    begin
      // Robust signed addition (no concatenation trick)
      tmp = $signed(p) + 9'sd128;  // -127..+127 -> 1..255

      if (tmp < 0) tmp = 0;
      if (tmp > 255) tmp = 255;

      pan_to_idx = tmp[7:0];
    end
  endfunction

  // ========== FSM: Synchronous ROM Read and Coefficient Calculation ==========
  // Computes: coeff = gain_lin * panX >> 14  (Q2.14 format)
  // Processes all 16 channels sequentially
  localparam ST_IDLE = 3'd0;  // Wait for start trigger
  localparam ST_REQ_ROM = 3'd1;  // Request ROM reads
  localparam ST_WAIT_ROM = 3'd2;  // Wait 1 cycle for ROM address
  localparam ST_COEFF_L = 3'd3;  // Calculate left coefficient
  localparam ST_COEFF_R = 3'd4;  // Calculate right coefficient
  localparam ST_WRITE = 3'd5;  // Write coefficients to memory
  localparam ST_WAIT_ROM2 = 3'd6;  // Wait for ROM data valid

  reg        [ 2:0] st;
  reg        [ 3:0] ch_idx;

  reg        [ 9:0] gain_cur;
  reg signed [ 7:0] pan_cur;

  reg        [15:0] gain_lin_q14;  // unsigned
  reg [15:0] panL_q14, panR_q14;
  reg [31:0] prod32;

  always @(posedge clk) begin
    if (rst) begin
      st <= ST_IDLE;
      busy <= 1'b0;
      done <= 1'b0;

      gain_rom_en <= 1'b0;
      gain_rom_addr <= 10'd0;
      pan_rom_en <= 1'b0;
      pan_rom_addr <= 8'd0;

      ch_idx <= 4'd0;
      gain_cur <= 10'd0;
      pan_cur <= 8'sd0;

      gain_lin_q14 <= 16'd0;
      panL_q14 <= 16'd0;
      panR_q14 <= 16'd0;
      prod32 <= 32'd0;

      for (i = 0; i < 16; i = i + 1) begin
        gL_mem[i]  <= 16'd0;
        gR_mem[i]  <= 16'd0;
        gain_on[i] <= 1'b0;
      end
    end else begin
      done <= 1'b0;
      gain_rom_en <= 1'b0;
      pan_rom_en <= 1'b0;

      case (st)
        ST_IDLE: begin
          busy <= 1'b0;
          if (start) begin
            busy <= 1'b1;
            ch_idx <= 4'd0;
            st <= ST_REQ_ROM;
          end
        end

        ST_REQ_ROM: begin
          // latch current inputs
          gain_cur <= sel_gain(ch_idx);
          pan_cur  <= sel_pan(ch_idx);

          // request sync ROM reads
          gain_rom_en   <= 1'b1;
          gain_rom_addr <= sel_gain(ch_idx);

          pan_rom_en    <= 1'b1;
          pan_rom_addr  <= pan_to_idx(sel_pan(ch_idx));

          st <= ST_WAIT_ROM;
        end

        ST_WAIT_ROM: begin
          st <= ST_WAIT_ROM2;
        end

        ST_WAIT_ROM2: begin
          // ROM outputs are now valid
          panL_q14 <= pan_rom_dout[31:16];
          panR_q14 <= pan_rom_dout[15:0];

          // Hysteresis selection for gain_lin (noise gate)
          if (gain_on[ch_idx]) begin
            if (gain_cur <= GAIN_OFF_TH) gain_lin_q14 <= 16'd0;
            else gain_lin_q14 <= gain_rom_dout;
          end else begin
            if (gain_cur >= GAIN_ON_TH) gain_lin_q14 <= gain_rom_dout;
            else gain_lin_q14 <= 16'd0;
          end

          st <= ST_COEFF_L;
        end

        ST_COEFF_L: begin
          // Multiply: gain_lin * panL (result is Q4.28)
          prod32 <= $unsigned(gain_lin_q14) * $unsigned(panL_q14);
          st <= ST_COEFF_R;
        end

        ST_COEFF_R: begin
          // Write left coefficient from previous multiply (extract Q2.14)
          gL_mem[ch_idx] <= prod32[COEFF_Q+15 : COEFF_Q];

          // Multiply: gain_lin * panR
          prod32 <= $unsigned(gain_lin_q14) * $unsigned(panR_q14);
          st <= ST_WRITE;
        end

        ST_WRITE: begin
          // Write right coefficient from previous multiply
          gR_mem[ch_idx] <= prod32[COEFF_Q+15 : COEFF_Q];

          // Update hysteresis state for this channel
          if (gain_on[ch_idx]) begin
            if (gain_cur <= GAIN_OFF_TH) gain_on[ch_idx] <= 1'b0;
          end else begin
            if (gain_cur >= GAIN_ON_TH) gain_on[ch_idx] <= 1'b1;
          end

          if (ch_idx == 4'd15) begin
            busy <= 1'b0;
            done <= 1'b1;
            st   <= ST_IDLE;
          end else begin
            ch_idx <= ch_idx + 4'd1;
            st <= ST_REQ_ROM;
          end
        end

        default: st <= ST_IDLE;
      endcase
    end
  end

endmodule

`default_nettype wire
