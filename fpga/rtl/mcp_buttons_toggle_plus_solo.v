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
 * MCP23017 Button Controller with Toggle and Solo Modes
 *
 * Features:
 * - Bank0: Normal toggle mode (multiple bits can be set) - mute buttons
 * - Bank1: Solo one-hot mode (only one bit at a time, press again to clear all)
 * - Hardware debouncing with configurable time
 * - Active-low button inputs with inversion
 * - LED output control
 */
`timescale 1ns / 1ps
`default_nettype none

module mcp_buttons_toggle_plus_solo #(
    parameter integer CLK_HZ = 50000000,
    parameter integer DEBOUNCE_MS = 20
) (
    input wire clk,
    input wire rst,

    // Button inputs (active-low from MCP23017)
    input wire [7:0] btn0_n_raw,  // MCP0 GPIOA (active-low)
    input wire [7:0] btn1_n_raw,  // MCP1 GPIOA (active-low)

    // Current status inputs
    input wire [7:0] cur_status0,  // Current mute status
    input wire [7:0] cur_status1,  // Current solo status (one-hot)

    // Status outputs
    output reg [7:0] status0,  // Mute toggle (multi-bit)
    output reg [7:0] status1,  // Solo one-hot

    output wire [7:0] led0,
    output wire [7:0] led1,

    output reg set_mute,
    output reg set_solo
);

  // LED outputs reflect current status
  assign led0 = cur_status0;
  assign led1 = cur_status1;

  // Invert active-low buttons (pressed=1)
  wire [7:0] btn0_raw = ~btn0_n_raw;
  wire [7:0] btn1_raw = ~btn1_n_raw;

  localparam integer DEBOUNCE_TICKS = (CLK_HZ / 1000) * DEBOUNCE_MS;

  reg [7:0] btn0_stable, btn1_stable;
  reg [7:0] btn0_prev, btn1_prev;

  integer i;
  reg [31:0] cnt0[0:7];
  reg [31:0] cnt1[0:7];

  function [7:0] onehot8;
    input [2:0] idx;
    begin
      onehot8 = (8'b00000001 << idx);
    end
  endfunction

  always @(posedge clk) begin
    if (rst) begin
      status0     <= 8'h00;
      status1     <= 8'h00;
      set_mute    <= 1'b0;
      set_solo    <= 1'b0;
      btn0_stable <= 8'h00;
      btn1_stable <= 8'h00;
      btn0_prev   <= 8'h00;
      btn1_prev   <= 8'h00;
      for (i = 0; i < 8; i = i + 1) begin
        cnt0[i] <= 32'd0;
        cnt1[i] <= 32'd0;
      end
    end else begin
      btn0_prev <= btn0_stable;
      btn1_prev <= btn1_stable;
      set_mute  <= 1'b0;
      set_solo  <= 1'b0;
      status0   <= 8'h00;

      // ========================================================
      // Debounce both button banks
      // ========================================================
      for (i = 0; i < 8; i = i + 1) begin
        if (btn0_raw[i] == btn0_stable[i]) cnt0[i] <= 32'd0;
        else begin
          if (cnt0[i] >= (DEBOUNCE_TICKS - 1)) begin
            btn0_stable[i] <= btn0_raw[i];
            cnt0[i] <= 32'd0;
          end else cnt0[i] <= cnt0[i] + 32'd1;
        end

        if (btn1_raw[i] == btn1_stable[i]) cnt1[i] <= 32'd0;
        else begin
          if (cnt1[i] >= (DEBOUNCE_TICKS - 1)) begin
            btn1_stable[i] <= btn1_raw[i];
            cnt1[i] <= 32'd0;
          end else cnt1[i] <= cnt1[i] + 32'd1;
        end
      end

      // ========================================================
      // Process button presses on rising edge
      // ========================================================
      for (i = 0; i < 8; i = i + 1) begin
        // Bank0: Normal toggle mode (mute)
        if (btn0_stable[i] & ~btn0_prev[i]) begin
          status0[i] <= 1'b1;
          set_mute   <= 1'b1;
        end

        // Bank1: Solo one-hot mode with toggle-off
        if (btn1_stable[i] & ~btn1_prev[i]) begin
          if (cur_status1[i]) status1 <= 8'h00;  // Toggle off if same button
          else status1 <= onehot8(i[2:0]);  // Solo this channel
          set_solo <= 1'b1;
        end
      end
    end
  end

endmodule
