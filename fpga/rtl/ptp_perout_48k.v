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
 * PTP-Synchronized 48kHz Periodic Output Generator
 *
 * Generates precise 48kHz pulses synchronized to PTP time
 * Uses Bresenham-like algorithm to distribute remainder evenly
 * across periods for accurate long-term frequency
 */
`timescale 1ns / 1ps
`default_nettype none

module ptp_perout_48k #(
    parameter integer BASE_PERIOD_NS = 32'd20833,  // Base period in nanoseconds (48kHz ≈ 20833ns)
    parameter integer REM_NS = 32'd16000,  // Remainder nanoseconds per second
    parameter integer REM_DEN = 32'd48000  // Number of pulses per second
) (
    input wire clk,  // 50 MHz clock (same domain as ptp_timebase)
    input wire rst,

    // PTP time input (running timestamp)
    input wire [47:0] ptp_sec,
    input wire [31:0] ptp_ns,

    output reg perout_48k  // 1-cycle pulse at ~48kHz
);

  // ================================================================
  // Internal registers
  // ================================================================
  // Next scheduled output time
  reg  [47:0] next_sec;
  reg  [31:0] next_ns;
  reg  [31:0] rem_acc;  // Remainder accumulator (Bresenham)
  reg         init_done;

  // Temporary calculation registers
  reg  [31:0] rem_sum;
  reg  [31:0] period_ns;
  reg  [31:0] ns_tmp;

  wire        time_ge;
  assign time_ge = (ptp_sec > next_sec) || ((ptp_sec == next_sec) && (ptp_ns >= next_ns));

  always @(posedge clk or posedge rst) begin
    if (rst) begin
      next_sec   <= 48'd0;
      next_ns    <= 32'd0;
      rem_acc    <= 32'd0;
      init_done  <= 1'b0;
      perout_48k <= 1'b0;
    end else begin
      perout_48k <= 1'b0;  // Default: no pulse

      // ========================================================
      // Initialization: set first target time
      // ========================================================
      if (!init_done) begin
        // Start shortly after current time
        // (Initial phase doesn't matter, just needs to be periodic)
        ns_tmp = ptp_ns + BASE_PERIOD_NS;
        next_sec <= ptp_sec;
        if (ns_tmp >= 32'd1000000000) begin
          next_sec <= ptp_sec + 48'd1;
          next_ns  <= ns_tmp - 32'd1000000000;
        end else begin
          next_ns <= ns_tmp;
        end
        rem_acc   <= 32'd0;
        init_done <= 1'b1;

        // ========================================================
        // Output pulse when PTP time reaches or exceeds target
        // ========================================================
      end else if (time_ge) begin
        // PTP time has reached or passed scheduled time -> pulse
        perout_48k <= 1'b1;

        // Bresenham remainder distribution: spread REM_NS over REM_DEN periods
        rem_sum   = rem_acc + REM_NS;
        period_ns = BASE_PERIOD_NS;

        if (rem_sum >= REM_DEN) begin
          rem_sum   = rem_sum - REM_DEN;
          period_ns = BASE_PERIOD_NS + 32'd1;  // This period gets +1 ns
        end

        rem_acc <= rem_sum;

        // Calculate next target time = previous target + period
        ns_tmp = next_ns + period_ns;
        next_sec <= next_sec;
        if (ns_tmp >= 32'd1000000000) begin
          next_sec <= next_sec + 48'd1;
          next_ns  <= ns_tmp - 32'd1000000000;
        end else begin
          next_ns <= ns_tmp;
        end
      end
    end
  end

endmodule
