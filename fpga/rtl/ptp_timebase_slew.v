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
// PTP Timebase with 1ns Slew Correction
// =============================================================================
// Maintains PTP time (seconds + nanoseconds) with gradual phase correction.
// Uses simple adders/comparators without multiplication.
//
// Features:
// - Hard step: Set time directly (for large corrections)
// - Soft slew: Gradual adjustment ±1ns per clock cycle
// - Phase correction accumulator with automatic drain
// - Configurable nanoseconds per clock tick
// =============================================================================

`timescale 1ns / 1ps

module ptp_timebase_slew #(
    parameter integer NS_PER_TICK = 20  // Example: 50 MHz -> 20 ns
) (
    input wire clk,
    input wire rst,

    // ========== Hard Time Step ==========
    input wire        set_time_valid,
    input wire [47:0] set_time_sec,
    input wire [31:0] set_time_ns,

    // ========== Fine Correction (from PI Servo) ==========
    // Correction value in nanoseconds
    input wire               corr_load,
    input wire signed [31:0] corr_ns,

    // ========== Current PTP Time Output ==========
    output reg [47:0] ptp_sec,
    output reg [31:0] ptp_ns,

    // ========== Debug: Internal Slew Status ==========
    output reg signed [31:0] phase_correction_ns
);

  reg signed [ 7:0] slew_step_ns;
  reg signed [31:0] ns_next;

  // ========== Phase Correction Accumulator ==========
  // Stores the correction value and gradually drains it
  always @(posedge clk or posedge rst) begin
    if (rst) begin
      phase_correction_ns <= 32'sd0;
    end else begin
      if (set_time_valid) begin
        // On hard step, reset phase correction
        phase_correction_ns <= 32'sd0;
      end else if (corr_load) begin
        // Load new correction from servo (e.g., every sync)
        phase_correction_ns <= corr_ns;
      end else begin
        // Gradually "drain" correction
        if (phase_correction_ns > 0) phase_correction_ns <= phase_correction_ns - 1;
        else if (phase_correction_ns < 0) phase_correction_ns <= phase_correction_ns + 1;
      end
    end
  end

  // ========== Slew Step Calculation ==========
  // Apply -1, 0, or +1 ns correction per tick
  always @* begin
    if (phase_correction_ns > 0) slew_step_ns = 1;
    else if (phase_correction_ns < 0) slew_step_ns = -1;
    else slew_step_ns = 0;
  end

  // ========== Time Advancement ==========
  // Increment time each clock cycle with slew correction
  always @(posedge clk or posedge rst) begin
    if (rst) begin
      ptp_sec <= 48'd0;
      ptp_ns  <= 32'd0;
    end else begin
      if (set_time_valid) begin
        // Hard time step to master
        ptp_sec <= set_time_sec;
        ptp_ns  <= set_time_ns;
      end else begin
        // Normal advancement with slew
        ns_next = $signed(ptp_ns) + $signed(NS_PER_TICK) + slew_step_ns;

        if (ns_next >= 32'd1000000000) begin
          ptp_ns  <= ns_next - 32'd1000000000;
          ptp_sec <= ptp_sec + 48'd1;
        end else if (ns_next < 0) begin
          ptp_ns  <= ns_next + 32'd1000000000;
          ptp_sec <= ptp_sec - 48'd1;
        end else begin
          ptp_ns <= ns_next;
        end
      end
    end
  end

endmodule
