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
// PTP Client Servo with Wrap-Safe Time Difference and PI Control
// =============================================================================
// Implements a PI controller for PTP clock synchronization with debug outputs.
// Handles wrap-around for second boundaries and applies either hard steps
// or gradual slew corrections.
//
// Features:
// - Wrap-safe time difference calculation (handles ns overflow)
// - Hard step for large errors (>5ms) or initial sync
// - PI servo for fine adjustments (Kp and Ki with right-shift division)
// - Integral term with anti-windup (configurable limit)
// - 3-stage pipeline for difference calculation and servo output
// - Debug outputs for error tracking and tuning
// =============================================================================

`timescale 1ns / 1ps

module ptp_client_servo_light_wrapsafe_dbg #(
    parameter integer HARD_STEP_THRESH_NS = 5_000_000,
    parameter integer INTEGRAL_LIMIT_NS   = 200_000_000,
    parameter integer KP_SHIFT            = 12,
    parameter integer KI_SHIFT            = 18
) (
    input wire clk,
    input wire rst,

    input wire        sync_valid,
    input wire [47:0] master_sec,
    input wire [31:0] master_ns,
    input wire [47:0] local_sec_sample,
    input wire [31:0] local_ns_sample,

    output reg        set_time_valid,
    output reg [47:0] set_time_sec,
    output reg [31:0] set_time_ns,

    output reg               corr_load,
    output reg signed [31:0] corr_ns,

    // Debug
    output reg               dbg_locked,
    output reg               dbg_hardstep_pulse,
    output reg        [31:0] dbg_hardstep_count,
    output reg signed [31:0] dbg_error_ns,
    output reg signed [47:0] dbg_d_sec_norm,
    output reg signed [31:0] dbg_d_ns_norm,
    output reg signed [31:0] dbg_integral_term,
    output reg signed [31:0] dbg_corr_candidate
);

  // ========== 3-Stage Pipeline Control ==========
  reg v1, v2;

  always @(posedge clk or posedge rst) begin
    if (rst) begin
      v1 <= 1'b0;
      v2 <= 1'b0;
    end else begin
      v1 <= sync_valid;
      v2 <= v1;
    end
  end

  // ========== Stage 1: Raw Time Differences ==========
  // Calculate master_time - local_time
  reg signed [47:0] d_sec_raw_r;
  reg signed [31:0] d_ns_raw_r;

  always @(posedge clk) begin
    if (sync_valid) begin
      d_sec_raw_r <= $signed(master_sec) - $signed(local_sec_sample);
      d_ns_raw_r  <= $signed(master_ns) - $signed(local_ns_sample);
    end
  end

  // ========== Stage 2: Normalization and Error Calculation ==========
  // Handle nanosecond overflow/underflow and calculate signed error
  reg signed [47:0] d_sec_norm_r;
  reg signed [31:0] d_ns_norm_r;
  reg signed [31:0] error_ns_r;

  always @(posedge clk) begin
    if (v1) begin
      d_sec_norm_r <= d_sec_raw_r;
      d_ns_norm_r  <= d_ns_raw_r;

      if (d_ns_raw_r < 0) begin
        d_ns_norm_r  <= d_ns_raw_r + 32'sd1000000000;
        d_sec_norm_r <= d_sec_raw_r - 48'sd1;
      end else if (d_ns_raw_r >= 32'sd1000000000) begin
        d_ns_norm_r  <= d_ns_raw_r - 32'sd1000000000;
        d_sec_norm_r <= d_sec_raw_r + 48'sd1;
      end

      if (d_sec_norm_r == 0) begin
        if (d_ns_norm_r <= 32'sd500000000) error_ns_r <= d_ns_norm_r;
        else error_ns_r <= -(32'sd1000000000 - d_ns_norm_r);
      end else if (d_sec_norm_r == 1) begin
        error_ns_r <= -(32'sd1000000000 - d_ns_norm_r);
      end else if (d_sec_norm_r == -1) begin
        error_ns_r <= d_ns_norm_r;
      end else begin
        error_ns_r <= 32'sd0;
      end
    end
  end

  wire [31:0] abs_error_ns = error_ns_r[31] ? (~error_ns_r + 1'b1) : error_ns_r;

  // ========== Stage 3: Servo Control (Hard Step or PI) ==========
  reg locked;
  reg signed [31:0] integral_term;

  // Saturation function for integral anti-windup
  function signed [31:0] sat32;
    input signed [31:0] val;
    input signed [31:0] limit;
    begin
      if (val > limit) sat32 = limit;
      else if (val < -limit) sat32 = -limit;
      else sat32 = val;
    end
  endfunction

  always @(posedge clk or posedge rst) begin
    if (rst) begin
      locked             <= 1'b0;
      set_time_valid     <= 1'b0;
      set_time_sec       <= 48'd0;
      set_time_ns        <= 32'd0;
      corr_load          <= 1'b0;
      corr_ns            <= 32'sd0;
      integral_term      <= 32'sd0;

      dbg_locked         <= 1'b0;
      dbg_hardstep_pulse <= 1'b0;
      dbg_hardstep_count <= 32'd0;
      dbg_error_ns       <= 32'sd0;
      dbg_d_sec_norm     <= 48'sd0;
      dbg_d_ns_norm      <= 32'sd0;
      dbg_integral_term  <= 32'sd0;
      dbg_corr_candidate <= 32'sd0;

    end else begin
      set_time_valid     <= 1'b0;
      corr_load          <= 1'b0;
      dbg_hardstep_pulse <= 1'b0;

      if (v2) begin
        dbg_error_ns   <= error_ns_r;
        dbg_d_sec_norm <= d_sec_norm_r;
        dbg_d_ns_norm  <= d_ns_norm_r;

        if (!locked || (d_sec_norm_r != 0) || (abs_error_ns > HARD_STEP_THRESH_NS[31:0])) begin

          set_time_valid     <= 1'b1;
          set_time_sec       <= master_sec;
          set_time_ns        <= master_ns;

          locked             <= 1'b1;
          integral_term      <= 32'sd0;
          corr_ns            <= 32'sd0;

          dbg_hardstep_pulse <= 1'b1;
          dbg_hardstep_count <= dbg_hardstep_count + 1;

        end else begin
          integral_term <= sat32(
              integral_term + (error_ns_r >>> KI_SHIFT), INTEGRAL_LIMIT_NS[31:0]
          );

          corr_ns <= (error_ns_r >>> KP_SHIFT) + integral_term;
          corr_load <= 1'b1;

          dbg_integral_term <= integral_term;
          dbg_corr_candidate <= corr_ns;
        end
      end

      dbg_locked <= locked;
    end
  end

endmodule
