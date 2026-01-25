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
// PTP Client with Stream Interface (50 MHz Domain)
// =============================================================================
// Complete PTP client implementation combining packet parser, servo controller,
// and timebase with slew correction.
//
// Features:
// - Parses PTP Sync/Follow_Up messages from stream interface
// - PI servo control with wrap-safe time difference calculation
// - Timebase with gradual slew correction
// - Hard step for large errors, soft slew for fine adjustments
// - Locked status output
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none
module ptp_client_stream_50m #(
    parameter integer NS_PER_TICK   = 20,        // 50 MHz
    parameter integer STEP_LIMIT_NS = 1_000_000
) (
    input wire clk,  // 50 MHz
    input wire rst,

    // ========== Packet Stream from FIFO (PTP payload only) ==========
    input  wire [7:0] s_data,
    input  wire       s_valid,
    output wire       s_ready,
    input  wire       s_start,
    input  wire       s_end,

    // ========== Local PTP Time Output ==========
    output wire [47:0] ptp_sec,
    output wire [31:0] ptp_ns,

    // ========== Status Outputs ==========
    output wire ptp_locked,
    output wire set_time
);

  // ========== Internal Signals ==========
  wire               hard_step_req;
  wire signed [63:0] phase_correction_ns;

  wire               set_time_valid;
  wire        [47:0] set_time_sec;
  wire        [31:0] set_time_ns;
  wire               corr_load;
  wire signed [31:0] corr_ns;

  assign set_time = set_time_valid;

  // ========== PTP Servo Controller ==========
  // Calculates time corrections based on sync messages
  ptp_client_servo_light_wrapsafe_dbg u_servo (
      .clk             (clk),
      .rst             (rst),
      .sync_valid      (event_valid),
      .master_sec      (master_sec),
      .master_ns       (master_ns),
      .local_sec_sample(local_sec_sample),
      .local_ns_sample (local_ns_sample),
      .set_time_valid  (set_time_valid),
      .set_time_sec    (set_time_sec),
      .set_time_ns     (set_time_ns),
      .corr_load       (corr_load),
      .corr_ns         (corr_ns),
      .dbg_locked      (ptp_locked)
  );

  // ========== PTP Timebase with Slew Correction ==========
  // Maintains local PTP time with gradual phase adjustments
  ptp_timebase_slew #(
      .NS_PER_TICK(NS_PER_TICK)  // 50 MHz = 20ns per tick
  ) u_timebase (
      .clk                (clk),
      .rst                (rst),
      .set_time_valid     (set_time_valid),
      .set_time_sec       (set_time_sec),
      .set_time_ns        (set_time_ns),
      .corr_load          (corr_load),
      .corr_ns            (corr_ns),
      .ptp_sec            (ptp_sec),
      .ptp_ns             (ptp_ns),
      .phase_correction_ns(  /* optional debug */)
  );

  // ========== PTP Sync/Follow_Up Parser ==========
  // Extracts timestamps from incoming PTP messages
  wire        event_valid;
  wire [47:0] master_sec;
  wire [31:0] master_ns;
  wire [47:0] local_sec_sample;
  wire [31:0] local_ns_sample;

  ptp_v2_sync_follow_parser_stream #(
      .TSTAMP_SEC_WIDTH(48),
      .TSTAMP_NS_WIDTH (32)
  ) u_parser (
      .clk         (clk),
      .rst         (rst),
      .s_data      (s_data),
      .s_valid     (s_valid),
      .s_ready     (s_ready),
      .s_start     (s_start),
      .s_end       (s_end),
      .ptp_time_sec(ptp_sec),
      .ptp_time_ns (ptp_ns),
      .event_valid (event_valid),
      .master_sec  (master_sec),
      .master_ns   (master_ns),
      .local_sec   (local_sec_sample),
      .local_ns    (local_ns_sample)
  );

endmodule

`default_nettype wire
