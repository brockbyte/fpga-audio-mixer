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
 * 8-Channel Quadrature Rotary Encoder Decoder with Debouncing
 * Supports: STEC12E08 or similar mechanical encoders
 *
 * Features:
 * - 8 independent quadrature (A/B) input channels
 * - Synchronous debouncing via integrator counters
 * - Quarter-step accumulator: Counts +/-1 per detent (not 4x per detent)
 * - External value override capability per channel
 * - Configurable per-channel direction inversion
 * - Active-low input support with internal inversion
 */
`timescale 1ns / 1ps
`default_nettype none

module rotary8_stec12e08_debounced #(
    parameter [7:0] DIR_INV = 8'h00,  // Per-channel direction inversion mask
    parameter integer DEB_BITS = 12,  // Debounce counter width (12 = ~4096 samples)
    parameter integer INPUT_ACTIVE_LOW = 1  // 1 = inputs are active-low, 0 = active-high
) (
    input wire clk,
    input wire rst,

    // Encoder inputs: 8 channels A/B signals
    input wire [7:0] enc_a,
    input wire [7:0] enc_b,

    // External value override (optional, per channel)
    input wire        [7:0] ext_we,    // External write enable
    input wire signed [7:0] ext_val0,
    input wire signed [7:0] ext_val1,
    input wire signed [7:0] ext_val2,
    input wire signed [7:0] ext_val3,
    input wire signed [7:0] ext_val4,
    input wire signed [7:0] ext_val5,
    input wire signed [7:0] ext_val6,
    input wire signed [7:0] ext_val7,

    // Current encoder values (8-bit signed counters)
    output reg signed [7:0] val0,
    output reg signed [7:0] val1,
    output reg signed [7:0] val2,
    output reg signed [7:0] val3,
    output reg signed [7:0] val4,
    output reg signed [7:0] val5,
    output reg signed [7:0] val6,
    output reg signed [7:0] val7,

    // Step pulses: 1-cycle active-high pulse per detent
    output reg [7:0] step_plus,
    output reg [7:0] step_minus
);

  // ================================================================
  // Internal signals and registers
  // ================================================================
  integer i;

  // Invert inputs if active-low configuration
  wire [7:0] a_in = (INPUT_ACTIVE_LOW != 0) ? ~enc_a : enc_a;
  wire [7:0] b_in = (INPUT_ACTIVE_LOW != 0) ? ~enc_b : enc_b;

  // Metastability protection: two-stage synchronizers
  reg [7:0] a_ff1, a_ff2;
  reg [7:0] b_ff1, b_ff2;

  // Debounce integrator counters (per input pin)
  reg [DEB_BITS-1:0] a_cnt[0:7];
  reg [DEB_BITS-1:0] b_cnt[0:7];

  // Debounced signals
  reg [7:0] a_db;
  reg [7:0] b_db;

  // Previous A/B state for edge detection
  reg [1:0] ab_last[0:7];

  // Quarter-step accumulator: -3..+3 range (4 = one full detent)
  reg signed [2:0] qacc[0:7];

  // ================================================================
  // Helper functions
  // ================================================================

  // Saturating add for 8-bit signed values
  function signed [7:0] sat_add1;
    input signed [7:0] v;
    input signed [1:0] d;
    reg signed [8:0] tmp;
    begin
      tmp = v + d;
      if (tmp > 9'sd127) sat_add1 = 8'sd127;
      else if (tmp < -9'sd127) sat_add1 = -8'sd127;
      else sat_add1 = tmp[7:0];
    end
  endfunction

  // Quadrature state machine: decode direction from A/B transitions
  function signed [1:0] quad_delta;
    input [1:0] last_ab;
    input [1:0] new_ab;
    reg [3:0] t;
    begin
      t = {last_ab, new_ab};
      case (t)
        4'b0001, 4'b0111, 4'b1110, 4'b1000: quad_delta = 2'sd1;  // Clockwise
        4'b0010, 4'b1011, 4'b1101, 4'b0100: quad_delta = -2'sd1;  // Counter-clockwise
        default:                            quad_delta = 2'sd0;  // Invalid/no change
      endcase
    end
  endfunction

  // Set value by index (task to handle array indexing)
  task set_val_by_idx;
    input integer idx;
    input signed [7:0] v;
    begin
      case (idx)
        0: val0 = v;
        1: val1 = v;
        2: val2 = v;
        3: val3 = v;
        4: val4 = v;
        5: val5 = v;
        6: val6 = v;
        7: val7 = v;
        default: ;
      endcase
    end
  endtask

  // Get current value by index
  function signed [7:0] get_val_by_idx;
    input integer idx;
    begin
      case (idx)
        0: get_val_by_idx = val0;
        1: get_val_by_idx = val1;
        2: get_val_by_idx = val2;
        3: get_val_by_idx = val3;
        4: get_val_by_idx = val4;
        5: get_val_by_idx = val5;
        6: get_val_by_idx = val6;
        7: get_val_by_idx = val7;
        default: get_val_by_idx = 8'sd0;
      endcase
    end
  endfunction

  // Get external value by index
  function signed [7:0] get_ext_by_idx;
    input integer idx;
    begin
      case (idx)
        0: get_ext_by_idx = ext_val0;
        1: get_ext_by_idx = ext_val1;
        2: get_ext_by_idx = ext_val2;
        3: get_ext_by_idx = ext_val3;
        4: get_ext_by_idx = ext_val4;
        5: get_ext_by_idx = ext_val5;
        6: get_ext_by_idx = ext_val6;
        7: get_ext_by_idx = ext_val7;
        default: get_ext_by_idx = 8'sd0;
      endcase
    end
  endfunction

  // Saturating increment
  function [DEB_BITS-1:0] inc_sat;
    input [DEB_BITS-1:0] x;
    begin
      if (&x) inc_sat = x;
      else inc_sat = x + {{(DEB_BITS - 1) {1'b0}}, 1'b1};
    end
  endfunction

  // Saturating decrement
  function [DEB_BITS-1:0] dec_sat;
    input [DEB_BITS-1:0] x;
    begin
      if (x == {DEB_BITS{1'b0}}) dec_sat = x;
      else dec_sat = x - {{(DEB_BITS - 1) {1'b0}}, 1'b1};
    end
  endfunction

  wire [DEB_BITS-1:0] CNT_MAX = {DEB_BITS{1'b1}};

  // ================================================================
  // Main decoder logic: debounce, decode, and accumulate
  // ================================================================
  // ================================================================
  // Main decoder logic: debounce, decode, and accumulate
  // ================================================================
  always @(posedge clk) begin
    if (rst) begin
      a_ff1 <= 8'h00;
      a_ff2 <= 8'h00;
      b_ff1 <= 8'h00;
      b_ff2 <= 8'h00;
      a_db <= 8'h00;
      b_db <= 8'h00;

      step_plus <= 8'h00;
      step_minus <= 8'h00;

      val0 <= 8'sd0;
      val1 <= 8'sd0;
      val2 <= 8'sd0;
      val3 <= 8'sd0;
      val4 <= 8'sd0;
      val5 <= 8'sd0;
      val6 <= 8'sd0;
      val7 <= 8'sd0;

      for (i = 0; i < 8; i = i + 1) begin
        a_cnt[i]   <= {DEB_BITS{1'b0}};
        b_cnt[i]   <= {DEB_BITS{1'b0}};
        ab_last[i] <= 2'b00;
        qacc[i]    <= 3'sd0;
      end
    end else begin
      // Clear step pulses (only asserted for one cycle)
      step_plus <= 8'h00;
      step_minus <= 8'h00;

      // Two-stage synchronizer for metastability protection
      a_ff1 <= a_in;
      a_ff2 <= a_ff1;
      b_ff1 <= b_in;
      b_ff2 <= b_ff1;

      // Process each encoder channel
      for (i = 0; i < 8; i = i + 1) begin : ENC
        reg signed [1:0] d;
        reg [1:0] ab_now;

        // Debounce A input: integrate up/down based on input state
        if (a_ff2[i]) a_cnt[i] <= inc_sat(a_cnt[i]);
        else a_cnt[i] <= dec_sat(a_cnt[i]);
        if (a_cnt[i] == CNT_MAX) a_db[i] <= 1'b1;
        else if (a_cnt[i] == {DEB_BITS{1'b0}}) a_db[i] <= 1'b0;

        // Debounce B input: integrate up/down based on input state
        if (b_ff2[i]) b_cnt[i] <= inc_sat(b_cnt[i]);
        else b_cnt[i] <= dec_sat(b_cnt[i]);
        if (b_cnt[i] == CNT_MAX) b_db[i] <= 1'b1;
        else if (b_cnt[i] == {DEB_BITS{1'b0}}) b_db[i] <= 1'b0;

        // Quadrature decode: detect and accumulate quarter-steps
        ab_now = {a_db[i], b_db[i]};
        if (ab_now != ab_last[i]) begin
          d = quad_delta(ab_last[i], ab_now);
          if (DIR_INV[i]) d = -d;

          // Invalid transition: reset accumulator (bounce/skip detected)
          if (d == 2'sd0) begin
            qacc[i] <= 3'sd0;
          end else begin
            // Direction reversal: reset accumulator (prevents phantom steps)
            if ((qacc[i] > 0 && d < 0) || (qacc[i] < 0 && d > 0)) qacc[i] <= 3'sd0;
            else qacc[i] <= qacc[i] + d;  // Accumulate +1 or -1

            // Check for full detent: +/-4 quarter-steps = one detent
            begin : ACCCHK
              reg signed [3:0] acc_next;
              acc_next = ( ((qacc[i] > 0 && d < 0) || (qacc[i] < 0 && d > 0)) || (d==0) )
                ? 4'sd0
                : (qacc[i] + d);

              // Full detent forward: emit step_plus and reset
              if (acc_next >= 4'sd4) begin
                step_plus[i] <= 1'b1;
                set_val_by_idx(i, sat_add1(get_val_by_idx(i), 2'sd1));
                qacc[i] <= 3'sd0;
                // Full detent backward: emit step_minus and reset
              end else if (acc_next <= -4'sd4) begin
                step_minus[i] <= 1'b1;
                set_val_by_idx(i, sat_add1(get_val_by_idx(i), -2'sd1));
                qacc[i] <= 3'sd0;
              end
            end
          end

          ab_last[i] <= ab_now;
        end

        // External value override: allows host to set encoder value
        if (ext_we[i]) begin
          set_val_by_idx(i, get_ext_by_idx(i));
          qacc[i] <= 3'sd0;  // Reset accumulator for clean restart
        end
      end
    end
  end

endmodule

`default_nettype wire
