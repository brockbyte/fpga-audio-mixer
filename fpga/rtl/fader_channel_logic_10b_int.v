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
 * Motor Fader Channel Controller with PI Servo Control
 *
 * Features:
 * - 10-bit ADC position sensing with 4-sample moving average filter
 * - Auto-calibration sequence (find min/max travel positions)
 * - Iterative position scaling (avoids hardware divider)
 * - Stable PI controller with integrator freeze and anti-windup
 * - Soft-landing approach near target (no minimum PWM)
 * - Slew-rate limited PWM output for smooth transitions
 * - 12-bit PWM generator for precise motor control
 *
 * Control Algorithm:
 * 1. ADC filtering: 4-sample moving average at 1kHz
 * 2. Calibration: Auto-detect fader travel range (up then down)
 * 3. Position scaling: Normalize raw ADC to 0-1023 range
 * 4. PI servo: Track target position with anti-windup protection
 * 5. PWM generation: Variable duty cycle motor control
 */
`timescale 1ns / 1ps
`default_nettype none

module fader_channel_logic_10b_int (
    input wire clk,
    input wire rst,

    // ADC interface (1kHz update rate)
    input wire       adc_valid,  // 1-cycle pulse (e.g., every 1 ms)
    input wire [9:0] adc_raw,    // 10-bit ADC value

    // Enable and calibration control
    input wire fader_enable,  // Enable motor control
    input wire start_cal,     // Trigger calibration sequence

    // Mixer interface
    input  wire [9:0] target_mixer,  // Target position from mixer (0-1023)
    output wire [9:0] actual_mixer,  // Actual fader position to mixer (0-1023)

    // Motor outputs
    output wire pwm,  // PWM signal for motor
    output wire up,   // Motor direction: up
    output wire down  // Motor direction: down
);

  // ================================================================
  // Control parameters (10-bit domain)
  // ================================================================
  localparam [9:0] STABLE_EPS10 = 10'd1;  // Position stability threshold
  localparam [7:0] STABLE_NEED = 8'd50;  // Cycles needed to consider stable

  localparam signed [10:0] START_DB10 = 11'sd16;  // Hysteresis: start moving
  localparam signed [10:0] STOP_DB10 = 11'sd7;  // Hysteresis: stop moving

  localparam [11:0] PWM_MIN = 12'd600;  // Minimum PWM (overcome friction)
  localparam [11:0] PWM_MAX = 12'd2450;  // Maximum PWM
  localparam [11:0] SLEW_STEP = 12'd8;  // PWM slew rate limit

  localparam [11:0] PWM_PERIOD = 12'd2499;  // PWM period (clk cycles)

  // ================================================================
  // Section 1: ADC filtering (4-sample moving average)
  // ================================================================
  reg     [ 9:0] history                       [0:3];
  reg     [12:0] sum;
  reg     [ 9:0] f_pos10;  // Filtered position
  integer        i;

  always @(posedge clk) begin
    if (rst) begin
      for (i = 0; i < 4; i = i + 1) history[i] <= 10'd0;
      sum     <= 13'd0;
      f_pos10 <= 10'd0;
    end else if (adc_valid) begin
      sum <= sum - history[3] + adc_raw;
      for (i = 3; i > 0; i = i - 1) history[i] <= history[i-1];
      history[0] <= adc_raw;
      f_pos10    <= sum >> 2;  // Divide by 4
    end
  end

  // ================================================================
  // Section 2: Calibration FSM (find min/max positions)
  // Sequence: UP first (find max), then DOWN (find min)
  // ================================================================
  reg [9:0] min_v10, max_v10;

  reg c_busy, c_en, c_dir;
  reg         [11:0] c_pwm;
  reg         [ 2:0] c_st;
  reg         [ 9:0] l_pos10;
  reg         [ 7:0] stable_cnt;

  wire signed [10:0] d_pos10 = $signed({1'b0, f_pos10}) - $signed({1'b0, l_pos10});
  wire        [10:0] ad_pos10 = d_pos10[10] ? (~d_pos10 + 11'd1) : d_pos10;

  always @(posedge clk) begin
    if (rst) begin
      c_st       <= 3'd0;
      c_busy     <= 1'b0;
      c_en       <= 1'b0;
      c_dir      <= 1'b0;
      c_pwm      <= 12'd0;
      l_pos10    <= 10'd0;
      stable_cnt <= 7'd0;
      min_v10    <= 10'd0;
      max_v10    <= 10'd1023;
    end else if (adc_valid) begin
      case (c_st)
        // State 0: IDLE - Wait for calibration trigger
        3'd0: begin
          c_busy <= 1'b0;
          c_en <= 1'b0;
          stable_cnt <= 7'd0;
          if (start_cal) begin
            c_busy  <= 1'b1;
            c_dir   <= 1'b1;  // UP direction
            c_pwm   <= 12'd1800;
            c_en    <= 1'b1;
            l_pos10 <= f_pos10;
            c_st    <= 3'd1;
          end
        end

        // State 1: UP - Move to maximum position
        3'd1: begin
          c_busy <= 1'b1;
          c_en   <= 1'b1;
          c_dir  <= 1'b1;

          if (ad_pos10 <= STABLE_EPS10) stable_cnt <= stable_cnt + 1'b1;
          else stable_cnt <= 7'd0;

          l_pos10 <= f_pos10;

          if (stable_cnt >= STABLE_NEED - 1) begin
            max_v10    <= f_pos10;
            stable_cnt <= 7'd0;
            c_dir      <= 1'b0;  // Switch to DOWN
            c_st       <= 3'd2;
          end
        end

        // State 2: DOWN - Move to minimum position
        3'd2: begin
          c_busy <= 1'b1;
          c_en   <= 1'b1;
          c_dir  <= 1'b0;

          if (ad_pos10 <= STABLE_EPS10) stable_cnt <= stable_cnt + 1'b1;
          else stable_cnt <= 7'd0;

          l_pos10 <= f_pos10;

          if (stable_cnt >= STABLE_NEED - 1) begin
            min_v10 <= f_pos10;
            c_st    <= 3'd3;
          end
        end

        // State 3: DONE - Calibration complete
        3'd3: begin
          c_busy <= 1'b0;
          c_en   <= 1'b0;
          c_st   <= 3'd0;
        end
      endcase

      // Ensure valid range (min < max)
      if (max_v10 <= min_v10 + 10'd2) max_v10 <= min_v10 + 10'd2;
    end
  end

  // ================================================================
  // Section 3: Position scaling and normalization
  // Iterative algorithm maps raw ADC (min_v10..max_v10) to (0..1023)
  // Avoids hardware divider by using accumulator approach
  // ================================================================
  reg  [ 9:0] cur_pos_10b;
  reg  [ 9:0] pos;
  reg  [20:0] accu;
  reg  [ 9:0] cnt;
  reg  [ 9:0] src;
  wire [ 9:0] span10 = (max_v10 > min_v10) ? (max_v10 - min_v10) : 10'd1;

  always @(posedge clk) begin
    if (rst) begin
      cur_pos_10b <= 10'd0;
      pos <= 10'd0;
      accu <= 21'd0;
      cnt <= 10'd0;
      src <= 10'd0;
    end else begin
      // Check for new ADC sample: start scaling calculation
      if (adc_valid && (cnt == 10'd0)) begin
        if (f_pos10 <= min_v10 + 10'd2) begin
          // At or below minimum: output 0
          cur_pos_10b <= 10'd0;
        end else if (f_pos10 >= max_v10 - 10'd2) begin
          // At or above maximum: output full scale (1023)
          cur_pos_10b <= 10'd1023;
        end else begin
          // Within range: perform iterative normalization
          src  <= f_pos10 - min_v10;
          pos  <= 10'd0;
          accu <= 21'd0;
          cnt  <= 10'd1023;  // Process 0..1023
        end
      end else if (cnt != 10'd0) begin
        // Iterative scaling: accumulate (src) and count up when threshold met
        if (accu + src >= span10) begin
          accu <= (accu + src) - span10;
          pos  <= pos + 1'b1;
        end else begin
          accu <= accu + src;
        end
        cnt <= cnt - 1'b1;

        // Store final result when done
        if (cnt == 10'd1) cur_pos_10b <= pos;
      end
    end
  end

  wire [9:0] m_target;

  // Direct passthrough: mixer target and actual position
  assign m_target = target_mixer;
  assign actual_mixer = cur_pos_10b;

  wire        [19:0] mul_target10 = m_target * span10;
  wire        [ 9:0] s_target10 = min_v10 + (mul_target10 >> 10);

  // ================================================================
  // Section 4: PI Servo Controller with Anti-Windup
  //
  // Control strategy:
  // - Proportional (P): Direct error multiplication (Kp=2)
  // - Integral (I): Slow accumulation via bit shift (Ki = 1/64)
  // - Freeze: Integrator locked near target or when motor disabled
  // - Anti-windup: Integrator clamped [-8000, +8000]
  // - Soft-landing: No minimum PWM near setpoint
  // - Updates: Only on adc_valid for 1kHz control loop
  // ================================================================
  reg                moving;
  reg                s_en;
  reg                s_dir;

  reg signed  [21:0] i_acc;  // Integrator accumulator
  reg signed  [21:0] u_cmd;  // PI output command
  reg         [11:0] s_pwm_cmd;
  reg         [11:0] s_pwm_slewed;

  // Error in 10-bit domain
  wire signed [10:0] err10 = $signed({1'b0, s_target10}) - $signed({1'b0, f_pos10});

  wire        [10:0] abs_err10 = err10[10] ? (~err10 + 11'd1) : err10;

  // PI controller gains
  localparam integer KP = 2;  // Proportional gain

  // Ki as shift: I += err / 2^KI_SHIFT per sample (1kHz)
  localparam integer KI_SHIFT = 6;  // Ki = 1/64 per sample

  // Integrator clamp (prevents windup)
  localparam signed [21:0] I_MAX = 22'sd8000;

  // Soft landing region (within this, allow PWM_MIN=0)
  localparam [10:0] SOFT_DB10 = 11'd35;
  localparam [11:0] PWM_MAX_SOFT = 12'd1100;

  always @(posedge clk) begin
    if (rst) begin
      moving    <= 1'b0;
      s_en      <= 1'b0;
      s_dir     <= 1'b0;
      i_acc     <= 22'sd0;
      u_cmd     <= 22'sd0;
      s_pwm_cmd <= 12'd0;
    end else if (adc_valid) begin
      // Local temporaries for consistent math
      integer           abs_i;
      reg signed [21:0] p_term;
      reg signed [21:0] i_next;
      reg signed [21:0] u_next;
      reg signed [21:0] u_abs;
      reg               freeze_i;
      reg               en_next;
      reg               dir_next;
      reg        [11:0] pwm_min_eff;
      reg        [11:0] pwm_max_eff;

      // Start/stop hysteresis (only on adc_valid)
      if (!moving) begin
        if ($signed({1'b0, abs_err10}) > START_DB10) moving <= 1'b1;
      end else begin
        if ($signed({1'b0, abs_err10}) < STOP_DB10) moving <= 1'b0;
      end

      en_next  = moving && fader_enable && !c_busy;
      dir_next = (err10 > 0);

      s_en  <= en_next;
      s_dir <= dir_next;

      // Effective PWM limits (soft landing near target)
      if (abs_err10 <= SOFT_DB10) pwm_max_eff = PWM_MAX_SOFT;
      else pwm_max_eff = PWM_MAX;

      if (abs_err10 <= SOFT_DB10) pwm_min_eff = 12'd0;
      else pwm_min_eff = PWM_MIN;

      // Proportional term
      p_term   = $signed(err10) * KP;

      // Anti-windup: freeze integrator in stop zone or when disabled
      freeze_i = (abs_err10 <= STOP_DB10) || !en_next || c_busy;

      // Calculate next integrator value
      i_next   = i_acc;
      if (!freeze_i) begin
        // Accumulate error step (divided by 2^KI_SHIFT = 64)
        i_next = i_acc + ($signed(err10) >>> KI_SHIFT);

        // Clamp integrator to prevent excessive wind-up
        if (i_next > I_MAX) i_next = I_MAX;
        if (i_next < -I_MAX) i_next = -I_MAX;
      end

      // Integrator bleed: reduce by half on zero-crossing
      if ((err10 > 0 && u_cmd < 0) || (err10 < 0 && u_cmd > 0)) begin
        i_next = i_next >>> 1;  // Divide by 2
      end

      // PI sum
      u_next = p_term + i_next;

      // Save integrator and command for next cycle
      i_acc <= i_next;
      u_cmd <= u_next;

      // Map control signal to PWM duty cycle with saturation
      if (!en_next) begin
        s_pwm_cmd <= 12'd0;
      end else begin
        u_abs = (u_next < 0) ? -u_next : u_next;

        // Clamp to max
        if (u_abs[21:0] > $signed({1'b0, pwm_max_eff})) s_pwm_cmd <= pwm_max_eff;
        else begin
          // Apply effective min only if nonzero command
          if (u_abs[21:0] < $signed({1'b0, pwm_min_eff})) s_pwm_cmd <= pwm_min_eff;
          else s_pwm_cmd <= u_abs[11:0];
        end
      end
    end
  end

  // ================================================================
  // Section 5: PWM slew-rate limiter (runs on clk)
  // ================================================================
  always @(posedge clk) begin
    if (rst) begin
      s_pwm_slewed <= 12'd0;
    end else begin
      if (s_pwm_slewed < s_pwm_cmd)
        s_pwm_slewed <= s_pwm_slewed +
            ((s_pwm_cmd - s_pwm_slewed > SLEW_STEP) ?
            SLEW_STEP : (s_pwm_cmd - s_pwm_slewed));
      else if (s_pwm_slewed > s_pwm_cmd)
        s_pwm_slewed <= s_pwm_slewed -
            ((s_pwm_slewed - s_pwm_cmd > SLEW_STEP) ?
            SLEW_STEP : (s_pwm_slewed - s_pwm_cmd));
    end
  end

  // ================================================================
  // Section 6: PWM generator
  // ================================================================
  reg [11:0] p_cnt;

  always @(posedge clk) begin
    if (rst) p_cnt <= 12'd0;
    else p_cnt <= (p_cnt >= PWM_PERIOD) ? 12'd0 : p_cnt + 1'b1;
  end

  // Mux between calibration and servo control
  wire        a_en = c_busy ? c_en : s_en;
  wire        a_dir = c_busy ? c_dir : s_dir;
  wire [11:0] a_duty = c_busy ? c_pwm : s_pwm_slewed;

  assign pwm  = a_en && (p_cnt < a_duty);
  assign up   = a_en && a_dir;
  assign down = a_en && !a_dir;

endmodule

`default_nettype wire
