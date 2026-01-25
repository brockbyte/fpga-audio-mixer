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
 * Motor Fader Controller with 8-channel ADC and PWM output
 * Controls audio mixer faders with motorized feedback
 */
module mixer_top_8ch (
    // Clock and reset
    input wire clk,
    input wire rst,

    // Fader enable signals (1 bit per channel)
    input wire [7:0] faders_en,

    // SPI interface to MCP3008 ADC
    output wire spi_sck,
    output wire spi_mosi,
    output wire spi_cs,
    input  wire spi_miso,

    // Mixer interface: target positions from mixer (8x10-bit)
    input wire [79:0] targets_flat,

    // Actual fader positions feedback to mixer (8x10-bit)
    output wire [79:0] actuals_flat,

    // Motor control outputs: PWM and direction signals per channel
    output wire [7:0] motor_pwm,
    output wire [7:0] motor_up,
    output wire [7:0] motor_down
);

  // ================================================================
  // Boot counter and calibration trigger for initialization
  // ================================================================
  reg [27:0] boot_cnt;
  reg        c_trig;
  reg [ 2:0] c_ch;

  always @(posedge clk) begin
    if (rst) begin
      boot_cnt <= 28'd0;
      c_trig   <= 1'b0;
    end else begin
      if (boot_cnt < 5000000) begin
        boot_cnt <= boot_cnt + 28'd1;
        // Trigger calibration pulse between 1M and 4M cycles
        c_trig   <= (boot_cnt > 1000000 && boot_cnt < 4000000);
      end
    end
  end

  // ================================================================
  // MCP3008 ADC interface: round-robin sampling of 8 channels
  // ================================================================
  mcp3008_rr u_adc (
      .clk   (clk),
      .rst   (rst),
      .cs_n  (spi_cs),
      .sclk  (spi_sck),
      .mosi  (spi_mosi),
      .miso  (spi_miso),
      .sample(adc_sample_data),
      .ch_idx(adc_sample_ch),
      .strobe(adc_sample_valid)
  );

  wire    [9:0] adc_sample_data;
  wire    [2:0] adc_sample_ch;
  wire          adc_sample_valid;

  // ================================================================
  // ADC sample storage and buffering
  // Store raw ADC values per channel with valid flags
  // ================================================================
  reg     [9:0] adcs             [0:7];
  reg           adc_valid        [0:7];
  reg     [2:0] adc_ch;
  integer       ch;

  // Store ADC samples inverted (full scale = 0, no touch = 1023)
  always @(posedge clk) begin
    if (rst) begin
      for (ch = 0; ch < 8; ch = ch + 1) begin
        adcs[ch]      <= 10'd0;
        adc_valid[ch] <= 1'b0;
      end
      adc_ch <= 3'd0;
    end else begin
      adc_ch <= adc_sample_ch;

      // Clear all valid flags each cycle
      for (ch = 0; ch < 8; ch = ch + 1) begin
        adc_valid[ch] <= 1'b0;
      end

      // Store new sample and set valid flag for this channel
      if (adc_sample_valid) begin
        adcs[adc_sample_ch]      <= 10'd1023 - adc_sample_data;
        adc_valid[adc_sample_ch] <= 1'b1;
      end
    end
  end

  // ================================================================
  // Fader channel instantiation: generates 8 controller instances
  // Each instance controls one motor fader with position feedback
  // ================================================================
  genvar i;
  generate
    for (i = 0; i < 8; i = i + 1) begin : ch_gen
      fader_channel_logic_10b_int logic_inst (
          .clk         (clk),
          .rst         (rst),
          .fader_enable(faders_en[i]),
          .start_cal   (c_trig),
          .adc_valid   (adc_valid[7-i]),
          .adc_raw     (adcs[7-i]),
          .target_mixer(targets_flat[i*10+:10]),
          .actual_mixer(actuals_flat[i*10+:10]),
          .pwm         (motor_pwm[7-i]),
          .up          (motor_up[7-i]),
          .down        (motor_down[7-i])
      );
    end
  endgenerate

endmodule
