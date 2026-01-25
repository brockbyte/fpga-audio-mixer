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

// ============================================================================
// 16-Channel Stereo Mixer with Q2.14 Gain Coefficients (Pipelined Architecture)
//
// Audio Mixing Algorithm:
// 1) Mute/Solo masking: Each channel multiplied by audibility mask
// 2) Per-channel gains: 16 parallel Q2.14 multipliers (left & right)
// 3) Pipelined summation tree: 4 stages for timing closure at 125 MHz
//    - Stage 0: Register inputs
//    - Stage 1: 16 parallel multiply operations
//    - Stage 2: 4-way summation (groups of 4 channels)
//    - Stage 3: Final 4-to-1 sum + saturation + bus headroom scaling
// 4) Saturation: 24-bit output clipping with configurable headroom
// 5) Bus Headroom: Optional additional attenuation (-6 dB/bit) to prevent clipping
//
// Timing: 3-cycle latency due to pipelining
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module mixer_16x2_q14_pipelined #(
    parameter integer SAT_HEADROOM      = 0,  // Saturation-Reserve (wie bisher)
    parameter integer BUS_HEADROOM_BITS = 2   // Additional attenuation on bus (2 ≈ -12 dB)
) (
    input wire clk,
    input wire rst,

    input wire tick,
    input wire ch_valid, // 1-cycle pulse when new 16ch frame is available

    input wire [15:0] solo_button,
    input wire [15:0] mute_button,

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

    input wire signed [15:0] g0L,
    input wire signed [15:0] g0R,
    input wire signed [15:0] g1L,
    input wire signed [15:0] g1R,
    input wire signed [15:0] g2L,
    input wire signed [15:0] g2R,
    input wire signed [15:0] g3L,
    input wire signed [15:0] g3R,
    input wire signed [15:0] g4L,
    input wire signed [15:0] g4R,
    input wire signed [15:0] g5L,
    input wire signed [15:0] g5R,
    input wire signed [15:0] g6L,
    input wire signed [15:0] g6R,
    input wire signed [15:0] g7L,
    input wire signed [15:0] g7R,
    input wire signed [15:0] g8L,
    input wire signed [15:0] g8R,
    input wire signed [15:0] g9L,
    input wire signed [15:0] g9R,
    input wire signed [15:0] g10L,
    input wire signed [15:0] g10R,
    input wire signed [15:0] g11L,
    input wire signed [15:0] g11R,
    input wire signed [15:0] g12L,
    input wire signed [15:0] g12R,
    input wire signed [15:0] g13L,
    input wire signed [15:0] g13R,
    input wire signed [15:0] g14L,
    input wire signed [15:0] g14R,
    input wire signed [15:0] g15L,
    input wire signed [15:0] g15R,

    output reg signed [23:0] out_l,
    output reg signed [23:0] out_r,
    output reg               out_valid
);

  // -----------------------------------------------------------------------------
  // Mute/Solo masking
  // -----------------------------------------------------------------------------
  wire solo_active;
  assign solo_active = |solo_button;  // Check if any solo bit is set

  // Per-channel audibility: (no solo active OR channel soloed) AND not muted
  wire [15:0] ch_audible;

  genvar gi;
  generate
    for (gi = 0; gi < 16; gi = gi + 1) begin : GEN_AUDIBLE
      assign ch_audible[gi] = (solo_active ? solo_button[gi] : 1'b1) & ~mute_button[gi];
    end
  endgenerate

  wire signed [23:0] ch_in_eff[0:15];

  assign ch_in_eff[0]  = ch_audible[0] ? ch0 : 24'sd0;
  assign ch_in_eff[1]  = ch_audible[1] ? ch1 : 24'sd0;
  assign ch_in_eff[2]  = ch_audible[2] ? ch2 : 24'sd0;
  assign ch_in_eff[3]  = ch_audible[3] ? ch3 : 24'sd0;
  assign ch_in_eff[4]  = ch_audible[4] ? ch4 : 24'sd0;
  assign ch_in_eff[5]  = ch_audible[5] ? ch5 : 24'sd0;
  assign ch_in_eff[6]  = ch_audible[6] ? ch6 : 24'sd0;
  assign ch_in_eff[7]  = ch_audible[7] ? ch7 : 24'sd0;
  assign ch_in_eff[8]  = ch_audible[8] ? ch8 : 24'sd0;
  assign ch_in_eff[9]  = ch_audible[9] ? ch9 : 24'sd0;
  assign ch_in_eff[10] = ch_audible[10] ? ch10 : 24'sd0;
  assign ch_in_eff[11] = ch_audible[11] ? ch11 : 24'sd0;
  assign ch_in_eff[12] = ch_audible[12] ? ch12 : 24'sd0;
  assign ch_in_eff[13] = ch_audible[13] ? ch13 : 24'sd0;
  assign ch_in_eff[14] = ch_audible[14] ? ch14 : 24'sd0;
  assign ch_in_eff[15] = ch_audible[15] ? ch15 : 24'sd0;

  // ========================
  // Stage -1: Input staging
  // ========================
  reg signed [23:0] st0, st1, st2, st3, st4, st5, st6, st7;
  reg signed [23:0] st8, st9, st10, st11, st12, st13, st14, st15;
  reg have_sample;

  // optional debug
  reg signed [23:0] ch0_reg, ch1_reg;

  // Valid shift register for 3 pipeline stages
  reg [2:0] v_sr;

  always @(posedge clk) begin
    if (rst) begin
      st0 <= 24'd0;
      st1 <= 24'd0;
      st2 <= 24'd0;
      st3 <= 24'd0;
      st4 <= 24'd0;
      st5 <= 24'd0;
      st6 <= 24'd0;
      st7 <= 24'd0;
      st8 <= 24'd0;
      st9 <= 24'd0;
      st10 <= 24'd0;
      st11 <= 24'd0;
      st12 <= 24'd0;
      st13 <= 24'd0;
      st14 <= 24'd0;
      st15 <= 24'd0;
      have_sample <= 1'b0;
    end else begin
      // derzeit: Frame wird nach Verwendung "verbraucht"
      have_sample <= 1'b0;

      if (ch_valid) begin
        ch0_reg <= ch0;
        ch1_reg <= ch1;

        st0 <= ch_in_eff[0];
        st1 <= ch_in_eff[1];
        st2 <= ch_in_eff[2];
        st3 <= ch_in_eff[3];
        st4 <= ch_in_eff[4];
        st5 <= ch_in_eff[5];
        st6 <= ch_in_eff[6];
        st7 <= ch_in_eff[7];
        st8 <= ch_in_eff[8];
        st9 <= ch_in_eff[9];
        st10 <= ch_in_eff[10];
        st11 <= ch_in_eff[11];
        st12 <= ch_in_eff[12];
        st13 <= ch_in_eff[13];
        st14 <= ch_in_eff[14];
        st15 <= ch_in_eff[15];

        have_sample <= 1'b1;
      end
    end
  end

  always @(posedge clk) begin
    if (rst) begin
      v_sr <= 3'b000;
    end else begin
      v_sr <= {v_sr[1:0], have_sample};
    end
  end

  // ========================
  // STAGE 0 → 1: register inputs for multiplies
  // ========================
  reg signed [23:0] s0, s1, s2, s3, s4, s5, s6, s7;
  reg signed [23:0] s8, s9, s10, s11, s12, s13, s14, s15;
  reg signed [15:0]
      k0L, k1L, k2L, k3L, k4L, k5L, k6L, k7L, k8L, k9L, k10L, k11L, k12L, k13L, k14L, k15L;
  reg signed [15:0]
      k0R, k1R, k2R, k3R, k4R, k5R, k6R, k7R, k8R, k9R, k10R, k11R, k12R, k13R, k14R, k15R;

  always @(posedge clk) begin
    if (rst) begin
      s0   <= 0;
      s1   <= 0;
      s2   <= 0;
      s3   <= 0;
      s4   <= 0;
      s5   <= 0;
      s6   <= 0;
      s7   <= 0;
      s8   <= 0;
      s9   <= 0;
      s10  <= 0;
      s11  <= 0;
      s12  <= 0;
      s13  <= 0;
      s14  <= 0;
      s15  <= 0;
      k0L  <= 0;
      k1L  <= 0;
      k2L  <= 0;
      k3L  <= 0;
      k4L  <= 0;
      k5L  <= 0;
      k6L  <= 0;
      k7L  <= 0;
      k8L  <= 0;
      k9L  <= 0;
      k10L <= 0;
      k11L <= 0;
      k12L <= 0;
      k13L <= 0;
      k14L <= 0;
      k15L <= 0;
      k0R  <= 0;
      k1R  <= 0;
      k2R  <= 0;
      k3R  <= 0;
      k4R  <= 0;
      k5R  <= 0;
      k6R  <= 0;
      k7R  <= 0;
      k8R  <= 0;
      k9R  <= 0;
      k10R <= 0;
      k11R <= 0;
      k12R <= 0;
      k13R <= 0;
      k14R <= 0;
      k15R <= 0;
    end else if (have_sample) begin
      s0   <= st0;
      s1   <= st1;
      s2   <= st2;
      s3   <= st3;
      s4   <= st4;
      s5   <= st5;
      s6   <= st6;
      s7   <= st7;
      s8   <= st8;
      s9   <= st9;
      s10  <= st10;
      s11  <= st11;
      s12  <= st12;
      s13  <= st13;
      s14  <= st14;
      s15  <= st15;

      k0L  <= g0L;
      k1L  <= g1L;
      k2L  <= g2L;
      k3L  <= g3L;
      k4L  <= g4L;
      k5L  <= g5L;
      k6L  <= g6L;
      k7L  <= g7L;
      k8L  <= g8L;
      k9L  <= g9L;
      k10L <= g10L;
      k11L <= g11L;
      k12L <= g12L;
      k13L <= g13L;
      k14L <= g14L;
      k15L <= g15L;

      k0R  <= g0R;
      k1R  <= g1R;
      k2R  <= g2R;
      k3R  <= g3R;
      k4R  <= g4R;
      k5R  <= g5R;
      k6R  <= g6R;
      k7R  <= g7R;
      k8R  <= g8R;
      k9R  <= g9R;
      k10R <= g10R;
      k11R <= g11R;
      k12R <= g12R;
      k13R <= g13R;
      k14R <= g14R;
      k15R <= g15R;
    end
  end

  // ========================
  // STAGE 1: 16 multiplies per side, register products
  // ========================
  reg signed [39:0]
      pL0, pL1, pL2, pL3, pL4, pL5, pL6, pL7, pL8, pL9, pL10, pL11, pL12, pL13, pL14, pL15;
  reg signed [39:0]
      pR0, pR1, pR2, pR3, pR4, pR5, pR6, pR7, pR8, pR9, pR10, pR11, pR12, pR13, pR14, pR15;

  always @(posedge clk) begin
    if (rst) begin
      pL0  <= 0;
      pL1  <= 0;
      pL2  <= 0;
      pL3  <= 0;
      pL4  <= 0;
      pL5  <= 0;
      pL6  <= 0;
      pL7  <= 0;
      pL8  <= 0;
      pL9  <= 0;
      pL10 <= 0;
      pL11 <= 0;
      pL12 <= 0;
      pL13 <= 0;
      pL14 <= 0;
      pL15 <= 0;
      pR0  <= 0;
      pR1  <= 0;
      pR2  <= 0;
      pR3  <= 0;
      pR4  <= 0;
      pR5  <= 0;
      pR6  <= 0;
      pR7  <= 0;
      pR8  <= 0;
      pR9  <= 0;
      pR10 <= 0;
      pR11 <= 0;
      pR12 <= 0;
      pR13 <= 0;
      pR14 <= 0;
      pR15 <= 0;
    end else begin
      if (v_sr[0]) begin
        pL0  <= $signed(s0) * $signed(k0L);
        pL1  <= $signed(s1) * $signed(k1L);
        pL2  <= $signed(s2) * $signed(k2L);
        pL3  <= $signed(s3) * $signed(k3L);
        pL4  <= $signed(s4) * $signed(k4L);
        pL5  <= $signed(s5) * $signed(k5L);
        pL6  <= $signed(s6) * $signed(k6L);
        pL7  <= $signed(s7) * $signed(k7L);
        pL8  <= $signed(s8) * $signed(k8L);
        pL9  <= $signed(s9) * $signed(k9L);
        pL10 <= $signed(s10) * $signed(k10L);
        pL11 <= $signed(s11) * $signed(k11L);
        pL12 <= $signed(s12) * $signed(k12L);
        pL13 <= $signed(s13) * $signed(k13L);
        pL14 <= $signed(s14) * $signed(k14L);
        pL15 <= $signed(s15) * $signed(k15L);

        pR0  <= $signed(s0) * $signed(k0R);
        pR1  <= $signed(s1) * $signed(k1R);
        pR2  <= $signed(s2) * $signed(k2R);
        pR3  <= $signed(s3) * $signed(k3R);
        pR4  <= $signed(s4) * $signed(k4R);
        pR5  <= $signed(s5) * $signed(k5R);
        pR6  <= $signed(s6) * $signed(k6R);
        pR7  <= $signed(s7) * $signed(k7R);
        pR8  <= $signed(s8) * $signed(k8R);
        pR9  <= $signed(s9) * $signed(k9R);
        pR10 <= $signed(s10) * $signed(k10R);
        pR11 <= $signed(s11) * $signed(k11R);
        pR12 <= $signed(s12) * $signed(k12R);
        pR13 <= $signed(s13) * $signed(k13R);
        pR14 <= $signed(s14) * $signed(k14R);
        pR15 <= $signed(s15) * $signed(k15R);
      end
    end
  end

  // ========================
  // STAGE 2: add tree level 1 (groups of 4), register
  // ========================
  reg signed [41:0] aL0, aL1, aL2, aL3;  // +2 bits headroom
  reg signed [41:0] aR0, aR1, aR2, aR3;

  always @(posedge clk) begin
    if (rst) begin
      aL0 <= 0;
      aL1 <= 0;
      aL2 <= 0;
      aL3 <= 0;
      aR0 <= 0;
      aR1 <= 0;
      aR2 <= 0;
      aR3 <= 0;
    end else if (v_sr[1]) begin
      aL0 <= $signed(
          {{2{pL0[39]}}, pL0}
      ) + $signed(
          {{2{pL1[39]}}, pL1}
      ) + $signed(
          {{2{pL2[39]}}, pL2}
      ) + $signed(
          {{2{pL3[39]}}, pL3}
      );
      aL1 <= $signed(
          {{2{pL4[39]}}, pL4}
      ) + $signed(
          {{2{pL5[39]}}, pL5}
      ) + $signed(
          {{2{pL6[39]}}, pL6}
      ) + $signed(
          {{2{pL7[39]}}, pL7}
      );
      aL2 <= $signed(
          {{2{pL8[39]}}, pL8}
      ) + $signed(
          {{2{pL9[39]}}, pL9}
      ) + $signed(
          {{2{pL10[39]}}, pL10}
      ) + $signed(
          {{2{pL11[39]}}, pL11}
      );
      aL3 <= $signed(
          {{2{pL12[39]}}, pL12}
      ) + $signed(
          {{2{pL13[39]}}, pL13}
      ) + $signed(
          {{2{pL14[39]}}, pL14}
      ) + $signed(
          {{2{pL15[39]}}, pL15}
      );

      aR0 <= $signed(
          {{2{pR0[39]}}, pR0}
      ) + $signed(
          {{2{pR1[39]}}, pR1}
      ) + $signed(
          {{2{pR2[39]}}, pR2}
      ) + $signed(
          {{2{pR3[39]}}, pR3}
      );
      aR1 <= $signed(
          {{2{pR4[39]}}, pR4}
      ) + $signed(
          {{2{pR5[39]}}, pR5}
      ) + $signed(
          {{2{pR6[39]}}, pR6}
      ) + $signed(
          {{2{pR7[39]}}, pR7}
      );
      aR2 <= $signed(
          {{2{pR8[39]}}, pR8}
      ) + $signed(
          {{2{pR9[39]}}, pR9}
      ) + $signed(
          {{2{pR10[39]}}, pR10}
      ) + $signed(
          {{2{pR11[39]}}, pR11}
      );
      aR3 <= $signed(
          {{2{pR12[39]}}, pR12}
      ) + $signed(
          {{2{pR13[39]}}, pR13}
      ) + $signed(
          {{2{pR14[39]}}, pR14}
      ) + $signed(
          {{2{pR15[39]}}, pR15}
      );
    end
  end

  // ========================
  // STAGE 3: final sum, scale, saturate, register output
  // ========================
  wire signed [43:0] bL = $signed(
      {{2{aL0[41]}}, aL0}
  ) + $signed(
      {{2{aL1[41]}}, aL1}
  ) + $signed(
      {{2{aL2[41]}}, aL2}
  ) + $signed(
      {{2{aL3[41]}}, aL3}
  );
  wire signed [43:0] bR = $signed(
      {{2{aR0[41]}}, aR0}
  ) + $signed(
      {{2{aR1[41]}}, aR1}
  ) + $signed(
      {{2{aR2[41]}}, aR2}
  ) + $signed(
      {{2{aR3[41]}}, aR3}
  );

  // Q2.14 scale + additional bus headroom attenuation
  // 14 = Fractionalbits der Q2.14-Gains
  // BUS_HEADROOM_BITS = additional global attenuation (per bit approx -6 dB)
  wire signed [43:0] mixL_q = bL >>> (14 + BUS_HEADROOM_BITS);
  wire signed [43:0] mixR_q = bR >>> (14 + BUS_HEADROOM_BITS);

  // Saturation helpers
  localparam signed [31:0] POS_MAX = 32'sh007FFFFF;
  localparam signed [31:0] NEG_MIN = -32'sh00800000;
  localparam signed [31:0] POS_MAX_HR = (SAT_HEADROOM == 0) ? POS_MAX : (POS_MAX >>> SAT_HEADROOM);
  localparam signed [31:0] NEG_MIN_HR = (SAT_HEADROOM == 0) ? NEG_MIN : (NEG_MIN >>> SAT_HEADROOM);

  function [23:0] sat24_hr;
    input signed [43:0] v;
    reg signed [31:0] vv;
    begin
      if (v > POS_MAX_HR) vv = POS_MAX_HR;
      else if (v < NEG_MIN_HR) vv = NEG_MIN_HR;
      else vv = v[31:0];
      sat24_hr = vv[23:0];
    end
  endfunction

  always @(posedge clk) begin
    if (rst) begin
      out_l     <= 24'd0;
      out_r     <= 24'd0;
      out_valid <= 1'b0;
    end else begin
      out_valid <= v_sr[2];  // Third pipeline stage

      if (v_sr[2]) begin
        out_l <= sat24_hr(mixL_q);
        out_r <= sat24_hr(mixR_q);
      end
    end
  end

endmodule

`resetall
`default_nettype wire
