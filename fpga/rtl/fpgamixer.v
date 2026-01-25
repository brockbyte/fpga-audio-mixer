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

// Language: Verilog 2001

`resetall
`timescale 1ns / 1ps
`default_nettype none

/*
 * FPGA top-level module
 * Instantiates PLL, reset synchronizers, debouncer, and FPGA core logic
 */
module fpgamixer (
    // ===== Clocks and Reset =====
    // 125MHz Ethernet clock
    input wire enet_clk_125m,
    // 50MHz clock input for audio PLL
    input wire c10_clk50m,
    // Reset button (active low)
    input wire c10_resetn,

    // ===== Audio Output =====
    output wire spdif_out,

    // ===== User GPIO (Push buttons, DIP switches, LEDs) =====
    input  wire [3:0] user_pb,
    input  wire [2:0] user_dip,
    output wire [3:0] user_led,

    // ===== I2C Interfaces =====
    // Main I2C bus (shared)
    inout wire i2c_sda,
    inout wire i2c_scl,

    // I2C for MCP23017 buttons/LEDs
    inout wire i2c_sda_t,
    inout wire i2c_scl_t,

    // I2C for MPR121 touch sensors
    inout wire i2c_sda_f,
    inout wire i2c_scl_f,

    // ===== Rotary Encoders =====
    input wire [7:0] enc_a,
    input wire [7:0] enc_b,

    // ===== MCP3008 SPI Interface (Fader ADCs) =====
    output wire adc_cs_n,
    output wire adc_sclk,
    output wire adc_mosi,
    input  wire adc_miso,

    // ===== TB6612 Motor Driver Control =====
    output wire [7:0] m_in1,
    output wire [7:0] m_in2,
    output wire [7:0] m_pwm,
    output wire       tb_stby,

    // ===== Ethernet: 1000BASE-T RGMII Interface =====
    input  wire       enet_rx_clk,
    input  wire [3:0] enet_rx_d,
    input  wire       enet_rx_dv,
    output wire       enet_tx_clk,
    output wire [3:0] enet_tx_d,
    output wire       enet_tx_en,
    output wire       enet_resetn,
    input  wire       enet_int
);

  // ================================================================
  // Clock Generation and Reset Synchronization
  // ================================================================

  // Internal 125MHz Ethernet clock and 90-degree phase for RGMII
  wire clk_int;
  wire clk90_int;
  wire rst_int;

  // Audio domain clock (50MHz and 12.288MHz)
  wire clk_50m;
  wire clk_12m288;
  wire audio_rst_int;

  // PLL control signals
  wire pll_rst;
  wire pll_locked;
  wire audio_pll_locked;

  assign pll_rst = ~c10_resetn;

  // ================================================================
  // PLL 1: 125MHz Ethernet clock with 90-degree phase shift
  // ================================================================
  altpll #(
      .bandwidth_type         ("AUTO"),
      .clk0_divide_by         (1),
      .clk0_duty_cycle        (50),
      .clk0_multiply_by       (1),
      .clk0_phase_shift       ("0"),
      .clk1_divide_by         (1),
      .clk1_duty_cycle        (50),
      .clk1_multiply_by       (1),
      .clk1_phase_shift       ("2000"),
      .compensate_clock       ("CLK0"),
      .inclk0_input_frequency (8000),
      .intended_device_family ("Cyclone 10 LP"),
      .operation_mode         ("NORMAL"),
      .pll_type               ("AUTO"),
      .port_activeclock       ("PORT_UNUSED"),
      .port_areset            ("PORT_USED"),
      .port_clkbad0           ("PORT_UNUSED"),
      .port_clkbad1           ("PORT_UNUSED"),
      .port_clkloss           ("PORT_UNUSED"),
      .port_clkswitch         ("PORT_UNUSED"),
      .port_configupdate      ("PORT_UNUSED"),
      .port_fbin              ("PORT_UNUSED"),
      .port_inclk0            ("PORT_USED"),
      .port_inclk1            ("PORT_UNUSED"),
      .port_locked            ("PORT_USED"),
      .port_pfdena            ("PORT_UNUSED"),
      .port_phasecounterselect("PORT_UNUSED"),
      .port_phasedone         ("PORT_UNUSED"),
      .port_phasestep         ("PORT_UNUSED"),
      .port_phaseupdown       ("PORT_UNUSED"),
      .port_pllena            ("PORT_UNUSED"),
      .port_scanaclr          ("PORT_UNUSED"),
      .port_scanclk           ("PORT_UNUSED"),
      .port_scanclkena        ("PORT_UNUSED"),
      .port_scandata          ("PORT_UNUSED"),
      .port_scandataout       ("PORT_UNUSED"),
      .port_scandone          ("PORT_UNUSED"),
      .port_scanread          ("PORT_UNUSED"),
      .port_scanwrite         ("PORT_UNUSED"),
      .port_clk0              ("PORT_USED"),
      .port_clk1              ("PORT_USED"),
      .port_clk2              ("PORT_UNUSED"),
      .port_clk3              ("PORT_UNUSED"),
      .port_clk4              ("PORT_UNUSED"),
      .port_clk5              ("PORT_UNUSED"),
      .port_clkena0           ("PORT_UNUSED"),
      .port_clkena1           ("PORT_UNUSED"),
      .port_clkena2           ("PORT_UNUSED"),
      .port_clkena3           ("PORT_UNUSED"),
      .port_clkena4           ("PORT_UNUSED"),
      .port_clkena5           ("PORT_UNUSED"),
      .port_extclk0           ("PORT_UNUSED"),
      .port_extclk1           ("PORT_UNUSED"),
      .port_extclk2           ("PORT_UNUSED"),
      .port_extclk3           ("PORT_UNUSED"),
      .self_reset_on_loss_lock("ON"),
      .width_clock            (5)
  ) altpll_component (
      .areset(pll_rst),
      .inclk({1'b0, enet_clk_125m}),
      .clk({clk90_int, clk_int}),
      .locked(pll_locked),
      .activeclock(),
      .clkbad(),
      .clkena({6{1'b1}}),
      .clkloss(),
      .clkswitch(1'b0),
      .configupdate(1'b0),
      .enable0(),
      .enable1(),
      .extclk(),
      .extclkena({4{1'b1}}),
      .fbin(1'b1),
      .fbmimicbidir(),
      .fbout(),
      .fref(),
      .icdrclk(),
      .pfdena(1'b1),
      .phasecounterselect({4{1'b1}}),
      .phasedone(),
      .phasestep(1'b1),
      .phaseupdown(1'b1),
      .pllena(1'b1),
      .scanaclr(1'b0),
      .scanclk(1'b0),
      .scanclkena(1'b1),
      .scandata(1'b0),
      .scandataout(),
      .scandone(),
      .scanread(1'b0),
      .scanwrite(1'b0),
      .sclkout0(),
      .sclkout1(),
      .vcooverrange(),
      .vcounderrange()
  );

  // ================================================================
  // PLL 2: Audio domain (12.288MHz for SPDIF, 50MHz for processing)
  // ================================================================
  altpll #(
      .bandwidth_type         ("AUTO"),
      .clk0_divide_by         (1),
      .clk0_duty_cycle        (50),
      .clk0_multiply_by       (1),
      .clk1_divide_by         (3125),
      .clk1_duty_cycle        (50),
      .clk1_multiply_by       (768),
      .compensate_clock       ("CLK0"),
      .inclk0_input_frequency (20000),
      .intended_device_family ("Cyclone 10 LP"),
      .operation_mode         ("NORMAL"),
      .pll_type               ("AUTO"),
      .port_activeclock       ("PORT_UNUSED"),
      .port_areset            ("PORT_USED"),
      .port_clkbad0           ("PORT_UNUSED"),
      .port_clkbad1           ("PORT_UNUSED"),
      .port_clkloss           ("PORT_UNUSED"),
      .port_clkswitch         ("PORT_UNUSED"),
      .port_configupdate      ("PORT_UNUSED"),
      .port_fbin              ("PORT_UNUSED"),
      .port_inclk0            ("PORT_USED"),
      .port_inclk1            ("PORT_UNUSED"),
      .port_locked            ("PORT_USED"),
      .port_pfdena            ("PORT_UNUSED"),
      .port_phasecounterselect("PORT_UNUSED"),
      .port_phasedone         ("PORT_UNUSED"),
      .port_phasestep         ("PORT_UNUSED"),
      .port_phaseupdown       ("PORT_UNUSED"),
      .port_pllena            ("PORT_UNUSED"),
      .port_scanaclr          ("PORT_UNUSED"),
      .port_scanclk           ("PORT_UNUSED"),
      .port_scanclkena        ("PORT_UNUSED"),
      .port_scandata          ("PORT_UNUSED"),
      .port_scandataout       ("PORT_UNUSED"),
      .port_scandone          ("PORT_UNUSED"),
      .port_scanread          ("PORT_UNUSED"),
      .port_scanwrite         ("PORT_UNUSED"),
      .port_clk0              ("PORT_USED"),
      .port_clk1              ("PORT_USED"),
      .port_clk2              ("PORT_UNUSED"),
      .port_clk3              ("PORT_UNUSED"),
      .port_clk4              ("PORT_UNUSED"),
      .port_clk5              ("PORT_UNUSED"),
      .port_clkena0           ("PORT_UNUSED"),
      .port_clkena1           ("PORT_UNUSED"),
      .port_clkena2           ("PORT_UNUSED"),
      .port_clkena3           ("PORT_UNUSED"),
      .port_clkena4           ("PORT_UNUSED"),
      .port_clkena5           ("PORT_UNUSED"),
      .port_extclk0           ("PORT_UNUSED"),
      .port_extclk1           ("PORT_UNUSED"),
      .port_extclk2           ("PORT_UNUSED"),
      .port_extclk3           ("PORT_UNUSED"),
      .self_reset_on_loss_lock("ON"),
      .width_clock            (5)
  ) altpll_component_audio (
      .areset(pll_rst),
      .inclk({1'b0, c10_clk50m}),
      .clk({clk_12m288, clk_50m}),
      .locked(audio_pll_locked),
      .activeclock(),
      .clkbad(),
      .clkena({6{1'b1}}),
      .clkloss(),
      .clkswitch(1'b0),
      .configupdate(1'b0),
      .enable0(),
      .enable1(),
      .extclk(),
      .extclkena({4{1'b1}}),
      .fbin(1'b1),
      .fbmimicbidir(),
      .fbout(),
      .fref(),
      .icdrclk(),
      .pfdena(1'b1),
      .phasecounterselect({4{1'b1}}),
      .phasedone(),
      .phasestep(1'b1),
      .phaseupdown(1'b1),
      .pllena(1'b1),
      .scanaclr(1'b0),
      .scanclk(1'b0),
      .scanclkena(1'b1),
      .scandata(1'b0),
      .scandataout(),
      .scandone(),
      .scanread(1'b0),
      .scanwrite(1'b0),
      .sclkout0(),
      .sclkout1(),
      .vcooverrange(),
      .vcounderrange()
  );

  // ================================================================
  // Reset Synchronizers (metastability protection)
  // ================================================================

  // Synchronize Ethernet PLL lock to clk_int domain
  sync_reset #(
      .N(4)
  ) sync_reset_inst (
      .clk(clk_int),
      .rst(~pll_locked),
      .out(rst_int)
  );

  // Synchronize Audio PLL lock to clk_50m domain
  sync_reset #(
      .N(4)
  ) sync_reset_inst_audio (
      .clk(clk_50m),
      .rst(~audio_pll_locked),
      .out(audio_rst_int)
  );

  // ================================================================
  // GPIO Debouncer for User Inputs
  // ================================================================

  wire [3:0] btn_int;
  wire [2:0] sw_int;
  wire [3:0] led_int;

  debounce_switch #(
      .WIDTH(7),
      .N(4),
      .RATE(125000)
  ) debounce_switch_inst (
      .clk(clk_int),
      .rst(rst_int),
      .in ({user_pb, user_dip}),
      .out({btn_int, sw_int})
  );

  // Invert LED output (active high to active low)
  assign user_led = ~led_int;

  // ================================================================
  // FPGA Core Logic Instantiation
  // ================================================================

  fpgamixer_core #(
      .TARGET("ALTERA")
  ) core_inst (
      // Clock and reset inputs
      .clk       (clk_int),
      .clk90     (clk90_int),
      .rst       (rst_int),
      .clk_50m   (clk_50m),
      .clk_12m288(clk_12m288),
      .rst_audio (audio_rst_int),

      // Audio output
      .spdif_out(spdif_out),

      // GPIO
      .btn(btn_int),
      .sw (sw_int),
      .led(led_int),

      // I2C buses
      .i2c_sda  (i2c_sda),
      .i2c_scl  (i2c_scl),
      .i2c_sda_t(i2c_sda_t),
      .i2c_scl_t(i2c_scl_t),
      .i2c_sda_f(i2c_sda_f),
      .i2c_scl_f(i2c_scl_f),

      // Rotary encoders
      .enc_a(enc_a),
      .enc_b(enc_b),

      // MCP3008 ADC SPI interface
      .adc_cs_n(adc_cs_n),
      .adc_sclk(adc_sclk),
      .adc_mosi(adc_mosi),
      .adc_miso(adc_miso),

      // TB6612 motor driver
      .m_in1  (m_in1),
      .m_in2  (m_in2),
      .m_pwm  (m_pwm),
      .tb_stby(tb_stby),

      // Ethernet RGMII interface
      .phy_rx_clk (enet_rx_clk),
      .phy_rxd    (enet_rx_d),
      .phy_rx_ctl (enet_rx_dv),
      .phy_tx_clk (enet_tx_clk),
      .phy_txd    (enet_tx_d),
      .phy_tx_ctl (enet_tx_en),
      .phy_reset_n(enet_resetn),
      .phy_int_n  (enet_int)
  );

endmodule

`resetall
