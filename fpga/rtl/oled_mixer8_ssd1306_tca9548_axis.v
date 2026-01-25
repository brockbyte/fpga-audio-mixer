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

`timescale 1ns / 1ps
`default_nettype none

// ============================================================================
// OLED Mixer Display Controller (SSD1306 with TCA9548 I2C Mux)
// 8-channel mixer display with peak meters, pan controls, and channel names
// Features:
//   - TCA9548 8-channel I2C multiplexer for 8 independent OLED displays
//   - SSD1306 128x64 monochrome display controller per channel
//   - Real-time peak meter visualization with hold/decay
//   - Signed pan value display (-64 to +63)
//   - Channel name management via dual-port BRAM
//   - I2C command/data FIFOs for decoupled I2C master interface
// Architecture: FSM-driven display updates, meter refresh at configurable interval
// Thresholds: Peak meter 12-segment display, Q.15 fixed-point peak values
// Clock Domains: 50MHz primary, 400kHz I2C
// ============================================================================
module oled_mixer8_ssd1306_tca9548_axis #(
    parameter integer       CLK_HZ       = 50000000,
    parameter integer       I2C_HZ       = 400000,
    parameter         [6:0] TCA_ADDR     = 7'h70,
    parameter         [6:0] SSD1306_ADDR = 7'h3C,
    parameter integer       METER_MS     = 20,

    parameter integer I2C_MAX_DATA   = 32,  // I2C burst transfer limit
    // SH1106 page mode specifics
    parameter integer SH1106_COL_OFF = 2,   // often 2 for SH1106

    // FIFO depths (axis_fifo from verilog-ethernet / forencich)
    parameter integer CMD_FIFO_DEPTH  = 32,
    parameter integer DATA_FIFO_DEPTH = 2048
) (
    input wire clk,
    input wire rst,

    // I2C physical (open-drain via *_t as in Forencich core)
    input  wire i2c_scl_i,
    output wire i2c_scl_o,
    output wire i2c_scl_t,
    input  wire i2c_sda_i,
    output wire i2c_sda_o,
    output wire i2c_sda_t,

    // Dirty pulses (1 clk wide, set bits)
    input wire [7:0] name_dirty_set,
    input wire [7:0] pan_dirty_set,

    // Q1.15 signed inputs
    input wire signed [15:0] peak0_q15,
    input wire signed [15:0] peak1_q15,
    input wire signed [15:0] peak2_q15,
    input wire signed [15:0] peak3_q15,
    input wire signed [15:0] peak4_q15,
    input wire signed [15:0] peak5_q15,
    input wire signed [15:0] peak6_q15,
    input wire signed [15:0] peak7_q15,

    input wire signed [7:0] pan0_s8,
    input wire signed [7:0] pan1_s8,
    input wire signed [7:0] pan2_s8,
    input wire signed [7:0] pan3_s8,
    input wire signed [7:0] pan4_s8,
    input wire signed [7:0] pan5_s8,
    input wire signed [7:0] pan6_s8,
    input wire signed [7:0] pan7_s8,

    // Name BRAM (8-bit, 256 deep, 16 channels -> addr {ch[3:0], idx[3:0]})
    output reg  [7:0] name_bram_addr,
    input  wire [7:0] name_bram_dout
);

  // ------------------------------------------------------------------------
  // AXIS -> FIFO -> i2c_master
  // ------------------------------------------------------------------------

  // FSM-side CMD signals (into CMD FIFO)
  reg  [6:0] cmd_address_reg = 7'd0;
  reg        cmd_start_reg = 1'b0;
  reg        cmd_read_reg = 1'b0;
  reg        cmd_write_reg = 1'b0;
  reg        cmd_wrm_reg = 1'b0;
  reg        cmd_stop_reg = 1'b0;
  reg        cmd_valid_reg = 1'b0;
  wire       cmd_ready;  // FIFO ready

  // FSM-side DATA signals (into DATA FIFO)
  reg  [7:0] data_tdata_reg = 8'd0;
  reg        data_tvalid_reg = 1'b0;
  wire       data_tready;  // FIFO ready
  reg        data_tlast_reg = 1'b0;

  // CMD FIFO -> i2c_master
  wire [6:0] cmd_address_int;
  wire       cmd_start_int;
  wire       cmd_read_int;
  wire       cmd_write_int;
  wire       cmd_write_multiple_int;
  wire       cmd_stop_int;
  wire       cmd_valid_int;
  wire       cmd_ready_int;  // i2c_master ready

  // DATA FIFO -> i2c_master
  wire [7:0] data_in_int;
  wire       data_in_valid_int;
  wire       data_in_ready_int;  // i2c_master ready
  wire       data_in_last_int;

  // Unused read channel
  wire [7:0] data_out_int;
  wire       data_out_valid_int;
  wire       data_out_last_int;

  // Status
  wire busy, bus_control, bus_active, missed_ack;

  localparam integer METER_W = 16;  // 12 pixels wide
  localparam integer METER_SEG = 12;  // 12 segments
  localparam integer SEG_H = 4;  // 4 pixels high
  localparam integer SEG_GAP = 1;  // 1 pixel spacing
  localparam integer SEG_STEP = SEG_H + SEG_GAP;  // 5 pixels

  // Peak-Hold/Decay in "meter_tick" intervals
  localparam integer HOLD_MS = 800;  // holds ~0.5s
  localparam integer DECAY_MS = 500;  // all ~0.12s 1 segment down
  localparam integer HOLD_TICKS = (HOLD_MS + METER_MS - 1) / METER_MS;  // e.g., 25 at 20ms
  localparam integer DECAY_TICKS = (DECAY_MS + METER_MS - 1) / METER_MS;  // e.g., 6 at 20ms


  // Prescale
  localparam integer PRESCALE_INT = (CLK_HZ / (I2C_HZ * 4));
  wire [15:0] prescale_cfg = (PRESCALE_INT < 1) ? 16'd1 :
    (PRESCALE_INT > 65535) ? 16'd65535 :
    PRESCALE_INT[15:0];

  // ------------------------------------------------------------------------
  // CMD FIFO (axis_fifo)
  // ------------------------------------------------------------------------
  axis_fifo #(
      .DEPTH(CMD_FIFO_DEPTH),
      .DATA_WIDTH(7 + 5),
      .KEEP_ENABLE(0),
      .LAST_ENABLE(0),
      .ID_ENABLE(0),
      .DEST_ENABLE(0),
      .USER_ENABLE(0),
      .FRAME_FIFO(0)
  ) i2c_cmd_fifo_inst (
      .clk(clk),
      .rst(rst),

      // input (FSM)
      .s_axis_tdata({
        cmd_address_reg, cmd_start_reg, cmd_read_reg, cmd_write_reg, cmd_wrm_reg, cmd_stop_reg
      }),
      .s_axis_tkeep(0),
      .s_axis_tvalid(cmd_valid_reg),
      .s_axis_tready(cmd_ready),
      .s_axis_tlast(1'b0),
      .s_axis_tid(0),
      .s_axis_tdest(0),
      .s_axis_tuser(1'b0),

      // output (to i2c_master)
      .m_axis_tdata({
        cmd_address_int,
        cmd_start_int,
        cmd_read_int,
        cmd_write_int,
        cmd_write_multiple_int,
        cmd_stop_int
      }),
      .m_axis_tkeep(),
      .m_axis_tvalid(cmd_valid_int),
      .m_axis_tready(cmd_ready_int),
      .m_axis_tlast(),
      .m_axis_tid(),
      .m_axis_tdest(),
      .m_axis_tuser()
  );

  // ------------------------------------------------------------------------
  // DATA FIFO (axis_fifo)
  // ------------------------------------------------------------------------
  axis_fifo #(
      .DEPTH(DATA_FIFO_DEPTH),
      .DATA_WIDTH(8),
      .KEEP_ENABLE(0),
      .LAST_ENABLE(1),
      .ID_ENABLE(0),
      .DEST_ENABLE(0),
      .USER_ENABLE(0),
      .FRAME_FIFO(0)
  ) i2c_data_fifo_inst (
      .clk(clk),
      .rst(rst),

      // input (FSM)
      .s_axis_tdata(data_tdata_reg),
      .s_axis_tkeep(0),
      .s_axis_tvalid(data_tvalid_reg),
      .s_axis_tready(data_tready),
      .s_axis_tlast(data_tlast_reg),
      .s_axis_tid(0),
      .s_axis_tdest(0),
      .s_axis_tuser(1'b0),

      // output (to i2c_master)
      .m_axis_tdata(data_in_int),
      .m_axis_tkeep(),
      .m_axis_tvalid(data_in_valid_int),
      .m_axis_tready(data_in_ready_int),
      .m_axis_tlast(data_in_last_int),
      .m_axis_tid(),
      .m_axis_tdest(),
      .m_axis_tuser()
  );

  // FIFO empty indicators (CRITICAL with decoupling!)
  wire cmd_fifo_empty = ~cmd_valid_int;
  wire data_fifo_empty = ~data_in_valid_int;

  // convenience: true when nothing pending anywhere
  wire i2c_all_idle = ~busy && cmd_fifo_empty && data_fifo_empty;

  // ------------------------------------------------------------------------
  // I2C master (consumes from FIFOs)
  // ------------------------------------------------------------------------
  i2c_master i2c_master_inst (
      .clk(clk),
      .rst(rst),

      .s_axis_cmd_address(cmd_address_int),
      .s_axis_cmd_start(cmd_start_int),
      .s_axis_cmd_read(cmd_read_int),
      .s_axis_cmd_write(cmd_write_int),
      .s_axis_cmd_write_multiple(cmd_write_multiple_int),
      .s_axis_cmd_stop(cmd_stop_int),
      .s_axis_cmd_valid(cmd_valid_int),
      .s_axis_cmd_ready(cmd_ready_int),

      .s_axis_data_tdata (data_in_int),
      .s_axis_data_tvalid(data_in_valid_int),
      .s_axis_data_tready(data_in_ready_int),
      .s_axis_data_tlast (data_in_last_int),

      // We don't read from slaves here
      .m_axis_data_tdata (data_out_int),
      .m_axis_data_tvalid(data_out_valid_int),
      .m_axis_data_tready(1'b0),
      .m_axis_data_tlast (data_out_last_int),

      .scl_i(i2c_scl_i),
      .scl_o(i2c_scl_o),
      .scl_t(i2c_scl_t),
      .sda_i(i2c_sda_i),
      .sda_o(i2c_sda_o),
      .sda_t(i2c_sda_t),

      .busy(busy),
      .bus_control(bus_control),
      .bus_active(bus_active),
      .missed_ack(missed_ack),

      .prescale(prescale_cfg),
      .stop_on_idle(1'b0)
  );

  // ------------------------------------------------------------
  // Font ROM (page-ready): addr -> dout (1-cycle latency)
  // ------------------------------------------------------------
  wire [7:0] font_dout;
  //reg  [9:0] font_addr;   // 0..607

  font7x12_pages_bram u_font (
      .clk (clk),
      .addr(font_addr),
      .dout(font_dout)
  );

  // ------------------------------------------------------------------------
  // Helpers
  // ------------------------------------------------------------------------
  function signed [15:0] get_peak;
    input [2:0] idx;
    begin
      case (idx)
        3'd0: get_peak = peak0_q15;
        3'd1: get_peak = peak1_q15;
        3'd2: get_peak = peak2_q15;
        3'd3: get_peak = peak3_q15;
        3'd4: get_peak = peak4_q15;
        3'd5: get_peak = peak5_q15;
        3'd6: get_peak = peak6_q15;
        default: get_peak = peak7_q15;
      endcase
    end
  endfunction

  function signed [7:0] get_pan;
    input [2:0] idx;
    begin
      case (idx)
        3'd0: get_pan = pan0_s8;
        3'd1: get_pan = pan1_s8;
        3'd2: get_pan = pan2_s8;
        3'd3: get_pan = pan3_s8;
        3'd4: get_pan = pan4_s8;
        3'd5: get_pan = pan5_s8;
        3'd6: get_pan = pan6_s8;
        default: get_pan = pan7_s8;
      endcase
    end
  endfunction

  function [6:0] peak_height56;
    input signed [15:0] q15;
    reg [15:0] mag;
    reg [31:0] prod;
    begin
      mag = q15[15] ? (~q15 + 1) : q15;
      prod = mag * 56;
      peak_height56 = prod[31:15];
      if (peak_height56 > 56) peak_height56 = 56;
    end
  endfunction

  function signed [7:0] pan_offset48;
    input signed [7:0] pan;  // -127..+127
    reg        [ 7:0] ap;  // abs(pan) as unsigned
    reg        [15:0] num;  // ap*48 fits in 16 bit (127*48=6096)
    reg        [ 9:0] aoff;  // abs(offset) 0..48
    reg signed [ 9:0] off;  // signed offset
    begin
      // abs(pan)
      if (pan[7]) ap = (~pan) + 8'd1;
      else ap = pan;

      // abs_offset = round(ap*48/127)
      num  = ap * 16'd48;
      aoff = (num + 16'd63) / 16'd127;

      // Vorzeichen wieder drauf (WICHTIG: keine 11-bit Konkatenation!)
      if (pan[7]) off = -$signed(aoff);  // <-- FIX
      else off = $signed(aoff);

      // Clamp for 96px field: -48..+47
      if (off > 10'sd47) off = 10'sd47;
      if (off < -10'sd48) off = -10'sd48;

      // Return (within -48..+47 fits safely in s8)
      pan_offset48 = $signed(off[7:0]);
    end
  endfunction

  function [6:0] ascii_to_charcode;
    input [7:0] ascii;
    reg [7:0] a;
    begin
      a = ascii;
      if (a < 8'd32) a = 8'd32;
      if (a > 8'd127) a = 8'd32;
      ascii_to_charcode = a - 8'd32;
    end
  endfunction

  function [2:0] first_set_bit;
    input [7:0] v;
    begin
      if (v[0]) first_set_bit = 3'd0;
      else if (v[1]) first_set_bit = 3'd1;
      else if (v[2]) first_set_bit = 3'd2;
      else if (v[3]) first_set_bit = 3'd3;
      else if (v[4]) first_set_bit = 3'd4;
      else if (v[5]) first_set_bit = 3'd5;
      else if (v[6]) first_set_bit = 3'd6;
      else first_set_bit = 3'd7;
    end
  endfunction

  function [2:0] col_in_char6;
    input [6:0] x;
    begin
      col_in_char6 = x % 6;  // 0..5
    end
  endfunction

  function [3:0] char_index16;
    input [6:0] x;
    begin
      char_index16 = x / 6;  // 0..15
    end
  endfunction

  function [7:0] render_pan_page4;
    input [2:0] disp;
    input [6:0] x;  // 0..95
    reg signed [7:0] off;
    reg signed [8:0] sx0, sx1;
    reg [6:0] x0, x1;
    reg [6:0] cx;
    reg [7:0] b;
    begin
      cx  = 7'd48;
      off = pan_offset48(get_pan(disp));  // -48..+47

      if (off >= 0) begin
        sx0 = cx;
        sx1 = cx + off;
      end else begin
        sx0 = cx + off;
        sx1 = cx;
      end

      if (sx0 < 0) x0 = 7'd0;
      else if (sx0 > 95) x0 = 7'd95;
      else x0 = sx0[6:0];

      if (sx1 < 0) x1 = 7'd0;
      else if (sx1 > 95) x1 = 7'd95;
      else x1 = sx1[6:0];

      b = 8'h00;

      if (x == cx) begin
        b = 8'hFF;
      end else if (off != 0) begin
        if (x >= x0 && x <= x1) begin
          b = 8'hFF;
        end
      end

      render_pan_page4 = b;
    end
  endfunction

  function [7:0] render_meter_byte;
    input [2:0] disp;
    input [2:0] page;  // 1..7
    input [4:0] col32;  // 0..31
    reg [6:0] peak_h;
    reg [7:0] b;
    integer i;
    reg [6:0] y;
    begin
      peak_h = peak_height56(get_peak(disp));
      b = 8'h00;
      for (i = 0; i < 8; i = i + 1) begin
        y = {page, 3'b000} + i[2:0];
        if ((63 - y) < peak_h) b[i] = 1'b1;
      end
      render_meter_byte = b;
    end
  endfunction

  function [7:0] init_cmd;
    input [4:0] p;  // 0..31
    begin
      case (p)
        0: init_cmd = 8'hAE;  // display OFF
        1: init_cmd = 8'hD5;
        2: init_cmd = 8'h80;
        3: init_cmd = 8'hA8;
        4: init_cmd = 8'h3F;
        5: init_cmd = 8'hD3;
        6: init_cmd = 8'h00;
        7: init_cmd = 8'h40;
        8: init_cmd = 8'h8D;
        9: init_cmd = 8'h14;
        10: init_cmd = 8'h20;
        11: init_cmd = 8'h00;  // horizontal addressing
        12: init_cmd = 8'hA1;
        13: init_cmd = 8'hC8;
        14: init_cmd = 8'hDA;
        15: init_cmd = 8'h12;
        16: init_cmd = 8'h81;
        17: init_cmd = 8'hCF;
        18: init_cmd = 8'hD9;
        19: init_cmd = 8'hF1;
        20: init_cmd = 8'hDB;
        21: init_cmd = 8'h40;
        22: init_cmd = 8'hA4;
        23: init_cmd = 8'hA6;
        24: init_cmd = 8'h2E;
        25: init_cmd = 8'h21;
        26: init_cmd = 8'h00;
        27: init_cmd = 8'h7F;
        28: init_cmd = 8'h22;
        29: init_cmd = 8'h00;
        30: init_cmd = 8'h07;
        31: init_cmd = 8'hAF;  // display ON
        default: init_cmd = 8'hAF;
      endcase
    end
  endfunction

  // ------------------------------------------------------------------------
  // Meter tick
  // ------------------------------------------------------------------------
  localparam integer METER_TICKS = (CLK_HZ / 1000) * METER_MS;
  reg [31:0] meter_cnt = 0;
  reg        meter_tick = 0;

  always @(posedge clk) begin
    if (rst) begin
      meter_cnt  <= 0;
      meter_tick <= 0;
    end else begin
      meter_tick <= 1'b0;
      if (meter_cnt == METER_TICKS - 1) begin
        meter_cnt  <= 0;
        meter_tick <= 1'b1;
      end else begin
        meter_cnt <= meter_cnt + 1;
      end
    end
  end

  // ------------------------------------------------------------------------
  // Caches & flags
  // ------------------------------------------------------------------------
  reg     [7:0] name_dirty;
  reg     [7:0] pan_dirty;
  reg           did_init   [ 0:7];
  reg     [7:0] name_cache [0:15];
  integer       di;

  // ------------------------------------------------------------------------
  // AXIS push helpers (into FIFOs)
  // ------------------------------------------------------------------------
  task automatic issue_cmd;
    input [6:0] addr;
    input do_start;
    input do_write;
    input do_wrm;
    input do_stop;
    begin
      cmd_address_reg <= addr;
      cmd_start_reg   <= do_start;
      cmd_read_reg    <= 1'b0;
      cmd_write_reg   <= do_write;
      cmd_wrm_reg     <= do_wrm;
      cmd_stop_reg    <= do_stop;
      cmd_valid_reg   <= 1'b1;
    end
  endtask

  task automatic issue_data;
    input [7:0] b;
    input last;
    begin
      data_tdata_reg  <= b;
      data_tlast_reg  <= last;
      data_tvalid_reg <= 1'b1;
    end
  endtask

  // ------------------------------------------------------------------------
  // METER: 10 “boxes”, 4px on + 1px gap = 5px pitch, bottom-aligned
  // Display area: right-aligned 16px wide: columns 112..127
  // Pages used: 1..7 (56px height)
  // We compute a stable displayed level (0..10) + peak-hold marker level (0..10)
  // ------------------------------------------------------------------------

  // thresholds for level (approx 0..-45 dBFS in 5 dB steps)
  localparam [15:0] TH0 = 16'd32767;  //  0 dB
  localparam [15:0] TH1 = 16'd18427;  // -5
  localparam [15:0] TH2 = 16'd10362;  // -10
  localparam [15:0] TH3 = 16'd5826;  // -15
  localparam [15:0] TH4 = 16'd3277;  // -20
  localparam [15:0] TH5 = 16'd1843;  // -25
  localparam [15:0] TH6 = 16'd1036;  // -30
  localparam [15:0] TH7 = 16'd582;  // -35
  localparam [15:0] TH8 = 16'd328;  // -40
  localparam [15:0] TH9 = 16'd184;  // -45

  function [3:0] level10_from_peak_q15;
    input signed [15:0] q15;
    reg [15:0] mag;
    begin
      mag = q15[15] ? (~q15 + 16'd1) : q15;

      if (mag >= TH0) level10_from_peak_q15 = 4'd10;
      else if (mag >= TH1) level10_from_peak_q15 = 4'd9;
      else if (mag >= TH2) level10_from_peak_q15 = 4'd8;
      else if (mag >= TH3) level10_from_peak_q15 = 4'd7;
      else if (mag >= TH4) level10_from_peak_q15 = 4'd6;
      else if (mag >= TH5) level10_from_peak_q15 = 4'd5;
      else if (mag >= TH6) level10_from_peak_q15 = 4'd4;
      else if (mag >= TH7) level10_from_peak_q15 = 4'd3;
      else if (mag >= TH8) level10_from_peak_q15 = 4'd2;
      else if (mag >= TH9) level10_from_peak_q15 = 4'd1;
      else level10_from_peak_q15 = 4'd0;
    end
  endfunction

  // displayed “bar” level and separate peak-hold level
  reg [3:0] meter_lvl    [0:7];  // 0..10 (stable)
  reg [4:0] meter_hold   [0:7];  // hold ticks
  reg [1:0] meter_rel    [0:7];  // release tick divider

  reg [3:0] peakhold_lvl [0:7];  // 0..10
  reg [4:0] peakhold_hold[0:7];
  reg [2:0] peakhold_rel [0:7];

  // tuning (with METER_MS=20ms):
  localparam integer METER_HOLD_TICKS = 15;  // 300ms
  localparam integer METER_REL_TICKS = 3;  // 60ms per step
  localparam integer PH_HOLD_TICKS = 25;  // 500ms
  localparam integer PH_REL_TICKS = 5;  // 100ms per step

  // render one 8-bit vertical slice for page=1..7 and col=0..15
  // page 1..7 -> y 8..63 (56px). Use y_local 0..55 from top of this region.
  // bottom index yb 0..55 from bottom.
  // each box uses 5 pixels pitch, active pixels when (yb%5)!=4, box_id=yb/5.
  // box_id 0 is bottom box.
  function [7:0] render_meter_byte_boxes16;
    input [3:0] lvl;  // 0..10
    input [3:0] ph;  // 0..10
    input [2:0] page;  // 1..7
    input [3:0] col16;  // 0..15
    reg     [7:0] b;
    integer       i;
    reg     [6:0] y;  // absolute y 0..63
    reg     [5:0] y_local;  // 0..55
    reg     [5:0] yb;  // 0..55 from bottom
    reg     [3:0] box_id;  // 0..11 possible
    reg     [2:0] mod5;
    reg           pixel_on;
    reg           peakline_on;
    reg     [6:0] peakline_yb;  // bottom-based y for peakline
    begin
      b = 8'h00;

      // peakline at top edge of held box: yb = ph*5 - 1 (if ph>0)
      if (ph != 0) begin
        peakline_yb = (ph * 7'd5) - 7'd1;
      end else begin
        peakline_yb = 7'd127;  // disabled
      end

      for (i = 0; i < 8; i = i + 1) begin
        // absolute y for this bit within selected pages 1..7
        y        = {page, 3'b000} + i[2:0];  // 8..63

        // convert to local 0..55
        y_local  = y - 7'd8;  // 0..55
        yb       = 6'd55 - y_local;  // 55..0 from top -> bottom-based

        box_id   = yb / 6'd5;
        mod5     = yb - (box_id * 6'd5);

        // within box fill region (4px) => mod5 != 4
        pixel_on = 1'b0;
        if (box_id < 4'd10) begin
          if (mod5 != 3'd4) begin
            if (box_id < lvl) pixel_on = 1'b1;
          end
        end

        // peak hold line: 1px line at yb == peakline_yb
        peakline_on = 1'b0;
        if (ph != 0) begin
          if (yb == peakline_yb[5:0]) begin
            peakline_on = 1'b1;
          end
        end

        if (pixel_on | peakline_on) begin
          b[i] = 1'b1;
        end
      end

      // col16 is always inside the 16px meter area, so no horizontal gating needed
      render_meter_byte_boxes16 = b;
    end
  endfunction

  integer k;
  reg [3:0] cur_lvl;

  always @(posedge clk) begin
    if (rst) begin

      for (di = 0; di < 8; di = di + 1) begin
        meter_lvl[di]     <= 4'd0;
        meter_hold[di]    <= 5'd0;
        meter_rel[di]     <= 2'd0;

        peakhold_lvl[di]  <= 4'd0;
        peakhold_hold[di] <= 5'd0;
        peakhold_rel[di]  <= 3'd0;
      end

    end else begin
      // update meter smoothing/hold at meter_tick (independent from I2C FSM)
      if (meter_tick) begin
        for (k = 0; k < 8; k = k + 1) begin
          cur_lvl = level10_from_peak_q15(get_peak(k[2:0]));  // 0..10

          // main displayed level (fast attack, hold, slow release)
          if (cur_lvl > meter_lvl[k]) begin
            meter_lvl[k]  <= cur_lvl;
            meter_hold[k] <= METER_HOLD_TICKS[4:0];
            meter_rel[k]  <= 2'd0;
          end else begin
            if (meter_hold[k] != 0) begin
              meter_hold[k] <= meter_hold[k] - 1'b1;
            end else begin
              if (meter_rel[k] == (METER_REL_TICKS - 1)) begin
                meter_rel[k] <= 2'd0;
                if (meter_lvl[k] != 0) meter_lvl[k] <= meter_lvl[k] - 1'b1;
              end else begin
                meter_rel[k] <= meter_rel[k] + 1'b1;
              end
            end
          end

          // separate peak-hold marker
          if (cur_lvl >= peakhold_lvl[k]) begin
            peakhold_lvl[k]  <= cur_lvl;
            peakhold_hold[k] <= PH_HOLD_TICKS[4:0];
            peakhold_rel[k]  <= 3'd0;
          end else begin
            if (peakhold_hold[k] != 0) begin
              peakhold_hold[k] <= peakhold_hold[k] - 1'b1;
            end else begin
              if (peakhold_rel[k] == (PH_REL_TICKS - 1)) begin
                peakhold_rel[k] <= 3'd0;
                if (peakhold_lvl[k] != 0) peakhold_lvl[k] <= peakhold_lvl[k] - 1'b1;
              end else begin
                peakhold_rel[k] <= peakhold_rel[k] + 1'b1;
              end
            end
          end
        end
      end
    end
  end

  reg signed [ 7:0] pan_s8_r0;  // Stage 0: pan latched
  reg        [ 7:0] pan_abs_r1;  // Stage 1: abs(pan)
  reg        [15:0] pan_num_r2;  // Stage 2: abs*48
  reg        [ 9:0] pan_aoff_r3;  // Stage 3: round(/127)
  reg signed [ 9:0] pan_off_r4;  // Stage 4: signed offset

  reg        [ 6:0] pan_x0_r5;  // Stage 5: clamped start (0..95)
  reg        [ 6:0] pan_x1_r5;  // Stage 5: clamped end   (0..95)

  reg        [ 2:0] pan_calc_cnt;  
  reg               pan_calc_busy;

  reg signed [ 9:0] pan_sum0_r5;
  reg signed [ 9:0] pan_sum1_r5;

  // ------------------------------------------------------------------------
  // FSM
  // ------------------------------------------------------------------------
  reg        [ 9:0] state = 0;

  reg        [ 2:0] disp_rr = 3'd0;
  reg        [ 2:0] disp_idx = 3'd0;
  reg        [ 1:0] mode = 2'd2;  // 0=name,1=pan,2=meter,3=init

  reg        [ 6:0] x96 = 0;
  reg        [ 4:0] x32 = 0;
  reg        [ 2:0] met_page = 3'd1;
  reg        [ 4:0] init_ptr = 0;

  reg        [ 3:0] x16 = 0;  // meter columns 0..15

  reg        [ 7:0] name_byte_buf = 8'h00;

  localparam [9:0]
ST_IDLE          = 10'd0,

ST_TCA_DATA      = 10'd10,
ST_TCA_CMD       = 10'd11,
ST_TCA_WAIT      = 10'd12,

ST_INIT_CTRL     = 10'd20,
ST_INIT_BYTES    = 10'd21,
ST_INIT_CMD      = 10'd22,
ST_INIT_WAIT     = 10'd23,

  // clear flow
  ST_CLR_HDR       = 10'd24,
ST_CLR_CMD       = 10'd25,
ST_CLR_WAIT      = 10'd26,
ST_CLR_DATA0     = 10'd27,
ST_CLR_DATA      = 10'd28,
ST_CLR_DATA_CMD  = 10'd29,
ST_CLR_DATA_WAIT = 10'd30,

ST_NAME_LOAD0    = 10'd31,
ST_NAME_LOAD1    = 10'd32,
ST_NAME_LOAD2    = 10'd33,

ST_NAME_HDR      = 10'd40,
ST_NAME_FONT0    = 10'd41,
ST_NAME_FONT1    = 10'd42,
ST_NAME_PUSH     = 10'd43,
ST_NAME_CMD      = 10'd44,
ST_NAME_WAIT     = 10'd45,

ST_NAME_DATA0     = 10'd46,
ST_NAME_FONT_ADDR = 10'd47,
ST_NAME_FONT_WAIT = 10'd48,
ST_NAME_DATA      = 10'd49,
ST_NAME_DATA_CMD  = 10'd100,
ST_NAME_DATA_WAIT = 10'd101,

ST_PAN_HDR       = 10'd50,
ST_PAN_PUSH      = 10'd51,
ST_PAN_CMD       = 10'd52,
ST_PAN_WAIT      = 10'd53,
ST_PAN_HDR_CMD   = 10'd54,
ST_PAN_DATA0     = 10'd55,
ST_PAN_DATA1     = 10'd56,
ST_PAN_DATA_CMD  = 10'd57,
ST_PAN_DATA_WAIT = 10'd58,

ST_MET_HDR       = 10'd60,
ST_MET_PUSH      = 10'd61,
ST_MET_CMD       = 10'd62,
ST_MET_WAIT      = 10'd63,
ST_MET_PUSH0     = 10'd64,
ST_MET_HDR_CMD   = 10'd65,
ST_MET_HDR_WAIT  = 10'd66,

ST_TST_HDR       = 10'd71,
ST_TST_CMD       = 10'd72,
ST_TST_WAIT      = 10'd73,
ST_TST_DATA0     = 10'd74,
ST_TST_DATA1     = 10'd75,
ST_TST_DATA_CMD  = 10'd76,
ST_TST_DATA_WAIT = 10'd77,

ST_MET_CMDX      = 10'd82,
ST_MET_WAIT0     = 10'd83,
ST_MET_DATA0     = 10'd84,
ST_MET_DATA      = 10'd85,
ST_MET_DATACMD   = 10'd86,
ST_MET_WAIT1     = 10'd87,

ST_PAN_LBL_HDR   = 10'd94,
ST_PAN_LBL_DATA0 = 10'd95,
ST_PAN_LBL_FONT1 = 10'd96,
ST_PAN_LBL_PUSH  = 10'd97,
ST_PAN_LBL_CMD   = 10'd98,
ST_PAN_LBL_WAIT  = 10'd99,
ST_PAN_LBL_DATA1 = 10'd110,
ST_PAN_LBL_DATA_CMD   = 10'd111,
ST_PAN_LBL_DATA_WAIT  = 10'd112;



  // --- kleine Helferregister:
  reg [2:0] p4_cmd_i = 3'd0;  // 0..3
  reg [1:0] p4_dat_i = 2'd0;  // 0..3

  // Test data (you can replace later)
  reg [7:0] p4_d0 = 8'hFF;
  reg [7:0] p4_d1 = 8'h81;
  reg [7:0] p4_d2 = 8'h42;
  reg [7:0] p4_d3 = 8'h18;

  // optional: SH1106 column offset
  localparam [7:0] COL_START = 8'd0;  // z.B. 8'd2 bei SH1106-Offset

  // PAN write helpers
  reg [ 6:0] pan_x96;  // 0..95
  reg [ 5:0] pan_chunk_cnt;  // 0..31
  reg [ 5:0] pan_chunk_len;  // 1..32
  reg [ 3:0] pan_cmd_i;  // 0..3 for cmd bytes
  reg [ 7:0] pan_col;  // aktuelle SH1106 col (inkl. offset)


  reg        name_page;  // 0..1
  reg [ 3:0] name_char;  // 0..15
  reg [ 2:0] name_col;  // 0..7 (7 glyph + 1 spacing)
  reg [ 7:0] name_chunk_cnt;  // 0..31
  reg [10:0] font_addr;  // large enough

  reg [ 3:0] hdr_i = 0;

  // clear/test counters
  reg [ 3:0] clr_hdr_i = 4'd0;
  reg [ 9:0] clr_cnt = 10'd0;  // 0..1023

  reg [ 2:0] met_page_pm;  // 0..7
  reg [ 3:0] met_col_pm;  // 0..(METER_W-1)
  reg [ 6:0] met_col_start;  // Startspalte im Display

  // ------------------------------------------------------------------------
  // Clear header (commands to set full window)
  // Transfer 1: 0x00, 0x21,0x00,0x7F, 0x22,0x00,0x07
  // Transfer 2: 0x40 + 1024x 0x00
  // ------------------------------------------------------------------------
  function [7:0] hdr_clear_cmd;
    input [3:0] i;
    begin
      case (i)
        0: hdr_clear_cmd = 8'h00;
        1: hdr_clear_cmd = 8'h21;
        2: hdr_clear_cmd = 8'h00;
        3: hdr_clear_cmd = 8'h7F;
        4: hdr_clear_cmd = 8'h22;
        5: hdr_clear_cmd = 8'h00;
        6: hdr_clear_cmd = 8'h07;
        default: hdr_clear_cmd = 8'h00;
      endcase
    end
  endfunction

  function [7:0] hdr_test_cmd;
    input [3:0] i;
    begin
      case (i)
        0: hdr_test_cmd = 8'h00;
        1: hdr_test_cmd = 8'h21;
        2: hdr_test_cmd = 8'h00;
        3: hdr_test_cmd = 8'h7F;
        4: hdr_test_cmd = 8'h22;
        5: hdr_test_cmd = 8'h04;
        6: hdr_test_cmd = 8'h04;
        default: hdr_test_cmd = 8'h00;
      endcase
    end
  endfunction

  function [7:0] hdr_name;
    input [3:0] i;
    input page;
    begin
      case (i)
        0: hdr_name = 8'h00;
        1: hdr_name = 8'h21;
        2: hdr_name = 8'h00;
        3: hdr_name = 8'h7F;
        4: hdr_name = 8'h22;
        5: hdr_name = 8'h00 | (page ? 8'd1 : 8'd0);
        6: hdr_name = 8'h00 | (page ? 8'd1 : 8'd0);
        default: hdr_name = 8'h00;
      endcase
    end
  endfunction

  function [7:0] hdr_pan;
    input [3:0] i;
    begin
      case (i)
        0: hdr_pan = 8'h00;
        1: hdr_pan = 8'h21;
        2: hdr_pan = 8'h00;
        3: hdr_pan = 8'h5F;
        4: hdr_pan = 8'h22;
        5: hdr_pan = 8'h04;
        6: hdr_pan = 8'h04;
        default: hdr_pan = 8'h00;
      endcase
    end
  endfunction

  function [7:0] hdr_meter;
    input [3:0] i;
    begin
      case (i)
        0: hdr_meter = 8'h00;
        1: hdr_meter = 8'h21;
        2: hdr_meter = 8'h70;
        3: hdr_meter = 8'h7F;
        4: hdr_meter = 8'h22;
        5: hdr_meter = 8'h01;
        6: hdr_meter = 8'h07;
        default: hdr_meter = 8'h00;
      endcase
    end
  endfunction

  function [5:0] ascii_to_glyph;
    input [7:0] a;
    begin
      if (a == 8'h20) ascii_to_glyph = 6'd0;  // space
      else if (a == 8'h2D) ascii_to_glyph = 6'd1;  // '-'
      else if (a >= 8'h30 && a <= 8'h39) ascii_to_glyph = 6'd2 + (a - 8'h30);  // 0..9
      else if (a >= 8'h41 && a <= 8'h5A) ascii_to_glyph = 6'd12 + (a - 8'h41);  // A..Z
      else ascii_to_glyph = 6'd0;
    end
  endfunction

  localparam integer FONT_GLYPHS = 38;
  localparam integer FONT_COLS = 8;
  localparam integer FONT_PAGE_STRIDE = FONT_GLYPHS * FONT_COLS;  // 304

  function [7:0] hdr_pan_lbl;
    input [3:0] i;
    begin
      case (i)
        0: hdr_pan_lbl = 8'h00;
        1: hdr_pan_lbl = 8'h21;
        2: hdr_pan_lbl = 8'h00;
        3: hdr_pan_lbl = 8'h5F;  // 0..95
        4: hdr_pan_lbl = 8'h22;
        5: hdr_pan_lbl = 8'h05;  // page 5 (unter page 4)
        6: hdr_pan_lbl = 8'h05;
        default: hdr_pan_lbl = 8'h00;
      endcase
    end
  endfunction

  // --------------------
  // Bit spiegeln (vertikal innerhalb der Page)
  // --------------------
  function [7:0] bitrev8;
    input [7:0] b;
    begin
      bitrev8 = {b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]};
    end
  endfunction

  // --------------------
  // 5x8 glyph columns (bit0..bit7) - danach optional spiegeln
  // --------------------
  function [7:0] glyph5x8_col_raw;
    input [7:0] ch;  // 'L','0','R',' '
    input [2:0] col;  // 0..4
    begin
      case (ch)
        "L": begin
          case (col)
            3'd0: glyph5x8_col_raw = 8'b01111111;
            3'd1: glyph5x8_col_raw = 8'b00000001;
            3'd2: glyph5x8_col_raw = 8'b00000001;
            3'd3: glyph5x8_col_raw = 8'b00000001;
            3'd4: glyph5x8_col_raw = 8'b00000001;
            default: glyph5x8_col_raw = 8'h00;
          endcase
        end

        "0": begin
          case (col)
            3'd0: glyph5x8_col_raw = 8'b00111110;
            3'd1: glyph5x8_col_raw = 8'b01000001;
            3'd2: glyph5x8_col_raw = 8'b01000001;
            3'd3: glyph5x8_col_raw = 8'b01000001;
            3'd4: glyph5x8_col_raw = 8'b00111110;
            default: glyph5x8_col_raw = 8'h00;
          endcase
        end

        "R": begin
          // 5x8 "R"
          // ####.
          // #...#
          // #...#
          // ####.
          // #.#..
          // #..#.
          // #...#
          // .....
          case (col)
            3'd0: glyph5x8_col_raw = 8'b01111111;  // left stem
            3'd1: glyph5x8_col_raw = 8'b01001000;  // top + mid
            3'd2: glyph5x8_col_raw = 8'b01001100;  // upper bowl + start leg
            3'd3: glyph5x8_col_raw = 8'b01001010;  // bowl + diagonal leg
            3'd4: glyph5x8_col_raw = 8'b00110001;  // right edge + leg
            default: glyph5x8_col_raw = 8'h00;
          endcase
        end

        default: glyph5x8_col_raw = 8'h00;
      endcase
    end
  endfunction

  function [7:0] glyph5x8_col;
    input [7:0] ch;
    input [2:0] col;
    reg [7:0] g;
    begin
      g = glyph5x8_col_raw(ch, col);
      // vertikal spiegeln:
      glyph5x8_col = bitrev8(g);
    end
  endfunction

  // --------------------
  // Pixelgenaues Rendern der Label-Zeile 0..95
  // - L bei x=0..4
  // - 0 zentriert unter cx=48  -> Start x=46..50
  // - R rechts bei x=91..95
  // --------------------
  function [7:0] pan_label_byte;
    input [6:0] x;  // 0..95
    reg [7:0] b;
    reg [2:0] col;
    begin
      b = 8'h00;

      // 'L' links
      if (x <= 7'd4) begin
        col = x[2:0];  // 0..4
        b   = glyph5x8_col("L", col);
      end  // '0' zentriert bei cx=48, Start 46
      else if (x >= 7'd46 && x <= 7'd50) begin
        col = x - 7'd46;  // 0..4
        b   = glyph5x8_col("0", col);
      end  // 'R' rechts (Start 91)
      else if (x >= 7'd91 && x <= 7'd95) begin
        col = x - 7'd91;  // 0..4
        b   = glyph5x8_col("R", col);
      end else begin
        b = 8'h00;
      end

      pan_label_byte = b;
    end
  endfunction





  // ------------------------------------------------------------------------
  // Main sequential
  // ------------------------------------------------------------------------
  always @(posedge clk) begin
    if (rst) begin
      state           <= ST_IDLE;

      // clear fifo-input valids
      cmd_valid_reg   <= 1'b0;
      data_tvalid_reg <= 1'b0;
      data_tlast_reg  <= 1'b0;

      disp_rr         <= 3'd0;
      disp_idx        <= 3'd0;
      mode            <= 2'd2;

      x96             <= 0;
      x32             <= 0;
      met_page        <= 3'd1;
      init_ptr        <= 0;
      hdr_i           <= 0;

      name_bram_addr  <= 8'h00;

      clr_hdr_i       <= 0;
      clr_cnt         <= 0;

      met_page_pm     <= 0;
      met_col_pm      <= 0;
      met_col_start   <= 8'd0;

      name_dirty      <= 8'h00;
      pan_dirty       <= 8'h00;

      for (di = 0; di < 8; di = di + 1) begin
        did_init[di] <= 1'b0;
      end

      pan_x96       <= 7'h00;  // 0..95
      pan_chunk_cnt <= 6'h00;  // 0..31
      pan_chunk_len <= 6'h00;  // 1..32
      pan_cmd_i     <= 4'h0;  // 0..3 for cmd bytes
      pan_col       <= 8'h00;

      pan_calc_busy <= 1'b0;
      pan_calc_cnt  <= 3'd0;
      pan_s8_r0     <= 8'sd0;
      pan_abs_r1    <= 8'd0;
      pan_num_r2    <= 16'd0;
      pan_aoff_r3   <= 10'd0;
      pan_off_r4    <= 10'sd0;
      pan_x0_r5     <= 7'd48;
      pan_x1_r5     <= 7'd48;

    end else begin

      if (pan_calc_busy) begin
        pan_calc_cnt <= pan_calc_cnt + 3'd1;

        // Stage 0: latch pan
        if (pan_calc_cnt == 3'd0) begin
          pan_s8_r0 <= get_pan(disp_idx);  // get_pan liefert signed[7:0]
        end

        // Stage 1: abs(pan)
        if (pan_calc_cnt == 3'd1) begin
          if (pan_s8_r0[7]) pan_abs_r1 <= (~pan_s8_r0) + 8'd1;
          else pan_abs_r1 <= pan_s8_r0;
        end

        // Stage 2: num = abs*48
        if (pan_calc_cnt == 3'd2) begin
          pan_num_r2 <= pan_abs_r1 * 16'd48;  // max 6096
        end

        // Stage 3: aoff = round(num/127)
        // (unsigned Division, stabil in Quartus)
        //if (pan_calc_cnt == 3'd3) begin
        //    pan_aoff_r3 <= (pan_num_r2 + 16'd63) / 16'd127;
        //end

        if (pan_calc_cnt == 3'd3) begin
          // 516 = floor(2^16/127)
          // +32768 for round-to-nearest on >>16
          pan_aoff_r3 <= ((pan_num_r2 * 16'd516) + 32'd32768) >> 16;
        end

        // Stage 4: sign wieder drauf + clamp (-48..+47)
        if (pan_calc_cnt == 3'd4) begin
          if (pan_s8_r0[7]) pan_off_r4 <= -$signed({1'b0, pan_aoff_r3});
          else pan_off_r4 <= $signed({1'b0, pan_aoff_r3});

          if (pan_off_r4 > 10'sd47) pan_off_r4 <= 10'sd47;
          if (pan_off_r4 < -10'sd48) pan_off_r4 <= -10'sd48;
        end

        // Stage 5: x0/x1 berechnen und clamp auf 0..95
        if (pan_calc_cnt == 3'd5) begin
          // sums bilden (signed)
          pan_sum0_r5 <= 10'sd48 + pan_off_r4;  // kann negativ sein
          pan_sum1_r5 <= 10'sd48 + pan_off_r4;

          // x0
          if (pan_off_r4 >= 0) begin
            pan_x0_r5 <= 7'd48;
          end else begin
            // x0 = 48 + off
            if ((10'sd48 + pan_off_r4) < 10'sd0) pan_x0_r5 <= 7'd0;
            else if ((10'sd48 + pan_off_r4) > 10'sd95) pan_x0_r5 <= 7'd95;
            else pan_x0_r5 <= (10'sd48 + pan_off_r4);  // passt in 0..95
          end

          // x1
          if (pan_off_r4 >= 0) begin
            // x1 = 48 + off
            if ((10'sd48 + pan_off_r4) < 10'sd0) pan_x1_r5 <= 7'd0;
            else if ((10'sd48 + pan_off_r4) > 10'sd95) pan_x1_r5 <= 7'd95;
            else pan_x1_r5 <= (10'sd48 + pan_off_r4);  // passt in 0..95
          end else begin
            pan_x1_r5 <= 7'd48;
          end

          // done
          pan_calc_busy <= 1'b0;
        end

      end

      // latch dirty pulses
      name_dirty <= name_dirty | name_dirty_set;
      pan_dirty  <= pan_dirty | pan_dirty_set;

      // drop CMD valid when accepted by CMD FIFO
      if (cmd_valid_reg && cmd_ready) begin
        cmd_valid_reg <= 1'b0;
        cmd_start_reg <= 1'b0;
        cmd_write_reg <= 1'b0;
        cmd_wrm_reg   <= 1'b0;
        cmd_stop_reg  <= 1'b0;
      end

      // drop DATA valid when accepted by DATA FIFO
      if (data_tvalid_reg && data_tready) begin
        data_tvalid_reg <= 1'b0;
        data_tlast_reg  <= 1'b0;
      end

      case (state)

        // ---------------------------
        ST_IDLE: begin
          // IMPORTANT: only start a new sequence when i2c_master AND FIFOs are drained
          if (i2c_all_idle) begin
            if (!did_init[disp_rr]) begin
              clr_hdr_i <= 0;
              clr_cnt <= 0;
              disp_idx <= disp_rr;
              mode <= 2'd3;
              state <= ST_TCA_DATA;
            end else if (pan_dirty != 8'h00) begin
              clr_hdr_i <= 0;
              clr_cnt <= 0;
              x96 <= 0;
              disp_idx <= first_set_bit(pan_dirty);
              mode <= 2'd1;
              state <= ST_TCA_DATA;
            end else if (name_dirty != 8'h00) begin
              clr_hdr_i <= 0;
              clr_cnt <= 0;
              disp_idx <= first_set_bit(name_dirty);
              mode <= 2'd0;
              state <= ST_TCA_DATA;
            end else if (meter_tick) begin
              disp_idx <= disp_rr;
              mode <= 2'd2;
              state <= ST_TCA_DATA;
            end
          end
        end

        // ---------------------------
        // Select TCA channel: write 1 byte mask to TCA_ADDR
        ST_TCA_DATA: begin
          if (!data_tvalid_reg) begin
            issue_data(8'h01 << disp_idx, 1'b1);
            //issue_data(8'h80 >> disp_idx, 1'b1);
          end
          if (data_tvalid_reg && data_tready) begin
            state <= ST_TCA_CMD;
          end
        end

        ST_TCA_CMD: begin
          if (!cmd_valid_reg) begin
            issue_cmd(TCA_ADDR, 1'b1, 1'b0, 1'b1, 1'b1);
          end
          if (cmd_valid_reg && cmd_ready) begin
            state <= ST_TCA_WAIT;
          end
        end

        ST_TCA_WAIT: begin
          // PATCH: wait for i2c_master AND both FIFOs to drain
          if (i2c_all_idle) begin
            if (mode == 2'd3) begin
              state <= ST_INIT_CTRL;
            end else if (mode == 2'd0) begin
              state <= ST_NAME_LOAD0;
              hdr_i <= 0;
            end else if (mode == 2'd1) begin
              //state <= ST_PAN_HDR;
              //state <= ST_P4_CMD;
              state <= ST_PAN_HDR;
              hdr_i <= 0;
            end else begin
              state <= ST_MET_HDR;
              hdr_i <= 0;
            end
          end
        end

        // ---------------------------
        // INIT: send 0x00 + init bytes
        ST_INIT_CTRL: begin
          init_ptr <= 0;
          if (!data_tvalid_reg) begin
            issue_data(8'h00, 1'b0);  // control byte for commands
          end
          if (data_tvalid_reg && data_tready) begin
            state <= ST_INIT_BYTES;
          end
        end

        ST_INIT_BYTES: begin
          if (!data_tvalid_reg) begin
            issue_data(init_cmd(init_ptr), (init_ptr == 5'd31));
          end
          if (data_tvalid_reg && data_tready) begin
            if (init_ptr == 5'd31) begin
              state <= ST_INIT_CMD;
            end else begin
              init_ptr <= init_ptr + 1'b1;
            end
          end
        end

        ST_INIT_CMD: begin
          if (!cmd_valid_reg) begin
            issue_cmd(SSD1306_ADDR, 1'b1, 1'b0, 1'b1, 1'b1);
          end
          if (cmd_valid_reg && cmd_ready) begin
            state <= ST_INIT_WAIT;
          end
        end

        ST_INIT_WAIT: begin
          // PATCH: wait for drain
          if (i2c_all_idle) begin
            clr_hdr_i <= 0;
            state <= ST_CLR_HDR;
          end
        end

        // ---------------------------
        // CLEAR transfer 1: command window
        ST_CLR_HDR: begin
          if (!data_tvalid_reg) begin
            issue_data(hdr_clear_cmd(clr_hdr_i), (clr_hdr_i == 4'd6));
          end
          if (data_tvalid_reg && data_tready) begin
            if (clr_hdr_i == 4'd6) begin
              state <= ST_CLR_CMD;
            end else begin
              clr_hdr_i <= clr_hdr_i + 1'b1;
            end
          end
        end

        ST_CLR_CMD: begin
          if (!cmd_valid_reg) begin
            issue_cmd(SSD1306_ADDR, 1'b1, 1'b0, 1'b1, 1'b1);
          end
          if (cmd_valid_reg && cmd_ready) begin
            state <= ST_CLR_WAIT;
          end
        end

        ST_CLR_WAIT: begin
          // PATCH: wait for drain
          if (i2c_all_idle) begin
            clr_cnt <= 0;
            state   <= ST_CLR_DATA0;
          end
        end

        // CLEAR transfer 2: 0x40 then 1024 bytes 0x00
        ST_CLR_DATA0: begin
          if (!data_tvalid_reg) begin
            issue_data(8'h40, 1'b0);
          end
          if (data_tvalid_reg && data_tready) begin
            state <= ST_CLR_DATA;
          end
        end

        ST_CLR_DATA: begin
          if (!data_tvalid_reg) begin
            issue_data(8'h00, (clr_cnt == 10'd1023));
          end
          if (data_tvalid_reg && data_tready) begin
            if (clr_cnt == 10'd1023) begin
              state <= ST_CLR_DATA_CMD;
            end else begin
              clr_cnt <= clr_cnt + 1'b1;
            end
          end
        end

        ST_CLR_DATA_CMD: begin
          if (!cmd_valid_reg) begin
            issue_cmd(SSD1306_ADDR, 1'b1, 1'b0, 1'b1, 1'b1);
          end
          if (cmd_valid_reg && cmd_ready) begin
            state <= ST_CLR_DATA_WAIT;
          end
        end

        ST_CLR_DATA_WAIT: begin
          // PATCH: wait for drain
          if (i2c_all_idle) begin
            //clr_hdr_i <= 0;
            //clr_cnt <= 0;
            //state <= ST_TST_HDR;
            // Fuer Label
            //did_init[disp_idx] <= 1'b1;
            //disp_rr <= disp_rr + 1'b1;
            //state <= ST_IDLE;
            hdr_i <= 0;
            state <= ST_PAN_LBL_HDR;
          end
        end

        // --------------------------------------------------
        // TEST: set window to (col=0,page=0) and write 0x40 + 0xFF
        ST_TST_HDR: begin
          if (!data_tvalid_reg) begin
            issue_data(hdr_test_cmd(clr_hdr_i), (clr_hdr_i == 4'd6));
          end
          if (data_tvalid_reg && data_tready) begin
            if (clr_hdr_i == 4'd6) begin
              state <= ST_TST_CMD;
            end else begin
              clr_hdr_i <= clr_hdr_i + 1'b1;
            end
          end
        end

        ST_TST_CMD: begin
          if (!cmd_valid_reg) begin
            issue_cmd(SSD1306_ADDR, 1'b1, 1'b0, 1'b1, 1'b1);
          end
          if (cmd_valid_reg && cmd_ready) begin
            state <= ST_TST_WAIT;
          end
        end

        ST_TST_WAIT: begin
          // PATCH: wait for drain
          if (i2c_all_idle) begin
            state <= ST_TST_DATA0;
            //state <= ST_PAN_CMD_WAIT;
          end
        end

        ST_TST_DATA0: begin
          if (!data_tvalid_reg) begin
            issue_data(8'h40, 1'b0);
          end
          if (data_tvalid_reg && data_tready) begin
            state <= ST_TST_DATA1;
          end
        end

        ST_TST_DATA1: begin
          if (!data_tvalid_reg) begin
            issue_data(8'hFF, (clr_cnt == 10'd20));
          end
          if (data_tvalid_reg && data_tready) begin
            if (clr_cnt == 10'd20) begin
              state <= ST_TST_DATA_CMD;
            end else begin
              clr_cnt <= clr_cnt + 1'b1;
            end
          end
        end

        ST_TST_DATA_CMD: begin
          if (!cmd_valid_reg) begin
            issue_cmd(SSD1306_ADDR, 1'b1, 1'b0, 1'b1, 1'b1);
          end
          if (cmd_valid_reg && cmd_ready) begin
            state <= ST_TST_DATA_WAIT;
          end
        end

        ST_TST_DATA_WAIT: begin
          // PATCH: wait for drain
          if (i2c_all_idle) begin
            did_init[disp_idx] <= 1'b1;
            disp_rr <= disp_rr + 1'b1;
            state <= ST_IDLE;
          end
        end


        // ---------------------------
        // PAN LABEL: header (page 5) then 96 bytes text row "L ... 0 ... R"
        ST_PAN_LBL_HDR: begin
          if (!data_tvalid_reg) begin
            issue_data(hdr_pan_lbl(hdr_i), (hdr_i == 4'd6));
          end
          if (data_tvalid_reg && data_tready) begin
            if (hdr_i == 4'd6) begin
              x96   <= 0;
              state <= ST_PAN_LBL_CMD;
            end else begin
              hdr_i <= hdr_i + 1'b1;
            end
          end
        end

        ST_PAN_LBL_CMD: begin
          if (!cmd_valid_reg) begin
            issue_cmd(SSD1306_ADDR, 1'b1, 1'b0, 1'b1, 1'b1);
          end
          if (cmd_valid_reg && cmd_ready) begin
            state <= ST_PAN_LBL_WAIT;
          end
        end

        ST_PAN_LBL_WAIT: begin
          if (i2c_all_idle) begin
            state <= ST_PAN_LBL_DATA0;
          end
        end

        ST_PAN_LBL_DATA0: begin
          if (!data_tvalid_reg) begin
            issue_data(8'h40, 1'b0);
          end
          if (data_tvalid_reg && data_tready) begin
            state <= ST_PAN_LBL_DATA1;
          end
        end

        ST_PAN_LBL_DATA1: begin

          if (!data_tvalid_reg) begin
            issue_data(pan_label_byte(x96), (x96 == 7'd95));
          end

          if (data_tvalid_reg && data_tready) begin
            if (x96 == 7'd95) begin
              state <= ST_PAN_LBL_DATA_CMD;
            end else begin
              x96 <= x96 + 1'b1;
            end
          end
        end

        ST_PAN_LBL_DATA_CMD: begin
          if (!cmd_valid_reg) begin
            issue_cmd(SSD1306_ADDR, 1'b1, 1'b0, 1'b1, 1'b1);
          end
          if (cmd_valid_reg && cmd_ready) begin
            state <= ST_PAN_LBL_DATA_WAIT;
          end
        end

        ST_PAN_LBL_DATA_WAIT: begin
          // PATCH: wait for drain
          if (i2c_all_idle) begin
            did_init[disp_idx] <= 1'b1;
            disp_rr <= disp_rr + 1'b1;
            state <= ST_IDLE;
          end
        end


        // --------------------------------------------------
        // TEST: set window to (col=0,page=0) and write 0x40 + 0xFF
        ST_PAN_HDR: begin
          if (!data_tvalid_reg) begin
            issue_data(hdr_test_cmd(clr_hdr_i), (clr_hdr_i == 4'd6));
          end
          if (data_tvalid_reg && data_tready) begin
            if (clr_hdr_i == 4'd6) begin
              state <= ST_PAN_CMD;
            end else begin
              clr_hdr_i <= clr_hdr_i + 1'b1;
            end
          end
        end

        ST_PAN_CMD: begin
          if (!cmd_valid_reg) begin
            issue_cmd(SSD1306_ADDR, 1'b1, 1'b0, 1'b1, 1'b1);
          end
          if (cmd_valid_reg && cmd_ready) begin
            state <= ST_PAN_WAIT;
          end
        end

        ST_PAN_WAIT: begin
          if (i2c_all_idle) begin
            // Pan-Geometrie vorberechnen (einmal pro Update)
            if (!pan_calc_busy) begin
              pan_calc_busy <= 1'b1;
              pan_calc_cnt  <= 3'd0;
            end
            // erst weiter, wenn Berechnung fertig
            if (!pan_calc_busy) begin
              state <= ST_PAN_DATA0;
            end
          end
        end

        ST_PAN_DATA0: begin
          if (!data_tvalid_reg) begin
            issue_data(8'h40, 1'b0);
          end
          if (data_tvalid_reg && data_tready) begin
            state <= ST_PAN_DATA1;
          end
        end

        ST_PAN_DATA1: begin
          if (!data_tvalid_reg) begin
            if (x96 == 7'd48) begin
              issue_data(8'hFF, (x96 == 7'd95));
            end else if (pan_off_r4 != 0 && (x96 >= pan_x0_r5) && (x96 <= pan_x1_r5)) begin
              issue_data(8'hFF, (x96 == 7'd95));
            end else begin
              issue_data(8'h00, (x96 == 7'd95));
            end
          end

          if (data_tvalid_reg && data_tready) begin
            if (x96 == 7'd95) begin
              state <= ST_PAN_DATA_CMD;
            end else begin
              x96 <= x96 + 1'b1;
            end
          end
        end

        ST_PAN_DATA_CMD: begin
          if (!cmd_valid_reg) begin
            issue_cmd(SSD1306_ADDR, 1'b1, 1'b0, 1'b1, 1'b1);
          end
          if (cmd_valid_reg && cmd_ready) begin
            state <= ST_PAN_DATA_WAIT;
          end
        end

        ST_PAN_DATA_WAIT: begin
          // PATCH: wait for drain
          if (i2c_all_idle) begin
            pan_dirty[disp_idx] <= 1'b0;
            state <= ST_IDLE;
          end
        end

        // ---------------------------
        // NAME load 16 bytes from BRAM into cache (assume 1-cycle read latency)
        ST_NAME_LOAD0: begin
          init_ptr <= 0;
          name_bram_addr <= {disp_idx, 4'h0};
          state <= ST_NAME_LOAD1;
        end

        ST_NAME_LOAD1: begin
          state <= ST_NAME_LOAD2;
        end

        ST_NAME_LOAD2: begin
          name_cache[init_ptr[3:0]] <= name_bram_dout;
          if (init_ptr == 4'd15) begin
            hdr_i          <= 0;
            name_page      <= 1'b0;
            name_char      <= 4'd0;
            name_col       <= 3'd0;
            name_chunk_cnt <= 8'd0;
            state          <= ST_NAME_HDR;
          end else begin
            init_ptr <= init_ptr + 1'b1;
            name_bram_addr <= {disp_idx, (init_ptr[3:0] + 1'b1)};
            state <= ST_NAME_LOAD1;
          end
        end

        // --------------------------------------------------
        // NAME: header -> cmd -> wait -> data stream (0x40 + bytes in 32er-chunks)
        // --------------------------------------------------

        ST_NAME_HDR: begin
          if (!data_tvalid_reg) begin
            issue_data(hdr_name(hdr_i, name_page), (hdr_i == 4'd6));
          end
          if (data_tvalid_reg && data_tready) begin
            if (hdr_i == 4'd6) begin
              state <= ST_NAME_CMD;
            end else begin
              hdr_i <= hdr_i + 1'b1;
            end
          end
        end

        ST_NAME_CMD: begin
          if (!cmd_valid_reg) begin
            issue_cmd(SSD1306_ADDR, 1'b1, 1'b0, 1'b1, 1'b1);
          end
          if (cmd_valid_reg && cmd_ready) begin
            state <= ST_NAME_WAIT;
          end
        end

        ST_NAME_WAIT: begin
          if (i2c_all_idle) begin
            // Start transfer with 0x40
            state <= ST_NAME_DATA0;
          end
        end

        ST_NAME_DATA0: begin
          if (!data_tvalid_reg) begin
            issue_data(8'h40, 1'b0);
          end
          if (data_tvalid_reg && data_tready) begin
            state <= ST_NAME_FONT_ADDR;
          end
        end

        // Font BRAM addr anlegen (sync ROM -> next state waits 1 cycle)
        ST_NAME_FONT_ADDR: begin
          font_addr <= {name_page, name_cache[name_char][6:0], name_col[2:0]};
          state <= ST_NAME_FONT_WAIT;
        end

        ST_NAME_FONT_WAIT: begin
          // 1-cycle latency for font_dout
          state <= ST_NAME_DATA;
        end

        ST_NAME_DATA: begin
          if (!data_tvalid_reg) begin
            // last, wenn Chunk zu Ende oder Page komplett zu Ende
            // Page-Ende: char==15 und col==7
            // Chunk-Ende: name_chunk_cnt==31
            issue_data(
                font_dout,
                ((name_chunk_cnt == 8'd127) || ((name_char == 4'd15) && (name_col == 3'd7))));
          end

          if (data_tvalid_reg && data_tready) begin
            // counters advance
            name_chunk_cnt <= name_chunk_cnt + 1'b1;

            if (name_col == 3'd7) begin
              //name_col <= 3'd0;
              if (name_char == 4'd15) begin
                // end of page bytes (128 bytes)
                state <= ST_NAME_DATA_CMD;
              end else begin
                name_char <= name_char + 1'b1;
                // next glyph/col
                if (name_chunk_cnt == 8'd127) begin
                  state <= ST_NAME_DATA_CMD;
                end else begin
                  name_col <= 3'd0;
                  state <= ST_NAME_FONT_ADDR;
                end
              end
            end else begin
              name_col <= name_col + 1'b1;
              if (name_chunk_cnt == 8'd127) begin
                state <= ST_NAME_DATA_CMD;
              end else begin
                state <= ST_NAME_FONT_ADDR;
              end
            end
          end
        end

        ST_NAME_DATA_CMD: begin
          if (!cmd_valid_reg) begin
            issue_cmd(SSD1306_ADDR, 1'b1, 1'b0, 1'b1, 1'b1);
          end
          if (cmd_valid_reg && cmd_ready) begin
            state <= ST_NAME_DATA_WAIT;
          end
        end

        ST_NAME_DATA_WAIT: begin
          if (i2c_all_idle) begin
            // Chunk beendet -> reset chunk counter und wieder 0x40
            if (name_chunk_cnt >= 8'd128) begin
              name_chunk_cnt <= 8'd0;
            end

            // Falls Page fertig (char=15,col=7 wurde gesendet)
            if ((name_char == 4'd15) && (name_col == 3'd7)) begin
              // Wir sind nach dem Senden bereits im "Ende Page" Pfad.
              // Next page or finished:
              if (name_page == 1'b0) begin
                name_page      <= 1'b1;
                name_char      <= 4'd0;
                name_col       <= 3'd0;
                name_chunk_cnt <= 8'd0;
                hdr_i          <= 4'd0;
                state          <= ST_NAME_HDR;
              end else begin
                // Done (or back to round-robin)
                name_dirty[disp_idx] <= 1'b0;
                state <= ST_IDLE;
              end
            end else begin
              // Not yet page-end -> continue with next chunk
              state <= ST_NAME_DATA0;
            end
          end
        end


        // ---------------------------
        // METER: header then 224 bytes (page 1..7, col 0..31)
        ST_MET_HDR: begin
          if (!data_tvalid_reg) begin
            issue_data(hdr_meter(hdr_i), (hdr_i == 4'd6));
          end
          if (data_tvalid_reg && data_tready) begin
            if (hdr_i == 4'd6) begin
              met_page <= 3'd1;
              x32      <= 5'd0;
              state    <= ST_MET_HDR_CMD;
            end else begin
              hdr_i <= hdr_i + 1'b1;
            end
          end
        end

        ST_MET_HDR_CMD: begin
          if (!cmd_valid_reg) begin
            issue_cmd(SSD1306_ADDR, 1'b1, 1'b0, 1'b1, 1'b1);
          end
          if (cmd_valid_reg && cmd_ready) begin
            state <= ST_MET_WAIT0;
          end
        end

        ST_MET_WAIT0: begin
          if (i2c_all_idle) begin
            met_col_pm <= 0;
            met_page_pm <= 1;
            x16 <= 4'd0;
            state <= ST_MET_DATA0;
          end
        end

        ST_MET_DATA0: begin
          if (!data_tvalid_reg) issue_data(8'h40, 1'b0);
          if (data_tvalid_reg && data_tready) state <= ST_MET_DATA;
        end


        ST_MET_DATA: begin
          if (!data_tvalid_reg) begin
            issue_data(render_meter_byte_boxes16(
                       meter_lvl[disp_idx], peakhold_lvl[disp_idx], met_page, x16),
                       (met_page == 3'd7 && x16 == 4'd15));
          end
          if (data_tvalid_reg && data_tready) begin
            if (met_page == 3'd7 && x16 == 4'd15) begin
              state <= ST_MET_DATACMD;
            end else begin
              if (x16 == 4'd15) begin
                x16 <= 4'd0;
                met_page <= met_page + 1'b1;
              end else begin
                x16 <= x16 + 1'b1;
              end
            end
          end
        end

        ST_MET_DATACMD: begin
          if (!cmd_valid_reg) issue_cmd(SSD1306_ADDR, 1'b1, 1'b0, 1'b1, 1'b1);
          if (cmd_valid_reg && cmd_ready) state <= ST_MET_WAIT1;
        end

        ST_MET_WAIT1: begin
          if (i2c_all_idle) begin
            // next display for meter rr
            disp_rr <= disp_rr + 1'b1;
            state   <= ST_IDLE;
          end
        end

        default: state <= ST_IDLE;
      endcase

      // optional: handle missed_ack
      // if (missed_ack) begin
      // end
    end
  end

endmodule

`default_nettype wire
