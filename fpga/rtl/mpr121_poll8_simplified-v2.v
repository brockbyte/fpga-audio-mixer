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
// MPR121 Capacitive Touch Sensor Controller (8-Channel Simplified)
// I2C-based multi-touch polling with debounce filtering and threshold control
// Features: 8 independent touch channels, configurable touch/release thresholds
// Functionality: Periodic I2C polling, register initialization, debounce FSM
// Output: 8-bit debounced touch status with 1-cycle valid pulse per poll update
// ============================================================================
module mpr121_poll8_simplified #(
    parameter integer       CLK_HZ         = 50000000,
    parameter integer       I2C_HZ         = 400000,
    parameter         [6:0] I2C_ADDR       = 7'h5A,
    parameter integer       POLL_MS        = 1,
    parameter         [7:0] TOUCH_TH       = 8'd20,
    parameter         [7:0] RELEASE_TH     = 8'd10,
    // Debounce: Number of poll cycles (at 1 ms: 10 => 10 ms debounce)
    parameter integer       DEBOUNCE_TICKS = 10
) (
    input wire clk,
    input wire rst,

    // I2C Pins (mit externen Pullups!)
    inout wire i2c_scl,
    inout wire i2c_sda,

    // Ergebnis (DEBOUNCED!)
    output reg [7:0] touch8,
    output reg       touch_valid,  // 1-Takt-Puls pro neuem Roh-Poll
    output reg       init_done,

    // Status aus i2c_master (nur zum Debug)
    output wire busy,
    output wire bus_control,
    output wire bus_active,
    output wire missed_ack
);

  // ------------------------------------------------------------------------
  // 1 ms Tick
  // ------------------------------------------------------------------------
  localparam integer TICK_CYCLES = (CLK_HZ / 1000) * POLL_MS;
  reg [31:0] tick_cnt = 0;
  reg        tick_1ms = 1'b0;

  always @(posedge clk) begin
    if (rst) begin
      tick_cnt <= 0;
      tick_1ms <= 1'b0;
    end else begin
      tick_1ms <= 1'b0;
      if (tick_cnt == TICK_CYCLES - 1) begin
        tick_cnt <= 0;
        tick_1ms <= 1'b1;
      end else begin
        tick_cnt <= tick_cnt + 1;
      end
    end
  end

  // ------------------------------------------------------------------------
  // Forencich i2c_master Host-Interface Signale
  // ------------------------------------------------------------------------
  reg  [6:0] s_axis_cmd_address_reg = 7'd0;
  reg        s_axis_cmd_start_reg = 1'b0;
  reg        s_axis_cmd_read_reg = 1'b0;
  reg        s_axis_cmd_write_reg = 1'b0;
  reg        s_axis_cmd_write_multiple_reg = 1'b0;
  reg        s_axis_cmd_stop_reg = 1'b0;
  reg        s_axis_cmd_valid_reg = 1'b0;
  wire       s_axis_cmd_ready;

  reg  [7:0] s_axis_data_tdata_reg = 8'd0;
  reg        s_axis_data_tvalid_reg = 1'b0;
  wire       s_axis_data_tready;
  reg        s_axis_data_tlast_reg = 1'b0;

  wire [7:0] m_axis_data_tdata;
  wire       m_axis_data_tvalid;
  reg        m_axis_data_tready_reg = 1'b0;
  wire       m_axis_data_tlast;

  localparam [15:0] I2C_PRESCALE = (CLK_HZ / (I2C_HZ * 4)) - 1;

  // ------------------------------------------------------------------------
  // I2C-Pad-Treiber (open drain)
  // ------------------------------------------------------------------------
  wire scl_i, scl_o, scl_t;
  wire sda_i, sda_o, sda_t;

  assign scl_i   = i2c_scl;
  assign i2c_scl = scl_t ? 1'bz : scl_o;

  assign sda_i   = i2c_sda;
  assign i2c_sda = sda_t ? 1'bz : sda_o;

  // ------------------------------------------------------------------------
  // i2c_master Instanz
  // ------------------------------------------------------------------------
  i2c_master i2c_master_inst (
      .clk(clk),
      .rst(rst),

      .s_axis_cmd_address       (s_axis_cmd_address_reg),
      .s_axis_cmd_start         (s_axis_cmd_start_reg),
      .s_axis_cmd_read          (s_axis_cmd_read_reg),
      .s_axis_cmd_write         (s_axis_cmd_write_reg),
      .s_axis_cmd_write_multiple(s_axis_cmd_write_multiple_reg),
      .s_axis_cmd_stop          (s_axis_cmd_stop_reg),
      .s_axis_cmd_valid         (s_axis_cmd_valid_reg),
      .s_axis_cmd_ready         (s_axis_cmd_ready),

      .s_axis_data_tdata (s_axis_data_tdata_reg),
      .s_axis_data_tvalid(s_axis_data_tvalid_reg),
      .s_axis_data_tready(s_axis_data_tready),
      .s_axis_data_tlast (s_axis_data_tlast_reg),

      .m_axis_data_tdata (m_axis_data_tdata),
      .m_axis_data_tvalid(m_axis_data_tvalid),
      .m_axis_data_tready(m_axis_data_tready_reg),
      .m_axis_data_tlast (m_axis_data_tlast),

      .scl_i(scl_i),
      .scl_o(scl_o),
      .scl_t(scl_t),
      .sda_i(sda_i),
      .sda_o(sda_o),
      .sda_t(sda_t),

      .busy       (busy),
      .bus_control(bus_control),
      .bus_active (bus_active),
      .missed_ack (missed_ack),

      .prescale   (I2C_PRESCALE),
      .stop_on_idle(1'b1)
  );

  // ------------------------------------------------------------------------
  // Init-ROM (reg, val)
  // ------------------------------------------------------------------------
  localparam integer INIT_LEN = 33;

  reg [7:0] init_idx = 8'd0;
  reg [7:0] init_reg;
  reg [7:0] init_val;

  always @* begin
    init_reg = 8'h00;
    init_val = 8'h00;
    case (init_idx)
      8'd0: begin
        init_reg = 8'h80;
        init_val = 8'h63;
      end  // SRST
      8'd1: begin
        init_reg = 8'h5E;
        init_val = 8'h00;
      end  // ECR stop
      // Filter
      8'd2: begin
        init_reg = 8'h2B;
        init_val = 8'h01;
      end
      8'd3: begin
        init_reg = 8'h2C;
        init_val = 8'h01;
      end
      8'd4: begin
        init_reg = 8'h2D;
        init_val = 8'h00;
      end
      8'd5: begin
        init_reg = 8'h2E;
        init_val = 8'h00;
      end
      8'd6: begin
        init_reg = 8'h2F;
        init_val = 8'h01;
      end
      8'd7: begin
        init_reg = 8'h30;
        init_val = 8'h01;
      end
      8'd8: begin
        init_reg = 8'h31;
        init_val = 8'hFF;
      end
      8'd9: begin
        init_reg = 8'h32;
        init_val = 8'h02;
      end
      8'd10: begin
        init_reg = 8'h33;
        init_val = 8'h00;
      end
      8'd11: begin
        init_reg = 8'h34;
        init_val = 8'h00;
      end
      8'd12: begin
        init_reg = 8'h35;
        init_val = 8'h00;
      end
      // CONFIG1/2
      8'd13: begin
        init_reg = 8'h5C;
        init_val = 8'h10;
      end
      8'd14: begin
        init_reg = 8'h5D;
        init_val = 8'h20;
      end
      // Debounce
      8'd15: begin
        init_reg = 8'h5B;
        init_val = 8'h22;
      end
      // Thresholds e0..e7
      8'd16: begin
        init_reg = 8'h41;
        init_val = TOUCH_TH;
      end
      8'd17: begin
        init_reg = 8'h42;
        init_val = RELEASE_TH;
      end
      8'd18: begin
        init_reg = 8'h43;
        init_val = TOUCH_TH;
      end
      8'd19: begin
        init_reg = 8'h44;
        init_val = RELEASE_TH;
      end
      8'd20: begin
        init_reg = 8'h45;
        init_val = TOUCH_TH;
      end
      8'd21: begin
        init_reg = 8'h46;
        init_val = RELEASE_TH;
      end
      8'd22: begin
        init_reg = 8'h47;
        init_val = TOUCH_TH;
      end
      8'd23: begin
        init_reg = 8'h48;
        init_val = RELEASE_TH;
      end
      8'd24: begin
        init_reg = 8'h49;
        init_val = TOUCH_TH;
      end
      8'd25: begin
        init_reg = 8'h4A;
        init_val = RELEASE_TH;
      end
      8'd26: begin
        init_reg = 8'h4B;
        init_val = TOUCH_TH;
      end
      8'd27: begin
        init_reg = 8'h4C;
        init_val = RELEASE_TH;
      end
      8'd28: begin
        init_reg = 8'h4D;
        init_val = TOUCH_TH;
      end
      8'd29: begin
        init_reg = 8'h4E;
        init_val = RELEASE_TH;
      end
      8'd30: begin
        init_reg = 8'h4F;
        init_val = TOUCH_TH;
      end
      8'd31: begin
        init_reg = 8'h50;
        init_val = RELEASE_TH;
      end
      // ECR: 8 Elektroden aktiv
      8'd32: begin
        init_reg = 8'h5E;
        init_val = 8'h88;
      end
      default: begin
        init_reg = 8'h5E;
        init_val = 8'h00;
      end
    endcase
  end

  // ------------------------------------------------------------------------
  // High-Level FSM (Init + Poll) - unchanged, but with touch_raw8
  // ------------------------------------------------------------------------
  localparam [3:0]
ST_INIT_START   = 4'd0,
ST_INIT_CMD     = 4'd1,
ST_INIT_DATA0   = 4'd2,
ST_INIT_DATA1   = 4'd3,

ST_RUN_WAIT     = 4'd4,
ST_PTR_CMD      = 4'd5,
ST_PTR_DATA     = 4'd6,

ST_R0_CMD       = 4'd7,
ST_R0_RECV      = 4'd8,

ST_R1_CMD       = 4'd9,
ST_R1_RECV      = 4'd10;

  reg [3:0] state = ST_INIT_START;

  reg [7:0] status_lsb = 8'h00;
  reg [7:0] status_msb = 8'h00;

  // Roh-Touch (direkt vom MPR121), vor Debounce
  reg [7:0] touch_raw8 = 8'h00;

  always @(posedge clk) begin
    if (rst) begin
      s_axis_cmd_address_reg        <= 7'd0;
      s_axis_cmd_start_reg          <= 1'b0;
      s_axis_cmd_read_reg           <= 1'b0;
      s_axis_cmd_write_reg          <= 1'b0;
      s_axis_cmd_write_multiple_reg <= 1'b0;
      s_axis_cmd_stop_reg           <= 1'b0;
      s_axis_cmd_valid_reg          <= 1'b0;

      s_axis_data_tdata_reg         <= 8'd0;
      s_axis_data_tvalid_reg        <= 1'b0;
      s_axis_data_tlast_reg         <= 1'b0;

      m_axis_data_tready_reg        <= 1'b0;

      init_idx                      <= 8'd0;
      init_done                     <= 1'b0;
      //touch8        <= 8'h00;
      touch_raw8                    <= 8'h00;
      touch_valid                   <= 1'b0;
      status_lsb                    <= 8'h00;
      status_msb                    <= 8'h00;

      state                         <= ST_INIT_START;
    end else begin
      touch_valid <= 1'b0;

      case (state)
        // --- INIT: For each entry, one Write transaction
        ST_INIT_START: begin
          if (init_done) begin
            state <= ST_RUN_WAIT;
          end else begin
            s_axis_cmd_address_reg        <= I2C_ADDR;
            s_axis_cmd_start_reg          <= 1'b1;
            s_axis_cmd_read_reg           <= 1'b0;
            s_axis_cmd_write_reg          <= 1'b1;
            s_axis_cmd_write_multiple_reg <= 1'b1;
            s_axis_cmd_stop_reg           <= 1'b1;
            s_axis_cmd_valid_reg          <= 1'b1;

            s_axis_data_tdata_reg         <= init_reg;
            s_axis_data_tlast_reg         <= 1'b0;
            s_axis_data_tvalid_reg        <= 1'b0;

            state                         <= ST_INIT_CMD;
          end
        end

        ST_INIT_CMD: begin
          if (s_axis_cmd_valid_reg && s_axis_cmd_ready) begin
            s_axis_cmd_valid_reg <= 1'b0;
            s_axis_data_tvalid_reg <= 1'b1;
            state <= ST_INIT_DATA0;
          end
        end

        ST_INIT_DATA0: begin
          if (s_axis_data_tvalid_reg && s_axis_data_tready) begin
            s_axis_data_tdata_reg <= init_val;
            s_axis_data_tlast_reg <= 1'b1;
            state <= ST_INIT_DATA1;
          end
        end

        ST_INIT_DATA1: begin
          if (s_axis_data_tvalid_reg && s_axis_data_tready) begin
            s_axis_data_tvalid_reg <= 1'b0;
            s_axis_data_tlast_reg  <= 1'b0;

            if (init_idx == INIT_LEN - 1) begin
              init_done <= 1'b1;
              state     <= ST_RUN_WAIT;
            end else begin
              init_idx <= init_idx + 1;
              state    <= ST_INIT_START;
            end
          end
        end

        // --- RUN: Poll alle 1 ms -------------------------------------------
        ST_RUN_WAIT: begin
          s_axis_cmd_valid_reg   <= 1'b0;
          s_axis_data_tvalid_reg <= 1'b0;
          m_axis_data_tready_reg <= 1'b0;

          if (tick_1ms) begin
            // 1) Pointer auf 0x00
            s_axis_cmd_address_reg        <= I2C_ADDR;
            s_axis_cmd_start_reg          <= 1'b1;
            s_axis_cmd_read_reg           <= 1'b0;
            s_axis_cmd_write_reg          <= 1'b1;
            s_axis_cmd_write_multiple_reg <= 1'b0;
            s_axis_cmd_stop_reg           <= 1'b1;
            s_axis_cmd_valid_reg          <= 1'b1;

            s_axis_data_tdata_reg         <= 8'h00;
            s_axis_data_tlast_reg         <= 1'b1;
            s_axis_data_tvalid_reg        <= 1'b0;

            state                         <= ST_PTR_CMD;
          end
        end

        ST_PTR_CMD: begin
          if (s_axis_cmd_valid_reg && s_axis_cmd_ready) begin
            s_axis_cmd_valid_reg   <= 1'b0;
            s_axis_data_tvalid_reg <= 1'b1;
            state                  <= ST_PTR_DATA;
          end
        end

        ST_PTR_DATA: begin
          if (s_axis_data_tvalid_reg && s_axis_data_tready) begin
            s_axis_data_tvalid_reg        <= 1'b0;
            s_axis_data_tlast_reg         <= 1'b0;

            // 2) Read LSB (0x00)
            s_axis_cmd_address_reg        <= I2C_ADDR;
            s_axis_cmd_start_reg          <= 1'b1;
            s_axis_cmd_read_reg           <= 1'b1;
            s_axis_cmd_write_reg          <= 1'b0;
            s_axis_cmd_write_multiple_reg <= 1'b0;
            s_axis_cmd_stop_reg           <= 1'b1;
            s_axis_cmd_valid_reg          <= 1'b1;

            m_axis_data_tready_reg        <= 1'b0;
            state                         <= ST_R0_CMD;
          end
        end

        ST_R0_CMD: begin
          if (s_axis_cmd_valid_reg && s_axis_cmd_ready) begin
            s_axis_cmd_valid_reg   <= 1'b0;
            m_axis_data_tready_reg <= 1'b1;
            state                  <= ST_R0_RECV;
          end
        end

        ST_R0_RECV: begin
          if (m_axis_data_tvalid) begin
            status_lsb                    <= m_axis_data_tdata;
            m_axis_data_tready_reg        <= 1'b0;

            // 3) Read MSB (0x01)
            s_axis_cmd_address_reg        <= I2C_ADDR;
            s_axis_cmd_start_reg          <= 1'b1;
            s_axis_cmd_read_reg           <= 1'b1;
            s_axis_cmd_write_reg          <= 1'b0;
            s_axis_cmd_write_multiple_reg <= 1'b0;
            s_axis_cmd_stop_reg           <= 1'b1;
            s_axis_cmd_valid_reg          <= 1'b1;

            state                         <= ST_R1_CMD;
          end
        end

        ST_R1_CMD: begin
          if (s_axis_cmd_valid_reg && s_axis_cmd_ready) begin
            s_axis_cmd_valid_reg   <= 1'b0;
            m_axis_data_tready_reg <= 1'b1;
            state                  <= ST_R1_RECV;
          end
        end

        ST_R1_RECV: begin
          if (m_axis_data_tvalid) begin
            status_msb <= m_axis_data_tdata;
            m_axis_data_tready_reg <= 1'b0;

            // Capture raw touch
            touch_raw8 <= status_lsb[7:0];
            touch_valid <= 1'b1;  // 1 Puls pro neuem Rohwert

            state <= ST_RUN_WAIT;
          end
        end

        default: state <= ST_INIT_START;
      endcase
    end
  end

  // ------------------------------------------------------------------------
  // Debounce: auf Basis von touch_raw8 und touch_valid
  // ------------------------------------------------------------------------
  integer i;
  reg [7:0] touch8_next;
  // Per-channel up/down counter
  reg [7:0] cnt[0:7];  // bis max 255 Ticks (reicht hier locker)

  always @(posedge clk) begin
    if (rst) begin
      touch8 <= 8'h00;
      for (i = 0; i < 8; i = i + 1) begin
        cnt[i] <= 0;
      end
    end else begin
      if (touch_valid) begin
        // nur einmal pro Roh-Update
        for (i = 0; i < 8; i = i + 1) begin
          // Up/down counter
          if (touch_raw8[i]) begin
            if (cnt[i] < DEBOUNCE_TICKS) cnt[i] <= cnt[i] + 1'b1;
          end else begin
            if (cnt[i] > 0) cnt[i] <= cnt[i] - 1'b1;
          end

          // Output-Regel:
          //  - ab DEBOUNCE_TICKS => 1
          //  - bei 0             => 0
          if (cnt[i] >= DEBOUNCE_TICKS) touch8[i] <= 1'b1;
          else if (cnt[i] == 0) touch8[i] <= 1'b0;
          // Zwischenwerte: letzter Zustand bleibt
        end
      end
    end
  end

endmodule

`default_nettype wire
