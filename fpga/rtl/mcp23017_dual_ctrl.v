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
// MCP23017 Dual Port Expander I2C Controller
// Controls two MCP23017 16-bit I/O expanders (16 buttons + 16 LED outputs)
// Features debounced button inputs with FIFO buffering and LED output control
// Supports polling-based status monitoring and control command transmission
// ============================================================================
module mcp23017_dual_ctrl #(
    parameter integer CLK_HZ      = 50000000,
    parameter integer POLL_MS     = 2,
    parameter integer DEBOUNCE_MS = 20,

    parameter [6:0] MCP0_ADDR = 7'h22,
    parameter [6:0] MCP1_ADDR = 7'h23,

    parameter integer CMD_FIFO_DEPTH  = 16,
    parameter integer DATA_FIFO_DEPTH = 64
) (
    input wire clk,
    input wire rst,

    inout wire i2c_scl,
    inout wire i2c_sda,

    input wire [7:0] cur_status0,  //mute
    input wire [7:0] cur_status1,  //solo

    output wire [7:0] status0,  //mute
    output wire [7:0] status1,  //solo

    output wire set_mute,
    output wire set_solo
);

  // -------------------------------------------------------------------------
  // MCP23017 regs (BANK=0)
  // Port A: LEDs (output) -> OLATA
  // Port B: Buttons (input) -> GPIOB, pullups -> GPPUB
  // -------------------------------------------------------------------------
  localparam [7:0] MCP_REG_IODIRA = 8'h00;
  localparam [7:0] MCP_REG_IODIRB = 8'h01;
  localparam [7:0] MCP_REG_GPPUA = 8'h0C;
  localparam [7:0] MCP_REG_GPPUB = 8'h0D;
  localparam [7:0] MCP_REG_GPIOA = 8'h12;
  localparam [7:0] MCP_REG_GPIOB = 8'h13;
  localparam [7:0] MCP_REG_OLATA = 8'h14;
  localparam [7:0] MCP_REG_OLATB = 8'h15;

  // -------------------------------------------------------------------------
  // Poll timer
  // -------------------------------------------------------------------------
  localparam integer POLL_TICKS = (CLK_HZ / 1000) * POLL_MS;
  reg [31:0] poll_cnt = 0;
  reg        poll_fire = 1'b0;

  always @(posedge clk) begin
    if (rst) begin
      poll_cnt  <= 0;
      poll_fire <= 1'b0;
    end else begin
      poll_fire <= 1'b0;
      if (poll_cnt >= (POLL_TICKS - 1)) begin
        poll_cnt  <= 0;
        poll_fire <= 1'b1;
      end else begin
        poll_cnt <= poll_cnt + 1;
      end
    end
  end

  // -------------------------------------------------------------------------
  // Raw buttons are on PORT B (active-low)
  // -------------------------------------------------------------------------
  reg [7:0] btn0_n_raw = 8'hFF;
  reg [7:0] btn1_n_raw = 8'hFF;

  wire [7:0] led0, led1;
  reg [7:0] led0_last = 8'h00;
  reg [7:0] led1_last = 8'h00;

  // status0: normal toggle (multi-bit) = mute
  // status1: solo one-hot with toggle-off on same button = solo
  mcp_buttons_toggle_plus_solo #(
      .CLK_HZ(CLK_HZ),
      .DEBOUNCE_MS(DEBOUNCE_MS)
  ) u_btn (
      .clk(clk),
      .rst(rst),
      .btn0_n_raw({
        btn1_n_raw[0],
        btn1_n_raw[1],
        btn1_n_raw[2],
        btn1_n_raw[6],
        btn1_n_raw[4],
        btn1_n_raw[5],
        btn1_n_raw[3],
        btn1_n_raw[7]
      }),
      .btn1_n_raw({
        btn0_n_raw[0],
        btn0_n_raw[1],
        btn0_n_raw[2],
        btn0_n_raw[3],
        btn0_n_raw[4],
        btn0_n_raw[6],
        btn0_n_raw[5],
        btn0_n_raw[7]
      }),
      .status0(status0),
      .status1(status1),
      .cur_status0(cur_status0),
      .cur_status1(cur_status1),
      .set_mute(set_mute),
      .set_solo(set_solo),
      //.led0({led1[5],led1[3],led1[4],led1[2],led1[6],led1[1],led1[7],led1[0]}),
      //.led0({led1[0],led1[1],led1[2],led1[3],led1[4],led1[5],led1[6],led1[7]}),
      .led0({led1[0], led1[1], led1[2], led1[6], led1[4], led1[5], led1[3], led1[7]}),
      .led1({led0[0], led0[1], led0[2], led0[3], led0[4], led0[6], led0[5], led0[7]})
  );

  // -------------------------------------------------------------------------
  // Forencich i2c_master
  // -------------------------------------------------------------------------
  wire [6:0] s_axis_cmd_address;
  wire       s_axis_cmd_start;
  wire       s_axis_cmd_read;
  wire       s_axis_cmd_write;
  wire       s_axis_cmd_write_multiple;
  wire       s_axis_cmd_stop;
  wire       s_axis_cmd_valid;
  wire       s_axis_cmd_ready;

  wire [7:0] s_axis_data_tdata;
  wire       s_axis_data_tvalid;
  wire       s_axis_data_tready;
  wire       s_axis_data_tlast;

  wire [7:0] m_axis_data_tdata;
  wire       m_axis_data_tvalid;
  reg        m_axis_data_tready = 1'b0;
  wire       m_axis_data_tlast;

  wire scl_i, scl_o, scl_t;
  wire sda_i, sda_o, sda_t;

  assign scl_i   = i2c_scl;
  assign i2c_scl = scl_t ? 1'bz : 1'b0;

  assign sda_i   = i2c_sda;
  assign i2c_sda = sda_t ? 1'bz : 1'b0;

  wire busy, bus_control, bus_active, missed_ack;

  // 50 MHz, 100 kHz: prescale = Fclk/(Fi2c*4) = 125
  wire [15:0] prescale = 16'd125;

  i2c_master u_i2c (
      .clk(clk),
      .rst(rst),

      .s_axis_cmd_address(s_axis_cmd_address),
      .s_axis_cmd_start(s_axis_cmd_start),
      .s_axis_cmd_read(s_axis_cmd_read),
      .s_axis_cmd_write(s_axis_cmd_write),
      .s_axis_cmd_write_multiple(s_axis_cmd_write_multiple),
      .s_axis_cmd_stop(s_axis_cmd_stop),
      .s_axis_cmd_valid(s_axis_cmd_valid),
      .s_axis_cmd_ready(s_axis_cmd_ready),

      .s_axis_data_tdata (s_axis_data_tdata),
      .s_axis_data_tvalid(s_axis_data_tvalid),
      .s_axis_data_tready(s_axis_data_tready),
      .s_axis_data_tlast (s_axis_data_tlast),

      .m_axis_data_tdata (m_axis_data_tdata),
      .m_axis_data_tvalid(m_axis_data_tvalid),
      .m_axis_data_tready(m_axis_data_tready),
      .m_axis_data_tlast (m_axis_data_tlast),

      .scl_i(scl_i),
      .scl_o(scl_o),
      .scl_t(scl_t),
      .sda_i(sda_i),
      .sda_o(sda_o),
      .sda_t(sda_t),

      .busy(busy),
      .bus_control(bus_control),
      .bus_active(bus_active),
      .missed_ack(missed_ack),

      .prescale(prescale),
      .stop_on_idle(1'b0)
  );

  // -------------------------------------------------------------------------
  // Two AXIS FIFOs: CMD + TX DATA
  // CMD packing: {addr[6:0], start, read, write, write_multiple, stop}
  // -------------------------------------------------------------------------
  localparam integer CMD_W = 12;

  wire [CMD_W-1:0] cmd_fifo_m_tdata;
  wire             cmd_fifo_m_tvalid;
  wire             cmd_fifo_m_tready;

  assign s_axis_cmd_address        = cmd_fifo_m_tdata[11:5];
  assign s_axis_cmd_start          = cmd_fifo_m_tdata[4];
  assign s_axis_cmd_read           = cmd_fifo_m_tdata[3];
  assign s_axis_cmd_write          = cmd_fifo_m_tdata[2];
  assign s_axis_cmd_write_multiple = cmd_fifo_m_tdata[1];
  assign s_axis_cmd_stop           = cmd_fifo_m_tdata[0];
  assign s_axis_cmd_valid          = cmd_fifo_m_tvalid;
  assign cmd_fifo_m_tready         = s_axis_cmd_ready;

  wire [CMD_W-1:0] cmd_fifo_s_tdata;
  wire             cmd_fifo_s_tvalid;
  wire             cmd_fifo_s_tready;

  axis_fifo #(
      .DEPTH(CMD_FIFO_DEPTH),
      .DATA_WIDTH(CMD_W),
      .KEEP_ENABLE(0),
      .LAST_ENABLE(0),
      .ID_ENABLE(0),
      .DEST_ENABLE(0),
      .USER_ENABLE(0),
      .FRAME_FIFO(0)
  ) u_cmd_fifo (
      .clk(clk),
      .rst(rst),
      .s_axis_tdata(cmd_fifo_s_tdata),
      .s_axis_tvalid(cmd_fifo_s_tvalid),
      .s_axis_tready(cmd_fifo_s_tready),
      .s_axis_tlast(1'b0),
      .m_axis_tdata(cmd_fifo_m_tdata),
      .m_axis_tvalid(cmd_fifo_m_tvalid),
      .m_axis_tready(cmd_fifo_m_tready),
      .m_axis_tlast()
  );

  wire [7:0] data_fifo_m_tdata;
  wire       data_fifo_m_tvalid;
  wire       data_fifo_m_tready;
  wire       data_fifo_m_tlast;

  assign s_axis_data_tdata  = data_fifo_m_tdata;
  assign s_axis_data_tvalid = data_fifo_m_tvalid;
  assign data_fifo_m_tready = s_axis_data_tready;
  assign s_axis_data_tlast  = data_fifo_m_tlast;

  wire [7:0] data_fifo_s_tdata;
  wire       data_fifo_s_tvalid;
  wire       data_fifo_s_tready;
  wire       data_fifo_s_tlast;

  axis_fifo #(
      .DEPTH(DATA_FIFO_DEPTH),
      .DATA_WIDTH(8),
      .KEEP_ENABLE(0),
      .LAST_ENABLE(1),
      .ID_ENABLE(0),
      .DEST_ENABLE(0),
      .USER_ENABLE(0),
      .FRAME_FIFO(0)
  ) u_data_fifo (
      .clk(clk),
      .rst(rst),
      .s_axis_tdata(data_fifo_s_tdata),
      .s_axis_tvalid(data_fifo_s_tvalid),
      .s_axis_tready(data_fifo_s_tready),
      .s_axis_tlast(data_fifo_s_tlast),
      .m_axis_tdata(data_fifo_m_tdata),
      .m_axis_tvalid(data_fifo_m_tvalid),
      .m_axis_tready(data_fifo_m_tready),
      .m_axis_tlast(data_fifo_m_tlast)
  );

  // -------------------------------------------------------------------------
  // Single FSM push interface
  // -------------------------------------------------------------------------
  reg       cmd_push;
  reg [6:0] cmd_addr7_r;
  reg cmd_start_r, cmd_read_r, cmd_write_r, cmd_wrmulti_r, cmd_stop_r;
  assign cmd_fifo_s_tdata = {
    cmd_addr7_r, cmd_start_r, cmd_read_r, cmd_write_r, cmd_wrmulti_r, cmd_stop_r
  };
  assign cmd_fifo_s_tvalid = cmd_push;

  reg       data_push;
  reg [7:0] data_push_byte;
  reg       data_push_last;
  assign data_fifo_s_tdata  = data_push_byte;
  assign data_fifo_s_tvalid = data_push;
  assign data_fifo_s_tlast  = data_push_last;

  wire cmd_can_push = cmd_fifo_s_tready;
  wire data_can_push = data_fifo_s_tready;

  task set_cmd_write_multi;
    input [6:0] a7;
    input stop;
    begin
      cmd_addr7_r   <= a7;
      cmd_start_r   <= 1'b0;
      cmd_read_r    <= 1'b0;
      cmd_write_r   <= 1'b0;
      cmd_wrmulti_r <= 1'b1;
      cmd_stop_r    <= stop;
    end
  endtask

  task set_cmd_read1;
    input [6:0] a7;
    input stop;
    begin
      cmd_addr7_r   <= a7;
      cmd_start_r   <= 1'b0;
      cmd_read_r    <= 1'b1;
      cmd_write_r   <= 1'b0;
      cmd_wrmulti_r <= 1'b0;
      cmd_stop_r    <= stop;
    end
  endtask

  // FSM states
  localparam [6:0]
  // init MCP0
  ST_I0_IODIRA_CMD  = 7'd0,
ST_I0_IODIRA_D0   = 7'd1,
ST_I0_IODIRA_D1   = 7'd2,
ST_I0_IODIRB_CMD  = 7'd3,
ST_I0_IODIRB_D0   = 7'd4,
ST_I0_IODIRB_D1   = 7'd5,
ST_I0_GPPUB_CMD   = 7'd6,
ST_I0_GPPUB_D0    = 7'd7,
ST_I0_GPPUB_D1    = 7'd8,

  // init MCP1
  ST_I1_IODIRA_CMD  = 7'd9,
ST_I1_IODIRA_D0   = 7'd10,
ST_I1_IODIRA_D1   = 7'd11,
ST_I1_IODIRB_CMD  = 7'd12,
ST_I1_IODIRB_D0   = 7'd13,
ST_I1_IODIRB_D1   = 7'd14,
ST_I1_GPPUB_CMD   = 7'd15,
ST_I1_GPPUB_D0    = 7'd16,
ST_I1_GPPUB_D1    = 7'd17,

ST_WAIT_POLL      = 7'd18,

  // read MCP0 GPIOB (buttons)
  ST_R0_PTR_CMD = 7'd19, ST_R0_PTR_D = 7'd20, ST_R0_READ_CMD = 7'd21, ST_R0_READ_WAIT = 7'd22,

  // read MCP1 GPIOB (buttons)
  ST_R1_PTR_CMD = 7'd23, ST_R1_PTR_D = 7'd24, ST_R1_READ_CMD = 7'd25, ST_R1_READ_WAIT = 7'd26,

  // write LEDs to OLATA (port A)
  ST_W0_CMD         = 7'd27,
ST_W0_D0          = 7'd28,
ST_W0_D1          = 7'd29,

ST_W1_CMD         = 7'd30,
ST_W1_D0          = 7'd31,
ST_W1_D1          = 7'd32;

  reg [6:0] st = ST_I0_IODIRA_CMD;

  always @(posedge clk) begin
    if (rst) begin
      st <= ST_I0_IODIRA_CMD;

      cmd_push <= 1'b0;
      data_push <= 1'b0;

      cmd_addr7_r <= 7'd0;
      cmd_start_r <= 1'b0;
      cmd_read_r <= 1'b0;
      cmd_write_r <= 1'b0;
      cmd_wrmulti_r <= 1'b0;
      cmd_stop_r <= 1'b0;

      data_push_byte <= 8'h00;
      data_push_last <= 1'b0;

      m_axis_data_tready <= 1'b0;

      btn0_n_raw <= 8'hFF;
      btn1_n_raw <= 8'hFF;

      led0_last <= 8'h00;
      led1_last <= 8'h00;
    end else begin
      cmd_push  <= 1'b0;
      data_push <= 1'b0;

      if (!(st == ST_R0_READ_WAIT || st == ST_R1_READ_WAIT)) begin
        m_axis_data_tready <= 1'b0;
      end

      case (st)
        // ---------------- INIT MCP0 ----------------
        // Port A LEDs: outputs
        ST_I0_IODIRA_CMD: begin
          if (cmd_can_push) begin
            set_cmd_write_multi(MCP0_ADDR, 1'b1);
            cmd_push <= 1'b1;
            st <= ST_I0_IODIRA_D0;
          end
        end
        ST_I0_IODIRA_D0: begin
          if (data_can_push) begin
            data_push <= 1'b1;
            data_push_byte <= MCP_REG_IODIRA;
            data_push_last <= 1'b0;
            st <= ST_I0_IODIRA_D1;
          end
        end
        ST_I0_IODIRA_D1: begin
          if (data_can_push) begin
            data_push <= 1'b1;
            data_push_byte <= 8'h00;
            data_push_last <= 1'b1;  // A outputs
            st <= ST_I0_IODIRB_CMD;
          end
        end

        // Port B buttons: inputs
        ST_I0_IODIRB_CMD: begin
          if (cmd_can_push) begin
            set_cmd_write_multi(MCP0_ADDR, 1'b1);
            cmd_push <= 1'b1;
            st <= ST_I0_IODIRB_D0;
          end
        end
        ST_I0_IODIRB_D0: begin
          if (data_can_push) begin
            data_push <= 1'b1;
            data_push_byte <= MCP_REG_IODIRB;
            data_push_last <= 1'b0;
            st <= ST_I0_IODIRB_D1;
          end
        end
        ST_I0_IODIRB_D1: begin
          if (data_can_push) begin
            data_push <= 1'b1;
            data_push_byte <= 8'hFF;
            data_push_last <= 1'b1;  // B inputs
            st <= ST_I0_GPPUB_CMD;
          end
        end

        // Pullups on Port B
        ST_I0_GPPUB_CMD: begin
          if (cmd_can_push) begin
            set_cmd_write_multi(MCP0_ADDR, 1'b1);
            cmd_push <= 1'b1;
            st <= ST_I0_GPPUB_D0;
          end
        end
        ST_I0_GPPUB_D0: begin
          if (data_can_push) begin
            data_push <= 1'b1;
            data_push_byte <= MCP_REG_GPPUB;
            data_push_last <= 1'b0;
            st <= ST_I0_GPPUB_D1;
          end
        end
        ST_I0_GPPUB_D1: begin
          if (data_can_push) begin
            data_push <= 1'b1;
            data_push_byte <= 8'hFF;
            data_push_last <= 1'b1;  // pullups B
            st <= ST_I1_IODIRA_CMD;
          end
        end

        // ---------------- INIT MCP1 ----------------
        ST_I1_IODIRA_CMD: begin
          if (cmd_can_push) begin
            set_cmd_write_multi(MCP1_ADDR, 1'b1);
            cmd_push <= 1'b1;
            st <= ST_I1_IODIRA_D0;
          end
        end
        ST_I1_IODIRA_D0: begin
          if (data_can_push) begin
            data_push <= 1'b1;
            data_push_byte <= MCP_REG_IODIRA;
            data_push_last <= 1'b0;
            st <= ST_I1_IODIRA_D1;
          end
        end
        ST_I1_IODIRA_D1: begin
          if (data_can_push) begin
            data_push <= 1'b1;
            data_push_byte <= 8'h00;
            data_push_last <= 1'b1;  // A outputs
            st <= ST_I1_IODIRB_CMD;
          end
        end

        ST_I1_IODIRB_CMD: begin
          if (cmd_can_push) begin
            set_cmd_write_multi(MCP1_ADDR, 1'b1);
            cmd_push <= 1'b1;
            st <= ST_I1_IODIRB_D0;
          end
        end
        ST_I1_IODIRB_D0: begin
          if (data_can_push) begin
            data_push <= 1'b1;
            data_push_byte <= MCP_REG_IODIRB;
            data_push_last <= 1'b0;
            st <= ST_I1_IODIRB_D1;
          end
        end
        ST_I1_IODIRB_D1: begin
          if (data_can_push) begin
            data_push <= 1'b1;
            data_push_byte <= 8'hFF;
            data_push_last <= 1'b1;  // B inputs
            st <= ST_I1_GPPUB_CMD;
          end
        end

        ST_I1_GPPUB_CMD: begin
          if (cmd_can_push) begin
            set_cmd_write_multi(MCP1_ADDR, 1'b1);
            cmd_push <= 1'b1;
            st <= ST_I1_GPPUB_D0;
          end
        end
        ST_I1_GPPUB_D0: begin
          if (data_can_push) begin
            data_push <= 1'b1;
            data_push_byte <= MCP_REG_GPPUB;
            data_push_last <= 1'b0;
            st <= ST_I1_GPPUB_D1;
          end
        end
        ST_I1_GPPUB_D1: begin
          if (data_can_push) begin
            data_push <= 1'b1;
            data_push_byte <= 8'hFF;
            data_push_last <= 1'b1;
            st <= ST_WAIT_POLL;
          end
        end

        // ---------------- MAIN LOOP ----------------
        ST_WAIT_POLL: begin
          if (poll_fire) st <= ST_R0_PTR_CMD;
        end

        // MCP0: pointer to GPIOB, then read 1 byte
        ST_R0_PTR_CMD: begin
          if (cmd_can_push) begin
            set_cmd_write_multi(MCP0_ADDR, 1'b0);
            cmd_push <= 1'b1;
            st <= ST_R0_PTR_D;
          end
        end
        ST_R0_PTR_D: begin
          if (data_can_push) begin
            data_push <= 1'b1;
            data_push_byte <= MCP_REG_GPIOB;
            data_push_last <= 1'b1;
            st <= ST_R0_READ_CMD;
          end
        end
        ST_R0_READ_CMD: begin
          if (cmd_can_push) begin
            set_cmd_read1(MCP0_ADDR, 1'b1);
            cmd_push <= 1'b1;
            m_axis_data_tready <= 1'b1;
            st <= ST_R0_READ_WAIT;
          end
        end
        ST_R0_READ_WAIT: begin
          if (m_axis_data_tvalid && m_axis_data_tready) begin
            btn0_n_raw <= m_axis_data_tdata;
            m_axis_data_tready <= 1'b0;
            st <= ST_R1_PTR_CMD;
          end
        end

        // MCP1: pointer to GPIOB, then read 1 byte
        ST_R1_PTR_CMD: begin
          if (cmd_can_push) begin
            set_cmd_write_multi(MCP1_ADDR, 1'b0);
            cmd_push <= 1'b1;
            st <= ST_R1_PTR_D;
          end
        end
        ST_R1_PTR_D: begin
          if (data_can_push) begin
            data_push <= 1'b1;
            data_push_byte <= MCP_REG_GPIOB;
            data_push_last <= 1'b1;
            st <= ST_R1_READ_CMD;
          end
        end
        ST_R1_READ_CMD: begin
          if (cmd_can_push) begin
            set_cmd_read1(MCP1_ADDR, 1'b1);
            cmd_push <= 1'b1;
            m_axis_data_tready <= 1'b1;
            st <= ST_R1_READ_WAIT;
          end
        end
        ST_R1_READ_WAIT: begin
          if (m_axis_data_tvalid && m_axis_data_tready) begin
            btn1_n_raw <= m_axis_data_tdata;
            m_axis_data_tready <= 1'b0;

            if (led0 != led0_last) st <= ST_W0_CMD;
            else if (led1 != led1_last) st <= ST_W1_CMD;
            else st <= ST_WAIT_POLL;
          end
        end

        // Write MCP0 OLATA=led0 (Port A LEDs)
        ST_W0_CMD: begin
          if (cmd_can_push) begin
            set_cmd_write_multi(MCP0_ADDR, 1'b1);
            cmd_push <= 1'b1;
            st <= ST_W0_D0;
          end
        end
        ST_W0_D0: begin
          if (data_can_push) begin
            data_push <= 1'b1;
            data_push_byte <= MCP_REG_OLATA;
            data_push_last <= 1'b0;
            st <= ST_W0_D1;
          end
        end
        ST_W0_D1: begin
          if (data_can_push) begin
            data_push <= 1'b1;
            data_push_byte <= led0;
            data_push_last <= 1'b1;
            led0_last <= led0;

            if (led1 != led1_last) st <= ST_W1_CMD;
            else st <= ST_WAIT_POLL;
          end
        end

        // Write MCP1 OLATA=led1
        ST_W1_CMD: begin
          if (cmd_can_push) begin
            set_cmd_write_multi(MCP1_ADDR, 1'b1);
            cmd_push <= 1'b1;
            st <= ST_W1_D0;
          end
        end
        ST_W1_D0: begin
          if (data_can_push) begin
            data_push <= 1'b1;
            data_push_byte <= MCP_REG_OLATA;
            data_push_last <= 1'b0;
            st <= ST_W1_D1;
          end
        end
        ST_W1_D1: begin
          if (data_can_push) begin
            data_push <= 1'b1;
            data_push_byte <= led1;
            data_push_last <= 1'b1;
            led1_last <= led1;
            st <= ST_WAIT_POLL;
          end
        end

        default: st <= ST_WAIT_POLL;
      endcase
    end
  end

endmodule

`default_nettype wire
