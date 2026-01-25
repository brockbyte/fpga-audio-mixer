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

// -----------------------------------------------------------------------------
// aes67_ctrl_name_rx_async_v5.v (Verilog-2001)
//
// Receives UDP payload byte-stream and decodes:
//  * CTRL v0x02 (60 bytes):
//      0..3  : 'C','T','R','L'
//      4     : version = 0x02
//      5..7  : reserved
//      8..39 : 16 x gain_u10 stored in u16 (big-endian), use bits [9:0]
//      40..55: 16 x pan_s8 (-127..+127, 0=center)
//      56..57: mute_mask (u16)
//      58..59: solo_mask (u16, one-hot or 0)
//
//  * CTRL legacy v0x01 (72 bytes):
//      8..39 : 16 x gain_q15 (signed/unsigned Q1.15)
//      40..71: 16 x pan_q15  (signed Q1.15)
//      (no mute/solo in v0x01)
//
//  * NAME v0x01 (24 bytes):
//      0..3  : 'N','A','M','E'
//      4     : version = 0x01
//      5     : channel_index (0..15)
//      6..7  : reserved
//      8..23 : 16 bytes ASCII name (0-padding)
//
// v5 change:
//  - No Q-format conversion and no L/R pan-law here anymore.
//    The mixer does conversion; this module outputs raw gain_u10/pan_s8.
// -----------------------------------------------------------------------------

`timescale 1ns / 1ps
`default_nettype none

module aes67_ctrl_name_rx (
    input wire clk,
    input wire rst,

    input  wire [7:0] udp_rx_payload_tdata,
    input  wire       udp_rx_payload_tvalid,
    output reg        udp_rx_payload_tready,
    input  wire       udp_rx_payload_start,
    input  wire       udp_rx_payload_end,

    // Raw controls (preferred)
    output wire [9:0] gain0,
    output wire [9:0] gain1,
    output wire [9:0] gain2,
    output wire [9:0] gain3,
    output wire [9:0] gain4,
    output wire [9:0] gain5,
    output wire [9:0] gain6,
    output wire [9:0] gain7,
    output wire [9:0] gain8,
    output wire [9:0] gain9,
    output wire [9:0] gain10,
    output wire [9:0] gain11,
    output wire [9:0] gain12,
    output wire [9:0] gain13,
    output wire [9:0] gain14,
    output wire [9:0] gain15,

    output wire signed [7:0] pan0,
    output wire signed [7:0] pan1,
    output wire signed [7:0] pan2,
    output wire signed [7:0] pan3,
    output wire signed [7:0] pan4,
    output wire signed [7:0] pan5,
    output wire signed [7:0] pan6,
    output wire signed [7:0] pan7,
    output wire signed [7:0] pan8,
    output wire signed [7:0] pan9,
    output wire signed [7:0] pan10,
    output wire signed [7:0] pan11,
    output wire signed [7:0] pan12,
    output wire signed [7:0] pan13,
    output wire signed [7:0] pan14,
    output wire signed [7:0] pan15,

    output reg [15:0] mute_button,
    output reg [15:0] solo_button,

    output reg set_gain,
    output reg set_pan,
    output reg set_solo,
    output reg set_mute,
    output reg set_name,

    output reg [15:0] pan_dirty,
    output reg [15:0] name_dirty,

    // Two read ports for name RAM
    input  wire [7:0] name_rd0_addr,
    output wire [7:0] name_rd0_data,
    input  wire [7:0] name_rd1_addr,
    output wire [7:0] name_rd1_data

    // Optional flat busses (kept 16*16 for compatibility)
    //output wire [16*16-1:0] gain_reg_flat,
    //output wire [16*16-1:0] pan_reg_flat
);

  // --------------------------------------------------------------------
  // Name-RAM parameters
  // --------------------------------------------------------------------
  localparam integer NAME_DEPTH = 256;
  localparam integer NAME_AW = 8;
  localparam integer NAME_LEN = 16;

  // --------------------------------------------------------------------
  // State/fields
  // --------------------------------------------------------------------
  reg [7:0] bcnt;
  reg       port_ok;

  reg [7:0] magic0, magic1, magic2, magic3;
  reg [7:0] version;
  reg [7:0] ch_index;

  localparam PKT_NONE = 2'd0;
  localparam PKT_CTRL = 2'd1;
  localparam PKT_NAME = 2'd2;
  reg        [ 1:0] pkt_kind;

  // CTRL raw values (v0x02)
  reg        [ 9:0] gain_u10     [0:15];
  reg        [ 7:0] gain_hi      [0:15];
  reg signed [ 7:0] pan_s8       [0:15];
  reg        [15:0] mute_mask_rx;
  reg        [15:0] solo_mask_rx;

  // CTRL legacy values (v0x01)
  //reg [15:0]        gain_q15 [0:15];
  //reg signed [15:0] pan_q15  [0:15];

  integer           i;

  // --------------------------------------------------------------------
  // Flattening for compatibility
  // gain_reg_flat: 16-bit word with gain in [9:0]
  // pan_reg_flat : 16-bit sign-extended s8
  // --------------------------------------------------------------------
  //    genvar j;
  //    generate
  //      for (j=0; j<16; j=j+1) begin : GEN_PACK
  //        assign gain_reg_flat[j*16 +: 16] = {6'd0, gain_u10[j]};
  //        assign pan_reg_flat [j*16 +: 16] = {{8{pan_s8[j][7]}}, pan_s8[j]};
  //      end
  //    endgenerate

  // Map arrays to discrete outputs
  assign gain0  = gain_u10[0];
  assign gain1  = gain_u10[1];
  assign gain2  = gain_u10[2];
  assign gain3  = gain_u10[3];
  assign gain4  = gain_u10[4];
  assign gain5  = gain_u10[5];
  assign gain6  = gain_u10[6];
  assign gain7  = gain_u10[7];
  assign gain8  = gain_u10[8];
  assign gain9  = gain_u10[9];
  assign gain10 = gain_u10[10];
  assign gain11 = gain_u10[11];
  assign gain12 = gain_u10[12];
  assign gain13 = gain_u10[13];
  assign gain14 = gain_u10[14];
  assign gain15 = gain_u10[15];

  assign pan0   = pan_s8[0];
  assign pan1   = pan_s8[1];
  assign pan2   = pan_s8[2];
  assign pan3   = pan_s8[3];
  assign pan4   = pan_s8[4];
  assign pan5   = pan_s8[5];
  assign pan6   = pan_s8[6];
  assign pan7   = pan_s8[7];
  assign pan8   = pan_s8[8];
  assign pan9   = pan_s8[9];
  assign pan10  = pan_s8[10];
  assign pan11  = pan_s8[11];
  assign pan12  = pan_s8[12];
  assign pan13  = pan_s8[13];
  assign pan14  = pan_s8[14];
  assign pan15  = pan_s8[15];

  // --------------------------------------------------------------------
  // Name RAM: 1 write, 2 reads
  // --------------------------------------------------------------------
  reg       name_we_a;
  reg [7:0] name_addr_a;
  reg [7:0] name_din_a;

  name_ram_1w2r #(
      .ADDR_WIDTH(8),
      .DATA_WIDTH(8)
  ) u_name_ram_1w2r (
      .clk     (clk),
      .we      (name_we_a),
      .wr_addr (name_addr_a),
      .wr_data (name_din_a),
      .rd0_addr(name_rd0_addr),
      .rd0_data(name_rd0_data),
      .rd1_addr(name_rd1_addr),
      .rd1_data(name_rd1_data)
  );

  function [NAME_AW-1:0] name_addr;
    input [3:0] ch;
    input [4:0] ci;
    begin
      name_addr = (ch * NAME_LEN) + ci;
    end
  endfunction

  // Optional RAM clear on reset
  reg init_ram;

  // --------------------------------------------------------------------
  // Main RX
  // --------------------------------------------------------------------
  always @(posedge clk) begin
    if (rst) begin
      udp_rx_payload_tready <= 1'b0;
      port_ok               <= 1'b0;
      bcnt                  <= 8'd0;
      pkt_kind              <= PKT_NONE;

      magic0                <= 8'd0;
      magic1                <= 8'd0;
      magic2                <= 8'd0;
      magic3                <= 8'd0;
      version               <= 8'd0;
      ch_index              <= 8'd0;

      for (i = 0; i < 16; i = i + 1) begin
        gain_u10[i] <= 10'd0;
        gain_hi[i]  <= 8'd0;
        pan_s8[i]   <= 8'sd0;
        //gain_q15[i] <= 16'd0;
        //pan_q15[i]  <= 16'sd0;
      end
      mute_mask_rx <= 16'd0;
      solo_mask_rx <= 16'd0;
      mute_button  <= 16'd0;
      solo_button  <= 16'd0;

      set_pan      <= 1'b0;
      set_solo     <= 1'b0;
      set_mute     <= 1'b0;
      set_gain     <= 1'b0;
      set_name     <= 1'b0;

      name_dirty   <= 16'd0;
      pan_dirty    <= 16'd0;

      init_ram     <= 1'b1;  //kein init
      name_we_a    <= 1'b0;
      name_addr_a  <= 8'd0;
      name_din_a   <= 8'h20;  // ' '

    end else begin
      udp_rx_payload_tready <= 1'b1;

      // defaults
      name_we_a             <= 1'b0;

      set_pan               <= 1'b0;
      set_solo              <= 1'b0;
      set_mute              <= 1'b0;
      set_gain              <= 1'b0;
      set_name              <= 1'b0;

      name_dirty            <= 16'd0;
      pan_dirty             <= 16'd0;

      // simple RAM clear sweep
      if (init_ram) begin
        //name_we_a   <= 1'b1;
        //name_din_a  <= 8'h20;
        //name_addr_a <= name_addr_a + 8'd1;
        //if (name_addr_a == 8'd255) begin
        init_ram   <= 1'b0;
        name_dirty <= 16'hFF;
        pan_dirty  <= 16'hFF;
        //end
      end

      // start of packet
      if (udp_rx_payload_tvalid && udp_rx_payload_start) begin
        port_ok  <= 1'b1;
        bcnt     <= 8'd1;
        pkt_kind <= PKT_NONE;
        magic0   <= udp_rx_payload_tdata;
      end

      if (udp_rx_payload_tvalid && port_ok) begin
        // header decode
        case (bcnt)
          8'd1: magic1 <= udp_rx_payload_tdata;
          8'd2: magic2 <= udp_rx_payload_tdata;
          8'd3: magic3 <= udp_rx_payload_tdata;
          8'd4: begin
            version <= udp_rx_payload_tdata;
            if (magic0 == "C" && magic1 == "T" && magic2 == "R" && magic3 == "L")
              pkt_kind <= PKT_CTRL;
            else if (magic0 == "N" && magic1 == "A" && magic2 == "M" && magic3 == "E")
              pkt_kind <= PKT_NAME;
            else pkt_kind <= PKT_NONE;
          end
          8'd5: begin
            if (pkt_kind == PKT_NAME) ch_index <= udp_rx_payload_tdata;
          end
          default: ;
        endcase

        // CTRL payload
        if (pkt_kind == PKT_CTRL) begin
          if (version == 8'h02) begin
            // gains u16 (big-endian), bytes 8..39
            if (bcnt >= 8'd8 && bcnt <= 8'd39) begin
              integer gi;
              gi = (bcnt - 8'd8) >> 1;
              if (((bcnt - 8'd8) & 8'd1) == 8'd0) begin
                gain_hi[gi[3:0]] <= udp_rx_payload_tdata;
              end else begin
                reg [15:0] gword;
                gword = {gain_hi[gi[3:0]], udp_rx_payload_tdata};
                gain_u10[gi[3:0]] <= gword[9:0];
              end
            end

            // pans s8, bytes 40..55
            if (bcnt >= 8'd40 && bcnt <= 8'd55) begin
              pan_s8[(bcnt-8'd40)&8'h0F] <= udp_rx_payload_tdata;
            end

            // mute/solo, bytes 56..59
            case (bcnt)
              8'd56:   mute_button[15:8] <= udp_rx_payload_tdata;
              8'd57:   mute_button[7:0] <= udp_rx_payload_tdata;
              8'd58:   solo_button[15:8] <= udp_rx_payload_tdata;
              8'd59:   solo_button[7:0] <= udp_rx_payload_tdata;
              default: ;
            endcase

          end
        end

        // NAME payload bytes 8-23
        if (pkt_kind == PKT_NAME && !init_ram) begin
          if (bcnt >= 8'd8 && bcnt <= 8'd23) begin
            name_we_a   <= 1'b1;
            name_addr_a <= name_addr(ch_index[3:0], (bcnt - 8'd8));
            name_din_a  <= udp_rx_payload_tdata;
          end
        end

        // End-of-packet actions
        if (udp_rx_payload_end) begin
          port_ok <= 1'b0;
          if (pkt_kind == PKT_CTRL) begin
            if (version == 8'h02) begin
              pan_dirty <= 16'hFF;
              set_solo  <= 1'b1;
              set_gain  <= 1'b1;
              set_mute  <= 1'b1;
              set_pan   <= 1'b1;
            end
          end else if (pkt_kind == PKT_NAME) begin
            name_dirty[ch_index[3:0]] <= 1'b1;
            set_name                  <= 1'b1;
          end
        end

        // byte counter
        bcnt <= bcnt + 8'd1;
      end
    end
  end

endmodule

`resetall
`default_nettype wire
