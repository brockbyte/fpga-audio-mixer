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
// SDP (Session Description Protocol) UDP Stream from ROM
// =============================================================================
// Periodically transmits SDP data from ROM via AXI-Stream interface.
// Implements a timer-triggered packet generator with configurable period.
//
// Features:
// - Reads SDP data from synchronous ROM (1-cycle latency)
// - Periodic transmission with configurable interval
// - Start/end packet markers
// - Availability signal for upstream mux
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

module sdp_udp_stream_from_rom #(
    parameter integer ADDR_WIDTH = 10,
    parameter integer ROM_LEN    = 157,    // Exact number of bytes to send
    parameter integer PERIOD_MS  = 1000    // Transmission period in milliseconds
) (
    input wire clk,
    input wire rst,

    input wire tick_1ms,

    // ROM (sync read, 1-cycle latency)
    output reg  [ADDR_WIDTH-1:0] rom_addr,
    input  wire [           7:0] rom_data,

    // Stream out
    output reg  [7:0] s_data,
    output reg        s_start,
    output reg        s_end,
    output reg        s_valid,
    output reg        s_avail,
    input  wire       s_ready
);

  reg [31:0] ms_cnt;
  reg        pending;

  localparam [1:0] ST_IDLE = 2'd0, ST_RD = 2'd1, ST_OUT = 2'd2;
  reg  [           1:0] st;

  reg  [ADDR_WIDTH-1:0] idx;
  reg  [           7:0] buffer;
  reg                   first_sent;

  wire                  fire = s_ready;  //s_valid && s_ready;

  always @(posedge clk) begin
    if (rst) begin
      ms_cnt     <= 32'd0;
      pending    <= 1'b0;
      st         <= ST_IDLE;
      idx        <= {ADDR_WIDTH{1'b0}};
      rom_addr   <= {ADDR_WIDTH{1'b0}};
      buffer     <= 8'd0;

      s_data     <= 8'd0;
      s_start    <= 1'b0;
      s_end      <= 1'b0;
      s_valid    <= 1'b0;
      s_avail    <= 1'b0;

      first_sent <= 1'b0;
    end else begin
      // defaults
      s_start <= 1'b0;
      s_end   <= 1'b0;
      s_valid <= 1'b0;

      // periodic trigger (every 1 second)
      if (tick_1ms) begin
        if (ms_cnt == PERIOD_MS - 1) begin
          ms_cnt  <= 32'd0;
          pending <= 1'b1;
        end else begin
          ms_cnt <= ms_cnt + 32'd1;
        end
      end

      s_avail <= pending;

      case (st)
        ST_IDLE: begin
          s_valid <= 1'b0;
          if (pending && s_ready) begin
            idx        <= {ADDR_WIDTH{1'b0}};
            rom_addr   <= {ADDR_WIDTH{1'b0}};
            first_sent <= 1'b0;
            st         <= ST_RD;
          end
        end

        ST_RD: begin
          // capture byte from ROM (1-cycle latency)
          //buffer <= rom_data;
          st       <= ST_OUT;
          idx      <= idx + {{(ADDR_WIDTH - 1) {1'b0}}, 1'b1};
          rom_addr <= idx + {{(ADDR_WIDTH - 1) {1'b0}}, 1'b1};
        end

        ST_OUT: begin
          s_data <= rom_data;


          if (fire) begin
            s_valid <= 1'b1;
            if (!first_sent) begin
              s_start    <= 1'b1;
              first_sent <= 1'b1;
            end

            if (idx == (ROM_LEN)) begin
              s_end   <= 1'b1;
              pending <= 1'b0;
              st      <= ST_IDLE;
            end else begin
              idx      <= idx + {{(ADDR_WIDTH - 1) {1'b0}}, 1'b1};
              rom_addr <= idx + {{(ADDR_WIDTH - 1) {1'b0}}, 1'b1};
            end
          end
        end

        default: st <= ST_IDLE;
      endcase
    end
  end

endmodule

`default_nettype wire
