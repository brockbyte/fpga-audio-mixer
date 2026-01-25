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
// Transmit Async FIFO with Clock Domain Crossing and AXI-Stream Adapter
// =============================================================================
// Dual-clock FIFO (write: producer clock, read: Ethernet 125MHz clock) with
// AXI-Stream interface. Includes start/end packet boundary detection.
//
// Features:
// - Async FIFO for clock domain crossing
// - 10-bit FIFO width: {start, end, data[7:0]}
// - Availability check based on payload length
// - Packet boundary detection and frame synchronization
// - 2-stage prefill pipeline for reliable data transfer
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

module tx_asyncfifo10b_axi #(

    parameter [15:0] PAYLOAD_LEN = 16'd30
) (

    // Write-side (producer)
    input  wire       wrclk,
    input  wire       wrrst,
    input  wire [7:0] w_data,
    input  wire       w_start,
    input  wire       w_end,
    input  wire       w_valid,
    output wire       w_ready,  // backpressure to producer (= !wrfull)

    // Read-side (to mux)
    input wire rdclk,
    input wire rdrst,

    output wire [7:0] s_data,
    output wire       s_start,
    output wire       s_end,
    output reg        s_valid,
    output wire       s_avail,
    input  wire       s_ready,

    // optional debug
    output wire        fifo_full,
    output wire        fifo_empty,
    output wire [11:0] fifo_level
);
  wire [9:0] fifo_din;
  wire [9:0] fifo_q;
  reg  [9:0] fifo_prefill1;
  reg  [9:0] fifo_prefill2;
  reg        fifo_rdreq;

  assign fifo_din = {w_start, w_end, w_data};
  assign w_ready  = (fifo_level < 12'd2038 - PAYLOAD_LEN[11:0]);
  //assign s_avail   = !fifo_empty;
  assign s_avail  = (fifo_level >= PAYLOAD_LEN[11:0] - 2);

  localparam TX_FIFO_WIDTH = 8 + 2;  // start + end + data

  localparam [3:0]
    ST_IDLE        = 4'd0,
    ST_SKIP1A      = 4'd1,
    ST_SKIP1       = 4'd2,
    ST_READ1A      = 4'd3,
    ST_READ1       = 4'd4,
    ST_LAST1       = 4'd5;


  reg [3:0] st;

  assign s_data  = fifo_prefill1[7:0];
  assign s_end   = fifo_prefill1[8];
  assign s_start = fifo_prefill1[9];

  always @(posedge rdclk or posedge rdrst) begin
    if (rdrst) begin
      s_valid    <= 1'd0;
      fifo_rdreq <= 1'b0;
      fifo_prefill1 <= 10'd0;
      fifo_prefill2 <= 10'd0;
      st    <= ST_IDLE;
    end else begin
      s_valid <= 1'b0;
      fifo_rdreq <= 1'b0;
      case (st)
        ST_IDLE: begin
          s_valid <= 1'b0;
          if (!fifo_empty && !s_start) begin
            st <= ST_SKIP1A;
            fifo_rdreq <= 1'b1;
          end else if (!fifo_empty && s_ready) begin
            fifo_rdreq <= 1'b1;
            st <= ST_READ1A;
          end
        end

        ST_SKIP1A: begin
          fifo_prefill1 <= fifo_prefill2;
          st <= ST_SKIP1;
        end

        ST_SKIP1: begin
          fifo_prefill2 <= fifo_q;
          if (!fifo_empty && !s_start) begin
            fifo_rdreq <= 1'b1;
            st <= ST_SKIP1A;
          end else st <= ST_IDLE;
        end

        ST_READ1A: begin
          if (s_ready) begin
            //fifo_prefill1 <= fifo_prefill2;
            fifo_rdreq <= 1'b1;
            s_valid <= 1'b1;
            st <= ST_READ1;
          end
        end

        ST_READ1: begin
          if (s_ready) begin
            fifo_prefill2 <= fifo_q;
            if (fifo_q[8]) begin
              fifo_prefill1 <= fifo_prefill2;
              st <= ST_LAST1;
              s_valid <= 1'b1;
            end else if (!fifo_empty) begin
              fifo_prefill1 <= fifo_prefill2;
              fifo_rdreq <= 1'b1;
              st <= ST_READ1;
              s_valid <= 1'b1;
            end
          end
        end

        ST_LAST1: begin
          if (s_ready) begin
            fifo_prefill1 <= fifo_prefill2;
            s_valid <= 1'b1;
            st <= ST_IDLE;
          end
        end

        default: st <= ST_IDLE;

      endcase

    end
  end


  net_async_fifo_10b u_fifo (
      .aclr   (wrrst),
      .data   (fifo_din),
      .rdclk  (rdclk),
      .rdreq  (fifo_rdreq),
      .wrclk  (wrclk),
      .wrreq  (w_valid),
      .q      (fifo_q),
      .rdempty(fifo_empty),
      .rdusedw(fifo_level),
      .wrfull (fifo_full)
  );



endmodule

`default_nettype wire

