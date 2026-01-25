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
// AXI-Stream UDP Payload Asynchronous FIFO
// =============================================================================
// Transfers UDP payload data across clock domains using an async FIFO.
// Encodes packet boundaries with start/end flags.
//
// Write Domain (s_clk): Receives AXI-Stream with TLAST
// Read Domain (m_clk): Outputs data with start/end flags
//
// Data Format:
// - start = 1 on first word of each UDP packet
// - end = 1 on last word (when TLAST is asserted)
// - FIFO width: {start[1], end[1], data[DATA_WIDTH]}
// =============================================================================

`timescale 1ns / 1ps

module axis_udp_payload_async_fifo #(
    parameter DATA_WIDTH = 8,  // Width of AXI-Stream tdata
    parameter ADDR_WIDTH = 8   // FIFO depth = 2^ADDR_WIDTH
) (
    // ========== Write Clock Domain (Network / AXI-Stream) ==========
    input wire s_clk,
    input wire s_rst,

    input  wire [DATA_WIDTH-1:0] s_axis_tdata,
    input  wire                  s_axis_tvalid,
    output wire                  s_axis_tready,
    input  wire                  s_axis_tlast,

    // ========== Read Clock Domain (e.g., Audio Clock) ==========
    input wire m_clk,
    input wire m_rst,

    output wire [DATA_WIDTH-1:0] m_data,
    output wire                  m_start,
    output wire                  m_end,
    output wire                  m_valid,
    input  wire                  m_ready
);

  // ========== FIFO Data Format ==========
  // FIFO width: start + end + data
  localparam FIFO_WIDTH = DATA_WIDTH + 2;

  // Write-domain signals
  reg                   in_frame;
  reg                   empty_delay;
  wire                  write_en;
  wire                  start_flag;
  wire                  end_flag;
  wire [FIFO_WIDTH-1:0] fifo_din;
  wire                  fifo_full;

  // Read-domain signals
  wire [FIFO_WIDTH-1:0] fifo_dout;
  wire                  fifo_empty;
  wire                  rd_en;

  wire [DATA_WIDTH-1:0] fifo_level;

  // ========== Write Domain: AXI-Stream to FIFO ==========
  assign s_axis_tready = !fifo_full;
  assign write_en      = s_axis_tvalid && s_axis_tready;

  assign start_flag    = write_en && !in_frame;
  assign end_flag      = write_en && s_axis_tlast;

  assign fifo_din      = {start_flag, end_flag, s_axis_tdata};

  // ========== Frame Tracking in Write Domain ==========
  // Track packet boundaries to generate start flag
  always @(posedge s_clk or posedge s_rst) begin
    if (s_rst) begin
      in_frame <= 1'b0;
    end else begin
      if (write_en) begin
        if (!in_frame) begin
          // Start eines Frames
          in_frame <= !s_axis_tlast;
        end else if (s_axis_tlast) begin
          // Ende des Frames
          in_frame <= 1'b0;
        end
      end
    end
  end

  always @(posedge m_clk or posedge m_rst) begin
    if (m_rst) begin
      empty_delay <= 1'b1;
    end else begin
      empty_delay <= fifo_empty;
    end
  end

  // ========== Asynchronous FIFO Instantiation ==========
  // Using Intel/Altera DCFIFO with independent read/write clocks
  net_async_fifo_10b net_async_fifo_inst (
      .data(fifo_din),
      .rdclk(m_clk),
      .rdreq(rd_en),
      .wrclk(s_clk),
      .wrreq(write_en),
      .q(fifo_dout),
      .rdempty(fifo_empty),
      .rdusedw(fifo_level),
      .wrfull(fifo_full)
  );

  // ========== Read Domain: FIFO to Output ==========

  assign m_valid = !empty_delay;
  assign rd_en = m_valid && m_ready;

  assign {m_start, m_end, m_data} = fifo_dout;

endmodule
