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
// Single-Port Read-Only BRAM
// =============================================================================
// Read-only BRAM for storing SDP (Session Description Protocol) or other
// constant data. Configurable address and data widths, with hex file init.
//
// Features:
// - Parameterizable address/data widths
// - Synchronous read (1-cycle latency)
// - Hex file initialization
// - Designed for FPGA BRAM resources
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

module sdp_rom #(
    parameter integer ADDR_WIDTH = 10,
    parameter integer ROM_LEN    = 157,
    // Verilog-2001: filename as plain parameter (packed array of chars)
    parameter         INIT_FILE  = "sdp2_rom.hex"  // up to 32 chars
) (
    input  wire                  clk,
    input  wire [ADDR_WIDTH-1:0] addr,
    output reg  [           7:0] data
);

  reg [7:0] mem[0:(1<<ADDR_WIDTH)-1];

  initial begin
    $readmemh(INIT_FILE, mem);
  end

  always @(posedge clk) begin
    data <= mem[addr];
  end

endmodule

`default_nettype wire

