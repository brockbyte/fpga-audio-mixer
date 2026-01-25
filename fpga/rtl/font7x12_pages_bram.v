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
// 7x12 Font BRAM with Paging
// =============================================================================
// Read-only BRAM containing 7x12 pixel font glyphs for OLED display.
// Organized as pages for efficient rendering.
//
// Features:
// - 128 glyphs (ASCII characters)
// - 8 columns per glyph (7 pixels + 1 spacing)
// - 2 pages organization
// - Synchronous read (1-cycle latency)
// - Initialized from font7x12_pages.hex
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

module font7x12_pages_bram #(
    parameter integer GLYPHS = 128,
    parameter integer COLS   = 8,    // 7 glyph + 1 spacing
    parameter integer PAGES  = 2
) (
    input  wire        clk,
    input  wire [10:0] addr,  // 0..607
    output reg  [ 7:0] dout   // 1 Takt Latenz
);

  (* ramstyle = "M9K" *) reg [7:0] mem[0:2047];

  initial begin
    $readmemh("font7x12_pages.hex", mem);
  end

  always @(posedge clk) begin
    dout <= mem[addr];
  end

endmodule

`default_nettype wire
