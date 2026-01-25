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
// Dual-Port RAM: Single Write / Single Read
// =============================================================================
// True dual-port RAM with Port A (write) and Port B (synchronous read).
// 1-cycle latency on read port. Supports optional hex file initialization.
//
// Parameters:
// - ADDR_WIDTH: Address bus width (8 = 256 locations)
// - DATA_WIDTH: Data width (typically 8 bits)
// - INIT_FILE: Optional hex file for initialization
// =============================================================================

// name_dp_ram.v  (Verilog-2001)
// True dual-port RAM: Port A (write), Port B (read)
// Synchronous read (1-cycle latency) on port B
`timescale 1ns / 1ps
`default_nettype none

module name_dp_ram #(
    parameter integer ADDR_WIDTH = 8,  // 256 deep -> 8
    parameter integer DATA_WIDTH = 8,
    parameter         INIT_FILE  = ""  // z.B. "name_init.hex" oder ""
) (
    input wire clk,

    // Port A: write
    input wire                  we_a,
    input wire [ADDR_WIDTH-1:0] addr_a,
    input wire [DATA_WIDTH-1:0] din_a,

    // Port B: read
    input  wire [ADDR_WIDTH-1:0] addr_b,
    output reg  [DATA_WIDTH-1:0] dout_b
);
  // Inferred RAM
  // reg [DATA_WIDTH-1:0] mem [0:(1<<ADDR_WIDTH)-1];

  localparam integer DEPTH = (1 << ADDR_WIDTH);

  reg [DATA_WIDTH-1:0] mem[0:DEPTH-1];

  integer i;
  initial begin
    // optional: default 0
    for (i = 0; i < DEPTH; i = i + 1) mem[i] = {DATA_WIDTH{1'b0}};

    // optional: load hex file
    if (INIT_FILE != "") begin
      $readmemh(INIT_FILE, mem);
    end
  end

  always @(posedge clk) begin
    // Write port
    if (we_a) begin
      mem[addr_a] <= din_a;
    end

    // Read port (sync)
    dout_b <= mem[addr_b];
  end
endmodule

`resetall
`default_nettype wire
