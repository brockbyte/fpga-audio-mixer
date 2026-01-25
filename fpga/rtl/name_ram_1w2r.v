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
// Dual-Read-Port RAM (1 Write, 2 Independent Reads)
// =============================================================================
// Synthesized as two identical RAM blocks with mirrored write port.
// Provides 1 write port and 2 independent synchronous read ports,
// each with 1-cycle latency. Used for channel name storage (BRAM).
//
// Features:
// - Dual DP-RAM instantiation for 2 independent reads
// - Synchronous reads with 1-cycle latency
// - Optional hex file initialization
// =============================================================================

// name_ram_1w2r.v  (Verilog-2001)
// 1 write port, 2 independent read ports (sync, 1-cycle latency)
// Implemented by duplicating RAM blocks (write mirrored)

`timescale 1ns / 1ps
`default_nettype none

module name_ram_1w2r #(
    parameter integer ADDR_WIDTH = 8,
    parameter integer DATA_WIDTH = 8,
    parameter         INIT_FILE  = "name_init.hex"  // neu
) (
    input wire clk,

    // Write port
    input wire                  we,
    input wire [ADDR_WIDTH-1:0] wr_addr,
    input wire [DATA_WIDTH-1:0] wr_data,

    // Read port 0
    input  wire [ADDR_WIDTH-1:0] rd0_addr,
    output wire [DATA_WIDTH-1:0] rd0_data,

    // Read port 1
    input  wire [ADDR_WIDTH-1:0] rd1_addr,
    output wire [DATA_WIDTH-1:0] rd1_data
);

  // RAM copy 0
  name_dp_ram #(
      .ADDR_WIDTH(ADDR_WIDTH),
      .DATA_WIDTH(DATA_WIDTH),
      .INIT_FILE (INIT_FILE)
  ) u_ram0 (
      .clk   (clk),
      .we_a  (we),
      .addr_a(wr_addr),
      .din_a (wr_data),
      .addr_b(rd0_addr),
      .dout_b(rd0_data)
  );

  // RAM copy 1
  name_dp_ram #(
      .ADDR_WIDTH(ADDR_WIDTH),
      .DATA_WIDTH(DATA_WIDTH),
      .INIT_FILE (INIT_FILE)
  ) u_ram1 (
      .clk   (clk),
      .we_a  (we),
      .addr_a(wr_addr),
      .din_a (wr_data),
      .addr_b(rd1_addr),
      .dout_b(rd1_data)
  );

endmodule

`resetall
`default_nettype wire
