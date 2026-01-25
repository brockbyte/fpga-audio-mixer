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

/*
 * MCP3008 Round-Robin SPI Reader (8 channels, 10-bit ADC)
 *
 * Features:
 * - Continuous round-robin sampling of all 8 channels
 * - Bit-accurate SPI Mode 0 timing (CPOL=0, CPHA=0)
 * - Configurable SCLK frequency and per-channel sampling rate
 * - CS-high inter-frame gap between conversions
 * - MSB-first data ordering
 *
 * SPI Sequence per conversion:
 * 1. CS asserted low
 * 2. MOSI: Send 5 control bits {Start(1), SGL/DIFF(1), D2, D1, D0}
 * 3. Wait 2 SCLK cycles for internal conversion
 * 4. MISO: Read 10 data bits continuously (MSB first)
 * 5. CS deasserted high, optional gap before next channel
 *
 * Timing:
 * - MISO sampled on rising edge of SCLK
 * - MOSI shifted on falling edge of SCLK
 * - Channel index latched with sample output
 */
`timescale 1ns / 1ps
`default_nettype none

module mcp3008_rr #(
    parameter integer CLK_HZ    = 50000000,  // System clock frequency
    parameter integer SCLK_HZ   = 1000000,   // SPI clock frequency (max 3.6 MHz at 5V)
    parameter integer PER_CH_HZ = 1000       // Target sampling rate per channel
) (
    input wire clk,
    input wire rst,

    // SPI interface signals
    output reg  cs_n,  // Chip select (active low)
    output reg  sclk,  // SPI clock
    output reg  mosi,  // Master out, slave in
    input  wire miso,  // Master in, slave out

    // Sample output interface
    output reg [9:0] sample,  // 10-bit ADC sample value
    output reg [2:0] ch_idx,  // Channel index (0-7)
    output reg       strobe   // 1-cycle pulse when sample valid
);

  // ================================================================
  // Parameter calculations
  // ================================================================
  // SCLK divider: toggle every DIV cycles => SCLK = CLK/(2*DIV)
  localparam integer DIV = (CLK_HZ / (2 * SCLK_HZ));
  // Total conversion rate for all 8 channels
  localparam integer CONV_HZ = PER_CH_HZ * 8;
  // Gap cycles between conversions to achieve target rate
  localparam integer GAP_CYCLES = (CLK_HZ / CONV_HZ);

  // ================================================================
  // Internal signals
  // ================================================================
  reg [31:0] div_ctr = 0;
  reg        sclk_en = 0;

  // FSM states
  localparam [2:0] ST_IDLE = 3'd0,  // Idle: prepare for next conversion
  ST_ASSERT = 3'd1,  // Assert CS and enable SCLK
  ST_SEND = 3'd2,  // Send 5 control bits
  ST_WAIT = 3'd3,  // Wait 2 SCLK cycles
  ST_READ = 3'd4,  // Read 10 data bits
  ST_DONE = 3'd5,  // Conversion complete
  ST_GAP = 3'd6;  // Inter-frame gap

  reg [ 2:0] st = ST_IDLE;

  // Counters for FSM phases
  reg [ 2:0] send_cnt;  // 0..4 for 5 control bits
  reg [ 1:0] wait_cnt;  // 0..1 for 2 wait clocks
  reg [ 3:0] read_cnt;  // 0..9 for 10 data bits

  reg [31:0] gap_ctr;

  // Control bits shift register (5 bits): {Start, SGL, D2, D1, D0}
  reg [ 4:0] ctrl_sr;

  // Channel management: separate next and latched channels
  reg [ 2:0] ch_next;  // Next conversion channel
  reg [ 2:0] ch_latched;  // Latched channel for current sample

  // ================================================================
  // SCLK generation (Mode 0: idle low, sample on rising edge)
  // ================================================================
  // ================================================================
  // SCLK generation (Mode 0: idle low, sample on rising edge)
  // ================================================================
  always @(posedge clk) begin
    if (rst) begin
      div_ctr <= 0;
      sclk    <= 1'b0;
    end else if (sclk_en) begin
      if (div_ctr >= (DIV - 1)) begin
        div_ctr <= 0;
        sclk    <= ~sclk;
      end else begin
        div_ctr <= div_ctr + 1'b1;
      end
    end else begin
      div_ctr <= 0;
      sclk    <= 1'b0;  // Hold low when disabled (Mode 0)
    end
  end

  // Edge detection qualifiers
  wire sclk_toggle = sclk_en && (div_ctr == (DIV - 1));
  wire next_is_rise = sclk_toggle && (sclk == 1'b0);  // Next cycle will be rising edge
  wire next_is_fall = sclk_toggle && (sclk == 1'b1);  // Next cycle will be falling edge

  // ================================================================
  // Main FSM: control conversion sequence and data shifting
  // ================================================================
  always @(posedge clk) begin
    if (rst) begin
      cs_n       <= 1'b1;
      sclk_en    <= 1'b0;
      mosi       <= 1'b0;

      sample     <= 10'd0;
      strobe     <= 1'b0;
      ch_idx     <= 3'd0;

      ch_next    <= 3'd0;
      ch_latched <= 3'd0;

      st         <= ST_IDLE;
      send_cnt   <= 3'd0;
      wait_cnt   <= 2'd0;
      read_cnt   <= 4'd0;
      ctrl_sr    <= 5'd0;
      gap_ctr    <= 32'd0;
    end else begin
      strobe <= 1'b0;

      case (st)
        // --------------------------------------------------------
        // IDLE: Prepare control bits for next channel
        // --------------------------------------------------------
        ST_IDLE: begin
          cs_n    <= 1'b1;
          sclk_en <= 1'b0;
          mosi    <= 1'b0;

          // Latch channel index for this conversion
          ch_latched <= ch_next;

          // Prepare control bits: Start=1, SGL=1, D2..D0=channel
          ctrl_sr  <= {1'b1, 1'b1, ch_next};

          send_cnt <= 3'd0;
          wait_cnt <= 2'd0;
          read_cnt <= 4'd0;

          st <= ST_ASSERT;
        end

        // --------------------------------------------------------
        // ASSERT: Enable CS and SCLK, present first MOSI bit
        // --------------------------------------------------------
        ST_ASSERT: begin
          cs_n    <= 1'b0;
          sclk_en <= 1'b1;

          // Present first MOSI bit before first rising edge
          // (Mode 0: data stable before rising edge)
          mosi    <= ctrl_sr[4];

          st <= ST_SEND;
        end

        // --------------------------------------------------------
        // SEND: Shift out 5 control bits on falling edges
        // --------------------------------------------------------
        // --------------------------------------------------------
        // SEND: Shift out 5 control bits on falling edges
        // --------------------------------------------------------
        ST_SEND: begin
          // Shift MOSI on falling edges so it is stable for next rising edge
          if (next_is_fall) begin
            // Already presented ctrl_sr[4] initially
            // After each falling edge, advance to next bit
            if (send_cnt < 3'd4) begin
              ctrl_sr  <= {ctrl_sr[3:0], 1'b0};
              mosi     <= ctrl_sr[3];  // Next bit (after shift)
              send_cnt <= send_cnt + 1'b1;
            end else begin
              // Finished sending 5 bits (Start, SGL, D2, D1, D0)
              mosi     <= 1'b0;  // Don't care afterwards
              st       <= ST_WAIT;
              wait_cnt <= 2'd0;
            end
          end
        end

        // --------------------------------------------------------
        // WAIT: Wait 2 SCLK cycles for ADC conversion
        // --------------------------------------------------------
        ST_WAIT: begin
          // Count 2 rising edges (safe, deterministic)
          if (next_is_rise) begin
            if (wait_cnt < 2'd1) begin
              wait_cnt <= wait_cnt + 1'b1;
            end else begin
              // Done waiting 2 rising edges
              read_cnt <= 4'd0;
              sample   <= 10'd0;
              st       <= ST_READ;
            end
          end
        end

        // --------------------------------------------------------
        // READ: Read 10 data bits continuously on rising edges
        // --------------------------------------------------------
        ST_READ: begin
          // Sample MISO on rising edges (MSB first)
          if (next_is_rise) begin
            sample <= {sample[8:0], miso};
            if (read_cnt < 4'd9) begin
              read_cnt <= read_cnt + 1'b1;
            end else begin
              st <= ST_DONE;
            end
          end
        end

        // --------------------------------------------------------
        // DONE: Deassert CS, output sample and advance channel
        // --------------------------------------------------------
        ST_DONE: begin
          cs_n    <= 1'b1;
          sclk_en <= 1'b0;
          mosi    <= 1'b0;

          // Output latched channel with this sample
          ch_idx  <= ch_latched;
          strobe  <= 1'b1;

          // Advance internal channel counter for next conversion
          ch_next <= ch_next + 1'b1;

          // Enter inter-frame gap
          gap_ctr <= 32'd0;
          st <= ST_GAP;
        end

        // --------------------------------------------------------
        // GAP: Inter-frame delay to control sample rate
        // --------------------------------------------------------
        ST_GAP: begin
          cs_n    <= 1'b1;
          sclk_en <= 1'b0;
          mosi    <= 1'b0;
          if (gap_ctr >= (GAP_CYCLES - 1)) begin
            st <= ST_IDLE;
          end else begin
            gap_ctr <= gap_ctr + 1'b1;
          end
        end

        default: st <= ST_IDLE;
      endcase
    end
  end

endmodule

`default_nettype wire
