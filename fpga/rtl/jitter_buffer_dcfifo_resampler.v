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
// Jitter Buffer with Adaptive Resampling
// =============================================================================
// Audio jitter buffer with 2-sample moving average smoothing for clock domain
// crossing. Implements adaptive buffering: inserts (duplicates) audio when
// buffer is too empty, skips samples when buffer is too full.
//
// Features:
// - Asynchronous FIFO for clock domain crossing (clk_in to clk_out)
// - Target fill level with hysteresis (target ±threshold)
// - Automatic sample duplication for underflow
// - Automatic sample skipping for overflow
// - Debug counters for fill level, duplicates, and skips
// =============================================================================

/*
* Module: AudioJitterBuffer
* Description: Jitter buffer with 4-sample moving average smoothing for
* clock domain crossing (24-bit L/R parallel).
*/

module AudioJitterBuffer (
    // Input Domain (clk_in)
    input wire        clk_in,
    input wire        rst_n,
    input wire [23:0] din_l,
    input wire [23:0] din_r,
    input wire        wr_en,

    // Output Domain (clk_out)
    input  wire        clk_out,
    input  wire        rd_en,
    output reg  [23:0] dout_l,
    output reg  [23:0] dout_r,
    output reg         out_valid,

    // Status & Debug
    output wire [ 9:0] fill_count,
    output reg  [31:0] cnt_fifo_reads,
    output reg  [31:0] cnt_out_samples,
    output reg  [31:0] cnt_duplicated,
    output reg  [31:0] cnt_skipped
);

  // Internal FIFO Signals
  wire [47:0] fifo_dout;
  reg         fifo_rd_en;
  wire        empty;

  // Threshold Parameters
  localparam TARGET_LEVEL = 144;
  localparam STARTUP_LEVEL = 144;
  localparam THRESHOLD = 48;

  // Instantiate Async FIFO
  custom_async_fifo custom_async_fifo_inst (
      .data({din_l, din_r}),
      .rdclk(clk_out),
      .rdreq(fifo_rd_en),
      .wrclk(clk_in),
      .wrreq(wr_en),
      .q(fifo_dout),
      .rdempty(empty),
      .rdusedw(fill_count),
      .wrfull()
  );

  // Smoothing History (4 Samples)
  reg signed [23:0] hist_l[0:1];
  reg signed [23:0] hist_r[0:1];

  reg [5:0] cooldown;

  // 4-Sample Moving Average Calculation
  wire signed [25:0] avg_l_sum = hist_l[0] + hist_l[1];
  wire signed [25:0] avg_r_sum = hist_r[0] + hist_r[1];
  wire signed [23:0] avg_val_l = avg_l_sum[24:1];
  wire signed [23:0] avg_val_r = avg_r_sum[24:1];

  reg buffer_ready;
  reg [3:0] state;
  localparam S_IDLE   = 4'd0,
S_READ   = 4'd1,
S_SKIP   = 4'd2,
S_PREFILL_WAIT = 4'd3,
S_PREFILL = 4'd4,
S_READ_WAIT = 4'd5,
S_SKIP_WAIT = 4'd6;

  always @(posedge clk_out or negedge rst_n) begin
    if (!rst_n) begin
      state <= S_IDLE;
      fifo_rd_en <= 0;
      out_valid <= 0;
      buffer_ready <= 0;
      cooldown <= 6'd0;
      cnt_fifo_reads <= 0;
      cnt_out_samples <= 0;
      cnt_duplicated <= 0;
      cnt_skipped <= 0;
      hist_l[0] <= 0;
      hist_r[0] <= 0;
      hist_l[1] <= 0;
      hist_r[1] <= 0;
    end else begin
      fifo_rd_en <= 0;
      out_valid  <= 0;

      if (!buffer_ready) begin
        if (fill_count >= STARTUP_LEVEL) begin
          buffer_ready <= 1;
          fifo_rd_en <= 1;
          state <= S_PREFILL_WAIT;
        end
      end else begin
        case (state)
          S_PREFILL_WAIT: begin
            state <= S_PREFILL;
          end
          S_PREFILL: begin
            hist_l[1] <= fifo_dout[47:24];
            hist_r[1] <= fifo_dout[23:0];
            state <= S_IDLE;
          end
          S_IDLE: begin
            if (rd_en) begin
              if (cooldown > 0) cooldown <= cooldown - 6'd1;
              // CASE: BUFFER TOO FULL -> SKIP
              if (fill_count > (TARGET_LEVEL + THRESHOLD)) begin
                fifo_rd_en <= 1;  // Start reading first sample to discard
                state <= S_SKIP_WAIT;
              end  // CASE: BUFFER TOO EMPTY -> INSERT (DUPLICATE/SMOOTH)
              else if (fill_count < (TARGET_LEVEL - THRESHOLD) && cooldown == 6'd0) begin
                dout_l <= avg_val_l;
                dout_r <= avg_val_r;
                out_valid <= 1;
                cnt_duplicated <= cnt_duplicated + 1;
                cnt_out_samples <= cnt_out_samples + 1;
                cooldown <= 6'd32;
                // Shift average back into history to keep signal moving
                //hist_l[0] <= avg_val_l; hist_l[1] <= hist_l[0]; hist_l[2] <= hist_l[1]; hist_l[3] <= hist_l[2];
                //hist_r[0] <= avg_val_r; hist_r[1] <= hist_r[0]; hist_r[2] <= hist_r[1]; hist_r[3] <= hist_r[2];
              end  // CASE: NORMAL OPERATION
              else begin
                fifo_rd_en <= 1;
                state <= S_READ_WAIT;
              end
            end
          end

          S_READ_WAIT: state <= S_READ;

          S_READ: begin
            hist_l[1] <= fifo_dout[47:24];
            hist_r[1] <= fifo_dout[23:0];
            dout_l <= hist_l[1];
            dout_r <= hist_r[1];
            out_valid <= 1;
            cnt_fifo_reads <= cnt_fifo_reads + 1;
            cnt_out_samples <= cnt_out_samples + 1;
            // Update History
            hist_l[0] <= hist_l[1];
            hist_r[0] <= hist_r[1];
            state <= S_IDLE;
          end

          S_SKIP_WAIT: begin
            state <= S_SKIP;
            fifo_rd_en <= 1;
          end

          S_SKIP: begin
            // Previous Takt was a read, now we read the NEXT one immediately
            // This effectively skips the first one by overwriting it
            //fifo_rd_en <= 1;
            cnt_skipped <= cnt_skipped + 1;
            cnt_fifo_reads <= cnt_fifo_reads + 1;  // We consumed one extra
            state <= S_READ;  // Go to output the second read sample
          end
        endcase
      end
    end
  end
endmodule
