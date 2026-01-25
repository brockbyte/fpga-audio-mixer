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
// PTPv2 Sync/Follow_Up Message Parser with Stream Interface
// =============================================================================
// Parses PTP v2 event messages (Sync and Follow_Up) from a packet stream.
// Timestamps Sync messages with local PTP time and matches Follow_Up by sequence ID.
//
// Input Stream Format:
// - start = 1 on first byte of packet
// - end = 1 on last byte of packet
// - data = 8-bit payload (PTP header starts at byte 0)
//
// Output: Generates event_valid pulse when matching Sync+Follow_Up pair received,
//         providing both master timestamp and local receipt timestamp.
// =============================================================================

`timescale 1ns / 1ps
`default_nettype none

module ptp_v2_sync_follow_parser_stream #(
    parameter integer TSTAMP_SEC_WIDTH = 48,
    parameter integer TSTAMP_NS_WIDTH  = 32
) (
    input wire clk,
    input wire rst,

    // ========== Packet Stream Input (from FIFO or wrapper) ==========
    input  wire [7:0] s_data,
    input  wire       s_valid,
    output wire       s_ready,
    input  wire       s_start,
    input  wire       s_end,

    // ========== Current Local PTP Time ==========
    // From timebase in 50 MHz domain
    input wire [TSTAMP_SEC_WIDTH-1:0] ptp_time_sec,
    input wire [ TSTAMP_NS_WIDTH-1:0] ptp_time_ns,

    // ========== Event Output for Servo ==========
    // Valid pulse with master and local timestamps
    output reg                        event_valid,
    output reg [TSTAMP_SEC_WIDTH-1:0] master_sec,
    output reg [ TSTAMP_NS_WIDTH-1:0] master_ns,
    output reg [TSTAMP_SEC_WIDTH-1:0] local_sec,
    output reg [ TSTAMP_NS_WIDTH-1:0] local_ns
);

  // No backpressure - always ready
  assign s_ready = 1'b1;

  // ========== Frame State Machine ==========
  reg       in_frame;
  reg [9:0] byte_cnt;  // Sufficient for PTP header + TLVs

  // ========== PTP Header Fields ==========
  reg [3:0] msg_type, ptp_vers;  // messageType (lower nibble of Byte 0)
  reg [                15:0] seq_id;  // sequenceId (Bytes 30-31)

  // ========== Timestamp from Follow_Up ==========
  reg [TSTAMP_SEC_WIDTH-1:0] fu_master_sec_reg;
  reg [ TSTAMP_NS_WIDTH-1:0] fu_master_ns_reg;

  // ========== Last Sync Message Storage ==========
  reg [                15:0] last_sync_seqid;
  reg [TSTAMP_SEC_WIDTH-1:0] last_sync_local_sec;
  reg [ TSTAMP_NS_WIDTH-1:0] last_sync_local_ns;
  reg                        have_last_sync;

  // ========== Local Time at Frame Start ==========
  reg [TSTAMP_SEC_WIDTH-1:0] frame_local_sec;
  reg [ TSTAMP_NS_WIDTH-1:0] frame_local_ns;
  reg                        frame_ts_valid;

  always @(posedge clk or posedge rst) begin
    if (rst) begin
      in_frame            <= 1'b0;
      byte_cnt            <= 10'd0;
      msg_type            <= 4'd0;
      ptp_vers            <= 4'd0;
      seq_id              <= 16'd0;
      fu_master_sec_reg   <= {TSTAMP_SEC_WIDTH{1'b0}};
      fu_master_ns_reg    <= {TSTAMP_NS_WIDTH{1'b0}};
      last_sync_seqid     <= 16'd0;
      last_sync_local_sec <= {TSTAMP_SEC_WIDTH{1'b0}};
      last_sync_local_ns  <= {TSTAMP_NS_WIDTH{1'b0}};
      have_last_sync      <= 1'b0;
      frame_local_sec     <= {TSTAMP_SEC_WIDTH{1'b0}};
      frame_local_ns      <= {TSTAMP_NS_WIDTH{1'b0}};
      frame_ts_valid      <= 1'b0;
      event_valid         <= 1'b0;
      master_sec          <= {TSTAMP_SEC_WIDTH{1'b0}};
      master_ns           <= {TSTAMP_NS_WIDTH{1'b0}};
      local_sec           <= {TSTAMP_SEC_WIDTH{1'b0}};
      local_ns            <= {TSTAMP_NS_WIDTH{1'b0}};
    end else begin
      event_valid <= 1'b0;  // Default

      if (s_valid && s_ready) begin
        // Start eines neuen Frames?
        if (s_start) begin
          in_frame        <= 1'b1;
          byte_cnt        <= 10'd1;
          msg_type        <= s_data[3:0];  // Byte 0: messageType in Lower-Nibble

          // Lokale PTP-Zeit beim Frame-Start samplen
          frame_local_sec <= ptp_time_sec;
          frame_local_ns  <= ptp_time_ns;
          //frame_ts_valid  <= 1'b1;
        end else if (in_frame) begin
          // Innerhalb des Frames
          byte_cnt <= byte_cnt + 10'd1;

          // relevante Felder des PTP-Headers extrahieren
          case (byte_cnt)
            10'd1:  ptp_vers <= s_data[3:0];
            // sequenceId: Bytes 30-31 (Big Endian)
            10'd30: seq_id[15:8] <= s_data;
            10'd31: seq_id[7:0] <= s_data;

            // preciseOriginTimestamp seconds (Bytes 34-39)
            10'd34: fu_master_sec_reg[47:40] <= s_data;
            10'd35: fu_master_sec_reg[39:32] <= s_data;
            10'd36: fu_master_sec_reg[31:24] <= s_data;
            10'd37: fu_master_sec_reg[23:16] <= s_data;
            10'd38: fu_master_sec_reg[15:8] <= s_data;
            10'd39: fu_master_sec_reg[7:0] <= s_data;

            // preciseOriginTimestamp nanoseconds (Bytes 40-43)
            10'd40: fu_master_ns_reg[31:24] <= s_data;
            10'd41: fu_master_ns_reg[23:16] <= s_data;
            10'd42: fu_master_ns_reg[15:8] <= s_data;
            10'd43: fu_master_ns_reg[7:0] <= s_data;

            default: ;
          endcase
        end

        // Frame end?
        if (s_end && in_frame) begin
          in_frame <= 1'b0;

          // Decide based on messageType
          if (msg_type == 4'h0 && ptp_vers == 4'h2) begin
            // Sync message
            last_sync_seqid     <= seq_id;
            last_sync_local_sec <= frame_local_sec;
            last_sync_local_ns  <= frame_local_ns;
            have_last_sync      <= 1'b1;
          end else if (msg_type == 4'h8 && ptp_vers == 4'h2) begin
            // Follow_Up message
            if (have_last_sync && seq_id == last_sync_seqid) begin
              event_valid <= 1'b1;
              master_sec  <= fu_master_sec_reg;
              master_ns   <= fu_master_ns_reg;
              local_sec   <= last_sync_local_sec;
              local_ns    <= last_sync_local_ns;
            end
          end
        end  // s_end
      end  // s_valid
    end
  end

endmodule
