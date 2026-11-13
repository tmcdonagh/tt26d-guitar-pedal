/*
 * Copyright (c) 2026 Tom McDonagh
 * SPDX-License-Identifier: Apache-2.0
 *
 * Serial shift-add multiplier: signed 16-bit x unsigned 8-bit, result
 * p = (a * b) >> 8 (arithmetic shift). 8 cycles from start to done.
 */

`default_nettype none

module serial_mult (
    input  wire               clk,
    input  wire               rst_n,
    input  wire signed [15:0] a,
    input  wire        [7:0]  b,
    input  wire               start,   // 1-clk pulse
    output reg  signed [15:0] p,
    output reg                done
);

  reg signed [23:0] acc;
  reg signed [23:0] a_sh;
  reg        [7:0]  b_sh;
  reg        [2:0]  cnt;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      acc  <= 24'sd0;
      a_sh <= 24'sd0;
      b_sh <= 8'd0;
      cnt  <= 3'd0;
      p    <= 16'sd0;
      done <= 1'b0;
    end else if (start) begin
      a_sh <= {{8{a[15]}}, a};  // sign-extend into shift register
      b_sh <= b;
      acc  <= 24'sd0;
      cnt  <= 3'd0;
      done <= 1'b0;
    end else if (!done) begin
      if (b_sh[0]) acc <= acc + a_sh;
      a_sh <= a_sh <<< 1;
      b_sh <= {1'b0, b_sh[7:1]};
      if (cnt == 3'd7) begin
        done <= 1'b1;
        p    <= (acc + (b_sh[0] ? a_sh : 24'sd0)) >>> 8;
      end else begin
        cnt <= cnt + 3'd1;
      end
    end
  end

endmodule

`default_nettype wire
