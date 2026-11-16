// SPDX-FileCopyrightText: © 2026 Thomas McDonagh
// SPDX-License-Identifier: Apache-2.0
//
// RC knob reader (v3 task 9). The pin sees a pot + cap to VDD: the reader
// discharges the cap by driving the pin low (drive_low = 1 -> uio_oe) for
// 1024 clk, then releases it (CHARGE) and counts clk in the 16-bit cnt
// until the 2-flop-synchronized pin reads 1:
//   raw = min(255, cnt >> 6)
// and present <= 1. If cnt reaches 65535 first (pin never rises: knob
// absent / pin tied low) the measurement times out: present <= 0, value
// holds. Either way it returns to DISCHARGE.
// Timing (pins the testbench knob model): the release edge leaves cnt = 0;
// a pin that rises right after clk edge N of the charge phase is seen
// through the synchronizer at edge N + 3 with cnt = N + 2 (SYNC_LAT = 2),
// and the charge phase lasts N + 3 clk.
// Smoothing per completed measurement: the first one while not present
// (after reset or an absence) loads acc = raw << 2 and value = raw
// (fast startup); later ones run the IIR
//   acc <= acc - (acc >> 2) + raw,  s = acc_new >> 2,
// and value <= s only if |s - value| >= 2 (hysteresis). acc fits 10 bits:
// acc <= 1023 and raw <= 255 give acc - acc/4 + raw <= 768 + 255.

`default_nettype none

module knob_rc (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       pin_in,
    output reg        drive_low,
    output reg  [7:0] value,
    output reg        present
);

  reg        pin_s1, pin_s2;   // synchronizer
  reg [15:0] cnt;              // discharge timer / charge count
  reg [9:0]  acc;              // IIR accumulator (4 x the smoothed value)

  wire [7:0] raw    = |cnt[15:14] ? 8'hFF : cnt[13:6];  // min(255, cnt >> 6)
  wire [9:0] acc_nx = acc - {2'b00, acc[9:2]} + {2'b00, raw};
  wire [7:0] s      = acc_nx[9:2];
  wire [7:0] dif    = (s > value) ? (s - value) : (value - s);
  wire       move   = (dif >= 8'd2);

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pin_s1    <= 1'b0;
      pin_s2    <= 1'b0;
      cnt       <= 16'd0;
      drive_low <= 1'b1;
      acc       <= 10'd0;
      value     <= 8'd0;
      present   <= 1'b0;
    end else begin
      pin_s1 <= pin_in;
      pin_s2 <= pin_s1;
      if (drive_low) begin                       // DISCHARGE, 1024 clk
        if (cnt == 16'd1023) begin
          drive_low <= 1'b0;
          cnt       <= 16'd0;
        end else begin
          cnt <= cnt + 16'd1;
        end
      end else if (pin_s2) begin                 // CHARGE done: measurement
        drive_low <= 1'b1;
        cnt       <= 16'd0;
        present   <= 1'b1;
        if (!present) begin                      // fast startup
          acc   <= {raw, 2'b00};
          value <= raw;
        end else begin
          acc <= acc_nx;
          if (move) value <= s;
        end
      end else if (cnt == 16'hFFFF) begin        // timeout: knob absent
        drive_low <= 1'b1;
        cnt       <= 16'd0;
        present   <= 1'b0;
      end else begin
        cnt <= cnt + 16'd1;
      end
    end
  end

endmodule

`default_nettype wire
