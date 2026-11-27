// SPDX-FileCopyrightText: © 2026 Thomas McDonagh
// SPDX-License-Identifier: Apache-2.0
//
// RC knob reader (v3 task 9; one shared round-robin reader for K1..K4
// since Task 15a). Each pin sees a pot + cap to VDD. The reader measures
// one knob at a time, knob 0, 1, 2, 3, 0, .. in turn (sel); every
// measurement is exactly the per-knob algorithm of the v3 task 9 reader:
// it discharges the selected cap by driving that pin low (drive_low[sel]
// = 1 -> uio_oe) for 1024 clk, then releases it (CHARGE) and counts clk
// in the 16-bit cnt until the selected pin's 2-flop synchronizer reads 1:
//   raw = min(255, cnt >> 6)
// and present[sel] <= 1. If cnt reaches 65535 first (pin never rises:
// knob absent / pin tied low) the measurement times out: present[sel] <=
// 0, value holds. Either way the next knob's DISCHARGE starts on the same
// edge (sel + 1, cnt = 0). Pins that are not selected are released
// (inputs): a present knob's cap charges to 1 between its turns and is
// pulled low again by its next 1024-clk discharge.
// Timing (pins the testbench knob model): the release edge leaves cnt = 0;
// a pin that rises right after clk edge N of the charge phase is seen
// through the synchronizer at edge N + 3 with cnt = N + 2 (SYNC_LAT = 2),
// and the charge phase lasts N + 3 clk. One rotation takes at most
// 4 x (1024 + 65536) clk (all absent).
// Smoothing per completed measurement, on the selected knob's state only:
// the first one while not present (after reset or an absence) loads
// acc = raw << 2 and value = raw (fast startup); later ones run the IIR
//   acc <= acc - (acc >> 2) + raw,  s = acc_new >> 2,
// and value <= s only if |s - value| >= 2 (hysteresis). acc fits 10 bits:
// acc <= 1023 and raw <= 255 give acc - acc/4 + raw <= 768 + 255.
// Shared: cnt, sel, the IIR datapath; per knob: a synchronizer, acc[9:0],
// value[7:0], present. value[8k+7:8k] / present[k] / drive_low[k] =
// knob k (K(k+1), uio[2+k]).

`default_nettype none

module knob_rc4 (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [3:0]  pin_in,
    output reg  [3:0]  drive_low,
    output reg  [31:0] value,
    output reg  [3:0]  present
);

  reg [3:0]  pin_s1, pin_s2;   // per-pin synchronizers
  reg [1:0]  sel;              // knob being measured
  reg [15:0] cnt;              // discharge timer / charge count
  reg [39:0] acc;              // per-knob IIR accumulators (4 x smoothed value)
  integer    k;                // knob index (write decode)

  wire       dis     = |drive_low;          // DISCHARGE phase (sel's pin driven)
  wire       pin_sel = pin_s2[sel];
  wire [9:0] acc_s   = acc[sel*10 +: 10];
  wire [7:0] val_s   = value[sel*8 +: 8];
  wire       pres_s  = present[sel];

  wire [7:0] raw    = |cnt[15:14] ? 8'hFF : cnt[13:6];  // min(255, cnt >> 6)
  wire [9:0] acc_nx = acc_s - {2'b00, acc_s[9:2]} + {2'b00, raw};
  wire [7:0] s      = acc_nx[9:2];
  // hysteresis |s - value| >= 2 as one 9-bit difference d = s - value:
  // move unless d is 0, +1 (d[8:1] == 0) or -1 (0x1FF)
  wire [8:0] dif    = {1'b0, s} - {1'b0, val_s};
  wire       move   = ~((dif[8:1] == 8'h00) | (dif == 9'h1FF));

  wire [1:0] sel_nx = sel + 2'd1;
  wire [3:0] drv_nx = 4'b0001 << sel_nx;     // next knob's discharge

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pin_s1    <= 4'd0;
      pin_s2    <= 4'd0;
      sel       <= 2'd0;
      cnt       <= 16'd0;
      drive_low <= 4'b0001;
      acc       <= 40'd0;
      value     <= 32'd0;
      present   <= 4'd0;
    end else begin
      pin_s1 <= pin_in;
      pin_s2 <= pin_s1;
      if (dis) begin                             // DISCHARGE, 1024 clk
        if (cnt == 16'd1023) begin
          drive_low <= 4'b0000;
          cnt       <= 16'd0;
        end else begin
          cnt <= cnt + 16'd1;
        end
      end else if (pin_sel) begin                // CHARGE done: measurement
        drive_low <= drv_nx;
        sel       <= sel_nx;
        cnt       <= 16'd0;
        for (k = 0; k < 4; k = k + 1)            // only knob sel's state
          if ({30'd0, sel} == k) begin
            present[k] <= 1'b1;
            // !present: fast startup (acc = raw << 2, value = raw);
            // else IIR, value moves only past the hysteresis
            acc[k*10 +: 10] <= pres_s ? acc_nx : {raw, 2'b00};
            if (!pres_s || move) value[k*8 +: 8] <= pres_s ? s : raw;
          end
      end else if (cnt == 16'hFFFF) begin        // timeout: knob absent
        drive_low    <= drv_nx;
        sel          <= sel_nx;
        cnt          <= 16'd0;
        for (k = 0; k < 4; k = k + 1)
          if ({30'd0, sel} == k) present[k] <= 1'b0;
      end else begin
        cnt <= cnt + 16'd1;
      end
    end
  end

endmodule

`default_nettype wire
