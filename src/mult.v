// SPDX-FileCopyrightText: © 2026 Thomas McDonagh
// SPDX-License-Identifier: Apache-2.0
//
// Shared multiplier (v3 task 9; serial since Task 15a): one 17 x 12
// signed-by-unsigned shift-add multiplier, instanced once in the top.
// The chain step that uses it raises `start` for one clk with its
// operands on a and b; they are captured on that edge (a into ra, b into
// the low 12 bits of the product register p). Then 12 iterations, one per
// clk, consume b LSB first:
//   sum = p[28:12] + (p[0] ? a : 0)      (18-bit signed)
//   p  <= {sum, p[11:1]}                  (arithmetic shift right by 1)
// so the high part H = p[28:12] holds floor(a * (b mod 2^i) / 2^i) after
// i iterations and the bits of b shift out as the product's low bits
// shift in. After the 12th, p = a * b exactly and done pulses for one clk
// (p then holds until the next start).
// Latency L = 12: start sampled at edge E0 -> done = 1 (and p final)
// after edge E12; the step registers its result at E13.
// Bounds: |a| <= 65536, b <= 4095 -> |p| < 2^28, so p never wraps in 29
// signed bits; H stays in [-65536, 65534] (17-bit signed) and H + a in
// [-131072, 131069] (the 18-bit sum).

`default_nettype none

module mult (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               start,
    input  wire signed [16:0] a,
    input  wire        [11:0] b,
    output reg  signed [28:0] p,
    output reg                done
);

  reg signed [16:0] ra;    // captured multiplicand
  reg        [3:0]  cnt;   // iterations left (0 = idle)

  wire signed [17:0] sum = {p[28], p[28:12]} + (p[0] ? {ra[16], ra} : 18'sd0);

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ra   <= 17'sd0;
      p    <= 29'sd0;
      cnt  <= 4'd0;
      done <= 1'b0;
    end else begin
      done <= 1'b0;
      if (start) begin
        ra  <= a;
        p   <= {17'd0, b};
        cnt <= 4'd12;
      end else if (cnt != 4'd0) begin
        p    <= {sum, p[11:1]};
        cnt  <= cnt - 4'd1;
        done <= (cnt == 4'd1);
      end
    end
  end

endmodule

`default_nettype wire
