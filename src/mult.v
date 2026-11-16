// SPDX-FileCopyrightText: © 2026 Thomas McDonagh
// SPDX-License-Identifier: Apache-2.0
//
// Shared multiplier (v3 task 9): one combinational 17 x 12 signed-by-
// unsigned product, instanced once in the top. Its operands are muxed by
// the chain `step`; each step that uses it registers a function of p.
// |a| <= 65536, b <= 4095 -> |p| <= 268,369,920 < 2^28: p never wraps in
// 29 signed bits.

`default_nettype none

module mult (
    input  wire signed [16:0] a,
    input  wire        [11:0] b,
    output wire signed [28:0] p
);

  assign p = a * $signed({1'b0, b});

endmodule

`default_nettype wire
