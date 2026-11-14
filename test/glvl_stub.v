// SPDX-FileCopyrightText: © 2026 Thomas McDonagh
// SPDX-License-Identifier: Apache-2.0
//
// Stand-in for sky130_fd_sc_hd__udp_pwrgood_pp$PG — a UDP in primitives.v,
// which Verilator-based tooling cannot compile. Semantics of the UDP:
// out = UDP_IN when VGND==0 && VPWR==1, X otherwise. In the FUNCTIONAL
// gate-level audio sim the power pins are tied good (tb_audio_gl.v:
// VPWR=1'b1, VGND=1'b0), so pass-through is exact.
// Used by wav_tool.py glvl_cmd(); never used by the iverilog GL path.

module sky130_fd_sc_hd__udp_pwrgood_pp$PG (out, UDP_IN, VGND, VPWR);
  output out;
  input  UDP_IN;
  input  VGND;
  input  VPWR;
  assign out = UDP_IN;
endmodule

module sky130_fd_sc_hd__udp_pwrgood_pp$P (out, UDP_IN, VPWR);
  output out;
  input  UDP_IN;
  input  VPWR;
  assign out = UDP_IN;
endmodule

// ground-only variant: pass when VGND==0 (X otherwise; tied good in sim)
module sky130_fd_sc_hd__udp_pwrgood_pp$G (out, UDP_IN, VGND);
  output out;
  input  UDP_IN;
  input  VGND;
  assign out = UDP_IN;
endmodule

// same table as pp$PG (VPWR=1 && VGND=0 -> pass)
module sky130_fd_sc_hd__udp_pwrgood$l_pp$PG (UDP_OUT, UDP_IN, VPWR, VGND);
  output UDP_OUT;
  input  UDP_IN;
  input  VPWR;
  input  VGND;
  assign UDP_OUT = UDP_IN;
endmodule

// sleep-variant: SLEEP=1 clamps out=0; tied SLEEP=0 in this sim -> pass
module sky130_fd_sc_hd__udp_pwrgood$l_pp$PG$S (UDP_OUT, UDP_IN, VPWR, VGND, SLEEP);
  output UDP_OUT;
  input  UDP_IN;
  input  VPWR;
  input  VGND;
  input  SLEEP;
  assign UDP_OUT = (SLEEP === 1'b1) ? 1'b0 : UDP_IN;
endmodule

// ---------------------------------------------------------------------
// Sequential/combinational UDP models (transcribed from the primitives.v
// UDP tables; Verilator cannot compile UDPs). Power guard mirrors the
// _pp$PG$N rows: any VPWR/VGND deviation from good forces X. NOTIFIER is
// an accepted-but-ignored port: under FUNCTIONAL its change row is "-"
// (hold). With clean stimulus the reduced-pessimism x rows never apply.
// ---------------------------------------------------------------------

module sky130_fd_sc_hd__udp_dff$P (Q, D, CLK);
  output reg Q;
  input  D, CLK;
  always @(posedge CLK) Q <= D;
endmodule

module sky130_fd_sc_hd__udp_dff$P_pp$PG$N (Q, D, CLK, NOTIFIER, VPWR, VGND);
  output reg Q;
  input  D, CLK, NOTIFIER, VPWR, VGND;
  always @(posedge CLK or VPWR or VGND) begin
    if (VPWR !== 1'b1 || VGND !== 1'b0) Q <= 1'bx;
    else Q <= D;
  end
endmodule

module sky130_fd_sc_hd__udp_dff$NSR (Q, SET, RESET, CLK_N, D);
  output reg Q;
  input  SET, RESET, CLK_N, D;
  // async set dominates async reset (both active high), posedge clk data
  always @(posedge CLK_N or posedge SET or posedge RESET) begin
    if (SET === 1'b1)       Q <= 1'b1;
    else if (RESET === 1'b1) Q <= 1'b0;
    else                     Q <= D;
  end
endmodule

module sky130_fd_sc_hd__udp_dff$NSR_pp$PG$N (Q, SET, RESET, CLK_N, D, NOTIFIER, VPWR, VGND);
  output reg Q;
  input  SET, RESET, CLK_N, D, NOTIFIER, VPWR, VGND;
  always @(posedge CLK_N or posedge SET or posedge RESET or VPWR or VGND) begin
    if (VPWR !== 1'b1 || VGND !== 1'b0) Q <= 1'bx;
    else if (SET === 1'b1)       Q <= 1'b1;
    else if (RESET === 1'b1)     Q <= 1'b0;
    else                         Q <= D;
  end
endmodule

module sky130_fd_sc_hd__udp_dff$PR (Q, D, CLK, RESET);
  output reg Q;
  input  D, CLK, RESET;
  // async reset active high, posedge clk data
  always @(posedge CLK or posedge RESET) begin
    if (RESET === 1'b1) Q <= 1'b0;
    else                Q <= D;
  end
endmodule

module sky130_fd_sc_hd__udp_dff$PR_pp$PG$N (Q, D, CLK, RESET, NOTIFIER, VPWR, VGND);
  output reg Q;
  input  D, CLK, RESET, NOTIFIER, VPWR, VGND;
  always @(posedge CLK or posedge RESET or VPWR or VGND) begin
    if (VPWR !== 1'b1 || VGND !== 1'b0) Q <= 1'bx;
    else if (RESET === 1'b1) Q <= 1'b0;
    else                     Q <= D;
  end
endmodule

module sky130_fd_sc_hd__udp_dff$PS (Q, D, CLK, SET);
  output reg Q;
  input  D, CLK, SET;
  // async set active high, posedge clk data
  always @(posedge CLK or posedge SET) begin
    if (SET === 1'b1) Q <= 1'b1;
    else              Q <= D;
  end
endmodule

module sky130_fd_sc_hd__udp_dff$PS_pp$PG$N (Q, D, CLK, SET, NOTIFIER, VPWR, VGND);
  output reg Q;
  input  D, CLK, SET, NOTIFIER, VPWR, VGND;
  always @(posedge CLK or posedge SET or VPWR or VGND) begin
    if (VPWR !== 1'b1 || VGND !== 1'b0) Q <= 1'bx;
    else if (SET === 1'b1) Q <= 1'b1;
    else                   Q <= D;
  end
endmodule

module sky130_fd_sc_hd__udp_dlatch$P (Q, D, GATE);
  output reg Q;
  input  D, GATE;
  always @(*) begin
    if (GATE === 1'b1) Q = D;   // transparent; holds when GATE==0
  end
endmodule

module sky130_fd_sc_hd__udp_dlatch$lP (Q, D, GATE);
  output reg Q;
  input  D, GATE;
  always @(*) begin
    if (GATE === 1'b1) Q = D;   // table identical to dlatch$P
  end
endmodule

module sky130_fd_sc_hd__udp_dlatch$P_pp$PG$N (Q, D, GATE, NOTIFIER, VPWR, VGND);
  output reg Q;
  input  D, GATE, NOTIFIER, VPWR, VGND;
  always @(*) begin
    if (VPWR !== 1'b1 || VGND !== 1'b0) Q = 1'bx;
    else if (GATE === 1'b1) Q = D;
  end
endmodule

module sky130_fd_sc_hd__udp_dlatch$lP_pp$PG$N (Q, D, GATE, NOTIFIER, VPWR, VGND);
  output reg Q;
  input  D, GATE, NOTIFIER, VPWR, VGND;
  always @(*) begin
    if (VPWR !== 1'b1 || VGND !== 1'b0) Q = 1'bx;
    else if (GATE === 1'b1) Q = D;
  end
endmodule

module sky130_fd_sc_hd__udp_dlatch$PR (Q, D, GATE, RESET);
  output reg Q;
  input  D, GATE, RESET;
  always @(*) begin
    if (RESET === 1'b1) Q = 1'b0;   // async clear dominates
    else if (GATE === 1'b1) Q = D;
  end
endmodule

module sky130_fd_sc_hd__udp_dlatch$PR_pp$PG$N (Q, D, GATE, RESET, NOTIFIER, VPWR, VGND);
  output reg Q;
  input  D, GATE, RESET, NOTIFIER, VPWR, VGND;
  always @(*) begin
    if (VPWR !== 1'b1 || VGND !== 1'b0) Q = 1'bx;
    else if (RESET === 1'b1) Q = 1'b0;
    else if (GATE === 1'b1) Q = D;
  end
endmodule

module sky130_fd_sc_hd__udp_mux_2to1 (X, A0, A1, S);
  output X;
  input  A0, A1, S;
  // x-reduction per table: equal data inputs pass that value regardless of S
  assign X = (A0 === A1) ? A0 :
             (S === 1'b0) ? A0 :
             (S === 1'b1) ? A1 : 1'bx;
endmodule

module sky130_fd_sc_hd__udp_mux_2to1_N (Y, A0, A1, S);
  output Y;
  input  A0, A1, S;
  assign Y = ~((A0 === A1) ? A0 :
               (S === 1'b0) ? A0 :
               (S === 1'b1) ? A1 : 1'bx);
endmodule

module sky130_fd_sc_hd__udp_mux_4to2 (X, A0, A1, A2, A3, S0, S1);
  output X;
  input  A0, A1, A2, A3, S0, S1;
  assign X = (A0 === A1 && A1 === A2 && A2 === A3) ? A0 :
             (S1 === 1'b0 && S0 === 1'b0) ? A0 :
             (S1 === 1'b0 && S0 === 1'b1) ? A1 :
             (S1 === 1'b1 && S0 === 1'b0) ? A2 :
             (S1 === 1'b1 && S0 === 1'b1) ? A3 : 1'bx;
endmodule
