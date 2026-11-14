// SPDX-FileCopyrightText: © 2026 Thomas McDonagh
// SPDX-License-Identifier: Apache-2.0
//
// Gate-level audio parity runner (v2.2 task 7): same harness as tb_audio.v
// — streams audio_in.hex through the real I2S input pins, one output
// sample per frame via $display — but compiled against the real
// gate_level_netlist.v instead of the RTL. Used by wav_tool.py (SIM=gl)
// and test/gl_wav_tool.md: every mode's output must be BIT-EXACT vs the
// RTL .wav in audio_out/ (netlist parity gate, plan Global Constraints).
//
// The netlist top keeps the RTL module name and ports plus the power
// pins, exactly like the cocotb GL path (tb.v + Makefile GATES=yes): the
// compile needs the same define set (-DGL_TEST -DFUNCTIONAL
// -DUSE_POWER_PINS -DSIM -DUNIT_DELAY=#1) and the same sky130 cell
// library sources in the same order (primitives.v, sky130_fd_sc_hd.v,
// gate_level_netlist.v). effect_sample survives synthesis as a public
// top-level wire name, so the dut.effect_sample probe compiles against
// the netlist (flat, no hierarchy below the top).
//
// Compile (from test/, with PDK_ROOT pointing at the sky130A PDK and
// gate_level_netlist.v copied in from the GDS run artifacts — printed by
// `python3 wav_tool.py gl-cmd`, built by `SIM=gl python3 wav_tool.py run`):
//   $ iverilog -g2012 -s tb_audio_gl -DGL_TEST -DFUNCTIONAL -DUSE_POWER_PINS \
//       -DSIM -DUNIT_DELAY=#1 -o sim_audio_gl.vvp tb_audio_gl.v \
//       $PDK_ROOT/sky130A/libs.ref/sky130_fd_sc_hd/verilog/primitives.v \
//       $PDK_ROOT/sky130A/libs.ref/sky130_fd_sc_hd/verilog/sky130_fd_sc_hd.v \
//       gate_level_netlist.v
// Run (per mode, same plusargs as the RTL build):
//   vvp sim_audio_gl.vvp +EN=2 +PARAMS=1 +N=2818047 +OUT=audio_out_gl/distX.hex
// Input:  audio_in.hex (one 16-bit sample per line)
// Output: one 16-bit sample per frame on stdout (captured per-mode by the
//         .log file in audio_out_gl/), converted to .wav by wav_tool.py
//
// +EN/+PARAMS/+N interface and ui/uio byte layout: identical to tb_audio.v
// (ui[7:4]=enables en[0]=fuzz..en[3]=trem, ui[3:1]=reserved driven 0,
// ui[0]=ADC data; P3..P0 on uio[5:2]).

`timescale 1ps/1ps

module tb_audio_gl;

  localparam integer CLK_HALF = 20267; // ps; ~24.576 MHz

  reg clk = 1'b0;
  reg rst_n = 1'b0;
  reg ena = 1'b1;
  reg [7:0] ui_in = 8'h00;
  reg [7:0] uio_in = 8'h00;

  // ~24.576 MHz clock
  always #(CLK_HALF) clk = ~clk;

  // Power pins for the gate-level netlist (tied, as tb.v's GL path does)
`ifdef GL_TEST
  wire VPWR = 1'b1;
  wire VGND = 1'b0;
`endif

  tt_um_tmcdonagh_guitar_pedal dut (

      // Include power ports for the gate-level test:
`ifdef GL_TEST
      .VPWR(VPWR),
      .VGND(VGND),
`endif

      .ui_in  (ui_in),
      .uo_out (),
      .uio_in (uio_in),
      .uio_out(),
      .uio_oe (),
      .ena    (ena),
      .clk    (clk),
      .rst_n  (rst_n)
  );

  reg [15:0] mem [0:3200000];
  integer n_samples = 0;
  integer en_dip = 0, params = 0, n_in = 0;
  integer i, slot;
  reg [15:0] word;
  integer dummy;

  initial begin
    dummy = $value$plusargs("EN=%d", en_dip);
    dummy = $value$plusargs("PARAMS=%d", params);
    dummy = $value$plusargs("N=%d", n_in);
    ui_in[7:1] = {en_dip[3:0], 3'b000};       // ui[7:4]=enables, ui[3:1]=reserved (driven 0)
    uio_in     = {2'b00, params[3:0], 2'b00}; // P3..P0 on uio[5:2]

    // samples go to stdout (captured per-mode by the .log file) — $fopen
    // with a string filename is not portable across Verilator/Icarus

    $readmemh("audio_in.hex", mem);
    n_samples = n_in;
    $display("loaded %0d samples, EN=%0d PARAMS=%0d", n_samples, en_dip, params);

    // reset
    rst_n = 1'b0;
    repeat (10) @(posedge clk);
    #1;
    rst_n = 1'b1;

    for (i = 0; i < n_samples; i = i + 1) begin
      word = mem[i];
      // drive one frame's left half: 32 slots x 8 clk, I2S data on ui[0]
      for (slot = 0; slot < 32; slot = slot + 1) begin
        ui_in[0] = (slot >= 1 && slot <= 16) ? word[16 - slot] : 1'b0;
        repeat (8) @(posedge clk);
      end
      // right half: RX already latched; let registered effects settle
      repeat (256) @(posedge clk);
      $display("%04h", dut.effect_sample);
      // frame complete: 256 (left) + 256 (right) = 512 clk
    end

    $display("done: %0d frames", n_samples);
    $finish;
  end

endmodule
