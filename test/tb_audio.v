// SPDX-FileCopyrightText: © 2026 Thomas McDonagh
// SPDX-License-Identifier: Apache-2.0
//
// Audio listening runner (Verilog-native, no cocotb): streams a hex file of
// 16-bit samples through the real I2S input pins and writes one output
// sample per frame. Runs entirely inside Icarus — ~30 s per mode.
//
// v3 task 7 byte layout: this tb drives the effect enables on ui[6:1]
// (+EN=, 0..63: bit order = chain order, en[0]=comp..en[5]=delay) and the
// fuzz octave switch on ui[7] (+OCT=, 0/1). The switches are debounced in
// the DUT (they reach the chain at frame 511; golden_model.EN_SETTLE).
// uio_in is 0: the knob pins uio[4:2] read low (absent) and uio[5] (SPI
// MISO) idles. ui[0] is the per-slot ADC data bit, assigned below.
// Each output sample is printed as a prefixed line "S:xxxx" on stdout;
// wav_tool.py parses only those lines.
//
// Compile (icarus; ../src/*.v picks up the top module and its sources):
//   $ iverilog -g2012 -o sim_audio.vvp tb_audio.v ../src/*.v
// Listening-sim build (Verilator; the binary wav_tool.py runs). The
// -Wno waivers cover WIDTH warnings in the trem arithmetic;
// --top-module pins the tb as the sim top:
//   $ verilator --binary --timing -j 16 -o sim_audio_vl --top-module tb_audio \
//       -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC tb_audio.v ../src/*.v
// Run (per mode, in parallel):
//   vvp sim_audio.vvp +EN=2 +OCT=1 +N=2818047 +OUT=audio_out/fuzzoct.hex
// Input:  audio_in.hex, or +IN=<file> (one 16-bit sample per line, from audio_in.wav)
// Output: one "S:xxxx" line per frame on stdout

`timescale 1ps/1ps

module tb_audio;

  localparam integer CLK_HALF = 20267; // ps; ~24.576 MHz

  reg clk = 1'b0;
  reg rst_n = 1'b0;
  reg ena = 1'b1;
  reg [7:0] ui_in = 8'h00;
  reg [7:0] uio_in = 8'h00;

  // ~24.576 MHz clock
  always #(CLK_HALF) clk = ~clk;

  tt_um_tmcdonagh_guitar_pedal dut (
      .ui_in  (ui_in),
      .uo_out (),
      .uio_in (uio_in),
      .uio_out (),
      .uio_oe (),
      .ena    (ena),
      .clk    (clk),
      .rst_n  (rst_n)
  );

  reg [15:0] mem [0:3200000];
  integer n_samples = 0;
  integer en_dip = 0, oct_sw = 0, n_in = 0;
  integer i, slot;
  reg [15:0] word;
  integer dummy;
  reg [8*256-1:0] in_file = "audio_in.hex";

  initial begin
    dummy = $value$plusargs("EN=%d", en_dip);
    dummy = $value$plusargs("OCT=%d", oct_sw);
    dummy = $value$plusargs("N=%d", n_in);
    dummy = $value$plusargs("IN=%s", in_file);
    ui_in[7:1] = {oct_sw[0], en_dip[5:0]};      // ui[6:1]=enables en[5:0], ui[7]=fuzz octave
    uio_in     = 8'h00;                         // knob pins uio[4:2] low (absent), uio[5] MISO idle

    // samples go to stdout (captured per-mode by the .log file) — $fopen
    // with a string filename is not portable across Verilator/Icarus

    $readmemh(in_file, mem);
    n_samples = n_in;
    $display("loaded %0d samples, EN=%0d OCT=%0d", n_samples, en_dip, oct_sw);

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
      $display("S:%04h", dut.effect_sample);
      // frame complete: 256 (left) + 256 (right) = 512 clk
    end

    $display("done: %0d frames", n_samples);
    $finish;
  end

endmodule
