// SPDX-FileCopyrightText: © 2026 Thomas McDonagh
// SPDX-License-Identifier: Apache-2.0
//
// Audio listening runner (Verilog-native, no cocotb): streams a hex file of
// 16-bit samples through the real I2S input pins and writes one output
// sample per frame. Runs entirely inside Icarus — ~30 s per mode.
//
// v2.2 task 3 byte layout: this tb drives the DIP effect enables on
// ui[7:4] (+EN=, 0..15: bit order = chain order, en[0]=fuzz..en[3]=trem)
// and the params on uio[5:2] (+PARAMS=, P3..P0: P2=trem rate, P3=trem
// depth, P0=dist knee, P1=crush depth base — all live as of task 5).
// ui[3:1] are reserved (v3) and stay 0 — en comes from the DIP byte
// alone; ui[0] is still the per-slot ADC data bit, assigned separately
// below.
//
// Compile (icarus; ../src/*.v picks up the top module and its sources):
//   $ iverilog -g2012 -o sim_audio.vvp tb_audio.v ../src/*.v
// Listening-sim build (Verilator; the binary wav_tool.py runs). The
// -Wno waivers cover WIDTH warnings in the trem arithmetic and the
// template placeholder src/project.v (the task-4 dist rewrite is
// warning-clean); --top-module pins the tb as the sim top:
//   $ verilator --binary --timing -j 16 -o sim_audio_vl --top-module tb_audio \
//       -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-MULTITOP tb_audio.v ../src/*.v
// Run (per mode, in parallel):
//   vvp sim_audio.vvp +EN=2 +PARAMS=1 +N=2818047 +OUT=audio_out/distX.hex
// Input:  audio_in.hex (one 16-bit sample per line, from audio_in.wav)
// Output: $OUT, one 16-bit sample per frame

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
