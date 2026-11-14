// SPDX-FileCopyrightText: © 2026 Tom McDonagh
// SPDX-License-Identifier: Apache-2.0
//
// Audio listening runner (Verilog-native, no cocotb): streams a hex file of
// 16-bit samples through the real I2S input pins and writes one output
// sample per frame. Runs entirely inside Icarus — ~30 s per mode.
//
// v2.2 pin map (task 1): BCLK/LRCLK/DAC data moved to uio[6]/uio[7]/uio[1]
// (MCLK stays uio[0]); ADC data stays on ui[0]. This tb drives ui[0] and
// taps dut.effect_sample directly, so its wiring is unchanged. Modes 3
// (ring mod) and 7 (test tone) are retired in v2.2 and now bypass.
//
// Compile:
//   iverilog -g2012 -o sim_audio.vvp tb_audio.v ../src/tt_um_tmcdonagh_guitar_pedal.v
// Run (per mode, in parallel):
//   vvp sim_audio.vvp +MODE=2 +PARAMS=2 +OUT=audio_out/dist2.hex
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
  integer mode = 0, params = 0, n_in = 0;
  integer i, slot;
  reg [15:0] word;
  integer dummy;

  initial begin
    dummy = $value$plusargs("MODE=%d", mode);
    dummy = $value$plusargs("PARAMS=%d", params);
    dummy = $value$plusargs("N=%d", n_in);
    ui_in = {params[3:0], mode[2:0], 1'b0}; // ui[7:4]=params, ui[3:1]=mode

    // samples go to stdout (captured per-mode by the .log file) — $fopen
    // with a string filename is not portable across Verilator/Icarus

    $readmemh("audio_in.hex", mem);
    n_samples = n_in;
    $display("loaded %0d samples, MODE=%0d PARAMS=%0d", n_samples, mode, params);

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
