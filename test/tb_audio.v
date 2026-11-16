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
// The knob pins uio[4:2] follow an RC pin model (+K1= +K2= +K3=, 0..255,
// -1 = absent = pin held low, the default) and uio[5] (SPI MISO) idles
// low. ui[0] is the per-slot ADC data bit, assigned below. A final
// "K1:value present moved" line reports the K1 settle probe.
// Each output sample is printed as a prefixed line "S:xxxx" on stdout;
// wav_tool.py parses only those lines.
//
// Compile (icarus; ../src/*.v picks up the top module and its sources):
//   $ iverilog -g2012 -o sim_audio.vvp tb_audio.v ../src/*.v
// Listening-sim build (Verilator; the binary wav_tool.py runs). The
// -Wno waivers cover WIDTH warnings in the tb arithmetic;
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
  wire [7:0] uio_in;
  wire [7:0] uio_oe;

  // ~24.576 MHz clock
  always #(CLK_HALF) clk = ~clk;

  tt_um_tmcdonagh_guitar_pedal dut (
      .ui_in  (ui_in),
      .uo_out (),
      .uio_in (uio_in),
      .uio_out (),
      .uio_oe (uio_oe),
      .ena    (ena),
      .clk    (clk),
      .rst_n  (rst_n)
  );

  // RC knob pin model (v3 task 9; same as test.py KnobPin): while the
  // reader drives its pin (uio_oe[2+k] = 1, discharge) the pin reads 0;
  // after the release it reads 1 exactly N = 64*v + 32 - SYNC_LAT clk
  // later (set at the falling clk edge closing clk N), so the reader's
  // charge count lands mid-bin at 64*v + 32 and raw == v. SYNC_LAT = 2
  // (the reader's 2-flop synchronizer). v = -1 (+Kn absent, the
  // default) holds the pin at 0 (knob absent). uio[5] (MISO) idles 0.
  localparam integer SYNC_LAT = 2;
  reg k1_pin = 1'b0, k2_pin = 1'b0, k3_pin = 1'b0;
  integer k1c = 0, k2c = 0, k3c = 0;
  assign uio_in = {3'b000, k3_pin, k2_pin, k1_pin, 2'b00};

  always @(negedge clk) begin
    if (k1v < 0 || uio_oe[2] !== 1'b0) begin k1_pin = 1'b0; k1c = 0; end
    else begin
      if (k1c == 64 * k1v + 32 - SYNC_LAT) k1_pin = 1'b1;
      k1c = k1c + 1;
    end
  end
  always @(negedge clk) begin
    if (k2v < 0 || uio_oe[3] !== 1'b0) begin k2_pin = 1'b0; k2c = 0; end
    else begin
      if (k2c == 64 * k2v + 32 - SYNC_LAT) k2_pin = 1'b1;
      k2c = k2c + 1;
    end
  end
  always @(negedge clk) begin
    if (k3v < 0 || uio_oe[4] !== 1'b0) begin k3_pin = 1'b0; k3c = 0; end
    else begin
      if (k3c == 64 * k3v + 32 - SYNC_LAT) k3_pin = 1'b1;
      k3c = k3c + 1;
    end
  end

  // Knob-settle probe (RTL only; read by wav_tool.py check): the golden
  // model applies knob_steady(k1) from frame 0, which is exact only if
  // the K1 reading (value, present) never moves from frame EN_SETTLE-1
  // (510) on. k1_moved latches any change after the reference is taken
  // at the start of frame 510; printed as "K1:value present moved".
  localparam integer EN_SETTLE = 511;
  reg [7:0] k1_ref = 8'd0;
  reg       k1_pref = 1'b0, k1_arm = 1'b0, k1_moved = 1'b0;
  always @(posedge clk)
    if (k1_arm && (dut.k1.value !== k1_ref || dut.k1.present !== k1_pref))
      k1_moved = 1'b1;

  reg [15:0] mem [0:3200000];
  integer n_samples = 0;
  integer en_dip = 0, oct_sw = 0, n_in = 0;
  integer k1v = -1, k2v = -1, k3v = -1;   // knob plusargs, -1 = absent
  integer i, slot;
  reg [15:0] word;
  integer dummy;
  reg [8*256-1:0] in_file = "audio_in.hex";

  initial begin
    dummy = $value$plusargs("EN=%d", en_dip);
    dummy = $value$plusargs("OCT=%d", oct_sw);
    dummy = $value$plusargs("N=%d", n_in);
    dummy = $value$plusargs("K1=%d", k1v);
    dummy = $value$plusargs("K2=%d", k2v);
    dummy = $value$plusargs("K3=%d", k3v);
    dummy = $value$plusargs("IN=%s", in_file);
    ui_in[7:1] = {oct_sw[0], en_dip[5:0]};      // ui[6:1]=enables en[5:0], ui[7]=fuzz octave

    // samples go to stdout (captured per-mode by the .log file) — $fopen
    // with a string filename is not portable across Verilator/Icarus

    $readmemh(in_file, mem);
    n_samples = n_in;
    $display("loaded %0d samples, EN=%0d OCT=%0d K1=%0d K2=%0d K3=%0d",
             n_samples, en_dip, oct_sw, k1v, k2v, k3v);

    // reset
    rst_n = 1'b0;
    repeat (10) @(posedge clk);
    #1;
    rst_n = 1'b1;

    for (i = 0; i < n_samples; i = i + 1) begin
      word = mem[i];
      if (i == EN_SETTLE - 1) begin
        k1_ref  = dut.k1.value;
        k1_pref = dut.k1.present;
        k1_arm  = 1'b1;
      end
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

    $display("K1:%0d %0d %0d", k1_ref, k1_pref, k1_moved);
    $display("done: %0d frames", n_samples);
    $finish;
  end

endmodule
