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
// gate_level_netlist.v). Capture is PORT-BASED: synthesis flattens the
// design and the RTL's internal `effect_sample` alias does not survive
// (first GL compile proved it — opt_clean collapsed it), so the output
// sample is reassembled from the DAC data pin `uio_out[1]` (slots 1..16,
// MSB first, sampled mid-slot — the same instant the codec would latch).
// The DAC emits each frame's result one frame late (TX reloads from the
// previous frame's effect at slot 0/32), so the loop prints the capture
// from frame i as log line i−1 and runs one extra flush frame; that
// matches the RTL log alignment (line j = effect of input j) exactly.
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
//   vvp sim_audio_gl.vvp +EN=2 +OCT=1 +N=2818048 +OUT=audio_out_gl/fuzzoct.hex
// Input:  audio_in.hex (one 16-bit sample per line)
// Output: one "S:xxxx" line per frame on stdout (captured per-mode by the
//         .log file in audio_out_gl/), converted to .wav by wav_tool.py
//
// +EN/+OCT/+N/+K1/+K2/+K3 interface and ui/uio byte layout: identical to
// tb_audio.v (ui[6:1]=enables en[0]=comp..en[5]=delay, ui[7]=fuzz octave
// via +OCT=, ui[0]=ADC data; knob pins uio[4:2] follow the same RC pin
// model, -1 = absent = held low; uio[5] MISO idles low). The K1 settle
// probe is RTL-only (tb_audio.v).
// Samples are printed as prefixed "S:xxxx" lines, as in tb_audio.v.

`timescale 1ps/1ps

module tb_audio_gl;

  localparam integer CLK_HALF = 20267; // ps; ~24.576 MHz

  reg clk = 1'b0;
  reg rst_n = 1'b0;
  reg ena = 1'b1;
  reg [7:0] ui_in = 8'h00;
  wire [7:0] uio_in;
  wire [7:0] uio_oe;

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
      .uio_out(uio_out),
      .uio_oe (uio_oe),
      .ena    (ena),
      .clk    (clk),
      .rst_n  (rst_n)
  );

  wire [7:0] uio_out;

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

  reg [15:0] mem [0:3200000];
  integer n_samples = 0;
  integer en_dip = 0, oct_sw = 0, n_in = 0;
  integer k1v = -1, k2v = -1, k3v = -1;   // knob plusargs, -1 = absent
  integer i, slot;
  reg [15:0] word;
  reg [15:0] cap;   // DAC-side sample reassembled from uio_out[1]
  integer dummy;

  initial begin
    dummy = $value$plusargs("EN=%d", en_dip);
    dummy = $value$plusargs("OCT=%d", oct_sw);
    dummy = $value$plusargs("N=%d", n_in);
    dummy = $value$plusargs("K1=%d", k1v);
    dummy = $value$plusargs("K2=%d", k2v);
    dummy = $value$plusargs("K3=%d", k3v);
    ui_in[7:1] = {oct_sw[0], en_dip[5:0]};      // ui[6:1]=enables en[5:0], ui[7]=fuzz octave

    // samples go to stdout (captured per-mode by the .log file) — $fopen
    // with a string filename is not portable across Verilator/Icarus

    $readmemh("audio_in.hex", mem);
    n_samples = n_in;
    $display("loaded %0d samples, EN=%0d OCT=%0d K1=%0d K2=%0d K3=%0d",
             n_samples, en_dip, oct_sw, k1v, k2v, k3v);

    // reset
    rst_n = 1'b0;
    repeat (10) @(posedge clk);
    #1;
    rst_n = 1'b1;

    // i runs one extra "flush" frame: frame i's left half emits the
    // effect of input i−1 on the DAC (one-frame lag); printing the
    // capture at the end of iteration i (for i ≥ 1) yields log line
    // i−1 = effect of input i−1 — identical alignment to the RTL log.
    for (i = 0; i < n_samples + 1; i = i + 1) begin
      word = (i < n_samples) ? mem[i] : 16'h0000;
      // drive one frame's left half: 32 slots x 8 clk, I2S data on ui[0];
      // reassemble the DAC sample from uio_out[1], slots 1..16, mid-slot
      for (slot = 0; slot < 32; slot = slot + 1) begin
        ui_in[0] = (slot >= 1 && slot <= 16) ? word[16 - slot] : 1'b0;
        repeat (4) @(posedge clk);
        if (slot >= 1 && slot <= 16) cap = {cap[14:0], uio_out[1]};
        repeat (4) @(posedge clk);
      end
      // right half: RX already latched; let registered effects settle
      repeat (256) @(posedge clk);
      if (i > 0) $display("S:%04h", cap);
      // frame complete: 256 (left) + 256 (right) = 512 clk
    end

    $display("done: %0d frames", n_samples);
    $finish;
  end

endmodule
