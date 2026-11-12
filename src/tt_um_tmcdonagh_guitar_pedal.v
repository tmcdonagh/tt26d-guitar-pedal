/*
 * Copyright (c) 2026 Tom McDonagh
 * SPDX-License-Identifier: Apache-2.0
 *
 * Silicon Guitar Pedal — multi-effect DSP core.
 * I2S master in/out (codec-agnostic), 8 runtime modes, LED level meter.
 * See docs/info.md and the spec in the ai-dev-archive.
 */

`default_nettype none

module tt_um_tmcdonagh_guitar_pedal (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock (24.576 MHz = 512 x 48 kHz)
    input  wire       rst_n     // reset_n - low to reset
);

  // ------------------------------------------------------------------
  // Frame timing scaffold: 512 clk per I2S frame, BCLK = clk/8 (64 fs),
  // LRCLK = clk/512 (48 kHz). I2S shift logic arrives in later tasks.
  // ------------------------------------------------------------------
  reg [8:0] lr_cnt;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) lr_cnt <= 9'd0;
    else        lr_cnt <= lr_cnt + 9'd1;
  end

  wire bclk  = lr_cnt[2];   // period 8 clk = 3.072 MHz
  wire lrclk = lr_cnt[8];   // period 512 clk = 48 kHz

  // ------------------------------------------------------------------
  // Effect mux (mode on ui[3:1]); effects filled in by later tasks.
  // sample_in is wired to 0 until the I2S receive path lands.
  // ------------------------------------------------------------------
  wire [2:0]  mode      = ui_in[3:1];
  wire [15:0] sample_in = 16'd0;

  reg [15:0] effect_sample;
  always @* begin
    case (mode)
      default: effect_sample = sample_in; // mode 0 bypass (+ stubs)
    endcase
  end

  // ------------------------------------------------------------------
  // Output pins (uo[7]=LRCLK, uo[6]=BCLK, uo[5:1]=LED bar, uo[0]=DAC data)
  // ------------------------------------------------------------------
  reg [4:0] led_bar;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) led_bar <= 5'd0;
    else        led_bar <= 5'd0; // meter arrives in a later task
  end

  reg dac_data;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) dac_data <= 1'b0;
    else        dac_data <= 1'b0; // tx shift-out arrives in a later task
  end

  assign uo_out = {lrclk, bclk, led_bar, dac_data};

  assign uio_out = {7'b0, clk}; // uio[0] = MCLK = clk (512 fs)
  assign uio_oe  = 8'b0000_0001;

  // List all unused inputs to prevent warnings
  wire _unused = &{ena, uio_in, ui_in[7:4], ui_in[0], 1'b0};

endmodule

`default_nettype wire
