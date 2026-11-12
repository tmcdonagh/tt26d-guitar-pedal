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

  wire [5:0] slot      = lr_cnt[8:3]; // 0..63, 0..31 = left (WS low) half
  wire [4:0] half_slot = lr_cnt[7:3]; // 0..31 within each half

  // Data window: slots 1..16 of each half (I2S one-slot delay, 16 MSBs)
  wire data_window = (half_slot >= 5'd1) && (half_slot <= 5'd16);

  // Left-half end: receive complete / processing starts
  wire sample_latch = (slot == 6'd32);

  // ------------------------------------------------------------------
  // I2S receive: ui_in[0] sampled once per slot on BCLK rising edges
  // (mid-slot, lr_cnt[2:0]==4), slots 1..16 of the left half only
  // ------------------------------------------------------------------
  reg [15:0] rx_shift;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) rx_shift <= 16'd0;
    else if (!lr_cnt[8] && data_window && lr_cnt[2:0] == 3'd4)
      rx_shift <= {rx_shift[14:0], ui_in[0]};
  end

  reg [15:0] sample_in;
  reg        sample_valid;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sample_in    <= 16'd0;
      sample_valid <= 1'b0;
    end else begin
      sample_valid <= sample_latch;
      if (sample_latch) sample_in <= rx_shift;
    end
  end

  // ------------------------------------------------------------------
  // Effect mux (mode on ui[3:1]); effects filled in by later tasks.
  // sample_in is wired to 0 until the I2S receive path lands.
  // ------------------------------------------------------------------
  wire [2:0]  mode = ui_in[3:1];

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

  // ------------------------------------------------------------------
  // I2S transmit: uo_out[0], MSB first. Loaded at slot 0 and slot 32
  // (start of each half), shifted out one bit per slot at the slot's
  // last clk; dac_data therefore only changes on BCLK falling edges.
  // ------------------------------------------------------------------
  reg [15:0] tx_shift;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) tx_shift <= 16'd0;
    else if (slot == 6'd0 || slot == 6'd32) tx_shift <= effect_sample;
    else if (data_window && lr_cnt[2:0] == 3'd7)
      tx_shift <= {tx_shift[14:0], 1'b0};
  end

  wire dac_data = (slot == 6'd0 || slot == 6'd32) ? 1'b0
                : (data_window ? tx_shift[15] : 1'b0);

  assign uo_out = {lrclk, bclk, led_bar, dac_data};

  assign uio_out = {7'b0, clk}; // uio[0] = MCLK = clk (512 fs)
  assign uio_oe  = 8'b0000_0001;

  // List all unused inputs to prevent warnings
  wire _unused = &{ena, uio_in, ui_in[7:4], 1'b0};

endmodule

`default_nettype wire
