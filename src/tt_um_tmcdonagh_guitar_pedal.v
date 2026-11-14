/*
 * Copyright (c) 2026 Tom McDonagh
 * SPDX-License-Identifier: Apache-2.0
 *
 * Silicon Guitar Pedal — multi-effect DSP core.
 * I2S master in/out (codec-agnostic), 5 effects + bypass (ring mod and
 * test tone retired in v2.2), LED level meter.
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
  wire       sample_strobe = sample_latch && (lr_cnt[2:0] == 3'd0); // 1-clk pulse
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sample_in    <= 16'd0;
      sample_valid <= 1'b0;
    end else begin
      sample_valid <= sample_strobe;
      if (sample_strobe) sample_in <= rx_shift;
    end
  end

  // ------------------------------------------------------------------
  // Bitcrusher (mode 5): quantize to N bits (mask low bits) + sample-hold
  // decimation. N: ui[5:4] 00->16, 01->10, 10->8, 11->6 (shift 0/6/8/10).
  // Decimation: ui[7:6] = 0..3 -> divide by 1,2,4,8.
  // ------------------------------------------------------------------
  wire [15:0] crushed = sample_in & ((ui_in[5:4] == 2'b00) ? 16'hFFFF
                                   : (ui_in[5:4] == 2'b01) ? 16'hFFC0
                                   : (ui_in[5:4] == 2'b10) ? 16'hFF00
                                   : 16'hFC00);

  wire [1:0] decim   = ui_in[7:6];
  wire [2:0] dec_mask = (3'b001 << decim) - 3'd1; // /2,/4,/8 -> 1,3,7
  reg  [2:0] decim_cnt;
  reg [15:0] crush_hold;
  wire crush_latch = sample_valid &&
                     ((decim == 2'd0) || ((decim_cnt & dec_mask) == 3'd0));

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      decim_cnt  <= 3'd0;
      crush_hold <= 16'd0;
    end else begin
      if (sample_valid) decim_cnt <= decim_cnt + 3'd1;
      if (crush_latch)  crush_hold <= crushed;
    end
  end

  // ------------------------------------------------------------------
  // Distortion (mode 2): out = sat16(in * G), G = 1 + ui[5:4] (1..4),
  // shift-add (G is one of 1,2,3,4 — no variable multiplier needed)
  // ------------------------------------------------------------------
  wire signed [16:0] dist_in = $signed(sample_in);
  reg signed [17:0] dist_prod;
  always @* begin
    case (ui_in[5:4])
      2'd1:    dist_prod = dist_in <<< 1;               // x2
      2'd2:    dist_prod = dist_in + (dist_in <<< 1);   // x3
      2'd3:    dist_prod = dist_in <<< 2;               // x4
      default: dist_prod = dist_in;                     // x1
    endcase
  end
  // overflow iff the result's top bits stop looking sign-extended
  wire dist_ovf = (dist_prod[17] != dist_prod[16]) || (dist_prod[16] != dist_prod[15]);
  wire [15:0] dist_sat = dist_ovf ? (dist_prod[17] ? 16'h8000 : 16'h7FFF)
                                  : dist_prod[15:0];

  // ------------------------------------------------------------------
  // Octave fuzz (mode 1): full-wave rectifier + parametric clip.
  // T: ui[5:4] 00->24576, 01->16384, 10->8192, 11->4096.
  // Fuzz->dist chain (mode 6): clip at fixed 16384, then sat x3.
  // ------------------------------------------------------------------
  wire [15:0] fuzz_t = (ui_in[5:4] == 2'b00) ? 16'd24576
                     : (ui_in[5:4] == 2'b01) ? 16'd16384
                     : (ui_in[5:4] == 2'b10) ? 16'd8192
                     : 16'd4096;
  wire [15:0] fuzz_out = (mag > fuzz_t) ? fuzz_t : mag;

  wire [15:0] chain_rect = (mag > 16'd16384) ? 16'd16384 : mag;
  wire signed [17:0] chain_prod = ($signed(chain_rect) <<< 1) + $signed(chain_rect); // x3
  // chain_rect <= 16384 so x3 fits 17 bits; overflow iff top bits disagree
  wire chain_ovf = chain_prod[16] != chain_prod[15];
  wire [15:0] chain_sat = chain_ovf ? 16'h7FFF : chain_prod[15:0];

  // ------------------------------------------------------------------
  // Tremolo (mode 4): stepped-gain tremolo: out = in >>> s, gain steps
  // 1, 1/2, 1/4, 1/8 driven by the triangle LFO (gap = distance from
  // full gain). Rate k = ui[7:5] (ramp = 2^(10-k) frames per direction);
  // depth ui[4]: 0 -> span 255 (4 steps), 1 -> span 127 (2 steps).
  // (v1: step-gain replaces the exact multiply to fit the tile; smooth
  // multiply is the documented refinement.)
  // ------------------------------------------------------------------
  wire [2:0] trem_k   = ui_in[7:5];
  wire [3:0] trem_exp = 4'd10 - {1'b0, trem_k};

  wire [7:0] trem_span = ui_in[4] ? 8'd127 : 8'd255;
  wire [3:0] trem_rshift = ui_in[4] ? 4'd7 : 4'd8;  // span = 2^rshift - 1

  reg  [10:0] trem_cnt;
  reg         trem_dir;   // 0 = falling (gain down), 1 = rising
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      trem_cnt <= 11'd0;
      trem_dir <= 1'b0;
    end else if (sample_strobe) begin
      if (trem_cnt == ((11'd1 << trem_exp) - 11'd1)) begin
        trem_cnt <= 11'd0;
        trem_dir <= ~trem_dir;
      end else begin
        trem_cnt <= trem_cnt + 11'd1;
      end
    end
  end

  wire [18:0] trem_ramp = ({8'd0, trem_cnt} << trem_rshift) - {11'd0, trem_cnt};
  wire [7:0]  trem_gap = trem_dir ? (trem_span - (trem_ramp >> trem_exp))
                                  : (trem_ramp >> trem_exp);
  wire [15:0] trem_out = $signed(sample_in) >>> (trem_gap >> 6);

  // ------------------------------------------------------------------
  // Effect mux (mode on ui[3:1]); ring mod (3) and test tone (7) were
  // retired in v2.2 and fall to the bypass default.
  // ------------------------------------------------------------------
  wire [2:0] mode = ui_in[3:1];

  reg [15:0] effect_sample;
  always @* begin
    case (mode)
      3'd1:    effect_sample = fuzz_out;
      3'd2:    effect_sample = dist_sat;
      3'd4:    effect_sample = trem_out;
      3'd6:    effect_sample = chain_sat;
      3'd5:    effect_sample = crush_hold;
      default: effect_sample = sample_in; // mode 0 bypass (+ retired 3/7)
    endcase
  end

  // ------------------------------------------------------------------
  // Output pins (v2.2 task 1): BCLK/LRCLK/DAC data moved from uo to uio
  // to free uo for the 7-seg display (later task). uio[0]=MCLK,
  // uio[1]=DAC data, uio[6]=BCLK, uio[7]=LRCLK; the LED bar keeps its
  // v1 pin uo[5:1] until the display lands.
  // ------------------------------------------------------------------
  // ------------------------------------------------------------------
  // LED level meter: bar = max(|sample| >> 11 (clamped 15), bar - 1),
  // updated once per sample on the sample_valid pulse
  // ------------------------------------------------------------------
  wire [15:0] mag   = (sample_in == 16'h8000) ? 16'h7FFF  // saturating |x|
                      : sample_in[15] ? (~sample_in + 16'd1) : sample_in;
  wire [4:0]  level = mag[15:10];  // 0..31, monotonic; uses all 5 LEDs
  wire [4:0]  decayed = (led_bar == 5'd0) ? 5'd0 : led_bar - 5'd1;

  reg [4:0] led_bar;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)          led_bar <= 5'd0;
    else if (sample_valid) led_bar <= (level > decayed) ? level : decayed;
  end

  // ------------------------------------------------------------------
  // I2S transmit: uio_out[1], MSB first. Loaded at slot 0 and slot 32
  // (start of each half), shifted out one bit per slot at the slot's
  // last clk; dac_data therefore only changes on BCLK falling edges.
  // ------------------------------------------------------------------
  reg [15:0] tx_shift;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) tx_shift <= 16'd0;
    // load once per half (first clk only): keeps L=R=previous sample
    // (mono contract) and immune to param bounce late in the slot
    else if ((slot == 6'd0 || slot == 6'd32) && lr_cnt[2:0] == 3'd0)
      tx_shift <= effect_sample;
    else if (data_window && lr_cnt[2:0] == 3'd7)
      tx_shift <= {tx_shift[14:0], 1'b0};
  end

  wire dac_data = (slot == 6'd0 || slot == 6'd32) ? 1'b0
                : (data_window ? tx_shift[15] : 1'b0);

  assign uo_out  = {2'b0, led_bar, 1'b0}; // uo[5:1] = LED bar (interim)

  assign uio_out = {lrclk, bclk, 4'b0, dac_data, clk}; // [7]=LRCLK, [6]=BCLK, [1]=DAC data, [0]=MCLK = clk (512 fs)
  assign uio_oe  = 8'b1100_0011; // uio[7:6] + uio[1:0] outputs (LRCLK/BCLK + MCLK/DAC data); uio[5:2] inputs (params, later task)

  // List all unused inputs to prevent warnings
  wire _unused = &{ena, uio_in, 1'b0};

endmodule

`default_nettype wire
