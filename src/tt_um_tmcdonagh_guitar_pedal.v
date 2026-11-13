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
  wire [5:0] qshift = (ui_in[5:4] == 2'b00) ? 6'd0
                    : ({3'd0, ui_in[5:4], 1'b0} + 6'd4);
  wire [15:0] crushed = sample_in & (16'hFFFF << qshift);

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
  // Ring modulator (mode 3): multiply by a square-LFO sign (+/-1).
  // LFO half-period = 2^(10-k) samples, k = ui[6:4].
  // ------------------------------------------------------------------
  wire [2:0] ring_k   = ui_in[6:4];
  wire [3:0] ring_exp = 4'd10 - {1'b0, ring_k};
  wire [10:0] ring_half = 11'd1 << ring_exp;

  reg [10:0] ring_cnt;
  reg        ring_sign;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ring_cnt  <= 11'd0;
      ring_sign <= 1'b0;
    end else if (sample_strobe) begin
      if (ring_cnt == ring_half - 11'd1) begin
        ring_cnt  <= 11'd0;
        ring_sign <= ~ring_sign;
      end else begin
        ring_cnt <= ring_cnt + 11'd1;
      end
    end
  end

  wire [15:0] ring_neg = ~sample_in + 16'd1; // two's complement negate

  // ------------------------------------------------------------------
  // Distortion (mode 2): out = sat16(in * G), G = 1 + ui[5:4] (1..5),
  // combinational shift-add multiply
  // ------------------------------------------------------------------
  wire [2:0] dist_g = {1'b0, ui_in[5:4]} + 3'd1; // 1..5
  wire signed [19:0] dist_prod = $signed(sample_in) * $signed({1'b0, dist_g});
  wire [15:0] dist_sat = (dist_prod > 20'sd32767)  ? 16'h7FFF
                       : (dist_prod < -20'sd32768) ? 16'h8000
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
  wire signed [19:0] chain_prod = $signed(chain_rect) * 20'sd3;
  wire [15:0] chain_sat = (chain_prod > 20'sd32767) ? 16'h7FFF : chain_prod[15:0];

  // ------------------------------------------------------------------
  // Tremolo (mode 4): out = (in * f) >> 8 via serial multiplier.
  // f = triangle between min and 255; rate k = ui[7:5] (8 rates, ramp
  // = 2^(10-k) frames per direction); depth ui[4]: 0 -> min 0 (full),
  // 1 -> min 128 (subtle). (Plan's ui[6:4]+ui[5:4] layout overlapped.)
  // ------------------------------------------------------------------
  wire [2:0] trem_k   = ui_in[7:5];
  wire [3:0] trem_exp = 4'd10 - {1'b0, trem_k};

  wire [7:0] trem_min = ui_in[4] ? 8'd128 : 8'd0;
  wire [7:0] trem_span = 8'd255 - trem_min;

  reg  [10:0] trem_cnt;
  reg         trem_dir;   // 0 = falling, 1 = rising
  reg  [15:0] trem_sample;
  reg         trem_start;
  reg  [15:0] trem_result;

  wire [18:0] trem_ramp = {8'd0, trem_cnt} * {1'b0, trem_span};
  wire [7:0]  trem_step = trem_ramp[18:0] >> trem_exp;
  wire [7:0]  trem_f = trem_dir ? (trem_min + trem_step)
                                : (8'd255 - trem_step);

  wire signed [15:0] trem_p;
  wire               trem_done;

  serial_mult u_trem_mult (
      .clk   (clk),
      .rst_n (rst_n),
      .a     (trem_sample),
      .b     (trem_f),
      .start (trem_start),
      .p     (trem_p),
      .done  (trem_done)
  );

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      trem_cnt    <= 11'd0;
      trem_dir    <= 1'b0;
      trem_sample <= 16'd0;
      trem_start  <= 1'b0;
      trem_result <= 16'd0;
    end else begin
      trem_start <= 1'b0;
      if (sample_strobe) begin
        trem_sample <= rx_shift;  // same edge as sample_in; reading sample_in
                                  // here would fetch the PREVIOUS frame's value
        trem_start  <= 1'b1;
        if (trem_cnt == ((11'd1 << trem_exp) - 11'd1)) begin
          trem_cnt <= 11'd0;
          trem_dir <= ~trem_dir;
        end else begin
          trem_cnt <= trem_cnt + 11'd1;
        end
      end
      if (trem_done) trem_result <= trem_p;
    end
  end

  // ------------------------------------------------------------------
  // Test tone (mode 7): internal square wave, half-period = 55 << k
  // frames (k = ui[6:4]); levels +/-16383; ADC input ignored.
  // ------------------------------------------------------------------
  wire [2:0] tone_k = ui_in[6:4];
  wire [8:0] tone_half = 9'd55 << tone_k; // 55..7040 frames

  reg [8:0] tone_cnt;
  reg       tone_sign;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tone_cnt  <= 9'd0;
      tone_sign <= 1'b0;
    end else if (sample_strobe) begin
      if (tone_cnt == tone_half - 9'd1) begin
        tone_cnt  <= 9'd0;
        tone_sign <= ~tone_sign;
      end else begin
        tone_cnt <= tone_cnt + 9'd1;
      end
    end
  end
  wire [15:0] tone_out = tone_sign ? 16'd16383 : (-16'd16383);

  // ------------------------------------------------------------------
  // Effect mux (mode on ui[3:1]); effects filled in by later tasks.
  // ------------------------------------------------------------------
  wire [2:0] mode = ui_in[3:1];

  reg [15:0] effect_sample;
  always @* begin
    case (mode)
      3'd1:    effect_sample = fuzz_out;
      3'd2:    effect_sample = dist_sat;
      3'd3:    effect_sample = ring_sign ? ring_neg : sample_in;
      3'd4:    effect_sample = trem_result;
      3'd6:    effect_sample = chain_sat;
      3'd7:    effect_sample = tone_out;
      3'd5:    effect_sample = crush_hold;
      default: effect_sample = sample_in; // mode 0 bypass (+ stubs)
    endcase
  end

  // ------------------------------------------------------------------
  // Output pins (uo[7]=LRCLK, uo[6]=BCLK, uo[5:1]=LED bar, uo[0]=DAC data)
  // ------------------------------------------------------------------
  // ------------------------------------------------------------------
  // LED level meter: bar = max(|sample| >> 11 (clamped 15), bar - 1),
  // updated once per sample on the sample_valid pulse
  // ------------------------------------------------------------------
  wire [15:0] mag   = sample_in[15] ? (~sample_in + 16'd1) : sample_in;
  wire [4:0]  level = (mag[15:11] > 5'd15) ? 5'd15 : mag[15:11];
  wire [4:0]  decayed = (led_bar == 5'd0) ? 5'd0 : led_bar - 5'd1;

  reg [4:0] led_bar;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)          led_bar <= 5'd0;
    else if (sample_valid) led_bar <= (level > decayed) ? level : decayed;
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
