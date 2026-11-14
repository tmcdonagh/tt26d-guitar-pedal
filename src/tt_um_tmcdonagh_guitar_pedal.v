/*
 * Copyright (c) 2026 Thomas McDonagh
 * SPDX-License-Identifier: Apache-2.0
 *
 * Silicon Guitar Pedal — multi-effect DSP core.
 * I2S master in/out (codec-agnostic), serial effect chain fuzz ->
 * distortion -> bitcrusher -> tremolo with per-stage enable bypass
 * (v2.2: ring mod, test tone, and the v1 8-way mode mux retired).
 * Enables: preset ROM (EC11 encoder on ui[3:1]) OR DIP ui[7:4], all
 * forced off by the latching stomp bypass. Params P3..P0 on uio[5:2].
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
  // Control plane (v2.2 task 2): EC11 encoder (A=ui[1], B=ui[2]) and
  // stomp (ui[3]), sampled once per audio frame on sample_valid.
  // pedal_panel debounces the lines, decodes quadrature x2, counts
  // presets 0..7 (ROM enable table), and drives the 7-seg preset digit
  // — blank while the bypass latch is engaged.
  // ------------------------------------------------------------------
  wire [3:0] panel_en;
  wire       panel_bypass;
  wire [6:0] panel_seg;

  pedal_panel u_panel (
      .clk       (clk),
      .rst_n     (rst_n),
      .tick      (sample_valid),
      .panel_in  (ui_in[3:1]),
      .preset_en (panel_en),
      .bypass    (panel_bypass),
      .seg       (panel_seg)
  );

  // ------------------------------------------------------------------
  // Effect chain (v2.2 task 3): serial fuzz -> dist -> crush -> trem,
  // enable-bypass per stage. en bit order = chain order: en[0]=fuzz,
  // en[1]=dist, en[2]=crush, en[3]=trem — preset ROM pattern OR DIP
  // ui[7:4], forced all-off while the stomp bypass latch is engaged.
  // ------------------------------------------------------------------
  wire [3:0] en = panel_bypass ? 4'b0000 : (panel_en | ui_in[7:4]);

  wire [15:0] s0 = sample_in;

  // ------------------------------------------------------------------
  // Stage 1 — octave fuzz: full-wave rectifier + fixed clip at 16384
  // (the v1 chain value; v2.2 gives fuzz no param bit).
  // ------------------------------------------------------------------
  wire [15:0] mag = (sample_in == 16'h8000) ? 16'h7FFF  // saturating |x|
                    : sample_in[15] ? (~sample_in + 16'd1) : sample_in;

  wire [15:0] fuzz_t   = 16'd16384;
  wire [15:0] fuzz_out = (mag > fuzz_t) ? fuzz_t : mag;

  wire [15:0] s1 = en[0] ? fuzz_out : s0;

  // ------------------------------------------------------------------
  // Stage 2 — distortion (v2.2 task 4): soft-knee piecewise-linear
  // shaper on the stage input's magnitude — unity below the knee, slope
  // 1/2 above it (a soft ceiling in place of the interim hard clip).
  // Knee from the P0 character bit (uio[2]): 0 -> 8192 (soft ceiling
  // 20479), 1 -> 4096 (ceiling 18431); both below the 16384 fuzz clip,
  // so a stacked fuzz always feeds the shaped region. No multipliers —
  // subtract + shift only.
  // ------------------------------------------------------------------
  wire [15:0] dist_mag = (s1 == 16'h8000) ? 16'h7FFF   // saturating |s1|
                        : s1[15] ? (~s1 + 16'd1) : s1;

  wire [15:0] dist_t = uio_in[2] ? 16'd4096 : 16'd8192;

  // a <= T: out = a (slope 1);  a > T: out = T + ((a - T) >> 1)
  // (a - T >= 1 in the shaped branch, so the shift is exact)
  wire [15:0] dist_over = dist_mag - dist_t;
  wire [15:0] dist_pw = (dist_mag > dist_t) ? (dist_t + (dist_over >> 1))
                                            : dist_mag;

  // sign reapplied through a sat16 clamp (unreachable: out <= 20479,
  // kept per the saturating-arithmetic convention)
  wire [16:0] dist_neg = 17'd0 - {1'b0, dist_pw};
  wire dist_neg_ovf = (dist_neg[16] != dist_neg[15]);
  wire [15:0] dist_out = s1[15] ? (dist_neg_ovf ? 16'h8000 : dist_neg[15:0])
                                : dist_pw;

  wire [15:0] s2 = en[1] ? dist_out : s1;

  // ------------------------------------------------------------------
  // Stage 3 — bitcrusher (interim, task 3): quantize-only, N=8 fixed
  // (mask 0xFF00), latched per audio frame (the v1 sample-hold
  // decimator is deleted). Task 5 replaces the constant mask with the
  // envelope-driven auto-lofi depth (P1 on uio[3]).
  // ------------------------------------------------------------------
  wire [15:0] crushed = s2 & 16'hFF00;

  reg [15:0] crush_hold;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)            crush_hold <= 16'd0;
    else if (sample_valid) crush_hold <= crushed;
  end

  wire [15:0] s3 = en[2] ? crush_hold : s2;

  // ------------------------------------------------------------------
  // Stage 4 — tremolo: stepped-gain tremolo as built (v1): out =
  // in >>> s with gain steps 1, 1/2, 1/4, 1/8, s = LFO gap >> 6
  // (gap = distance from full gain), triangle LFO. v2.2 params: rate
  // P2 = uio[4] (k=0 -> ramp 2^10 frames per direction ~23.4 Hz,
  // k=1 -> 2^9 ~46.9 Hz); depth P3 = uio[5] (0 -> span 255 = 4 steps,
  // 1 -> span 127 = 2 steps).
  // ------------------------------------------------------------------
  wire [2:0] trem_k   = uio_in[4] ? 3'd1 : 3'd0;
  wire [3:0] trem_exp = 4'd10 - {1'b0, trem_k};

  wire [7:0] trem_span = uio_in[5] ? 8'd127 : 8'd255;
  wire [3:0] trem_rshift = uio_in[5] ? 4'd7 : 4'd8;  // span = 2^rshift - 1

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
  wire [15:0] trem_out = $signed(s3) >>> (trem_gap >> 6);

  wire [15:0] s4 = en[3] ? trem_out : s3;

  // ------------------------------------------------------------------
  // Chain output (name kept from v1: the audio tooling probes
  // dut.effect_sample).
  // ------------------------------------------------------------------
  reg [15:0] effect_sample;
  always @* effect_sample = s4;

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

  assign uo_out  = {1'b0, panel_seg}; // uo[6:0] = 7-seg a..g; uo[7] = dp clip LED (task 6)

  assign uio_out = {lrclk, bclk, 4'b0, dac_data, clk}; // [7]=LRCLK, [6]=BCLK, [1]=DAC data, [0]=MCLK = clk (512 fs)
  assign uio_oe  = 8'b1100_0011; // uio[7:6] + uio[1:0] outputs (LRCLK/BCLK + MCLK/DAC data); uio[5:2] inputs (params P3..P0)

  // List all unused inputs to prevent warnings
  // (uio[3] = P1 lands in task 5; uio[7:6]/[1:0] are clock/data
  // outputs, their input side is unused)
  wire _unused = &{ena, uio_in[7:6], uio_in[3], uio_in[1:0], 1'b0};

endmodule

`default_nettype wire
