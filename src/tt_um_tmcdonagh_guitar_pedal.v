/*
 * Copyright (c) 2026 Thomas McDonagh
 * SPDX-License-Identifier: Apache-2.0
 *
 * Silicon Guitar Pedal — multi-effect DSP core (v3: 2x2 silicon
 * pedalboard, Rev A). I2S master in/out (codec-agnostic), serial
 * effect chain comp -> fuzz -> distortion -> phaser -> trem -> delay
 * with per-stage enable bypass (comp: smoothed-envelope compressor,
 * slewed gain through the shared multiplier, task 10; fuzz: asymmetric
 * clip + octave blend on its chain input, task 4; phaser and delay are
 * identity stubs, Rev B; the v2.2 crush slot is removed, its envelope
 * follower retained for the fuzz bias).
 * Switches: DIPs on ui[6:1] (en[0]=comp .. en[4]=trem, en[5] = the
 * delay slot, a reserved identity stub; bit order = chain order) and the
 * fuzz octave switch on ui[7], all debounced (v3 task 7: 2-flop sync,
 * two agreeing samples 256 frames apart). uio[4:2] are the RC knob pins
 * K1..K3 (knob_rc readers, v3 task 9: K1 = trem rate; K2/K3 read, no
 * consumer yet); uio[5] reserved (SPI MISO).
 * uo[7] = dp clip LED; uo[2:0] = SPI idle (SCK=0, CS=1, MOSI=0, Rev B
 * PSRAM); uo[6:3] driven 0.
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
  // Effect chain + enables (v3 task 1, 2x2 Rev A): serial comp ->
  // fuzz -> dist -> phaser -> trem -> delay, enable-bypass per stage,
  // en bit order = chain order (en[0]=comp .. en[5]=delay) — DIP
  // switches on ui[6:1] (debounced), ui[7] = fuzz octave switch.
  // Bypass stays true bypass, wired externally with a 3PDT switch
  // around the pedal (all-off DIP = "digital clean": audio still runs
  // through the ADC -> identity chain -> DAC, a distinct mode from
  // true bypass). Comp is the slewed-gain compressor (task 10);
  // phaser and delay are identity stubs in Rev A.
  // ------------------------------------------------------------------
  // ------------------------------------------------------------------
  // Switch debounce (v3 task 7). Each of ui[7:1] passes a 2-flop
  // synchronizer; an 8-bit frame divider counts sample_strobe and every
  // 256th strobe (deb_div wrapping) is a tick that samples the
  // synchronized bits into deb_s. If the new sample equals the previous
  // tick's sample the switch bank sw updates (two agreeing ticks 256
  // frames ~ 5.3 ms apart), else it holds — so contact bounce shorter
  // than a tick pair never reaches the chain. sw = {oct, en[5:0]}
  // (ui[7], ui[6:1]); everything resets to 0 (pedal boots with all
  // effects off; the switch inputs have external pull-downs), so a
  // switch set at power-up reaches the chain at strobe 512 (frame 511's
  // sample is the first processed with it; golden_model EN_SETTLE).
  // ------------------------------------------------------------------
  reg [6:0] sw_s1, sw_s2;   // synchronizer
  reg [7:0] deb_div;        // frame divider, wraps every 256 frames
  reg [6:0] deb_s;          // previous tick's sample
  reg [6:0] sw;             // debounced {oct, en[5:0]}
  wire      deb_tick = sample_strobe && (deb_div == 8'hFF);
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sw_s1   <= 7'd0;
      sw_s2   <= 7'd0;
      deb_div <= 8'd0;
      deb_s   <= 7'd0;
      sw      <= 7'd0;
    end else begin
      sw_s1 <= ui_in[7:1];
      sw_s2 <= sw_s1;
      if (sample_strobe) deb_div <= deb_div + 8'd1;
      if (deb_tick) begin
        deb_s <= sw_s2;
        if (sw_s2 == deb_s) sw <= sw_s2;
      end
    end
  end

  wire [5:0] en = sw[5:0];   // en[5] (ui[6]) = delay slot enable, unused (identity stub)

  wire [15:0] s0 = sample_in;

  // ------------------------------------------------------------------
  // Step 1 — compressor (v3 task 10): smoothed envelope, 1.5 dB gain
  // table, slewed gain through the shared multiplier. The state runs
  // every frame whatever en[0] is (so cenv can also key the gate, task 11):
  //   cenv (24-bit 16.8) on m = sat_mag(sample_in), the pre-chain input:
  //     attack ((m << 8) > cenv): cenv += ((m << 8) - cenv) >> 5
  //     release:                  cenv -= (cenv >> 12) + (cenv != 0)
  //     (the 1-LSB floor makes the release terminate at exactly 0)
  //   L = cenv[23:8]; lvl_q = 4*e + m2 (e = L's MSB index, m2 = the two
  //     bits below it, zero-filled for e < 2; 0 for L == 0): 1.5 dB steps
  //   over = max(0, lvl_q - 44)  (knee at L = 2048)
  //   gq = max(-8, 6 - ((over * 5) >> 3))  (ratio 8/5 above the knee)
  //   target = MANT[gq & 3] shifted by gq >>> 2, MANT = 256/304/362/431
  //     (2^(k/4) in 8.8); reachable gq is 6..-3 -> target 724..152
  //   g (12-bit, reset 256 = unity) <= g + ((target - g) >>> 3)
  //   s1 = en[0] ? sat16((s0 * g) >>> 8) : s0, the multiplier at step 1
  // cenv and g update on the sample_valid edge, each from the other's
  // pre-edge value (g slews toward the target of the pre-edge cenv);
  // step 1 multiplies by the post-edge g.
  // ------------------------------------------------------------------
  reg  [23:0] cenv;
  reg  [11:0] comp_g;
  wire signed [28:0] mul_p;   // shared multiplier product (u_mult below)
  wire [15:0] mag = (sample_in == 16'h8000) ? 16'h7FFF  // saturating |x|, cenv and env
                    : sample_in[15] ? (~sample_in + 16'd1) : sample_in;

  // attack test and gap from one 25-bit subtract: (m << 8) > cenv iff
  // the difference is non-negative and non-zero (mag[15] is 0, so the
  // shifted magnitude fits 24 bits)
  wire [24:0] cenv_gap = {1'b0, mag, 8'h00} - {1'b0, cenv};
  wire        cenv_atk = ~cenv_gap[24] & (|cenv_gap[23:0]);
  // release decrement (cenv >> 12) + (cenv != 0): cenv < 2^23, so
  // cenv[23:12] <= 2047 and the 12-bit sum cannot carry out
  wire [11:0] cenv_dec = cenv[23:12] + {11'd0, |cenv};

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)            cenv <= 24'd0;
    else if (sample_valid) cenv <= cenv_atk ? cenv + {5'd0, cenv_gap[23:5]}
                                            : cenv - {12'd0, cenv_dec};
  end

  // level: leading-one index e of L and the two bits below it ({L, 2'b00}
  // supplies the zero fill for e < 2)
  wire [15:0] comp_l  = cenv[23:8];
  wire [17:0] comp_lx = {comp_l, 2'b00};
  reg  [3:0]  comp_e;
  integer     ci;
  always @(*) begin
    comp_e = 4'd0;
    for (ci = 1; ci < 16; ci = ci + 1)
      if (comp_l[ci]) comp_e = ci[3:0];
  end
  wire [1:0] comp_m2   = comp_lx[comp_e +: 2];
  wire [5:0] comp_lvl  = (comp_l == 16'd0) ? 6'd0 : {comp_e, comp_m2};

  // gain index: over <= 63 - 44 = 19, red = (5 * over) >> 3 <= 11, so
  // gq = 6 - red >= -5 and the -8 floor below is unreachable (kept per
  // the brief's table definition)
  wire [5:0] comp_over = (comp_lvl > 6'd44) ? comp_lvl - 6'd44 : 6'd0;
  wire [7:0] comp_ov5  = {2'b00, comp_over} + {comp_over, 2'b00};
  wire signed [5:0] comp_gq_r = 6'sd6 - $signed({1'b0, comp_ov5[7:3]});
  wire signed [3:0] comp_gq   = (comp_gq_r < -6'sd8) ? -4'sd8 : comp_gq_r[3:0];

  // target: mantissa by gq[1:0], exponent gq >>> 2 in -2..1
  wire [8:0] comp_mant = (comp_gq[1:0] == 2'd0) ? 9'd256
                       : (comp_gq[1:0] == 2'd1) ? 9'd304
                       : (comp_gq[1:0] == 2'd2) ? 9'd362 : 9'd431;
  wire [11:0] comp_tgt = (comp_gq[3:2] == 2'b01) ? {2'b00, comp_mant, 1'b0}  // << 1
                       : (comp_gq[3:2] == 2'b00) ? {3'b000, comp_mant}       // x1
                       : (comp_gq[3:2] == 2'b11) ? {4'b0000, comp_mant[8:1]} // >> 1
                                                 : {5'b00000, comp_mant[8:2]}; // >> 2

  // slew: g + ((target - g) >>> 3), 13-bit signed gap; g stays within
  // [min target, max target] (64..724), so the 12-bit sum never wraps
  wire signed [12:0] comp_gap = $signed({1'b0, comp_tgt}) - $signed({1'b0, comp_g});
  wire signed [12:0] comp_stp = comp_gap >>> 3;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)            comp_g <= 12'd256;
    else if (sample_valid) comp_g <= comp_g + comp_stp[11:0];
  end

  // output: (s0 * g) >>> 8 is p[28:8]; it fits 16 bits iff p[28:23]
  // all equal the sign, else clamp to the rails
  wire        comp_ovf = ~((&mul_p[28:23]) | ~(|mul_p[28:23]));
  wire [15:0] comp_out = comp_ovf ? (mul_p[28] ? 16'h8000 : 16'h7FFF)
                                  : mul_p[23:8];

  wire [15:0] s1_w = en[0] ? comp_out : s0;

  // Sequenced chain (v3 task 8): one registered stage per clk, stepped
  // after sample_valid. step = 1 on the clk after sample_valid (env,
  // the trem phase and sw are post-edge by then), runs to 6, idles at 0.
  // Stage outputs are registered; stage n's logic reads stage n-1's
  // register. Step map:
  //   step 1: comp            -> s1_r
  //   step 2: fuzz            -> s2_r
  //   step 3: dist            -> s3_r
  //   step 4: tone/gate slot  -> s4_r (identity for now)
  //   step 5: phaser stub     -> s5_r
  //   step 6: trem (mult)     -> effect_sample (delay stub is identity)
  reg [2:0]  step;
  reg [15:0] s1_r, s2_r, s3_r, s4_r, s5_r;
  reg        chain_done;   // 1-clk pulse, the clk after effect_sample loads

  // ------------------------------------------------------------------
  // Step 2 — fuzz (v3 task 4): Fuzz Face-inspired asymmetric clip +
  // octave blend on the stage's CHAIN input s1_r — the task-3 routed
  // re-coupling (v2.2's cell-saving trim read the PRE-CHAIN sample_in
  // here, masking comp in every fuzz-on stack, probe-verified en=0x03
  // == en=0x02; fuzz now consumes comp's output, and dist below
  // consumes fuzz's). Input x = s1_r, signed; constants provisional:
  //   a      = sat_mag(x)  — the saturating-|x| idiom on s1_r
  //   t_pos  = 19660       (~ +0.6 FS)
  //   t_neg  = sat16(13107 + (env >>> 2))  (~ -0.4 FS at env 0,
  //             relaxing to -21298 at env 32766 — bias-starve
  //             dynamics: the negative clip point eases up with the
  //             input level. env is read POST-update (env updates on
  //             the sample_valid edge, step 2 runs after it), so the
  //             word transmitted in frame N+1 is fuzz(x_N, env_N).
  //             env <= 32766 -> env>>>2
  //             <= 8191 -> the sum <= 21298 < 32767: the sat16 clamp
  //             below is unreachable, kept as a guard for by-ear
  //             retunes; it tests bits [16:15] because the sum fits a
  //             positive 16-bit value iff both are clear)
  //   shaped = a > t ? t : a              (asym clip on the magnitude)
  //   fuzz_core = x[15] ? (-shaped) : shaped (sat16 negate; shaped <=
  //             21298 -> -shaped >= -21298 > -32768: that clamp is
  //             unreachable too, kept per the convention)
  // The octave switch (ui[7]) blends the full-wave-rectified copy (the octave):
  //   fuzz_out = oct ? sat16(shaped + a) : fuzz_core
  // shaped + a >= 0 always (both non-negative), so the blend's sat16
  // clamps UPPER only (max 21298 + 32767 -> 0x7FFF): with the octave on
  // fuzz_out is non-negative in every reachable state; with it off it
  // is signed (the plain asym fuzz); dist below takes |s2| either way.
  // ------------------------------------------------------------------
  wire [15:0] fuzz_mag = (s1_r == 16'h8000) ? 16'h7FFF   // saturating |s1_r|
                       : s1_r[15] ? (~s1_r + 16'd1) : s1_r;

  wire fuzz_oct = sw[6];   // debounced ui[7]: octave blend

  // negative clip point sat16(13107 + (env >>> 2))
  wire [16:0] fuzz_tn_sum = 17'd13107 + {3'b000, env[15:2]};
  wire [15:0] fuzz_tneg = |fuzz_tn_sum[16:15] ? 16'h7FFF : fuzz_tn_sum[15:0];
  wire [15:0] fuzz_t = s1_r[15] ? fuzz_tneg : 16'd19660;

  wire [15:0] fuzz_shaped = (fuzz_mag > fuzz_t) ? fuzz_t : fuzz_mag;

  // sat16 negate (clamp unreachable, kept per convention)
  wire [16:0] fuzz_neg = 17'd0 - {1'b0, fuzz_shaped};
  wire        fuzz_neg_ovf = (fuzz_neg[16] != fuzz_neg[15]);
  wire [15:0] fuzz_core = s1_r[15] ? (fuzz_neg_ovf ? 16'h8000 : fuzz_neg[15:0])
                                 : fuzz_shaped;

  // octave blend: sat16(shaped + a), upper clamp only — both terms are
  // non-negative so the sum cannot underflow, and it fits a positive
  // 16-bit magnitude iff <= 0x7FFF, i.e. iff bits [16:15] are both
  // clear (their OR is exactly "sum > 0x7FFF" for any 17-bit sum; the
  // plain [16] carry test would MISS sums in [0x8000, 0xFFFF] — the
  // max here is 21298 + 32767 = 54065, so [16] never sets on its own)
  wire [16:0] fuzz_blend_w = {1'b0, fuzz_shaped} + {1'b0, fuzz_mag};
  wire [15:0] fuzz_blend = |fuzz_blend_w[16:15] ? 16'h7FFF : fuzz_blend_w[15:0];

  wire [15:0] fuzz_out = fuzz_oct ? fuzz_blend : fuzz_core;

  wire [15:0] s2_w = en[1] ? fuzz_out : s1_r;

  // ------------------------------------------------------------------
  // Step 3 — distortion (v3 task 5): fixed drive x4 into a 3-piece
  // diode-knee PWL on the stage input's magnitude. a = sat_mag(s2);
  // w = 4a (18-bit, NO intermediate saturation — keeps the top soft):
  //   w <= 8192   : out = w                        (slope 1)
  //   w <= 32768  : out = 8192 + ((w - 8192) >> 2) (slope 1/4, 14336 at K2)
  //   else        : out = 14336 + ((w - 32768) >> 4) (slope 1/16)
  // Max out = 20479 (a = 32767), so the sign negate below never overflows.
  // Drive is the constant 4 (the <<2); a retune is that shift plus the
  // knee constants. Shift-add only, no multipliers.
  // ------------------------------------------------------------------
  // Direct saturating |x| on the actual stage input s2 (sign stays
  // s2_r[15] below). Correct for every enable/octave combination, including
  // fuzz's signed core output with the octave off.
  wire [15:0] dist_mag = (s2_r == 16'h8000) ? 16'h7FFF   // saturating |s2_r|
                       : s2_r[15] ? (~s2_r + 16'd1) : s2_r;

  wire [17:0] dist_w = {dist_mag, 2'b00};
  wire [17:0] dist_o1 = 18'd8192 + ((dist_w - 18'd8192) >> 2);
  wire [17:0] dist_o2 = 18'd14336 + ((dist_w - 18'd32768) >> 4);
  wire [15:0] dist_pw = (dist_w <= 18'd8192)  ? dist_w[15:0]
                      : (dist_w <= 18'd32768) ? dist_o1[15:0]
                                              : dist_o2[15:0];

  // sign reapplied through a sat16 clamp (unreachable: out <= 20479,
  // kept per the saturating-arithmetic convention)
  wire [16:0] dist_neg = 17'd0 - {1'b0, dist_pw};
  wire dist_neg_ovf = (dist_neg[16] != dist_neg[15]);
  wire [15:0] dist_out = s2_r[15] ? (dist_neg_ovf ? 16'h8000 : dist_neg[15:0])
                                : dist_pw;

  wire [15:0] s3_w = en[2] ? dist_out : s2_r;

  // ------------------------------------------------------------------
  // Step 5 — phaser: identity stub on s4_r (Rev B allpass stack; the
  // enable is a no-op in Rev A). Step 4 (tone/gate slot) is a plain
  // register copy s3_r -> s4_r for now.
  // ------------------------------------------------------------------
  wire [15:0] phaser_out = s4_r;   // Rev B

  wire [15:0] s5_w = en[3] ? phaser_out : s4_r;

  // ------------------------------------------------------------------
  // Fast envelope (v3): the v2.2 auto-lofi envelope follower, retained
  // after the crush slot's removal — the fuzz negative clip point
  // (task 4) reads it; the compressor has its own smoothed cenv.
  // 16-bit envelope of the PRE-CHAIN input magnitude (mag, shared with
  // cenv and declared with it above, the
  // saturating |sample_in| idiom — env's own source since task 4 moved
  // stage 2 onto its chain input; fix-round ruling — the player's
  // dynamics live in the raw input):
  //   attack (mag > env): env <= (env + mag) >>> 1   — 17-bit add,
  //     >> 1 back to 16; halves the gap, ~12 frames to converge;
  //   release: env <= env - (env>>>8) - (env != 0)   — tau ~256 frames
  //     ~5.3 ms; the 1-LSB floor makes the decay terminate at exactly
  //     0 (the plain subtract-shift stalls anywhere below 256).
  // env tops out at 32766 — env[15] never sets. One update per audio
  // frame on sample_valid.
  // ------------------------------------------------------------------
  reg  [15:0] env;
  wire [16:0] env_atk = {1'b0, env} + {1'b0, mag};       // 17-bit env+mag
  wire        env_nz  = (env != 16'd0);                   // release floor

  // release decrement (env>>>8) + (env != 0) in one 8-bit sum: the
  // carry-out cannot occur (env <= 32766, so env[15:8] <= 127) — keeps
  // the release a single 16-bit subtractor against a zero-extended
  // byte instead of two cascaded full-width subtractors
  wire [7:0]  env_dec8 = env[15:8] + {7'b0, env_nz};
  wire [15:0] env_dec  = {8'h00, env_dec8};

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)             env <= 16'd0;
    else if (sample_valid)  begin
      if (mag > env)        env <= env_atk[16:1];         // (env+mag) >>> 1
      else                  env <= env - env_dec;
    end
  end

  // ------------------------------------------------------------------
  // RC knob readers (v3 task 9): K1 uio[2], K2 uio[3], K3 uio[4]. Each
  // discharges its pin (drive_low -> uio_oe, uio_out = 0) then times the
  // RC charge; value is the smoothed 8-bit reading, present = 0 after a
  // timeout (knob absent). K1 sets the trem rate; K2/K3 have no consumer
  // yet (listed in _unused until their tasks).
  // ------------------------------------------------------------------
  wire [7:0] k1_value, k2_value, k3_value;
  wire       k1_present, k2_present, k3_present;
  wire       k1_drv, k2_drv, k3_drv;

  knob_rc k1 (.clk(clk), .rst_n(rst_n), .pin_in(uio_in[2]), .drive_low(k1_drv),
              .value(k1_value), .present(k1_present));
  knob_rc k2 (.clk(clk), .rst_n(rst_n), .pin_in(uio_in[3]), .drive_low(k2_drv),
              .value(k2_value), .present(k2_present));
  knob_rc k3 (.clk(clk), .rst_n(rst_n), .pin_in(uio_in[4]), .drive_low(k3_drv),
              .value(k3_value), .present(k3_present));

  // ------------------------------------------------------------------
  // Step 6 — tremolo (v3 task 9): smooth triangle-LFO gain through the
  // shared multiplier. 22-bit phase P steps on sample_valid while trem
  // is enabled (P <= P + inc), else holds at 0:
  //   inc  = K1 present ? (64 + value[5:0]) << value[7:6] : 437
  //          (64..1016: 2^22/inc frames per cycle, ~0.73..11.6 Hz at
  //          48 kHz; absent default 437 ~5.0 Hz)
  //   tri  = P[21] ? ~P[20:13] : P[20:13]     (8-bit triangle)
  //   gain = 256 - (tri >> 1) - (tri >> 2)    (66..256, 75% depth)
  //   out  = (s5_r * gain) >>> 8              (|out| <= |s5_r|: no clamp)
  // ------------------------------------------------------------------
  reg  [21:0] trem_p;
  wire [9:0]  trem_inc_k = {3'b000, 1'b1, k1_value[5:0]} << k1_value[7:6];
  wire [9:0]  trem_inc   = k1_present ? trem_inc_k : 10'd437;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)            trem_p <= 22'd0;
    else if (sample_valid) trem_p <= en[4] ? (trem_p + {12'd0, trem_inc}) : 22'd0;
  end

  wire [7:0] trem_tri  = trem_p[21] ? ~trem_p[20:13] : trem_p[20:13];
  wire [8:0] trem_gain = 9'd256 - {2'b00, trem_tri[7:1]} - {3'b000, trem_tri[7:2]};

  // Shared multiplier, operands muxed by step: comp at step 1 (s0 x g),
  // trem at step 6 (s5_r x gain). Each step registers its own function
  // of mul_p (comp_out above, trem_out below).
  wire               mul_comp = (step == 3'd1);
  wire signed [16:0] mul_a = mul_comp ? {s0[15], s0} : {s5_r[15], s5_r};
  wire        [11:0] mul_b = mul_comp ? comp_g : {3'b000, trem_gain};
  mult u_mult (.a(mul_a), .b(mul_b), .p(mul_p));

  wire [15:0] trem_out = mul_p[23:8];   // >>> 8; fits 16 bits (gain <= 256)

  wire [15:0] s6_w = en[4] ? trem_out : s5_r;

  // ------------------------------------------------------------------
  // Chain output (name kept from v1: the audio tooling probes
  // dut.effect_sample).
  // ------------------------------------------------------------------
  reg [15:0] effect_sample;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      step <= 3'd0;
      s1_r <= 16'd0; s2_r <= 16'd0; s3_r <= 16'd0;
      s4_r <= 16'd0; s5_r <= 16'd0;
      effect_sample <= 16'd0;
      chain_done    <= 1'b0;
    end else begin
      chain_done <= (step == 3'd6);
      if (sample_valid)      step <= 3'd1;
      else if (step == 3'd6) step <= 3'd0;
      else if (step != 3'd0) step <= step + 3'd1;
      case (step)
        3'd1: s1_r <= s1_w;            // comp
        3'd2: s2_r <= s2_w;            // fuzz
        3'd3: s3_r <= s3_w;            // dist
        3'd4: s4_r <= s3_r;            // tone/gate slot (identity for now)
        3'd5: s5_r <= s5_w;            // phaser stub
        3'd6: effect_sample <= s6_w;   // trem (the delay stub is identity)
        default: ;
      endcase
    end
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

  // ------------------------------------------------------------------
  // Clip LED (v2.2 task 6): dp on uo[7] with ~2048-frame (~43 ms)
  // peak-hold. hit = (effect_sample == 16'h7FFF) || (effect_sample ==
  // 16'h8000) — the two saturation rails, which are bitwise
  // complements, so no single equality covers both. Written as the
  // value-equal reduction form — one mux of two 15-bit reduction trees
  // instead of two 16-bit comparators:
  //   x[15]=0 -> &x[14:0]  (x == 0x7FFF)
  //   x[15]=1 -> ~|x[14:0] (x == 0x8000)
  // The hit is taken at chain_done: the clk after effect_sample loads
  // (step 6, ~8 clk after the sample_valid edge), once per output
  // frame, so the detect sees each transmitted sample exactly once (the
  // very value the TX shift sends next frame). Hold: a hit lights dp_q
  // and reloads the 11-bit counter to 0; otherwise, while lit, the
  // counter runs up to 2047 and clears dp_q on the edge it wraps
  // (2047 + 1 = 0 in 11 bits, so the terminal state needs no separate
  // reset path). A hit mid-hold restarts the hold.
  // ------------------------------------------------------------------
  wire hit = effect_sample[15] ? ~|effect_sample[14:0]   // 0x8000
                               :  &effect_sample[14:0];  // 0x7FFF

  reg        dp_q;
  reg [10:0] dp_cnt;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      dp_q   <= 1'b0;
      dp_cnt <= 11'd0;
    end else if (chain_done) begin
      if (hit) begin
        dp_q   <= 1'b1;
        dp_cnt <= 11'd0;
      end else if (dp_q) begin
        dp_cnt <= dp_cnt + 11'd1;
        if (dp_cnt == 11'd2047) dp_q <= 1'b0;
      end
    end
  end

  // uo[7] = dp clip LED (peak-hold); uo[6:3] reserved, driven 0; uo[2:0] =
  // SPI idle for Rev B PSRAM: SCK = 0, CS = 1 (deselected), MOSI = 0
  assign uo_out  = {dp_q, 4'b0000, 3'b010};

  // [7]=LRCLK, [6]=BCLK, [5]=0 (MISO input), [4:2]=0 (knob discharge
  // level, enabled only while each reader drives), [1]=DAC data,
  // [0]=MCLK = clk (512 fs)
  assign uio_out = {lrclk, bclk, 1'b0, 3'b000, dac_data, clk};
  // uio[7:6] + uio[1:0] outputs; uio[5] input (reserved SPI MISO);
  // uio[4:2] = knob pins K3..K1, outputs only while discharging
  assign uio_oe  = {2'b11, 1'b0, k3_drv, k2_drv, k1_drv, 2'b11};

  // List all unused inputs to prevent warnings
  // (ui[7:1] = switches and ui[0] = ADC data are live; uio[7:6]/[1:0]
  // are clock/data outputs, their input side is unused; uio[5] is the
  // reserved SPI MISO; uio[4:2] feed the knob readers. K2/K3 readings
  // have no consumer until their tasks; the multiplier's fraction bits
  // below the >>> 8 are unused)
  wire _unused = &{ena, uio_in[7:5], uio_in[1:0], en[5],
                   k2_value, k2_present, k3_value, k3_present,
                   mul_p[7:0], trem_tri[0], 1'b0};

endmodule

`default_nettype wire
