/*
 * Copyright (c) 2026 Thomas McDonagh
 * SPDX-License-Identifier: Apache-2.0
 *
 * Silicon Guitar Pedal — multi-effect DSP core (v3: 2x2 silicon
 * pedalboard, Rev A). I2S master in/out (codec-agnostic), serial
 * effect chain comp -> fuzz -> distortion -> phaser -> trem -> delay
 * with per-stage enable bypass (comp: smoothed-envelope compressor,
 * slewed gain through the shared multiplier, task 10; fuzz: asymmetric
 * clip + octave blend on its chain input, task 4; a post-dirt tone
 * low-pass and noise gate follow fuzz/dist, task 11; the phaser is a
 * Phase 90-style 4-stage allpass, Rev B1 Task 16; the delay is an
 * identity stub until Rev B3; the v2.2 crush slot is removed, its
 * envelope follower retained for the fuzz bias).
 * Switches: DIPs on ui[6:1] (en[0]=comp .. en[4]=trem, en[5] = the
 * delay slot, a reserved identity stub; bit order = chain order) and the
 * fuzz octave switch on ui[7], all debounced (v3 task 7: 2-flop sync,
 * two agreeing samples 256 frames apart). uio[5:2] are the RC knob pins
 * K1..K4 (one round-robin knob_rc4 reader, v3 task 9 / Task 15a: K1 =
 * trem + phaser rate, K2 = tone, task 11; K3 = dist drive; K4 = delay time, uio[5],
 * Rev B1: read, not yet used).
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
  // true bypass). Comp is the slewed-gain compressor (task 10); the
  // phaser is live from Rev B1 (Task 16); the delay is an identity stub.
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

  wire [5:0] en = sw[5:0];   // en[5] (ui[6]) = delay slot enable, unused (identity stub; ui[6] is also PSRAM MISO from B3)

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
  //     (2^(k/4) in 8.8); reachable gq is 6..-5 -> target 724..107 (the table floor
  //     -8 is never reached, see comp_gq below)
  //   g (12-bit, reset 256 = unity) <= g + ((target - g) >>> 3)
  //   s1 = en[0] ? sat16((s0 * g) >>> 8) : s0, the multiplier at step 1
  // cenv and g update on the sample_valid edge, each from the other's
  // pre-edge value (g slews toward the target of the pre-edge cenv);
  // step 1 multiplies by the post-edge g. The gate below keys on the
  // same pre-edge L = cenv[23:8].
  // ------------------------------------------------------------------
  reg  [23:0] cenv;
  reg  [11:0] comp_g;
  // knob reader outputs (instances below; declared first: the dist drive,
  // tone shift and trem rate read them above their instances)
  wire [7:0] k1_value, k2_value, k3_value, k4_value;
  wire       k1_present, k2_present, k3_present, k4_present;
  wire       k1_drv, k2_drv, k3_drv, k4_drv;
  wire signed [28:0] mul_p;   // shared multiplier product (u_mult below)
  wire               mul_done; // u_mult: p final (1-clk pulse)
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
  // gq = 6 - red >= -5: the reachable floor is -5; the -8 floor below is the
  // table's definition and is never reached (kept for the saturating convention)
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

  // Sequenced chain (v3 task 8; serial multiplier since Task 15a): one
  // registered stage per step, stepped after sample_valid. step = 1 on
  // the clk after sample_valid (env, the trem phase, gate state and sw are
  // post-edge by then), runs to CHAIN_LAST, idles at 0. A step without the
  // multiplier lasts 1 clk. A step with it (step_mul) raises mul_start
  // on its first clk (the multiplier captures the step's operands), holds
  // while mwait, and registers its result on the clk mul_done is high:
  // 1 + L + 1 = 14 clk (L = 12). Adding a multiplier step = one more
  // step_mul term + its operand case arm + its chain case arm. Per frame
  // (Task 16): 8 mult steps x 14 + 4 single-clk steps = 116 clk from
  // sample_valid to chain_done (bound 200, test_chain_latency; the TX load
  // is 256 clk after sample_valid).
  // One running chain register cs (Task 17a): step 1 loads cs from s0, each
  // later step reads cs (the previous stage's output) and writes its own
  // output back. No stage needs an earlier output after its successor has
  // run, so no per-stage tap registers exist: cs is unchanged across the
  // non-writing steps (4, 6..9), which is what the tone filter (step 4),
  // the gate passthrough (step 5) and the phaser (steps 6, 9, 10) rely on
  // (they read the s3 / s4 value cs still holds). DO NOT write cs at steps
  // 4 or 6..9: the held s3 / s4 values would be lost. Step map:
  //   step 1: comp (mult)     -> cs
  //   step 2: fuzz            -> cs
  //   step 3: dist (mult)     -> cs   (= the s3 value, held through step 5)
  //   step 4: tone            -> ytone (every frame; 16.4 low-pass of cs)
  //   step 5: gate (mult)     -> cs (fuzz/dist on: tone * gg, else cs); = s4
  //   steps 6..9: phaser allpass stages 0..3 (mult) -> ph_* state (every frame)
  //   step 10: phaser mix     -> cs (s4 held through step 10)
  //   step 11: trem (mult)    -> cs
  //   step 12: delay slot     -> effect_sample (identity until Task 22)
  // The shared multiplier serves comp (1), dist (3), gate (5), the phaser
  // stages (6..9) and trem (11).
  localparam [3:0] CHAIN_LAST = 4'd12;
  reg [3:0]  step;
  reg        mwait;        // the current step's multiply is running
  reg [15:0] cs;           // running chain sample: each step reads cs and writes its own output back
  reg        chain_done;   // 1-clk pulse, the clk after effect_sample loads
  // per-step "uses the multiplier" flag, start and advance
  wire       ph_step   = (step >= 4'd6) & (step <= 4'd9);   // phaser stages 0..3
  wire       step_mul  = (step == 4'd1) | (step == 4'd3) | (step == 4'd5)
                       | ph_step | (step == 4'd11);
  wire       mul_start = step_mul & ~mwait;
  wire       step_adv  = (step != 4'd0) & (~step_mul | mul_done);

  // ------------------------------------------------------------------
  // Step 2 — fuzz (v3 task 4): Fuzz Face-inspired asymmetric clip +
  // octave blend on the stage's CHAIN input cs — the task-3 routed
  // re-coupling (v2.2's cell-saving trim read the PRE-CHAIN sample_in
  // here, masking comp in every fuzz-on stack, probe-verified en=0x03
  // == en=0x02; fuzz now consumes comp's output, and dist below
  // consumes fuzz's). Input x = cs, signed; constants provisional:
  //   a      = sat_mag(x)  — the saturating-|x| idiom on cs
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
  wire [15:0] fuzz_mag = (cs == 16'h8000) ? 16'h7FFF   // saturating |cs|
                       : cs[15] ? (~cs + 16'd1) : cs;

  wire fuzz_oct = sw[6];   // debounced ui[7]: octave blend

  // negative clip point sat16(13107 + (env >>> 2))
  wire [16:0] fuzz_tn_sum = 17'd13107 + {3'b000, env[15:2]};
  wire [15:0] fuzz_tneg = |fuzz_tn_sum[16:15] ? 16'h7FFF : fuzz_tn_sum[15:0];
  wire [15:0] fuzz_t = cs[15] ? fuzz_tneg : 16'd19660;

  wire [15:0] fuzz_shaped = (fuzz_mag > fuzz_t) ? fuzz_t : fuzz_mag;

  // sat16 negate (clamp unreachable, kept per convention)
  wire [16:0] fuzz_neg = 17'd0 - {1'b0, fuzz_shaped};
  wire        fuzz_neg_ovf = (fuzz_neg[16] != fuzz_neg[15]);
  wire [15:0] fuzz_core = cs[15] ? (fuzz_neg_ovf ? 16'h8000 : fuzz_neg[15:0])
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

  wire [15:0] s2_w = en[1] ? fuzz_out : cs;

  // ------------------------------------------------------------------
  // Step 3 — distortion (v3 task 5, K3 drive task 12): drive into a
  // 3-piece diode-knee PWL on the stage input's magnitude. a = sat_mag(s2);
  // w = (a * drive) >> 8 through the shared multiplier (18-bit, NO
  // intermediate saturation — keeps the top soft):
  //   w <= 8192   : out = w                        (slope 1)
  //   w <= 32768  : out = 8192 + ((w - 8192) >> 2) (slope 1/4, 14336 at w = 32768)
  //   else        : out = 14336 + ((w - 32768) >> 4) (slope 1/16)
  // drive (Q4.8) = K3 present ? 256 + 7 * value3 (x1 .. x7.97) : 1024 (x4).
  // w < 32768 * 2041 / 256 = 261,248 < 2^18. Max out = 14336 + (261248 -
  // 32768) / 16 = 28,615 < 32767 (x4 default: 20479), so the sign negate
  // below never overflows.
  // ------------------------------------------------------------------
  // Direct saturating |x| on the actual stage input s2 (sign stays
  // cs[15] below). Correct for every enable/octave combination, including
  // fuzz's signed core output with the octave off.
  wire [15:0] dist_mag = (cs == 16'h8000) ? 16'h7FFF   // saturating |cs|
                       : cs[15] ? (~cs + 16'd1) : cs;

  wire [11:0] dist_drive = k3_present
      ? (12'd256 + {1'b0, k3_value, 3'b000} - {4'b0000, k3_value})
      : 12'd1024;
  wire [17:0] dist_w = mul_p[25:8];     // a * drive >> 8 (p >= 0, < 2^26)
  wire [17:0] dist_o1 = 18'd8192 + ((dist_w - 18'd8192) >> 2);
  wire [17:0] dist_o2 = 18'd14336 + ((dist_w - 18'd32768) >> 4);
  wire [15:0] dist_pw = (dist_w <= 18'd8192)  ? dist_w[15:0]
                      : (dist_w <= 18'd32768) ? dist_o1[15:0]
                                              : dist_o2[15:0];

  // sign reapplied through a sat16 clamp (unreachable: out <= 28,615 at
  // max K3 drive; kept per the saturating-arithmetic convention)
  wire [16:0] dist_neg = 17'd0 - {1'b0, dist_pw};
  wire dist_neg_ovf = (dist_neg[16] != dist_neg[15]);
  wire [15:0] dist_out = cs[15] ? (dist_neg_ovf ? 16'h8000 : dist_neg[15:0])
                                : dist_pw;

  wire [15:0] s3_w = en[2] ? dist_out : cs;

  // ------------------------------------------------------------------
  // Steps 4-5 — post-dirt tone + noise gate (v3 task 11), applied only
  // while fuzz or dist is on (dirt_on; the two are the only stages that
  // add noise and harshness).
  //   Tone (step 4): ytone (20-bit signed, 16.4) one-pole low-pass of
  //     cs, updated EVERY frame whatever the enables are (so it is
  //     warm when dirt comes on):
  //       ytone <= ytone + (((cs <<< 4) - ytone) >>> kt)
  //       kt = K2 present ? 4 - value2[7:6] : 2   (1..4; knob max = bright)
  //     tone = ytone >>> 4 (floor; the shift rounds the state to within
  //     a few LSB of cs at DC).
  //   Gate (frame state on sample_valid, each from pre-edge values; the
  //     chain steps see the post-edge ones): key L = cenv[23:8] of the
  //     compressor (its envelope runs whatever en[0] is). Hysteresis:
  //       open & L < 128 -> open <= 0;  !open & L >= 256 -> open <= 1
  //     and a 9-bit gain gg (reset 0) ramps toward 256 while open
  //     (gg + ((256 - gg) >> 2) + 1, capped) and down by 1 while closed:
  //       gg <= min(256, gg + ((256 - gg) >> 2) + 1) if open,
  //       gg <= gg - (gg != 0) otherwise.
  //   Step 5 (shared multiplier): cs = dirt_on ? (tone * gg) >>> 8
  //     : cs. |tone * gg| <= 2^23, so p[23:8] fits 16 bits (no clamp).
  // ------------------------------------------------------------------
  wire dirt_on = en[1] | en[2];

  reg signed [19:0] ytone;
  reg               gate_open;
  reg [8:0]         gate_gg;
  wire [2:0] tone_kt = k2_present ? (3'd4 - {1'b0, k2_value[7:6]}) : 3'd2;
  wire signed [20:0] tone_d = {cs[15], cs, 4'b0000} - {ytone[19], ytone};
  wire signed [20:0] tone_s = tone_d >>> tone_kt;
  wire signed [15:0] tone   = ytone[19:4];     // >>> 4

  wire [8:0] gate_rem = 9'd256 - gate_gg;
  wire [8:0] gate_up  = gate_gg + {2'b00, gate_rem[8:2]} + 9'd1;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      gate_open <= 1'b0;
      gate_gg   <= 9'd0;
    end else if (sample_valid) begin
      if (gate_open) begin
        gate_gg <= (gate_up > 9'd256) ? 9'd256 : gate_up;
        if (comp_l < 16'd128) gate_open <= 1'b0;
      end else begin
        gate_gg <= gate_gg - {8'd0, |gate_gg};
        if (comp_l >= 16'd256) gate_open <= 1'b1;
      end
    end
  end

  wire [15:0] s4_w = dirt_on ? mul_p[23:8] : cs;

  // ------------------------------------------------------------------
  // Steps 6..10 — phaser (Rev B1, Task 16): Phase 90-style, 4 first-order
  // allpass stages (no feedback) + 50/50 dry mix, golden_model.Phaser.
  //   pp (phaser_p, 22-bit) <= en[3] ? pp + inc : 0 on sample_valid, inc =
  //     K1 present ? the trem increment (shared knob) : 87 (~1.0 Hz, prov)
  //   t = pp[20:9], inverted if pp[21] (12-bit triangle)
  //   v = (3 t) >> 2 (0..3071); d = ((1024 + v[9:0]) << v[11:10]) >> 2
  //     (256..2047, exponential sweep); c = 4096 - d (Q12, 2049..3840)
  //   stage k (step 6 + k, input s; stage 0: cs), states x1[k], y1[k]:
  //     y = sat16(x1[k] + (((y1[k] - s) * c) >>> 12))   (shared multiplier)
  //     x1[k] <= s; y1[k] <= y; s = y
  //   mix (step 10): ph_out = (cs + y3) >>> 1 (17-bit sum, floor)
  // The stages run every frame whatever en[3] is (warm on switch-on).
  // Lean state (bit-exact with the golden's 8 registers): x1[k] for k >= 1
  // always equals y1[k-1] (both load stage k-1's output in the same frame,
  // both reset 0), so only y1[0..3] are kept, as a 4-deep shift chain
  // ph_y0..ph_y3 whose head ph_y0 is the current stage's y1[k]: each stage
  // step shifts it and pushes the new y at ph_y3 (four shifts per frame
  // restore the order; ph_y3 = the previous stage's y = this stage's
  // input). One more register ph_x holds the stage's x1[k]: on each stage
  // step it takes the consumed head (old y1[k] = x1[k+1]), and at stage 3
  // the frame's cs instead (= x1[0] for the next frame).
  // Bounds: |y1 - s| <= 65535 (17-bit a); |p >>> 12| < 61440, so the 18-bit
  // x1 + p[28:12] cannot wrap before the sat16 clamp.
  // Timing: the ph_* shift happens on each of steps 6..9, so all four must
  // complete inside every frame (the chain finishes at ~116 clk, well before
  // the next sample_valid at 512 clk); a frame cut short would leave the
  // ph_y chain rotated.
  // ------------------------------------------------------------------
  reg  [21:0] phaser_p;
  reg  [15:0] ph_x, ph_y0, ph_y1, ph_y2, ph_y3;
  wire [9:0]  trem_inc_k;                     // K1 increment (trem block below)
  wire [9:0]  phaser_inc = k1_present ? trem_inc_k : 10'd87;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)            phaser_p <= 22'd0;
    else if (sample_valid) phaser_p <= en[3] ? (phaser_p + {12'd0, phaser_inc}) : 22'd0;
  end

  wire [11:0] ph_t     = phaser_p[21] ? ~phaser_p[20:9] : phaser_p[20:9];
  wire [13:0] ph_v3    = {2'b00, ph_t} + {1'b0, ph_t, 1'b0};        // 3 t <= 12285
  wire [11:0] ph_v     = ph_v3[13:2];                                // 0..3071
  wire [13:0] ph_dx    = {3'b000, 1'b1, ph_v[9:0]} << ph_v[11:10];  // <= 2047 << 2
  wire [11:0] phaser_c = 12'd0 - {1'b0, ph_dx[12:2]};                // 4096 - d (d >= 256)

  wire [15:0] ph_s   = (step == 4'd6) ? cs : ph_y3;               // stage input
  wire signed [16:0] ph_a = $signed({ph_y0[15], ph_y0}) - $signed({ph_s[15], ph_s});
  wire signed [17:0] ph_sum = $signed({{2{ph_x[15]}}, ph_x}) + $signed({mul_p[28], mul_p[28:12]});
  wire        ph_ovf = ~((&ph_sum[17:15]) | ~(|ph_sum[17:15]));
  wire [15:0] ph_y   = ph_ovf ? (ph_sum[17] ? 16'h8000 : 16'h7FFF) : ph_sum[15:0];

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ph_x  <= 16'd0;
      ph_y0 <= 16'd0; ph_y1 <= 16'd0; ph_y2 <= 16'd0; ph_y3 <= 16'd0;
    end else if (step_adv & ph_step) begin      // stage done (mul_done)
      ph_x  <= (step == 4'd9) ? cs : ph_y0;
      ph_y0 <= ph_y1;
      ph_y1 <= ph_y2;
      ph_y2 <= ph_y3;
      ph_y3 <= ph_y;
    end
  end

  wire [16:0] ph_mix     = {cs[15], cs} + {ph_y3[15], ph_y3};
  wire [15:0] phaser_out = ph_mix[16:1];        // (s4 + y3) >>> 1

  wire [15:0] s5_w = en[3] ? phaser_out : cs;

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
  // RC knob reader (v3 task 9; one shared round-robin reader since Task
  // 15a): K1 uio[2], K2 uio[3], K3 uio[4], K4 uio[5], measured one at a
  // time in turn. Each turn discharges that pin (drive_low -> uio_oe,
  // uio_out = 0) then times the RC charge; the other pins are released.
  // value is the smoothed 8-bit reading, present = 0 after a timeout
  // (knob absent). K1 sets the trem and phaser rate, K2 the tone shift; K3 sets the
  // dist drive. K4 (Rev B1): delay time; not consumed until the delay
  // lands (B3), so it sits in _unused for now. The kN_value/kN_present
  // wires are the testbench probe points.
  // ------------------------------------------------------------------

  knob_rc4 knobs (.clk(clk), .rst_n(rst_n), .pin_in(uio_in[5:2]),
                  .drive_low({k4_drv, k3_drv, k2_drv, k1_drv}),
                  .value({k4_value, k3_value, k2_value, k1_value}),
                  .present({k4_present, k3_present, k2_present, k1_present}));

  // ------------------------------------------------------------------
  // Step 11 — tremolo (v3 task 9): smooth triangle-LFO gain through the
  // shared multiplier. 22-bit phase P steps on sample_valid while trem
  // is enabled (P <= P + inc), else holds at 0:
  //   inc  = K1 present ? (64 + value[5:0]) << value[7:6] : 437
  //          (64..1016: 2^22/inc frames per cycle, ~0.73..11.6 Hz at
  //          48 kHz; absent default 437 ~5.0 Hz)
  //   tri  = P[21] ? ~P[20:13] : P[20:13]     (8-bit triangle)
  //   gain = 256 - (tri >> 1) - (tri >> 2)    (66..256, 75% depth)
  //   out  = (cs * gain) >>> 8              (|out| <= |cs|: no clamp)
  // ------------------------------------------------------------------
  reg  [21:0] trem_p;
  assign      trem_inc_k = {3'b000, 1'b1, k1_value[5:0]} << k1_value[7:6];
  wire [9:0]  trem_inc   = k1_present ? trem_inc_k : 10'd437;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)            trem_p <= 22'd0;
    else if (sample_valid) trem_p <= en[4] ? (trem_p + {12'd0, trem_inc}) : 22'd0;
  end

  wire [7:0] trem_tri  = trem_p[21] ? ~trem_p[20:13] : trem_p[20:13];
  wire [8:0] trem_gain = 9'd256 - {2'b00, trem_tri[7:1]} - {3'b000, trem_tri[7:2]};

  // Shared multiplier, operands decoded per step (captured on mul_start):
  // comp at step 1 (s0 x g), dist at step 3 (|cs| x drive), gate at
  // step 5 (tone x gg), the phaser stages at steps 6..9 ((y1 - s) x c),
  // trem at step 11 (cs x gain); every other step drives 0 (no
  // fall-through to any stage's operands). Each step registers its own
  // function of mul_p (comp_out, dist_w, s4_w, ph_y, trem_out) on the
  // mul_done clk.
  reg signed [16:0] mul_a;
  reg        [11:0] mul_b;
  always @(*) begin
    case (step)
      4'd1:    begin mul_a = {s0[15], s0};     mul_b = comp_g;               end
      4'd3:    begin mul_a = {1'b0, dist_mag}; mul_b = dist_drive;           end
      4'd5:    begin mul_a = {tone[15], tone}; mul_b = {3'b000, gate_gg};    end
      4'd6, 4'd7, 4'd8, 4'd9:
               begin mul_a = ph_a;             mul_b = phaser_c;             end
      4'd11:   begin mul_a = {cs[15], cs}; mul_b = {3'b000, trem_gain};  end
      default: begin mul_a = 17'sd0;           mul_b = 12'd0;                end
    endcase
  end
  mult u_mult (.clk(clk), .rst_n(rst_n), .start(mul_start), .a(mul_a), .b(mul_b),
               .p(mul_p), .done(mul_done));

  wire [15:0] trem_out = mul_p[23:8];   // >>> 8; fits 16 bits (gain <= 256)

  wire [15:0] s6_w = en[4] ? trem_out : cs;

  // Step 12 — delay slot (en[5]): identity until Task 22 (Rev B3).
  wire [15:0] s7_w = cs;

  // ------------------------------------------------------------------
  // Chain output (name kept from v1: the audio tooling probes
  // dut.effect_sample).
  // ------------------------------------------------------------------
  reg [15:0] effect_sample;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      step  <= 4'd0;
      mwait <= 1'b0;
      cs    <= 16'd0;
      ytone <= 20'sd0;
      effect_sample <= 16'd0;
      chain_done    <= 1'b0;
    end else begin
      chain_done <= step_adv & (step == CHAIN_LAST);
      if (sample_valid) begin
        step  <= 4'd1;
        mwait <= 1'b0;
      end else begin
        if (mul_start)     mwait <= 1'b1;
        else if (mul_done) mwait <= 1'b0;
        if (step_adv) step <= (step == CHAIN_LAST) ? 4'd0 : step + 4'd1;
      end
      if (step_adv) begin
        case (step)
          4'd1: cs <= s1_w;            // comp (mult)
          4'd2: cs <= s2_w;            // fuzz
          4'd3: cs <= s3_w;            // dist (mult)
          4'd4: ytone <= ytone + tone_s[19:0];   // tone (every frame)
          4'd5: cs <= s4_w;            // gate (mult)
          4'd10: cs <= s5_w;           // phaser mix (stages 6..9: ph_* above)
          4'd11: cs <= s6_w;           // trem (mult)
          4'd12: effect_sample <= s7_w;  // delay slot (identity stub)
          default: ;
        endcase
      end
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
  // (step 12, 116 clk after the sample_valid edge), once per output
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

  // uo[7] = dp clip LED (peak-hold); uo[6:4] reserved, driven 0; uo[3] =
  // 595 RCLK (idle 0, driven from Rev B2); uo[2:0] = SPI idle for the Rev B
  // PSRAM: SCK = 0, CS = 1 (deselected), MOSI = 0
  assign uo_out  = {dp_q, 4'b0000, 3'b010};

  // [7]=LRCLK, [6]=BCLK, [5:2]=0 (knob discharge
  // level, enabled only while the reader discharges that pin), [1]=DAC data,
  // [0]=MCLK = clk (512 fs)
  assign uio_out = {lrclk, bclk, 1'b0, 3'b000, dac_data, clk};
  // uio[7:6] + uio[1:0] outputs; uio[5:2] = knob pins K4..K1, outputs
  // only while discharging (one at a time, round robin)
  assign uio_oe  = {2'b11, k4_drv, k3_drv, k2_drv, k1_drv, 2'b11};

  // List all unused inputs to prevent warnings
  // (ui[7:1] = switches and ui[0] = ADC data are live; uio[7:6]/[1:0]
  // are clock/data outputs, their input side is unused; uio[5:2] feed the
  // knob reader K1..K4. K3 sets the dist drive; K4 is not
  // consumed yet; the multiplier's fraction bits below the >>> 8 are
  // unused; the phaser's 3t low bits fall off the >> 2, and d's bit 13 is
  // always 0; the mix floor drops ph_mix[0])
  wire _unused = &{ena, uio_in[7:6], uio_in[1:0], en[5], gate_rem[1:0],
                   k4_value, k4_present,
                   mul_p[7:0], trem_tri[0], ph_v3[1:0], ph_dx[13], ph_dx[1:0], ph_mix[0], 1'b0};

endmodule

`default_nettype wire
