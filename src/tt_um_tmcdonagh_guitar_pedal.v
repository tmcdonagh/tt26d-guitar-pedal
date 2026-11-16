/*
 * Copyright (c) 2026 Thomas McDonagh
 * SPDX-License-Identifier: Apache-2.0
 *
 * Silicon Guitar Pedal — multi-effect DSP core (v3: 2x2 silicon
 * pedalboard, Rev A). I2S master in/out (codec-agnostic), serial
 * effect chain comp -> fuzz -> distortion -> phaser -> trem -> delay
 * with per-stage enable bypass (comp: env-driven shift-ladder gain,
 * task 3; fuzz: asymmetric clip + octave blend on its chain input,
 * task 4; phaser and delay are identity stubs, Rev B; the v2.2 crush
 * slot is removed, its envelope follower retained as the shared
 * dynamics source).
 * Switches: DIPs on ui[6:1] (en[0]=comp .. en[4]=trem, en[5] = the
 * delay slot, a reserved identity stub; bit order = chain order) and the
 * fuzz octave switch on ui[7], all debounced (v3 task 7: 2-flop sync,
 * two agreeing samples 256 frames apart). uio[4:2] are knob pins (RC
 * inputs, read nowhere until the knob task); uio[5] reserved (SPI MISO).
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
  // true bypass). Comp carries the env-driven gain ladder (task 3);
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

  wire [5:0] en = sw[5:0];   // en[5] (ui[6]) only feeds the delay identity stub

  wire [15:0] s0 = sample_in;

  // ------------------------------------------------------------------
  // Stage 1 — compressor (v3 task 3): env-driven shift-ladder makeup
  // gain toward the fixed reference TH = 16384 — no division, no
  // multiplier, gain in {x1, x2, x4} via arithmetic left shifts on
  // the sign-extended sample; fixed character, no param bit. Reads
  // the shared env block below (the retained v2.2 envelope follower
  // on the PRE-CHAIN input magnitude, one update per audio frame):
  //   env >= 24576 -> x1   (loud: at/above 1.5*TH)
  //   16384..24575 -> x2   (mid: TH .. 1.5*TH)
  //   env  < 16384 -> x4   (quiet: below TH)
  // Decoded on env[14:13] — the top two bits of env's REACHABLE
  // range: env tops out at 32766 (the (env+mag)>>>1 attack rounds
  // down from 32767), so env[15] NEVER sets, the live range is the
  // 15 bits [14:0], and the quadrant size is 8192 — 2'b11 -> env >=
  // 3*8192 = 24576 -> x1, 2'b10 -> [16384, 24576) -> x2, 2'b0x ->
  // env < 16384 -> x4. (The plan line wrote env[15:14] — dead code:
  // 2'b11/2'b10 are unreachable there, gain would pin at x4 forever
  // and the pinned loud probe could never pass; [14:13] is that
  // predicate on the reachable range, giving the ruled x4 boundary
  // at exactly env < 16384.) TH = 16384 provisional (by-ear retune
  // sanctioned). Saturation: s0 <<< gain_sh is exact in an 18-bit
  // sign-extended container (|s0| <= 32768 -> |s0 <<< 2| <= 131072
  // = 2^17, inside the 18-bit signed range with one spare MSB); the
  // shifted value fits 16 bits iff the two bits above [15] replicate
  // the sign (comp_sh[17:16] == {2{comp_sh[15]}}), else clamp to the
  // rails 0x7FFF / 0x8000.
  // ------------------------------------------------------------------
  wire [1:0] gain_sh = (env[14:13] == 2'b11) ? 2'd0
                     : (env[14:13] == 2'b10) ? 2'd1
                                             : 2'd2;

  wire [17:0] comp_sh  = {{2{s0[15]}}, s0} <<< gain_sh;
  wire        comp_ovf = (comp_sh[17:16] != {2{comp_sh[15]}});
  wire [15:0] comp_out = comp_ovf ? (comp_sh[17] ? 16'h8000 : 16'h7FFF)
                                  : comp_sh[15:0];

  wire [15:0] s1 = en[0] ? comp_out : s0;

  // ------------------------------------------------------------------
  // Stage 2 — fuzz (v3 task 4): Fuzz Face-inspired asymmetric clip +
  // octave blend on the stage's CHAIN input s1 — the task-3 routed
  // re-coupling (v2.2's cell-saving trim read the PRE-CHAIN sample_in
  // here, masking comp in every fuzz-on stack, probe-verified en=0x03
  // == en=0x02; fuzz now consumes comp's output, and dist below
  // consumes fuzz's). Input x = s1, signed; constants provisional:
  //   a      = sat_mag(x)  — the saturating-|x| idiom on s1
  //   t_pos  = 19660       (~ +0.6 FS)
  //   t_neg  = sat16(13107 + (env >>> 2))  (~ -0.4 FS at env 0,
  //             relaxing to -21298 at env 32766 — bias-starve
  //             dynamics: the negative clip point eases up with the
  //             input level. env is read POST-update — the same
  //             combinational-into-TX timing as comp, so frame N+1's
  //             output is fuzz(x_N, env_N). env <= 32766 -> env>>>2
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
  wire [15:0] fuzz_mag = (s1 == 16'h8000) ? 16'h7FFF   // saturating |s1|
                       : s1[15] ? (~s1 + 16'd1) : s1;

  wire fuzz_oct = sw[6];   // debounced ui[7]: octave blend

  // negative clip point sat16(13107 + (env >>> 2))
  wire [16:0] fuzz_tn_sum = 17'd13107 + {3'b000, env[15:2]};
  wire [15:0] fuzz_tneg = |fuzz_tn_sum[16:15] ? 16'h7FFF : fuzz_tn_sum[15:0];
  wire [15:0] fuzz_t = s1[15] ? fuzz_tneg : 16'd19660;

  wire [15:0] fuzz_shaped = (fuzz_mag > fuzz_t) ? fuzz_t : fuzz_mag;

  // sat16 negate (clamp unreachable, kept per convention)
  wire [16:0] fuzz_neg = 17'd0 - {1'b0, fuzz_shaped};
  wire        fuzz_neg_ovf = (fuzz_neg[16] != fuzz_neg[15]);
  wire [15:0] fuzz_core = s1[15] ? (fuzz_neg_ovf ? 16'h8000 : fuzz_neg[15:0])
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

  wire [15:0] s2 = en[1] ? fuzz_out : s1;

  // ------------------------------------------------------------------
  // Stage 3 — distortion (v3 task 5): fixed drive x4 into a 3-piece
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
  // s2[15] below). Correct for every enable/octave combination, including
  // fuzz's signed core output with the octave off.
  wire [15:0] dist_mag = (s2 == 16'h8000) ? 16'h7FFF   // saturating |s2|
                       : s2[15] ? (~s2 + 16'd1) : s2;

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
  wire [15:0] dist_out = s2[15] ? (dist_neg_ovf ? 16'h8000 : dist_neg[15:0])
                                : dist_pw;

  wire [15:0] s3 = en[2] ? dist_out : s2;

  // ------------------------------------------------------------------
  // Stage 4 — phaser: identity stub (Rev B allpass stack; the enable
  // is a no-op in Rev A).
  // ------------------------------------------------------------------
  wire [15:0] phaser_out = s3;   // Rev B

  wire [15:0] s4 = en[3] ? phaser_out : s3;

  // ------------------------------------------------------------------
  // Shared dynamics source (v3): the v2.2 auto-lofi envelope follower,
  // retained after the crush slot's removal — the comp gain ladder
  // (task 3) and the fuzz negative clip point (task 4) read it.
  // 16-bit envelope of the PRE-CHAIN input magnitude (mag below, the
  // saturating |sample_in| idiom — env's own source since task 4 moved
  // stage 2 onto its chain input; fix-round ruling — the player's
  // dynamics live in the raw input):
  //   attack (mag > env): env <= (env + mag) >>> 1   — 17-bit add,
  //     >> 1 back to 16; halves the gap, ~12 frames to converge;
  //   release: env <= env - (env>>>8) - (env != 0)   — tau ~256 frames
  //     ~5.3 ms; the 1-LSB floor makes the decay terminate at exactly
  //     0 (the plain subtract-shift stalls anywhere below 256).
  // env tops out at 32766 — env[15] never sets. One update per audio
  // frame on sample_valid. Feeds the comp gain ladder (task 3) and the
  // fuzz negative clip point (task 4) above.
  // ------------------------------------------------------------------
  reg  [15:0] env;
  wire [15:0] mag = (sample_in == 16'h8000) ? 16'h7FFF  // saturating |x|
                    : sample_in[15] ? (~sample_in + 16'd1) : sample_in;
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
  // Stage 5 — tremolo: stepped-gain tremolo as built (v1): out =
  // in >>> s with gain steps 1, 1/2, 1/4, 1/8, s = LFO gap >> 6
  // (gap = distance from full gain), triangle LFO. v3 task 2 rate
  // ladder base: rate k = 0 fixed (exp 13 -> 2^13-frame half-periods ->
  // 2.93 Hz LFO full cycle at 48 kHz) until the knob stage (task 9)
  // returns the rate control; the old rate pins are knob pins now.
  // Depth span FIXED 255 = 4 gain steps (span/rshift are constants).
  // ------------------------------------------------------------------
  wire [3:0] trem_exp = 4'd13;

  wire [7:0] trem_span   = 8'd255;  // constant (see above)
  wire [3:0] trem_rshift = 4'd8;    // span = 2^rshift - 1

  reg  [13:0] trem_cnt;
  reg         trem_dir;   // 0 = falling (gain down), 1 = rising
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      trem_cnt <= 14'd0;
      trem_dir <= 1'b0;
    end else if (sample_strobe) begin
      if (trem_cnt == ((14'd1 << trem_exp) - 14'd1)) begin
        trem_cnt <= 14'd0;
        trem_dir <= ~trem_dir;
      end else begin
        trem_cnt <= trem_cnt + 14'd1;
      end
    end
  end

  // Ramp width derivation (the v2.2 x5-drive lesson: the ramp
  // arithmetic is exactly where a width bug hides): ramp = (cnt<<8)
  // - cnt = cnt*255 with cnt <= 2^13-1 (k=0, the widest half-period
  // — larger k only shrinks the terminal count, never these widths).
  // Then cnt<<8 <= (2^13-1)*256 = 2,096,896 < 2^22 and ramp <=
  // (2^13-1)*255 = 2,087,205 < 2^22: a 22-bit concat/shift/subtract
  // chain cannot truncate at ANY k (k=3's small counter with the
  // large gap shift only ever divides the 22-bit ramp down — falling
  // gap <= 254, rising gap = 255 - that, both 8-bit-safe). The two
  // subtract operands are the same 22-bit zero-extension of cnt, so
  // the whole expression runs at 22 bits with no underflow
  // (256*cnt >= cnt) and no assignment truncation.
  wire [21:0] trem_ramp = ({8'd0, trem_cnt} << trem_rshift) - {8'd0, trem_cnt};
  wire [7:0]  trem_gap = trem_dir ? (trem_span - (trem_ramp >> trem_exp))
                                  : (trem_ramp >> trem_exp);
  wire [15:0] trem_out = $signed(s4) >>> (trem_gap >> 6);

  wire [15:0] s5 = en[4] ? trem_out : s4;

  // ------------------------------------------------------------------
  // Stage 6 — delay/chorus: identity stub (Rev B QSPI-PSRAM block;
  // the enable is a no-op in Rev A).
  // ------------------------------------------------------------------
  wire [15:0] delay_out = s5;   // Rev B

  wire [15:0] s6 = en[5] ? delay_out : s5;

  // ------------------------------------------------------------------
  // Chain output (name kept from v1: the audio tooling probes
  // dut.effect_sample).
  // ------------------------------------------------------------------
  reg [15:0] effect_sample;
  always @* effect_sample = s6;

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
  // 16'h8000) — the two saturation rails (full-scale positive and
  // -32768), which are bitwise complements, so no single equality
  // covers both. Written as the value-equal reduction form (0x7FFF =
  // 0_111111111111111: bit 15 clear, low 15 all ones; 0x8000 =
  // 1_000000000000000: bit 15 set, low 15 all zeros) — one mux of two
  // 15-bit reduction trees instead of two 16-bit comparator expands:
  //   x[15]=0 -> &x[14:0]  (x == 0x7FFF)
  //   x[15]=1 -> ~|x[14:0] (x == 0x8000)
  // The compare is evaluated once per output frame on sample_valid —
  // the same edge env clocks on, one clk after the strobe latched the
  // frame's input — so the detect sees each transmitted sample exactly
  // once (bypass and the comb stages: the very value the TX shift
  // sends next frame). Hold: a hit lights dp_q and reloads
  // the 11-bit counter to 0; otherwise, while lit, the counter runs up
  // to 2047 and clears dp_q on the edge it wraps (2047 + 1 = 0 in 11
  // bits, so the terminal state needs no separate reset path). A hit
  // mid-hold restarts the hold.
  // ------------------------------------------------------------------
  wire hit = effect_sample[15] ? ~|effect_sample[14:0]   // 0x8000
                               :  &effect_sample[14:0];  // 0x7FFF

  reg        dp_q;
  reg [10:0] dp_cnt;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      dp_q   <= 1'b0;
      dp_cnt <= 11'd0;
    end else if (sample_valid) begin
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

  assign uio_out = {lrclk, bclk, 4'b0, dac_data, clk}; // [7]=LRCLK, [6]=BCLK, [1]=DAC data, [0]=MCLK = clk (512 fs)
  assign uio_oe  = 8'b1100_0011; // uio[7:6] + uio[1:0] outputs (LRCLK/BCLK + MCLK/DAC data); uio[5:2] inputs (knob pins K1..K3 on uio[4:2], reserved SPI MISO on uio[5])

  // List all unused inputs to prevent warnings
  // (ui[7:1] = switches and ui[0] = ADC data are live; uio[7:6]/[1:0]
  // are clock/data outputs, their input side is unused; uio[4:2] knob
  // pins are read nowhere until the knob task and uio[5] is the
  // reserved SPI MISO. env feeds the comp ladder and the fuzz clip point)
  wire _unused = &{ena, uio_in, 1'b0};

endmodule

`default_nettype wire
