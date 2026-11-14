/*
 * Copyright (c) 2026 Thomas McDonagh
 * SPDX-License-Identifier: Apache-2.0
 *
 * Silicon Guitar Pedal — multi-effect DSP core.
 * I2S master in/out (codec-agnostic), serial effect chain fuzz ->
 * distortion -> bitcrusher -> tremolo with per-stage enable bypass
 * (v2.2: ring mod, test tone, and the v1 8-way mode mux retired).
 * Enables: DIP switches direct on ui[7:4] (v2.2 amendment: the EC11
 * encoder/preset ROM, 7-seg digit, and stomp bypass are dropped;
 * ui[3:1] and uo[6:0] are reserved for the 2x2 v3 project, and true
 * bypass is wired externally with a 3PDT switch). Params P3..P0 on
 * uio[5:2]; uo[7] = dp clip LED.
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
  // Effect chain + enables (v2.2 amendment): serial fuzz -> dist ->
  // crush -> trem, enable-bypass per stage, en bit order = chain order
  // (en[0]=fuzz, en[1]=dist, en[2]=crush, en[3]=trem) — now DIP
  // switches direct on ui[7:4]. The panel control plane (EC11 encoder,
  // preset ROM, 7-seg digit, latching stomp bypass) was dropped;
  // ui[3:1] are unused inputs (reserved for the v3 project) and bypass
  // is true bypass, wired externally with a 3PDT switch around the
  // pedal (all-off DIP = "digital clean": audio still runs through
  // the ADC -> identity chain -> DAC, a distinct mode from true bypass).
  // ------------------------------------------------------------------
  wire [3:0] en = ui_in[7:4];

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
  // Shaper input magnitude, shared from the chain (area trim (b)):
  // s1 = fuzz_out when fuzz is on — already a non-negative magnitude
  // clamped to [0, 16384], so never the 0x8000 saturation edge — and
  // sample_in otherwise, whose saturating |x| IS the stage-1 mag wire.
  // Value-equal to saturating |s1| in every reachable state; sharing
  // drops a second |x| block (comparator + negater + muxes).
  wire [15:0] dist_mag = en[0] ? fuzz_out : mag;

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
  // Stage 3 — auto-lofi bitcrusher (v2.2 task 5): quantize depth N
  // follows the envelope — quiet passages get crunchier. Peak-ish
  // envelope follower, one update per audio frame on sample_valid —
  // the same enable pattern as crush_hold, so env is always warm the
  // moment the stage switches on. env tracks the PRE-CHAIN input
  // magnitude (mag, stage 1's saturating |sample_in|; fix-round
  // ruling): on |s2| the fuzz clip (16384) + dist ceiling cap the
  // envelope below the loud threshold in stacked chains, pinning the
  // full stack coarse regardless of playing dynamics — the player's
  // dynamics live in the raw input. Standalone crush is unchanged
  // (s2 == sample_in with fuzz/dist off):
  //   attack (mag > env): env <= (env + mag) >>> 1   — 17-bit add,
  //     >> 1 back to 16; halves the gap, ~12 frames to converge;
  //   release: env <= env - (env>>>8) - (env != 0)   — tau ~256 frames
  //     ~5.3 ms; the 1-LSB floor makes the decay terminate at exactly
  //     0 (the plain subtract-shift stalls anywhere below 256).
  // loud = |env[15:14]| (env >= 16384). env tops out at 32766 — env[15]
  // never sets, so a reduction-& here would pin the stage quiet
  // forever; | is the threshold the pinned test values require.
  // Depth {P1 = uio[3], loud}: 11 -> N=8 (mask FF00, 8 bits kept),
  // 01/10 -> N=6 (FC00, 6 bits), 00 -> N=4 (F000, 4 bits).
  // Frame timing: env and crush_hold clock on the SAME sample_valid
  // edge, so frame N's latch masks with the env value after frame
  // N-1's update — a sample's own magnitude never sets its own depth
  // (one-frame env lag, pinned by test_autolofi_env's quiet probes).
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

  wire loud = |env[15:14];                            // env >= 16384

  wire [1:0]  crush_sel  = {uio_in[3], loud};
  wire [15:0] crush_mask = crush_sel == 2'b11 ? 16'hFF00 :  // {11}: N=8
                           crush_sel != 2'b00 ? 16'hFC00 :  // {01,10}: N=6
                                               16'hF000;    // {00}: N=4

  wire [15:0] crushed = s2 & crush_mask;

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
  // the same edge env and crush_hold clock on, one clk after the
  // strobe latched the frame's input — so the detect sees each
  // transmitted sample exactly once (bypass and the comb stages: the
  // very value the TX shift sends next frame; crush's hold register
  // contributes its usual one-frame lag when enabled — the same phase
  // the env mask already pins). Hold: a hit lights dp_q and reloads
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

  assign uo_out  = {dp_q, 7'b000_0000}; // uo[7] = dp clip LED (peak-hold); uo[6:0] reserved for v3, driven 0

  assign uio_out = {lrclk, bclk, 4'b0, dac_data, clk}; // [7]=LRCLK, [6]=BCLK, [1]=DAC data, [0]=MCLK = clk (512 fs)
  assign uio_oe  = 8'b1100_0011; // uio[7:6] + uio[1:0] outputs (LRCLK/BCLK + MCLK/DAC data); uio[5:2] inputs (params P3..P0)

  // List all unused inputs to prevent warnings
  // (uio[5:2] = params P3..P0, all live after task 5; uio[7:6]/[1:0] are
  // clock/data outputs, their input side is unused; ui[3:1] reserved
  // for v3, unconnected after the panel was dropped)
  wire _unused = &{ena, ui_in[3:1], uio_in[7:6], uio_in[1:0], 1'b0};

endmodule

`default_nettype wire
