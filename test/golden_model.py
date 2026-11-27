# SPDX-FileCopyrightText: © 2026 Thomas McDonagh
# SPDX-License-Identifier: Apache-2.0
"""Independent golden model of the effect chain (written from the spec/plan
formulas, not by executing the RTL). One output per input frame, signed
16-bit ints. wav_tool.py `check` compares the Verilator sim against it
bit-exactly; `model` renders listening WAVs from it.

Chain order: comp -> dist -> tone -> gate -> phaser -> trem -> delay;
tone + gate act only when dist is on. Rev B3 final (2026-11-26): fuzz is
removed; the switches are ui[5:0] in chain order, en bits EN_COMP ..
EN_FREEZE below.
"""
import math

# en bits = debounced ui_in[5:0] (Rev B3 final, switches in chain order)
EN_COMP, EN_DIST, EN_PHASER, EN_TREM, EN_DELAY, EN_FREEZE = 1, 2, 4, 8, 16, 32


def sat16(v):
    return max(-32768, min(32767, v))


def smag(x):
    """Saturating |x|."""
    return 32767 if x == -32768 else abs(x)


class KnobRC:
    """RC knob reader smoothing (RTL knob_rc4.v), one call per measurement.
    A completed measurement of raw (0..255) while not present loads
    acc = raw << 2 and value = raw directly (fast startup); later ones run
    the IIR acc <= acc - (acc >> 2) + raw, s = acc >> 2, and move value to
    s only if |s - value| >= 2 (hysteresis). A timeout clears present
    (value holds)."""

    def __init__(self):
        self.acc = 0
        self.value = 0
        self.present = 0

    def measure(self, raw):
        if not self.present:
            self.acc, self.value = raw << 2, raw
        else:
            self.acc = self.acc - (self.acc >> 2) + raw
            s = self.acc >> 2
            if abs(s - self.value) >= 2:
                self.value = s
        self.present = 1
        return self.value

    def timeout(self):
        self.present = 0


def knob_steady(v):
    """Steady knob value after repeated measurements of raw v from reset,
    simulated exactly (fast startup + IIR + hysteresis). With the fast
    startup the IIR starts at its fixed point, so this is v."""
    k = KnobRC()
    prev = None
    while prev != k.value:
        prev = k.value
        for _ in range(64):
            k.measure(v)
    return k.value


TREM_INC_ABSENT = 437   # K1 absent: ~5 Hz (2^22 / 437 = 9598 frames)


def trem_inc(k1):
    """Per-frame phase increment from the K1 knob value (None = absent)."""
    if k1 is None:
        return TREM_INC_ABSENT
    v = knob_steady(k1)
    return (64 + (v & 63)) << (v >> 6)


def trem_gain(p):
    """22-bit phase -> triangle tri (8-bit) -> gain 256 - tri/2 - tri/4
    (75% depth, 66..256)."""
    t = (p >> 13) & 0xFF
    if (p >> 21) & 1:
        t ^= 0xFF
    return 256 - (t >> 1) - (t >> 2)


# Compressor (v3 task 10): a smoothed 16.8 envelope cenv on the pre-chain
# magnitude, a log-ish level lvl_q (4 steps per octave), a 1.5 dB-step gain
# table above the knee and a slewed gain g applied through the multiplier.
COMP_MANT = (256, 304, 362, 431)   # 2^(k/4) in 8.8, k = 0..3
COMP_KNEE = 44                     # lvl_q of L = 2048 (e = 11, m2 = 0)
COMP_G_RESET = 256                 # unity


def comp_level(cenv):
    """lvl_q = 4*e + m2 of L = cenv >> 8: e = index of L's MSB, m2 = the
    two bits below it (zero-filled when e < 2); 0 for L == 0."""
    L = cenv >> 8
    if L == 0:
        return 0
    e = L.bit_length() - 1
    m2 = (L >> (e - 2)) & 3 if e >= 2 else (L << (2 - e)) & 3
    return 4 * e + m2


def comp_target(cenv):
    """Gain target from the level: 1.5 dB per lvl_q step, ratio 8/5 above
    the knee (red = 5/8 of the overshoot), floored at gq = -8."""
    over = max(0, comp_level(cenv) - COMP_KNEE)
    gq = max(-8, 6 - ((over * 5) >> 3))
    m, sh = COMP_MANT[gq & 3], gq >> 2
    return m << sh if sh >= 0 else m >> -sh


def cenv_release(c):
    """cenv release with the 1-LSB floor: terminates at exactly 0."""
    return c - (c >> 12) - (1 if c else 0)


class Comp:
    """Compressor state, one step per frame on the sample_valid edge from
    the pre-edge values (g slews toward the target of the pre-edge cenv);
    runs whether or not comp is enabled. step() returns the post-edge g,
    the gain the chain applies to this frame's sample."""

    def __init__(self):
        self.cenv = 0
        self.g = COMP_G_RESET

    def step(self, x):
        t = comp_target(self.cenv)
        self.g += (t - self.g) >> 3
        m = smag(x) << 8
        if m > self.cenv:
            self.cenv += (m - self.cenv) >> 5
        else:
            self.cenv = cenv_release(self.cenv)
        return self.g


def comp(s, g):
    """sat16((s * g) >>> 8) — the shared multiplier at step 1."""
    return sat16((s * g) >> 8)


DRIVE_ABSENT = 1024   # K3 absent: x4 (Q4.8)


def drive_q8(k3):
    """Dist drive (Q4.8) from the K3 knob value (None = absent):
    256 + 7*v (x1 .. x7.97), x4 when absent."""
    if k3 is None:
        return DRIVE_ABSENT
    v = knob_steady(k3)
    return 256 + (v << 3) - v


def dist(s, drive_q8=DRIVE_ABSENT):
    """Diode-knee PWL on w = (sat_mag(s) * drive_q8) >> 8 (Q4.8 drive, no
    intermediate saturation; w < 2^18): slope 1 to 8192, 1/4 to 32768, 1/16
    above (max 28.6k at drive x7.97; 20479 at the default x4)."""
    w = (smag(s) * drive_q8) >> 8
    if w <= 8192:
        o = w
    elif w <= 32768:
        o = 8192 + ((w - 8192) >> 2)
    else:
        o = 14336 + ((w - 32768) >> 4)
    return -o if s < 0 else o


# Post-dirt tone + noise gate (v3 task 11). Tone: ytone (16.4) one-pole
# low-pass of the dist-stage output s3, every frame whatever the enables
# are: ytone += ((s3 << 4) - ytone) >> kt, kt = 4 - value2[7:6] (K2 max =
# bright, kt 1), 2 with K2 absent. Gate: frame state on the sample_valid
# edge from pre-edge values, keyed by L = cenv >> 8 of the compressor's
# pre-edge cenv: open needs L >= 256 to open, L < 128 to close (hysteresis);
# gg ramps 0..256 (open: gg + ((256 - gg) >> 2) + 1 capped at 256, closed:
# gg - 1 down to 0). With dist on, out = (tone * gg) >> 8.
GATE_OPEN_L, GATE_CLOSE_L, GATE_MAX = 256, 128, 256
TONE_KT_ABSENT = 2


def tone_kt(k2):
    """Tone shift from the K2 knob value (None = absent)."""
    if k2 is None:
        return TONE_KT_ABSENT
    return 4 - (knob_steady(k2) >> 6)


class ToneGate:
    """Tone filter + gate state. gate() runs on the valid edge with the
    pre-edge cenv (call it before Comp.step); tone() is the chain's step 4
    (every frame), apply() step 5."""

    def __init__(self, kt=TONE_KT_ABSENT):
        self.kt = kt
        self.y = 0
        self.open = 0
        self.gg = 0

    def gate(self, cenv):
        L = cenv >> 8
        if self.open:
            self.gg = min(GATE_MAX, self.gg + ((GATE_MAX - self.gg) >> 2) + 1)
            if L < GATE_CLOSE_L:
                self.open = 0
        else:
            self.gg -= 1 if self.gg else 0
            if L >= GATE_OPEN_L:
                self.open = 1

    def tone(self, s3):
        self.y += ((s3 << 4) - self.y) >> self.kt
        return self.y >> 4

    def apply(self, t):
        return (t * self.gg) >> 8


# Phaser (Rev B1, Task 16): Phase 90-style, 4 first-order allpass stages,
# no feedback, 50/50 dry mix. Its own 22-bit phase pp steps on the
# sample_valid edge while en[3] is on (else held at 0), at the trem rate
# from K1 (shared knob) or PHASER_INC_ABSENT (~1.0 Hz) with K1 absent.
# The stage states run every frame whatever en[3] is (warm on switch-on).
PHASER_INC_ABSENT = 87   # prov: 2^22 / 87 = 48210 frames ~ 1.0 Hz


def phaser_inc(k1):
    """Phaser phase increment: trem_inc(k1) with K1 present, else 87."""
    return PHASER_INC_ABSENT if k1 is None else trem_inc(k1)


def phaser_coef(p):
    """22-bit phase -> allpass coefficient c (Q12, 2049..3840): 12-bit
    triangle t = p[20:9] (inverted when p[21]), v = (3 t) >> 2 (0..3071),
    d = ((1024 + v[9:0]) << v[11:10]) >> 2 (256..2047, exponential in v),
    c = 4096 - d."""
    t = (p >> 9) & 0xFFF
    if (p >> 21) & 1:
        t ^= 0xFFF
    v = (t * 3) >> 2
    d = ((1024 + (v & 1023)) << (v >> 10)) >> 2
    return 4096 - d


class Phaser:
    """Four allpass stages y = sat16(x1 + (((y1 - s) * c) >>> 12)) with
    16-bit states x1[k] (last input) and y1[k] (last output), reset 0;
    the shared multiplier forms (y1 - s) * c at chain steps 6..9."""

    def __init__(self):
        self.x1 = [0] * 4
        self.y1 = [0] * 4

    def step(self, s4, p):
        """One frame on input s4 at phase p: updates every stage state and
        returns the mix (s4 + y3) >> 1 (17-bit sum, floor)."""
        c = phaser_coef(p)
        s = s4
        for k in range(4):
            y = sat16(self.x1[k] + (((self.y1[k] - s) * c) >> 12))
            self.x1[k], self.y1[k] = s, y
            s = y
        return (s4 + s) >> 1


def trem(s, p):
    """(s * gain) >>> 8 (the shared multiplier); |out| <= |s|, no clamp."""
    return (s * trem_gain(p)) >> 8


# Delay (Rev B3, Task 22): PT2399-style echo in the external PSRAM. One
# quantized write + one read per frame, both issued by the engine after
# chain_done (go = chain_done & ok). Per frame N (golden order):
#   dly slews +/-1 toward the K4 target, applied from EN_SETTLE (the
#   settled knob, like every other knob consumer; absent = 16800 = 350 ms
#   *prov*; target = ((64 + (v & 63)) << (v >> 6)) * 28 for v = the steady
#   K4 value, 1792..28448 frames)
#   rd   = (ok && filled) ? the previous frame's read : 0
#   lp  += (rd - lp) >>> 2                                  (16-bit state)
#   mixv = (lp >>> 2) + (lp >>> 3) + (lp >>> 6)              (*prov* ~0.39)
#   out = en5 ? sat16(x + mixv) : x      (x = the trem-stage output s6)
#   wv   = out (the feedback uses the same mix; both are *prov* separately)
#   w    = quant12(wv): sign-magnitude, |wv| with the low 4 bits cleared,
#          sign reapplied (magnitude-symmetric)
#   SPI: write w at word widx; read (widx + 1 - dly) mod 32768; widx += 1
# filled: 1 after FILL_FRAMES writes since ok rose. ok timing (the
# engine's): the probe completes mid-frame 16 (pu 8192 clk + ~436 clk),
# so ok is high for every frame >= 17: the first write is frame 17's
# (at that frame's chain_done), rd_N = the read issued at frame N-1
# (it completes at lr ~56 of frame N, before the sample edge ~lr 135).
# The ring models the testbench PSRAM's LFSR fill (word = bytes 2a, 2a+1)
# so unwritten addresses are bit-exact under the FILL_FRAMES test
# override; on real silicon pre-fill content is undefined.
DELAY_ABSENT = 16800   # *prov* 350 ms


def delay_target(k4):
    """Delay length in frames from the K4 knob value (None = absent)."""
    if k4 is None:
        return DELAY_ABSENT
    v = knob_steady(k4)
    return ((64 + (v & 63)) << (v >> 6)) * 28


def quant12(w):
    """12-magnitude-bit sign-magnitude quantization (low 4 bits cleared)."""
    m = smag(w) & ~0xF
    return -m if w < 0 else m


class Delay:
    """The delay stage's per-frame state (dly, lp, the ring write index,
    the fill counter). step() is one frame in the golden order above."""

    def __init__(self, k4=None, fill=32768, psram=True):
        # Rev B3 final: the delay-enable rising edge reprobes the PSRAM only
        # while NO chip is detected (ok = 0) — a working chip keeps its ring
        # and `filled` across delay toggles. With a chip fitted, ok is high
        # from frame 17 on, so no reprobe ever happens here; without one,
        # every probe fails (ok stays 0).
        self.dly = DELAY_ABSENT
        self.lp = 0
        self.widx = 0
        self.fill_cnt = 0
        self.filled = False
        self.rd_prev = 0
        self.f = 0
        self.fill = fill
        self.psram = psram
        self.tgt = delay_target(k4)
        self.ring = None
        if psram:
            lfsr = 0xACE1
            mem = bytearray(131072)
            for i in range(131072):
                mem[i] = lfsr & 0xFF
                lfsr = ((lfsr >> 1) ^ 0xB400) if (lfsr & 1) else (lfsr >> 1)
            # the engine's rdata is a SIGNED 16-bit word (the audio and the
            # model's LFSR fill alike) — the unwritten addresses must be
            # sign-interpreted to match the RTL
            self.ring = [(((mem[2 * a] << 8) | mem[2 * a + 1]) + 32768) % 65536 - 32768
                         for a in range(32768)]

    def step(self, x, en5, freeze=False):
        """One frame. freeze (Rev B3 final, ui[5]): the ring write word is
        quant12(rd) — the word just read, the loop recirculates unchanged
        and no new audio enters — instead of quant12(out). Independent of
        the delay enable en5 (a loop frozen while bypassed plays when the
        delay comes back on); rd is 0 before `filled`, so freezing an
        unfilled ring holds silence."""
        f = self.f
        self.f += 1
        ok = self.psram and f >= 17
        tgt = self.tgt if f >= EN_SETTLE else DELAY_ABSENT
        # the delay time HOLDS while frozen (a K4 move would otherwise
        # re-slew the read pointer through a ring that only recirculates)
        if not freeze:
            if self.dly < tgt:
                self.dly += 1
            elif self.dly > tgt:
                self.dly -= 1
        rd = self.rd_prev if (ok and self.filled) else 0
        self.lp += (rd - self.lp) >> 2
        mixv = (self.lp >> 2) + (self.lp >> 3) + (self.lp >> 6)
        out = sat16(x + mixv) if en5 else x
        if ok:
            a = self.widx
            self.ring[a] = quant12(rd if freeze else out)
            self.rd_prev = self.ring[(a + 1 - self.dly) % 32768]
            self.widx = (a + 1) % 32768
            self.fill_cnt += 1
            if self.fill_cnt >= self.fill:
                self.filled = True
        return out



# Switch-debounce settle (RTL: 2-flop sync -> deb_div, an 8-bit counter on
# sample_strobe; every 256th strobe is a tick that samples the switches,
# and sw updates when two consecutive ticks agree, from reset-0 state).
# Strobe n (n = 1, 2, ...) latches input frame n-1, so tick 1 is strobe 256
# (frame 255, sample differs from the reset-0 previous tick, sw unchanged)
# and tick 2 is strobe 512 (frame 511): sw updates on that strobe's clock
# edge, the same edge that latches frame 511's sample, so frame 511 is the
# first output computed with the switch values. Frames before it render
# with en = 0 (switches held constant from reset).
EN_SETTLE = 511


class Chain:
    """Whole-chain frame state (env, comp, tone/gate, trem phase) with a
    per-frame step(); render() is this driven by the settled switches.
    Tests that change the enables mid-run drive it directly: silent
    frames leave every stage's output 0, so the enable split inside a
    settle run does not matter.
    Rev B3 final: no fuzz stage; no tuner/595 display."""

    def __init__(self, k1=None, k2=None, k3=None, tgb=False, k4=None,
                 fill=32768, psram=True, reprobe=False):
        self.cst = Comp()
        self.tg = ToneGate(tone_kt(k2))
        self.inc = trem_inc(k1)
        self.pinc = phaser_inc(k1)
        self.ph = Phaser()
        self.pp = 0
        self.dq = drive_q8(k3)
        self.tgb = tgb
        self.p = 0
        self.frame = 0
        self.s3 = 0
        self.dly = Delay(k4, fill, psram)   # reprobe: unused since Rev B3 final (see Delay)

    def step(self, x, en_i):
        """One frame with switches en_i in force. Returns
        (out, s3): the output word and the dist-stage output s3."""
        i = self.frame
        self.frame += 1
        self.tg.gate(self.cst.cenv)          # from the pre-edge cenv
        g = self.cst.step(x)
        s = x
        # trem phase: steps on sample_valid while enabled, else held at 0
        self.p = (self.p + self.inc) & 0x3FFFFF if en_i & EN_TREM else 0
        # phaser phase: the same rule on its own enable
        self.pp = (self.pp + self.pinc) & 0x3FFFFF if en_i & EN_PHASER else 0
        if en_i & EN_COMP:
            s = comp(s, g)
        if en_i & EN_DIST:
            s = dist(s, self.dq)
        s3 = self.s3 = s
        t = self.tg.tone(s)                  # every frame, dirt or not
        if (en_i & EN_DIST) and not self.tgb:
            s = self.tg.apply(t)
        ph = self.ph.step(s, self.pp)        # every frame, warm when off
        if en_i & EN_PHASER:
            s = ph
        if en_i & EN_TREM:
            s = trem(s, self.p)
        # step 12: the delay (Rev B3); the ring write + read are part of
        # the same frame (the engine issues them after chain_done)
        s = self.dly.step(s, en_i & EN_DELAY, bool(en_i & EN_FREEZE))
        return s, s3


def switch_frame(at):
    """The first frame processed with switches the tb sets at the start of
    input frame `at` (> 0): the debounce ticks at strobes 256*j (strobe n
    latches frame n-1) and updates sw on the second agreeing tick."""
    m = -(-(at + 1) // 256) * 256
    return m + 255


def render(xs, en, k1=None, k2=None, k3=None, tgb=False, tap=None, k4=None,
           psram=True, fill=32768, sched=()):
    """k1 (0..255 or None = absent) sets the trem and phaser rate; k2 (None = absent)
    the tone shift; k3 the dist drive (drive_q8); k4 (0..255 or None = absent)
    the delay time (Rev B3: the PSRAM echo, delay_target); psram False
    models an absent PSRAM chip (ok = 0: dry output, no ring writes);
    fill = FILL_FRAMES (the tb parameter override in the short-horizon
    tests); tgb (tone/gate bypass, model-only; unused since fuzz was removed).
    en is the switch value held from reset: it takes effect at
    frame EN_SETTLE (debounce). The knob value is knob_steady(k1) from
    frame 0: valid because trem only reads it from EN_SETTLE on, long
    after the reader's first measurement (fast startup, ~34 frames).
    K2 timing approximation: golden applies K2's tone shift from frame 0,
    while the RTL uses the absent default (kt 2) for the first ~34 frames
    until the reader's first measurement. ytone is a state, but the two
    trajectories merge by frame <= 173 on the excerpt, and `check` is
    bit-exact, so a future non-merge would fail loudly.
    K3 is likewise steady from frame 0, and that is exact: the dist stage
    reads K3 only once en[2] takes effect (EN_SETTLE, reader settled by
    then), and dist has no state of its own (unlike ytone, whose
    pre-EN_SETTLE history is a state), so frames before EN_SETTLE never
    see the drive.
    tap (a list) collects each frame's dist-stage output s3, the value
    the tone filter sees (the RTL cs value held at step 4, sampled by test.py _s3_monitor)."""
    ch = Chain(k1, k2, k3, tgb, k4, fill, psram)
    out = []
    # sched: ((at, en_new), ...) switch changes the tb makes at input frame at
    chg = sorted((switch_frame(at), e) for at, e in sched)
    for i, x in enumerate(xs):
        en_i = en if i >= EN_SETTLE else 0
        for f0, e in chg:
            if i >= f0:
                en_i = e
        s, s3 = ch.step(x, en_i)
        if tap is not None:
            tap.append(s3)
        out.append(s)
    return out


def selftest():
    """cenv release from full scale reaches exactly 0 (the 1-LSB floor;
    the plain c - (c >> 12) stalls below 4096). Returns the frame count."""
    c = (32767 << 8) - 31          # attack fixed point at full scale
    n = 0
    while c:
        c = cenv_release(c)
        n += 1
        assert n < 100000, "cenv release does not terminate"
    return n


if __name__ == "__main__":
    import sys
    if sys.argv[1:] == ["--selftest"]:
        print(f"selftest: cenv release full scale -> 0 in {selftest()} frames: PASS")
    else:
        raise SystemExit("usage: golden_model.py --selftest")
