# SPDX-FileCopyrightText: © 2026 Thomas McDonagh
# SPDX-License-Identifier: Apache-2.0
"""Independent golden model of the effect chain (written from the spec/plan
formulas, not by executing the RTL). One output per input frame, signed
16-bit ints. wav_tool.py `check` compares the Verilator sim against it
bit-exactly; `model` renders listening WAVs from it.

Chain order: comp -> fuzz (+octave) -> dist -> phaser (stub) -> trem ->
delay (stub). en bit i enables stage i (en[0]=comp .. en[5]=delay).
"""


def sat16(v):
    return max(-32768, min(32767, v))


def smag(x):
    """Saturating |x|."""
    return 32767 if x == -32768 else abs(x)


class Env:
    """Peak-follower envelope: fast attack (average toward peak), slow
    release. The post-update value is what the chain sees on this frame."""

    def __init__(self):
        self.v = 0

    def step(self, x):
        m = smag(x)
        if m > self.v:
            self.v = (self.v + m) >> 1
        else:
            self.v -= (self.v >> 8) + (1 if self.v else 0)
        return self.v


class KnobRC:
    """RC knob reader smoothing (RTL knob_rc.v), one call per measurement.
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


def fuzz(s, env, oct):
    a = smag(s)
    t = min(32767, 13107 + (env >> 2)) if s < 0 else 19660
    shaped = min(a, t)
    if oct:
        return min(32767, shaped + a)
    return -shaped if s < 0 else shaped


def dist(s):
    """Diode-knee PWL on w = 4*sat_mag(s), no intermediate saturation:
    slope 1 to 8192, 1/4 to 32768, 1/16 above (max 20479)."""
    w = smag(s) << 2
    if w <= 8192:
        o = w
    elif w <= 32768:
        o = 8192 + ((w - 8192) >> 2)
    else:
        o = 14336 + ((w - 32768) >> 4)
    return -o if s < 0 else o


def trem(s, p):
    """(s * gain) >>> 8 (the shared multiplier); |out| <= |s|, no clamp."""
    return (s * trem_gain(p)) >> 8


# Switch-debounce settle (RTL: 2-flop sync -> deb_div, an 8-bit counter on
# sample_strobe; every 256th strobe is a tick that samples the switches,
# and sw updates when two consecutive ticks agree, from reset-0 state).
# Strobe n (n = 1, 2, ...) latches input frame n-1, so tick 1 is strobe 256
# (frame 255, sample differs from the reset-0 previous tick, sw unchanged)
# and tick 2 is strobe 512 (frame 511): sw updates on that strobe's clock
# edge, the same edge that latches frame 511's sample, so frame 511 is the
# first output computed with the switch values. Frames before it render
# with en = 0, oct = 0 (switches held constant from reset).
EN_SETTLE = 511


def render(xs, en, oct=0, k1=None, k2=None, k3=None, tgb=False):
    """k1 (0..255 or None = absent) sets the trem rate; k2/k3 and tgb
    (tone/gate bypass, model-only) are reserved for later voicing stages.
    en/oct are the switch values held from reset: they take effect at
    frame EN_SETTLE (debounce). The knob value is knob_steady(k1) from
    frame 0: valid because trem only reads it from EN_SETTLE on, long
    after the reader's first measurement (fast startup, ~34 frames)."""
    env = Env()
    cst = Comp()
    inc = trem_inc(k1)
    p = 0
    out = []
    for i, x in enumerate(xs):
        e = env.step(x)
        g = cst.step(x)
        s = x
        en_i, oct_i = (en, oct) if i >= EN_SETTLE else (0, 0)
        # trem phase: steps on sample_valid while enabled, else held at 0
        p = (p + inc) & 0x3FFFFF if en_i & 16 else 0
        if en_i & 1:
            s = comp(s, g)
        if en_i & 2:
            s = fuzz(s, e, oct_i)
        if en_i & 4:
            s = dist(s)
        if en_i & 16:
            s = trem(s, p)
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
