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


def comp(s, env):
    sh = 0 if env >= 24576 else (1 if env >= 16384 else 2)
    return sat16(s << sh)


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
    inc = trem_inc(k1)
    p = 0
    out = []
    for i, x in enumerate(xs):
        e = env.step(x)
        s = x
        en_i, oct_i = (en, oct) if i >= EN_SETTLE else (0, 0)
        # trem phase: steps on sample_valid while enabled, else held at 0
        p = (p + inc) & 0x3FFFFF if en_i & 16 else 0
        if en_i & 1:
            s = comp(s, e)
        if en_i & 2:
            s = fuzz(s, e, oct_i)
        if en_i & 4:
            s = dist(s)
        if en_i & 16:
            s = trem(s, p)
        out.append(s)
    return out
