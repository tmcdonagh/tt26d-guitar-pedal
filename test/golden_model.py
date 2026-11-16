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


class Trem:
    """Triangle LFO counter: tcnt counts 0..2^exp-1 per frame, tdir flips."""

    def __init__(self, exp):
        self.exp = exp
        self.tcnt = 0
        self.tdir = 0

    def step(self):
        if self.tcnt == (1 << self.exp) - 1:
            self.tcnt, self.tdir = 0, self.tdir ^ 1
        else:
            self.tcnt += 1


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


def trem(s, t):
    ramp = t.tcnt * 255
    gap = (255 - (ramp >> t.exp)) if t.tdir else (ramp >> t.exp)
    return s >> (gap >> 6)


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
    """k1..k3 (0..255 or None) and tgb (tone/gate bypass, model-only) are
    reserved for later voicing stages. Trem is fixed at k=0 (2.93 Hz)
    until the knob stage. en/oct are the switch values held from reset:
    they take effect at frame EN_SETTLE (debounce)."""
    env = Env()
    t = Trem(13)
    out = []
    for i, x in enumerate(xs):
        t.step()
        e = env.step(x)
        s = x
        en_i, oct_i = (en, oct) if i >= EN_SETTLE else (0, 0)
        if en_i & 1:
            s = comp(s, e)
        if en_i & 2:
            s = fuzz(s, e, oct_i)
        if en_i & 4:
            s = dist(s)
        if en_i & 16:
            s = trem(s, t)
        out.append(s)
    return out
