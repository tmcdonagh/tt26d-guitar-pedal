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
    """Soft-knee PWL, T=8192, slope 1/2."""
    a = smag(s)
    o = 8192 + ((a - 8192) >> 1) if a > 8192 else a
    return -o if s < 0 else o


def trem(s, t):
    ramp = t.tcnt * 255
    gap = (255 - (ramp >> t.exp)) if t.tdir else (ramp >> t.exp)
    return s >> (gap >> 6)


def render(xs, en, oct=0, k1=None, k2=None, k3=None, tgb=False, trem_k=0):
    """k1..k3 (0..255 or None) and tgb (tone/gate bypass, model-only) are
    reserved for later voicing stages. trem_k (0..3, rate 2.93 Hz << k) is
    the current trem rate field, removed once k* pots replace it."""
    env = Env()
    t = Trem(13 - trem_k)
    out = []
    for x in xs:
        t.step()
        e = env.step(x)
        s = x
        if en & 1:
            s = comp(s, e)
        if en & 2:
            s = fuzz(s, e, oct)
        if en & 4:
            s = dist(s)
        if en & 16:
            s = trem(s, t)
        out.append(s)
    return out


def render_params(xs, en, params):
    """Current RTL params byte (uio[4:2]): oct = params & 1, trem rate =
    params >> 1. Removed in task 7."""
    return render(xs, en, oct=params & 1, trem_k=(params >> 1) & 3)
