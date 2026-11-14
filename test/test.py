# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-FileCopyrightText: © 2026 Thomas McDonagh
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, Timer


@cocotb.test()
async def test_skeleton_baseline(dut):
    """Task 1 baseline: reset state, pin wiring, and running frame clock."""
    dut._log.info("Start")

    # ~24.576 MHz (40535 ps exact isn't half-representable at 1 ps precision)
    clock = Clock(dut.clk, 40534, unit="ps", period_high=20267)
    cocotb.start_soon(clock.start())

    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    # Reset state: DAC data low, preset digit "0" on the 7-seg (uo[6:0])
    assert dut.uio_out[1].value == 0, "DAC data not low after reset"
    assert int(dut.uo_out.value) == 0x3F, \
        "preset digit must show 0 after reset (uo[6:0]=0x3F, dp=0)"

    # MCLK output enabled on uio[0] and following clk.
    # Sample away from clock edges (comb outputs need settle time).
    assert dut.uio_oe.value == 0b1100_0011, "uio_oe must enable only uio[7:6] and uio[1:0]"
    vals = []
    toggles = 0
    prev = None
    for _ in range(16):
        await Timer(10134, unit="ps")  # quarter period
        v = int(dut.uio_out[0].value)
        if prev is not None and v != prev:
            toggles += 1
        prev = v
        vals.append(v)
    assert toggles >= 2, f"MCLK must follow clk (sampled {vals})"

    # Frame clock is running: LRCLK (uio[7]) must toggle within one frame (512 clk)
    start = dut.uio_out[7].value
    toggled = False
    for _ in range(600):
        await ClockCycles(dut.clk, 1)
        if dut.uio_out[7].value != start:
            toggled = True
            break
    assert toggled, "LRCLK never toggled — frame counter not running"

    # DIP enables all on (en = 0xF), panel lines idle, zero input: every
    # stage maps 0 to 0, so DAC data stays low (v2.2: no mode mux — ui[7:4]
    # are the effect enables, ui[3:1] the panel lines).
    dut.ui_in.value = 0b1111_0000  # en[3:0] = 1111, encoder idle, stomp up
    await ClockCycles(dut.clk, 1)
    assert dut.uio_out[1].value == 0, "unexpected DAC output with zero input"


# ---------------------------------------------------------------------
# I2S helpers (used by all frame-level tests).
#
# Framing: 512 clk per frame; BCLK (uio[6]) rises mid-slot (clk cycles
# 8k+4) and falls at slot boundaries (8k). Slots 0..31 = left half (WS
# low), 32..63 = right half. Data bits ride slots 1..16, MSB first,
# after the one-slot I2S delay. ADC data in on ui[0], DAC data out on
# uio[1]. All sampling uses a settle Timer to avoid edge races.
# ---------------------------------------------------------------------

CLK_PERIOD_PS = 40534


async def _bclk_edge(dut, want_rising, settle_ns=1):
    """Wait for the next BCLK edge (uio[6] transition), return after settle."""
    prev = int(dut.uio_out[6].value)
    while True:
        await ClockCycles(dut.clk, 1)
        await Timer(settle_ns, unit="ns")
        cur = int(dut.uio_out[6].value)
        if cur != prev:
            if (cur == 1) == want_rising:
                return
            prev = cur
        else:
            prev = cur


async def _wait_left_start(dut):
    """Wait for the next WS falling edge (LRCLK 1->0) = start of a left
    (data) half-frame."""
    prev = int(dut.uio_out[7].value)
    while True:
        await ClockCycles(dut.clk, 1)
        await Timer(1, unit="ns")
        cur = int(dut.uio_out[7].value)
        if prev == 1 and cur == 0:
            return
        prev = cur


async def i2s_send_and_capture(dut, word24):
    """Drive one left-channel frame on ui[0] (24-bit word, slots 1..24)
    and capture the 32-slot left half of the same frame from uio[1].

    Returns the 32 captured bits (index = slot). Called right after a WS
    edge (start of left half)."""
    bits = []
    for slot in range(32):
        # Drive ADC data for this slot before its BCLK rising edge
        if 1 <= slot <= 24:
            bit = (word24 >> (24 - slot)) & 1
        else:
            bit = 0
        dut.ui_in[0].value = bit
        await _bclk_edge(dut, True)  # mid-slot: master samples here
        bits.append(int(dut.uio_out[1].value))
        await _bclk_edge(dut, False)  # slot boundary
    return bits


def tx_data_bits(bits):
    """Extract the 16 transmitted data bits from a captured frame
    (slots 1..16, MSB first)."""
    v = 0
    for k in range(16):
        v = (v << 1) | bits[1 + k]
    return v


async def _trem_probe(dut, cur, count, x=16384):
    """LFO-count-locked capture for the trem tests. With the sim at the
    start of frame `cur` (right after a WS fall; frames < cur may already
    have been driven), advance/drive/capture so the returned value is
    frame `count`'s output — the effect of frame count-1's input x,
    produced with post-strobe LFO count == count (frame F's strobe
    leaves count F+1). Caveat: that count == count correspondence holds
    only before the first LFO wrap — once the counter wraps at 2^exp-1,
    the post-strobe count is count mod 2^exp and the direction has
    flipped (the k=1 CONFIG's count-512 probe samples post-wrap count
    0, rising). Probe counts here stay in the falling direction unless
    they cross a wrap. Consecutive counts reuse the previous capture
    round's drive (it drove the same x). Returns (new cur, out).

    Requires: count >= cur (increasing probes) and, when count == cur,
    the previous round must have driven x on frame count-1."""
    if cur <= count - 1:
        await ClockCycles(dut.clk, 512 * (count - 1 - cur))
        await i2s_send_and_capture(dut, (x & 0xFFFF) << 8)   # drive frame count-1
        await _wait_left_start(dut)                          # start of frame count
    bits = await i2s_send_and_capture(dut, (x & 0xFFFF) << 8)  # capture frame count
    await _wait_left_start(dut)                              # start of frame count+1
    out = tx_data_bits(bits)
    return count + 1, (out - 0x10000 if out & 0x8000 else out)


@cocotb.test()
async def test_bclk_lrclk_ratios(dut):
    """BCLK toggles every 4 clk (period 8); LRCLK toggles once per 256 clk."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    # Poll with quarter-period spacing so no edge race can hide a toggle.
    # One iteration = 1/4 clk period; 1620 iterations ~ 405 clk.
    bclk_rises = 0
    lr_toggles = 0
    prev_b = int(dut.uio_out[6].value)
    prev_l = int(dut.uio_out[7].value)
    for _ in range(1620):
        await Timer(10134, unit="ps")
        b = int(dut.uio_out[6].value)
        l = int(dut.uio_out[7].value)
        if b == 1 and prev_b == 0:
            bclk_rises += 1
        if l != prev_l:
            lr_toggles += 1
        prev_b, prev_l = b, l
    assert 49 <= bclk_rises <= 52, f"BCLK rises in 405 clk: {bclk_rises}"
    assert lr_toggles == 1, f"LRCLK toggles in 405 clk after reset: {lr_toggles}"


@cocotb.test()
async def test_i2s_tx_frame(dut):
    """Sample 0x55AA driven on ui[0] in frame N must appear MSB-first on
    uio[1] slots 1..16 in frame N+1; slot 0 low (I2S delay), slots 17..31 low."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    word24 = 0x55AA << 8  # 24-bit I2S word: 16 data MSBs + 8 zero LSBs

    await _wait_left_start(dut)
    # Pipeline latency: frame N's injected sample is transmitted in
    # frame N+1. So: inject through frame 0, capture during frame 1
    # (injecting the same word again — it only affects frame 2).
    bits0 = await i2s_send_and_capture(dut, word24)
    await _wait_left_start(dut)
    bits1 = await i2s_send_and_capture(dut, word24)

    assert tx_data_bits(bits1) == 0x55AA, \
        f"TX data bits {tx_data_bits(bits1):04x} != 55AA"
    assert bits1[0] == 0, "slot 0 must be low (I2S one-slot delay)"
    assert all(b == 0 for b in bits1[17:32]), \
        f"slots 17..31 must stay low: {bits1[17:32]}"


@cocotb.test()
async def test_stereo_halves_match(dut):
    """I2S-3 fix: left and right halves of frame N must carry the SAME
    (previous) sample — the mono contract in docs/info.md."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0  # en = 0 (identity chain; L=R mono contract)
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    async def send_capture_both_halves(word16):
        """Drive one frame's left half with word16; capture BOTH halves.
        Must be called with the sim at a left-half start (the previous
        call's right-half loop ends exactly there)."""
        lhalf = []
        for slot in range(32):
            dut.ui_in[0].value = (1 <= slot <= 16) and ((word16 >> (16 - slot)) & 1) or 0
            await _bclk_edge(dut, True)
            lhalf.append(int(dut.uio_out[1].value))
            await _bclk_edge(dut, False)
        # right half: no drive needed, just capture slots 33..48
        rhalf = []
        for slot in range(32):
            await _bclk_edge(dut, True)
            rhalf.append(int(dut.uio_out[1].value))
            await _bclk_edge(dut, False)
        return lhalf, rhalf

    await _wait_left_start(dut)
    await send_capture_both_halves(0x0000)   # flush the reset frame
    await send_capture_both_halves(0x1234)   # drive frame: inject 0x1234
    lhalf, rhalf = await send_capture_both_halves(0x0000)  # echo frame
    def data_bits(bits):
        v = 0
        for k in range(16):
            v = (v << 1) | bits[1 + k]
        return v
    assert data_bits(lhalf) == 0x1234, "left half must carry prior frame's input"
    assert data_bits(rhalf) == 0x1234, \
        f"right half must match left (mono), got {data_bits(rhalf):#06x}"


@cocotb.test()
async def test_bypass_loopback(dut):
    """en = 0 (preset 0 ROM all-off, DIP clear, stomp up) echoes samples
    through the full pipeline, one frame later: +100, -100, and -32768
    (saturation edge) must all round-trip."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0  # en = 0: preset 0 + DIP clear -> identity
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    for word16 in (0x0064, 0xFF9C, 0x8000):  # +100, -100, -32768
        word24 = word16 << 8
        await _wait_left_start(dut)
        await i2s_send_and_capture(dut, word24)     # frame N: inject
        await _wait_left_start(dut)
        bits = await i2s_send_and_capture(dut, word24)  # frame N+1: echo
        got = tx_data_bits(bits)
        assert got == word16, \
            f"loopback {word16:#06x} -> {got:#06x}"


@cocotb.test()
async def test_reset_state(dut):
    """After reset (with junk driven on the ADC line), junk may flush
    through for exactly one frame (reset releases mid-frame); the next
    frame with a clean (zero) input must be silent, preset digit 0."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    dut.ui_in[0].value = 1  # junk on the data line during reset
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    await _wait_left_start(dut)
    await i2s_send_and_capture(dut, 0)          # frame A: junk may flush
    await _wait_left_start(dut)
    bits = await i2s_send_and_capture(dut, 0)   # frame B: must be silent
    assert tx_data_bits(bits) == 0, \
        f"second frame after reset must be silent, got {tx_data_bits(bits):#06x}"
    assert int(dut.uo_out.value) == 0x3F, \
        "preset digit must show 0 after reset (uo[6:0]=0x3F, dp=0)"


# ---------------------------------------------------------------------
# Panel control plane (v2.2 task 2): EC11 encoder A=ui[1], B=ui[2] and
# stomp ui[3], sampled once per audio frame (512 clk) on sample_valid.
# The bounce filter accepts a level only after 48 consecutive equal
# frame samples, so tests hold each level 50 frames (a real detent) or
# 47 (a sub-detent glitch, which must be rejected). Quadrature is x2 on
# accepted A edges: rose -> B low +1 / B high -1; fell -> B high +1 /
# B low -1; preset_cnt wraps mod 8. Tests interleave B between A edges
# (a real EC11 never changes both lines on the same frame; the decode
# reads the settled B level at each A edge). The display blanks while
# the bypass latch is engaged.
# ---------------------------------------------------------------------

async def _detent(dut, a, b):
    """Set encoder levels A=ui[1], B=ui[2] and hold 50 frames (512 clk
    each) — long enough for the 48-consecutive-sample bounce filter to
    accept them. Settles 1 ns so comb outputs are readable after."""
    dut.ui_in[1].value = a
    dut.ui_in[2].value = b
    await ClockCycles(dut.clk, 512 * 50)
    await Timer(1, unit="ns")


@cocotb.test()
async def test_encoder_fwd_rev(dut):
    """x2 quadrature on accepted A edges, direction from accepted B:
    rose with B low +1 / B high -1; fell with B high +1 / B low -1
    (controller ruling — standard x2 table). A full forward detent
    (A 0->1->0 with B interleaved 0->1 between the edges, as a real
    EC11 does) counts +2; reversed counts -2; rev from 0 wraps to 7;
    fwd from 7 wraps to 0. B-only steps settle B one hold before the
    A edge that reads it (the panel never sees A and B change on the
    same frame)."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    def pc():
        # tb.v instantiates the design top as user_project
        return int(dut.user_project.u_panel.preset_cnt.value)

    assert pc() == 0, "preset must start at 0 after reset"

    # forward detent (A edges: rose with B low, fell with B high) -> +2
    await _detent(dut, 1, 0)
    assert pc() == 1, "rose with B low must be +1"
    await _detent(dut, 1, 1)                 # B 0->1 (no A edge: no count)
    assert pc() == 1, "B-only step must not count"
    await _detent(dut, 0, 1)
    assert pc() == 2, "fell with B high must be +1"
    await _detent(dut, 0, 0)                 # back to idle (A=B=0)
    assert pc() == 2, "full forward detent must count +2"

    # reversed detent (A edges: rose with B high, fell with B low) -> -2
    await _detent(dut, 0, 1)                 # B lead-in
    assert pc() == 2, "B-only step must not count"
    await _detent(dut, 1, 1)
    assert pc() == 1, "rose with B high must be -1"
    await _detent(dut, 1, 0)                 # B lead-in
    assert pc() == 1, "B-only step must not count"
    await _detent(dut, 0, 0)
    assert pc() == 0, "fell with B low must be -1 (full reverse detent -2)"

    # reverse from 0 wraps to 7 (rose with B high)
    await _detent(dut, 0, 1)                 # B lead-in
    await _detent(dut, 1, 1)
    assert pc() == 7, "rev from 0 must wrap to 7"

    # forward from 7 wraps to 0 (fell with B high; B already settled)
    await _detent(dut, 0, 1)
    assert pc() == 0, "fwd from 7 must wrap to 0"


@cocotb.test()
async def test_encoder_bounce(dut):
    """Bounce filter: A high for 47 frames — one short of the
    48-consecutive-sample threshold — is ignored; a level held exactly
    48 frames (the acceptance boundary itself) still counts; a full
    50-frame level also counts (sub-detent glitches rejected, real
    detents counted). The 48-frame hold is frame-locked (started just
    after a WS fall) so its window provably spans ticks f0..f47 — 48
    equal samples — pinning the a_cnt == 46 acceptance edge that the
    47/50 bracket alone leaves ambiguous (Task 2 review carry-in)."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    def pc():
        # tb.v instantiates the design top as user_project
        return int(dut.user_project.u_panel.preset_cnt.value)

    assert pc() == 0, "preset must start at 0 after reset"

    # 47-frame glitch on A: below the 48-sample acceptance threshold
    dut.ui_in[1].value = 1
    await ClockCycles(dut.clk, 512 * 47)
    dut.ui_in[1].value = 0                # back to idle
    await ClockCycles(dut.clk, 512 * 50)
    await Timer(1, unit="ns")
    assert pc() == 0, "47-frame glitch must be rejected"

    # control: a full 50-frame level is accepted (rose, B low -> +1)
    await _detent(dut, 1, 0)
    assert pc() == 1, "50-frame level must be counted"

    # exactly 48 frames: the boundary itself must count. A 1->0 fall
    # with B low counts -1 (pc 1 -> 0). Frame-locked start so the
    # 512*48 window covers exactly the 48 equal samples f0..f47.
    await _wait_left_start(dut)
    dut.ui_in[1].value = 0                # A falls; hold EXACTLY 48 frames
    await ClockCycles(dut.clk, 512 * 48)
    await Timer(1, unit="ns")
    assert pc() == 0, "exact 48-frame hold must be counted (fell, B low: -1)"


@cocotb.test()
async def test_stomp_toggle(dut):
    """Stomp (ui[3]) is a latching toggle on the debounced press edge:
    press -> bypass on and the display goes blank; release changes
    nothing; a second press toggles back (v2.2 errata 2: latching,
    not momentary)."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await Timer(1, unit="ns")

    # tb.v instantiates the design top as user_project
    u_panel = dut.user_project.u_panel

    assert int(u_panel.bypass.value) == 0, "bypass must start clear"
    assert int(dut.uo_out.value) == 0x3F, "preset 0 digit must be visible"

    # press 1: bypass latched, digit blank (uo[6:0] = 0)
    dut.ui_in[3].value = 1
    await ClockCycles(dut.clk, 512 * 50)
    await Timer(1, unit="ns")
    assert int(u_panel.bypass.value) == 1, "press must latch bypass"
    assert (int(dut.uo_out.value) & 0x7F) == 0, "bypass must blank the digit"

    # release: the latch holds (latching, not momentary)
    dut.ui_in[3].value = 0
    await ClockCycles(dut.clk, 512 * 50)
    await Timer(1, unit="ns")
    assert int(u_panel.bypass.value) == 1, "release must not clear the latch"
    assert (int(dut.uo_out.value) & 0x7F) == 0, "digit must stay blank"

    # press 2: toggles back — effect on, digit visible again
    dut.ui_in[3].value = 1
    await ClockCycles(dut.clk, 512 * 50)
    await Timer(1, unit="ns")
    assert int(u_panel.bypass.value) == 0, "second press must toggle back"
    assert int(dut.uo_out.value) == 0x3F, "digit must return with the effect"


@cocotb.test()
async def test_seg_patterns(dut):
    """7-seg patterns (seg[0]=a..seg[6]=g, active high) and the preset
    ROM enable table, walked preset 0..7 with one forward A edge each
    (rose with B low and fell with B high, alternating — +1 per edge,
    B settled one hold ahead). The dp (uo[7]) stays dark here — no
    I2S input is driven, so effect_sample never saturates (dp clip
    LED behavior lives in test_clip_dp)."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await Timer(1, unit="ns")

    # index = preset: segment patterns and ROM enables (provisional)
    seg_pats = [0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07]
    rom_pats = [0b0000, 0b0001, 0b0010, 0b0100,
                0b1000, 0b1100, 0b1101, 0b1111]

    def pc():
        # tb.v instantiates the design top as user_project
        return int(dut.user_project.u_panel.preset_cnt.value)

    def pe():
        return int(dut.user_project.u_panel.preset_en.value)

    assert pc() == 0, "preset must start at 0 after reset"
    assert int(dut.uo_out.value) == 0x3F, "preset 0 must show digit 0"
    assert pe() == rom_pats[0], "preset 0 ROM pattern"

    # walk presets 1..7 one +1 edge at a time, alternating the two
    # forward A edges (rose with B low, fell with B high) with a B-only
    # settle hold between edges — B must be accepted before the A edge
    # that reads it
    for k in range(1, 8):
        if k % 2:                 # odd k: A rises with B low (from A=0, B=0)
            await _detent(dut, 1, 0)
        else:                     # even k: A falls with B high (from A=1, B=1)
            await _detent(dut, 0, 1)
        assert pc() == k, f"step {k}: preset must advance to {k}"
        assert int(dut.uo_out.value) == seg_pats[k], \
            f"preset {k}: uo {int(dut.uo_out.value):#04x} != {seg_pats[k]:#04x}"
        assert pe() == rom_pats[k], \
            f"preset {k}: ROM pattern {pe():#06b}"
        if k < 7:                 # settle B for the next edge (no count)
            await _detent(dut, k % 2, k % 2)


# ui byte layout (v2.2): ui[0]=I2S data (driven by the helper), ui[3:1]=
# panel lines {stomp, enc_b, enc_a} (kept 0 = idle/stomp-up in these
# tests, so preset stays 0 and the ROM contributes nothing), ui[7:4]=DIP
# effect enables en[3:0] (en[0]=fuzz .. en[3]=trem = chain order).
# uio byte layout: uio[5:2]=params P3..P0, all four live as of task 5 —
# P0=uio[2] (dist knee), P1=uio[3] (crush depth base), P2=uio[4] (trem
# rate), P3=uio[5] (trem depth).

@cocotb.test()
async def test_autolofi_env(dut):
    """Crush stage (en[2]), v2.2 task 5: auto-lofi — quantize depth N
    follows the envelope of the PRE-CHAIN input (mag of sample_in; fix-
    round ruling: on |s2| the fuzz clip + dist ceiling cap the
    envelope below the loud threshold in stacked chains, pinning the
    full stack coarse regardless of playing dynamics — the player's
    dynamics live in the raw input). Base from P1 (uio[3]):
      {P1,loud} 11 -> N=8 (mask 0xFF00), 01/10 -> N=6 (0xFC00),
      00 -> N=4 (0xF000)
    The distinguishing sample 0x7F73 maps to 0x7F00 / 0x7C00 / 0x7000
    (N=6 is 0x7C00 = 0x7F73 & 0xFC00 — the plan line's 0x7FC0 is
    unreachable through any AND mask, 0x73 bit 7 is clear).
    Frame N's crush latch masks with the env value AFTER frame N-1's
    update (env and crush_hold clock on the same sample_valid edge; a
    freshly attacked env is visible only one frame later). Pinned by
    the quiet probes: the 0x7F73 probe frame's own attack raises env to
    ~19674 (loud), so a mask that wrongly used the post-update env
    would give 0x7F00, not the expected quiet 0x7C00.
    Per P1 setting, from reset (env provably 0 — the deepest silence):
    100 loud frames of 0x7FFF re-raise env past 16384 (16383 after
    frame 1, 24575 after frame 2, converges to 32766) — attack
    re-raise after silence, Review Focus 4; 400 silent frames decay it
    back below 16384 (to ~6721, crossed at ~frame 176); long silence
    terminates at EXACTLY 0 (the release's 1-LSB floor — plain
    env - (env>>>8) stalls anywhere below 256)."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0b0100_0000  # en[2] crush only; panel idle (preset 0)
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    def env():
        # tb.v instantiates the design top as user_project
        return int(dut.user_project.env.value)

    for p1 in (1, 0):
        dut.uio_in.value = 0x08 if p1 else 0x00   # P1 = uio[3]
        dut.rst_n.value = 0
        await ClockCycles(dut.clk, 10)
        dut.rst_n.value = 1

        await _wait_left_start(dut)
        assert env() == 0, "env must reset to 0"

        # 100 loud frames (0x7FFF). RX only listens in left halves, so
        # each frame = one I2S drive (per-slot bits) + an exact 256-clk
        # right-half skip; ui[0] sits at 0 between drives.
        for _ in range(100):
            await i2s_send_and_capture(dut, 0x7FFF << 8)
            await ClockCycles(dut.clk, 256)
        assert env() >= 16384, "100 loud frames must attack env past 16384"

        # loud probe: 0x7F73 in frame N, captured in frame N+1 (the
        # one-frame I2S pipeline); the latch masked with env after the
        # last 0x7FFF frame (32766 — loud)
        await i2s_send_and_capture(dut, 0x7F73 << 8)
        await ClockCycles(dut.clk, 256)
        bits = await i2s_send_and_capture(dut, 0)
        exp, got = (0x7F00 if p1 else 0x7C00), tx_data_bits(bits)
        assert got == exp, f"P1={p1} loud: {got:#06x} != {exp:#06x}"

        # 400 silent frames (the loud capture frame above drove the
        # first): env releases below 16384 — the quiet probe's latch
        # masks with env ~6721
        await ClockCycles(dut.clk, 256)              # rest of capture frame
        await ClockCycles(dut.clk, 512 * 399)        # 399 more silent frames
        assert env() < 16384, "400 silent frames must decay env below 16384"

        # quiet probe: env_501 ~6721 masks the latch; env crosses back
        # up only at the NEXT frame (the pinned one-frame lag)
        await i2s_send_and_capture(dut, 0x7F73 << 8)
        await ClockCycles(dut.clk, 256)
        bits = await i2s_send_and_capture(dut, 0)
        exp, got = (0x7C00 if p1 else 0x7000), tx_data_bits(bits)
        assert got == exp, f"P1={p1} quiet: {got:#06x} != {exp:#06x}"

    # termination, once after the loop — env's update logic is
    # P1-independent (P1 only reads env through the mask mux), and both
    # iterations leave env attacked to ~19674 by their final probe
    # frame: release reaches exact 0 in 1258 frames; 1400 covers it
    # with margin. The 1-LSB floor is what terminates it (plain
    # env - (env>>>8) stalls anywhere below 256).
    await ClockCycles(dut.clk, 256)              # rest of capture frame
    await ClockCycles(dut.clk, 512 * 1400)
    assert env() == 0, "long silence must decay env to exactly 0"

    # fix-round ruling pin (env source = pre-chain input): with
    # fuzz+dist+crush stacked (en=0x7), the chain caps |s2| at 12288
    # (fuzz clip 16384 -> dist ceiling 8192+4096), forever below the
    # 16384 loud threshold — an s2-tracked envelope could never go
    # loud in the full stack. 20 raw 0x7FFF frames drive env to 32766
    # (an s2-tracked env would converge to 12288, quiet). The probe
    # word 12000 stays under the fuzz clip so its chain output is
    # off-grid: s2 = dist(12000) = 8192 + 3808/2 = 10096 = 0x2770;
    # P1=0 loud -> N=6: 0x2770 & 0xFC00 = 0x2400 (an s2-tracked quiet
    # N=4 would give 0x2000).
    dut.ui_in.value = 0b0111_0000            # en[2:0] fuzz+dist+crush
    dut.uio_in.value = 0x00                  # P1=0 (loud -> N=6)
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await _wait_left_start(dut)
    for _ in range(20):
        await i2s_send_and_capture(dut, 0x7FFF << 8)
        await ClockCycles(dut.clk, 256)
    assert env() >= 16384, \
        "raw-tracked env must go loud under the fuzz+dist ceiling"
    await i2s_send_and_capture(dut, 12000 << 8)
    await ClockCycles(dut.clk, 256)
    bits = await i2s_send_and_capture(dut, 0)
    got = tx_data_bits(bits)
    assert got == 0x2400, f"stacked fuzz+dist+crush loud: {got:#06x} != 0x2400"


@cocotb.test()
async def test_dist_softknee(dut):
    """Dist stage (en[1]), v2.2 task 4: soft-knee piecewise-linear
    shaper on the stage input's magnitude — unity below the knee, slope
    1/2 above it (a soft ceiling in place of the interim hard clip).
    Knee T from the P0 character bit (uio[2]): 0 -> 8192 (soft ceiling
    20479), 1 -> 4096 (ceiling 18431); both below the 16384 fuzz clip,
    so a stacked fuzz always feeds the shaped region. Fuzz is off here,
    so the shaper input is the raw sample — sign kept, no rectify.
    Pinned exact points on both knees, both signs, plus -32768 (mag
    saturates to 32767, then shapes and keeps its sign)."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0b0010_0000  # en[1] dist only; panel idle
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    async def dist_once(p0, word16):
        dut.uio_in.value = 0x04 if p0 else 0x00  # P0 = uio[2]
        await _wait_left_start(dut)
        await i2s_send_and_capture(dut, (word16 & 0xFFFF) << 8)
        await _wait_left_start(dut)
        bits = await i2s_send_and_capture(dut, 0)
        out = tx_data_bits(bits)
        return out - 0x10000 if out & 0x8000 else out

    # P0=0 (T=8192): sub-knee unity; above it T + (a-T)>>1
    assert await dist_once(0, 4096) == 4096, "sub-knee must pass at unity"
    assert await dist_once(0, 16384) == 12288    # 8192 + 8192/2
    assert await dist_once(0, 32767) == 20479    # 8192 + 24575/2 = soft ceiling
    assert await dist_once(0, -16384) == -12288  # sign reapplied
    assert await dist_once(0, -32768) == -20479  # mag sat 32767 -> shape -> negate

    # P0=1 (T=4096): same shape, lower knee (soft ceiling 18431)
    assert await dist_once(1, 8192) == 6144      # 4096 + 4096/2
    assert await dist_once(1, 16384) == 10240    # 4096 + 12288/2
    assert await dist_once(1, 32767) == 18431    # 4096 + 28671/2 = soft ceiling


@cocotb.test()
async def test_fuzz_rectify(dut):
    """Fuzz stage (en[0]): full-wave rectifier + fixed clip at 16384 —
    the v1 chain value (v2.2: fuzz gets no param bit)."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0b0001_0000  # en[0] fuzz only; panel idle
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    async def fuzz_once(word16):
        await _wait_left_start(dut)
        await i2s_send_and_capture(dut, word16 << 8)
        await _wait_left_start(dut)
        bits = await i2s_send_and_capture(dut, 0)
        return tx_data_bits(bits)

    # negative folds positive; small signal stays below the clip
    assert await fuzz_once(-1000) == 1000
    assert await fuzz_once(3000) == 3000
    # |x| > 16384 clips, both input polarities (v1 chain vectors)
    assert await fuzz_once(-20000) == 16384
    assert await fuzz_once(20000) == 16384
    # -32768: |x| saturates to 32767, then clips
    assert await fuzz_once(-32768) == 16384


@cocotb.test()
async def test_fuzz_dist_chain(dut):
    """Fuzz + dist stacked (en = 0x3, P0=0): rectify, clip at 16384,
    then the soft-knee shaper (T=8192, below the fuzz clip — the
    shaper always sees the clipped magnitude and shapes it). 10000
    pins the shaped branch reached without the fuzz clip."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0b0011_0000  # en[1:0] fuzz + dist
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    async def chain_once(word16):
        await _wait_left_start(dut)
        await i2s_send_and_capture(dut, word16 << 8)
        await _wait_left_start(dut)
        bits = await i2s_send_and_capture(dut, 0)
        out = tx_data_bits(bits)
        return out - 0x10000 if out & 0x8000 else out

    assert await chain_once(-20000) == 12288   # -20000 -> clip 16384 -> 8192+4096
    assert await chain_once(5000) == 5000      # below both clip and knee
    assert await chain_once(10000) == 9096     # 10000 -> 8192 + 1808/2
    assert await chain_once(20000) == 12288    # clip 16384, then shape


@cocotb.test()
async def test_tremolo(dut):
    """Trem stage (en[3]): stepped-gain triangle LFO as built (v1) —
    out = in >>> s, s = LFO gap >> 6 (gains 1, 1/2, 1/4, 1/8). v2.2
    params on uio: P2=uio[4] rate, P3=uio[5] depth. The LFO counter
    value sampled by each output is the post-increment one; frame F's
    input is output in frame F+1 with count F+1 (validated against the
    v1 k=7 sequence). 16 exact values across both P2 rates x both P3
    depths, probed at the gain-step boundary counts, plus a sign probe
    and a wrap/direction probe (v1 parity)."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0b1000_0000  # en[3] trem only; panel idle
    dut.uio_in.value = 0           # P2=0, P3=0 (k=0, span 255) — set per config below
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    # (uio byte, probes) per P2 x P3 setting: uio[4]=P2 rate (k=0 ->
    # 2^10-frame ramp/dir ~23.4 Hz, k=1 -> 2^9 ~46.9 Hz), uio[5]=P3
    # depth (0 -> span 255 = 4 gain steps, 1 -> span 127 = 2 steps).
    # Each probe is (post-strobe LFO count, input x, expected output):
    # out = x >>> (gap >> 6) with gap = ((cnt << rs) - cnt) >> exp
    # (falling) — counts chosen at the gain-step boundaries so the ramp
    # arithmetic is pinned exactly (gap 63/64, 127/128, 191/192).
    CONFIGS = [
        # k=0, span 255 (rs 8, exp 10): boundary counts 257/258/515/772
        (0x00, [(257, 16384, 16384), (258, 16384, 8192),
                (515, 16384, 4096), (772, 16384, 2048)]),
        # k=1, span 255 (rs 8, exp 9): boundaries 128/129/258/386, sign
        # probe at 302, wrap probe at 512 (rising starts at max gap 255)
        (0x10, [(128, 16384, 16384), (129, 16384, 8192),
                (258, 16384, 4096), (302, -16384, -4096),
                (386, 16384, 2048), (512, 16384, 2048)]),
        # k=0, span 127 (rs 7, exp 10): s=1 boundary at 516/517
        (0x20, [(256, 16384, 16384), (516, 16384, 16384),
                (517, 16384, 8192), (900, 16384, 8192)]),
        # k=1, span 127 (rs 7, exp 9): s=1 boundary at 258/259
        (0x30, [(128, 16384, 16384), (258, 16384, 16384),
                (259, 16384, 8192), (450, 16384, 8192)]),
    ]

    for uio_byte, probes in CONFIGS:
        dut.uio_in.value = uio_byte
        dut.rst_n.value = 0
        await ClockCycles(dut.clk, 10)
        dut.rst_n.value = 1
        await _wait_left_start(dut)
        cur = 1
        for count, x, exp in probes:
            cur, out = await _trem_probe(dut, cur, count, x)
            assert out == exp, \
                f"uio={uio_byte:#04x} count={count} x={x}: got {out}, exp {exp}"


@cocotb.test()
async def test_chain_enables(dut):
    """Serial chain enable matrix (v2.2 task 3): with preset 0 (ROM
    all-off) and the panel idle, ui[7:4] alone sets en. For each stage:
    ON applies its effect to known vectors (incl. -32768 — the output
    also proves every DOWNSTREAM stage passes bit-exact when disabled);
    with all en=0 the chain is bit-exact identity and +-32768 round-trip
    (Review Focus 3: no disabled stage may corrupt the sample). Fuzz +
    dist stacked feeds the shaper the clipped magnitude (both knees sit
    below the fuzz clip; dist is the task-4 soft knee, P0=0)."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    async def stage_once(en_nibble, word16):
        dut.ui_in.value = en_nibble << 4      # ui[7:4] = en; panel idle
        await _wait_left_start(dut)
        await i2s_send_and_capture(dut, (word16 & 0xFFFF) << 8)
        await _wait_left_start(dut)
        bits = await i2s_send_and_capture(dut, 0)
        out = tx_data_bits(bits)
        return out - 0x10000 if out & 0x8000 else out

    # trem first (needs the LFO advanced from reset: gap >> 6 = 2 at
    # post-strobe counts 600/603 on the k=0 / span-255 default params)
    dut.ui_in.value = 0b1000_0000             # en[3] trem only
    await _wait_left_start(dut)
    cur = 1
    cur, out = await _trem_probe(dut, cur, 600, 16384)
    assert out == 4096, f"trem ON: 16384 >>> 2, got {out}"
    cur, out = await _trem_probe(dut, cur, 603, -32768)
    assert out == -8192, f"trem ON: -32768 >>> 2, got {out}"

    # fuzz ON: rectify + clip 16384 (-32768 folds and clips; 5000 below)
    assert await stage_once(0x1, -32768) == 16384
    assert await stage_once(0x1, 5000) == 5000

    # dist ON: soft-knee PWL (P0=0, T=8192); 1234 pins the fuzz-off
    # pass-through exact (sub-knee unity), 20000 the shaped branch
    assert await stage_once(0x2, -32768) == -20479
    assert await stage_once(0x2, 1234) == 1234
    assert await stage_once(0x2, 20000) == 14096

    # crush ON: auto-lofi depth (task 5) — the mask now follows env,
    # so the vectors pin the state, not just the masking: the raw
    # -32768 words driven in the fuzz/dist sections above attacked env
    # (pre-chain tracked, fix-round ruling) to ~30.7k (>> 16384) and it
    # decays only ~0.4%/frame, so the stage sits LOUD for these probes;
    # P1=0 (uio_in=0 throughout this test) -> N=6 mask 0xFC00: 0x0FFF
    # -> 0x0C00, and -32768 keeps its sign through the mask (0x8000).
    # Depth transitions are pinned by test_autolofi_env; this stays
    # the enable-matrix check.
    assert int(dut.user_project.env.value) >= 16384, \
        "crush vectors require env loud (>= 16384; the -32768 attacks above)"
    assert await stage_once(0x4, 0x0FFF) == 0x0C00
    assert await stage_once(0x4, -32768) == -32768

    # fuzz + dist stacked: rectify + clip 16384, then shape at the
    # 8192 knee (soft-knee dist replaces the v1 x3 tail)
    assert await stage_once(0x3, -20000) == 12288
    assert await stage_once(0x3, 5000) == 5000
    assert await stage_once(0x3, 20000) == 12288

    # all stages OFF: bit-exact identity, +-full-scale through
    assert await stage_once(0x0, 32767) == 32767
    assert await stage_once(0x0, -32768) == -32768


@cocotb.test()
async def test_clip_dp(dut):
    """dp clip LED (uo[7]), v2.2 task 6: peak-hold on output saturation.
    hit = (effect_sample == 0x7FFF) || (effect_sample == 0x8000)
    (0x7FFF = 0_111111111111111, 0x8000 = 1_000000000000000 — bitwise
    complements, so no single equality covers both), evaluated once per
    output frame on sample_valid — the same edge env/crush_hold clock
    on, one clk after the strobe latched the frame's input, so the
    compare sees exactly the value the TX shift sends for the frame
    (en=0 here: effect_sample IS sample_in). Hold: a hit lights dp_q
    and reloads an 11-bit counter to 0; without a hit the counter runs
    to 2047 (~2048 frames ~ 43 ms at 48 kHz) and clears dp_q on the
    wrap edge; a new hit mid-hold restarts it (peak-hold).
    Full-scale is driven with en=0 (bypass): dist never emits
    full-scale (soft ceiling 20479) and fuzz clips at 16384, but the
    identity chain passes sample_in = 0x7FFF exactly. Phases, preset 0
    / DIP clear / stomp up throughout:
      - 100 mid-level (0x1000) frames: dp stays dark (the LED is
        sticky for ~2048 frames, so one end-of-phase probe covers all
        100 frames);
      - ONE 0x7FFF frame: dp still dark right after the drive (the hit
        clocks at the frame's sample_valid edge, early in the right
        half), lit by the end of that frame; uo[6:0] still tracks
        panel_seg (preset-0 digit 0x3F);
      - input returns to 0x1000: dp holds on its own — still lit 1000
        frames later, and at hold-frame 2040 (a half-length hold would
        already be dark; the clear comes at 2048);
      - cleared by clip-frame + 2048 + 64. Frames after the return
        frame advance in bulk (ui[0] idles low — silence, exactly as
        non-hitting as 0x1000; the hold is what's pinned)."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0     # en=0: preset-0 ROM all-off + DIP clear -> identity
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    def dp():
        return int(dut.uo_out[7].value)

    def seg_ok():
        # the dp must never disturb the preset digit: uo[6:0] == panel_seg
        return (int(dut.uo_out.value) & 0x7F) == int(dut.user_project.panel_seg.value)

    await _wait_left_start(dut)   # start of frame 0

    # 100 mid-level frames (frames 0..99): dp never lights
    for _ in range(100):
        await i2s_send_and_capture(dut, 0x1000 << 8)
        await ClockCycles(dut.clk, 256)
    await Timer(1, unit="ns")
    assert dp() == 0, "mid-level frames must never light the dp"
    assert seg_ok(), "uo[6:0] must track panel_seg with the dp dark"

    # frame 100: ONE full-scale frame. The helper returns at the start
    # of the right half, before the strobe edge, so dp is still dark
    # here; the hit clocks at the sample_valid edge (lr_cnt 257) and dp
    # is lit by the end of the frame.
    await i2s_send_and_capture(dut, 0x7FFF << 8)
    await Timer(1, unit="ns")
    assert dp() == 0, "dp must not light before the clip frame's valid edge"
    await ClockCycles(dut.clk, 256)              # rest of the clip frame
    await Timer(1, unit="ns")
    assert dp() == 1, "one full-scale frame must light the dp (uo[7])"
    assert seg_ok() and int(dut.user_project.panel_seg.value) == 0x3F, \
        "7-seg must be unchanged during clip (preset-0 digit still 0x3F)"

    # frame 101: input returns to mid-level — from here the dp must
    # hold with no full-scale anywhere in the input
    await i2s_send_and_capture(dut, 0x1000 << 8)
    await ClockCycles(dut.clk, 256)
    await Timer(1, unit="ns")
    assert dp() == 1, "dp must stay lit right after the input returns to mid-level"

    # frame 1101 = 1000 frames after the return frame: still lit — the
    # hold, not the input, keeps the dp on
    await ClockCycles(dut.clk, 512 * 999)
    await Timer(1, unit="ns")
    assert dp() == 1, "dp must hold lit 1000 frames after the input returns to mid-level"

    # frame 2040: still lit — pins the hold near its full length (the
    # clear comes at clip-frame + 2048 = 2148; 8 frames of slack)
    await ClockCycles(dut.clk, 512 * 939)
    await Timer(1, unit="ns")
    assert dp() == 1, "dp must still hold at hold-frame 2040 (~2048-frame hold)"

    # frame 2212 = clip frame + 2048 + 64: cleared
    await ClockCycles(dut.clk, 512 * 172)
    await Timer(1, unit="ns")
    assert dp() == 0, "dp must clear within 2048+64 frames of the clip frame"
    assert seg_ok(), "uo[6:0] must still track panel_seg after the dp clears"
