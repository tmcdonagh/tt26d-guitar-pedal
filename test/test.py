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

    # Mode select bits reach the mux (retired mode 7 falls to bypass; sample_in=0)
    dut.ui_in.value = 0b0000_1110  # mode 7, params 0
    await ClockCycles(dut.clk, 1)
    assert dut.uio_out[1].value == 0, "unexpected DAC output in skeleton"


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
    dut.ui_in.value = 0  # bypass
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
    """Mode 0 echoes samples through the full pipeline, one frame later:
    +100, -100, and -32768 (saturation edge) must all round-trip."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0  # mode 0 = bypass
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
    48-consecutive-sample threshold — is ignored; a full 50-frame
    level still counts (sub-detent glitches rejected, real detents
    counted)."""
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
    B settled one hold ahead). uo[7] is the dp placeholder (0 until
    task 6)."""
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


# ui byte layout (interim): ui[0]=I2S data (driven by helper), ui[3:1]=mode
# (also the panel lines {stomp, enc_b, enc_a} since task 2 — the mode decode
# moves off them in task 3), ui[5:4]=bit-depth sel, ui[7:6]=decimation sel

@cocotb.test()
async def test_bitcrush_quantize(dut):
    """Mode 5, no decimation: output = input with low (16-N) bits masked.
    N=16 keeps all, N=10 masks 6, N=8 masks 8."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    async def crush_once(param_byte, word16):
        dut.ui_in.value = param_byte  # ui[0] rewritten by helper
        await _wait_left_start(dut)
        await i2s_send_and_capture(dut, word16 << 8)
        await _wait_left_start(dut)
        bits = await i2s_send_and_capture(dut, 0)
        return tx_data_bits(bits)

    # N=8 (ui[5:4]=10 -> mask 0xFF00), decim=0
    assert await crush_once(0b0_10_101_0, 0x0FFF) == 0x0F00
    # N=10 (ui[5:4]=01 -> mask 0xFFC0)
    assert await crush_once(0b0_01_101_0, 0x0FFF) == 0x0FC0
    # N=16 (ui[5:4]=00 -> no mask)
    assert await crush_once(0b0_00_101_0, 0x0FFF) == 0x0FFF
    # negative values keep sign through masking
    assert await crush_once(0b0_10_101_0, 0xFF9C) == 0xFF00


@cocotb.test()
async def test_bitcrush_decimate(dut):
    """Mode 5 with divide-by-2 sample-hold. The latch phase after mid-frame
    reset is arbitrary, so inject each value twice on consecutive frames;
    regardless of phase, the output stream must show each value repeated
    twice, in order (shifted by 0 or 1 frame)."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0b01_00_101_0  # N=16 (ui[5:4]=00), decim=/2 (ui[7:6]=01)
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    vals = [0x1111, 0x3333, 0x5555, 0x7777, 0xAAAA, 0xCCCC]
    inputs = [v for v in vals for _ in (0, 1)]  # each value twice
    outs = []
    for word16 in inputs:
        await _wait_left_start(dut)
        outs.append(tx_data_bits(await i2s_send_and_capture(dut, word16 << 8)))

    expected = inputs  # held stream = each value twice
    # find the phase: an 8-sample window of outs must equal the held pattern
    ok = any(outs[o:o + 8] == expected[0:8] for o in (1, 2))
    assert ok, f"held pattern not found; outs={[hex(o) for o in outs]}"


@cocotb.test()
async def test_distortion_gain(dut):
    """Mode 2: out = sat16(in * G), G = 1 + ui[5:4] (1..4)."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    async def dist_once(param_byte, word16):
        dut.ui_in.value = param_byte
        await _wait_left_start(dut)
        await i2s_send_and_capture(dut, word16 << 8)
        await _wait_left_start(dut)
        bits = await i2s_send_and_capture(dut, 0)
        out = tx_data_bits(bits)
        return out - 0x10000 if out & 0x8000 else out

    # G=3 (ui[5:4]=10): 1000*3 = 3000
    assert await dist_once(0b00_10_010_0, 1000) == 3000
    # G=4 (ui[5:4]=11): 20000*4 saturates (G max is 4)
    assert await dist_once(0b00_11_010_0, 20000) == 32767
    assert await dist_once(0b00_11_010_0, -20000) == -32768  # spec sat16: clamp to -32768
    # G=1 (ui[5:4]=00): clean pass-through
    assert await dist_once(0b00_00_010_0, 1234) == 1234


@cocotb.test()
async def test_fuzz_rectify(dut):
    """Mode 1: full-wave rectifier (|x|) with parametric clip T
    (ui[5:4]: 00->24576, 01->16384, 10->8192, 11->4096)."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    async def fuzz_once(param_byte, word16):
        dut.ui_in.value = param_byte
        await _wait_left_start(dut)
        await i2s_send_and_capture(dut, word16 << 8)
        await _wait_left_start(dut)
        bits = await i2s_send_and_capture(dut, 0)
        return tx_data_bits(bits)

    # T=24576 (default 00): negative input folds positive, small signal unclipped
    assert await fuzz_once(0b00_00_001_0, -1000) == 1000
    assert await fuzz_once(0b00_00_001_0, 3000) == 3000
    # T=8192 (ui[5:4]=10): |−30000| clips to 8192
    assert await fuzz_once(0b00_10_001_0, -30000) == 8192
    # T=4096 (ui[5:4]=11): |+3000| = 3000 stays below T
    assert await fuzz_once(0b00_11_001_0, 3000) == 3000
    assert await fuzz_once(0b00_11_001_0, -30000) == 4096


@cocotb.test()
async def test_fuzz_dist_chain(dut):
    """Mode 6: rectify, clip at fixed 16384, then saturating x3."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0b00_00_110_0  # mode 6
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

    assert await chain_once(-20000) == 32767   # 20000->clip 16384->x3->sat
    assert await chain_once(5000) == 15000     # 5000->5000->x3
    assert await chain_once(20000) == 32767    # clip then x3


@cocotb.test()
async def test_tremolo(dut):
    """Mode 4: stepped-gain tremolo: out = in >>> s, s = LFO gap >> 6
    (gains 1, 1/2, 1/4, 1/8; min=0 via ui[4]=0). k=7 -> ramp of 8 frames.
    The LFO counter value sampled by the shift is the post-increment one;
    with the partial reset frame consuming count 0 the expected sequence
    (DC 16384 in) is exactly:"""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0b11101000  # mode 4 (ui[3:1]=100), k=7 (ui[7:5]=111), min=0 (ui[4]=0)
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    def gapfall(c): return (c * 255) >> 3
    def grise(c): return (c * 255) >> 3
    def out_for(gap): return 16384 >> (gap >> 6)
    exp_outs = [0,                                   # partial frame: input 0
                out_for(gapfall(2)), out_for(gapfall(3)),
                out_for(gapfall(4)), out_for(gapfall(5)),
                out_for(gapfall(6)), out_for(gapfall(7)),
                out_for(255),                        # count wraps: rising starts at max gap
                out_for(255 - grise(1)), out_for(255 - grise(2)),
                out_for(255 - grise(3)), out_for(255 - grise(4)),
                out_for(255 - grise(5)), out_for(255 - grise(6)),
                out_for(255 - grise(7)),
                out_for(gapfall(0))]                 # wraps again: full gain

    outs = []
    for _ in range(len(exp_outs)):
        await _wait_left_start(dut)
        outs.append(tx_data_bits(await i2s_send_and_capture(dut, 16384 << 8)))
    assert outs == exp_outs, \
        f"tremolo sequence mismatch: got {[hex(o) for o in outs]} exp {exp_outs}"

    # sign preservation: negative DC in, capture one frame later
    await _wait_left_start(dut)
    await i2s_send_and_capture(dut, ((-16384) & 0xFFFF) << 8)
    await _wait_left_start(dut)
    out_neg = tx_data_bits(await i2s_send_and_capture(dut, 0))
    out_neg = out_neg - 0x10000 if out_neg & 0x8000 else out_neg
    assert out_neg == -(16384 >> (gapfall(2) >> 6)), f"negative DC: got {out_neg}"


@cocotb.test()
async def test_bitcrush_n6(dut):
    """Bitcrush N=6 (ui[5:4]=11, mask 0xFC00): deep quantize."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0b0_11_101_0  # mode 5, N=6, decim=0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    async def crush_once(word16):
        await _wait_left_start(dut)
        await i2s_send_and_capture(dut, word16 << 8)
        await _wait_left_start(dut)
        return tx_data_bits(await i2s_send_and_capture(dut, 0))

    assert await crush_once(0x0FFF) == 0x0C00, "N=6 mask 0xFC00"
    assert await crush_once(0xFF9C) == 0xFC00, "negative keeps sign through mask"
