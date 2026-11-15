# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-FileCopyrightText: © 2026 Thomas McDonagh
# SPDX-License-Identifier: Apache-2.0
# Internal-state probes are RTL-only (guarded getattr — names may vanish
# in the flat GL netlist); port-level assertions carry the GL coverage.

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

    # Reset state: DAC data low, uo dark — dp clip LED (uo[7]) reset low,
    # uo[6:0] reserved for v3 and driven 0
    assert dut.uio_out[1].value == 0, "DAC data not low after reset"
    assert int(dut.uo_out.value) == 0x00, \
        "uo must read 0 after reset (dp dark, uo[6:0] reserved driven 0)"

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

    # DIP enables all on (en = 0x3F), zero input: every stage maps 0 to 0
    # (the stub slots are identity), so DAC data stays low (v3: ui[6:1]
    # are the effect enables; ui[7] reserved Rev B, driven 0 here).
    dut.ui_in.value = 0b0111_1110  # en[5:0] = 111111, ui[7] = 0 (reserved)
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
    """en = 0 (DIP all-off — the "digital clean" mode) echoes samples
    through the full pipeline, one frame later: +100, -100, and -32768
    (saturation edge) must all round-trip."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0  # en = 0: DIP all-off -> identity chain
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
    frame with a clean (zero) input must be silent, uo dark."""
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
    assert int(dut.uo_out.value) == 0x00, \
        "uo must read 0 after reset (dp dark, uo[6:0] reserved driven 0)"



# ui byte layout (v3 task 1): ui[0]=I2S data (driven by the helper),
# ui[6:1]=DIP effect enables en[5:0] (en[0]=comp .. en[5]=delay, bit
# order = chain order), ui[7] reserved (Rev B delay character) — driven 0.
# uio byte layout: params P0..P2 on uio[4:2] — P0=uio[2] (v3 fuzz octave
# blend, unused until task 4 — the v2.2 dist-knee role is retired and the
# pin is trapped in _unused), P1=uio[3] + P2=uio[4] = trem rate bits
# k={uio[4],uio[3]}.
# uio[5] is reserved (Rev B PSRAM MISO): the v2.2 trem depth bit is gone —
# the trem span is a fixed constant now.

@cocotb.test()
async def test_dist_softknee(dut):
    """Dist stage (en[2]), v2.2 task 4 (kept until v3 task 5's diode knee):
    soft-knee piecewise-linear shaper on the stage input's magnitude —
    unity below the knee, slope 1/2 above it (a soft ceiling in place of
    the interim hard clip). Knee FIXED at T=8192 (soft ceiling 20479),
    below the 16384 fuzz clip, so a stacked fuzz always feeds the shaped
    region — param-free: the v2.2 P0 character bit (uio[2]) is retired
    (P0 becomes the fuzz octave blend in task 4; uio[2] is trapped in
    _unused). Fuzz is off here, so the shaper input is the raw sample —
    sign kept, no rectify. Pinned exact points on the 8192 knee, both
    signs, plus -32768 (mag saturates to 32767, then shapes and keeps
    its sign)."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0b0000_1000  # en[2] dist only (ui[3]); ui[7] = 0 (reserved)
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    async def dist_once(word16):
        await _wait_left_start(dut)
        await i2s_send_and_capture(dut, (word16 & 0xFFFF) << 8)
        await _wait_left_start(dut)
        bits = await i2s_send_and_capture(dut, 0)
        out = tx_data_bits(bits)
        return out - 0x10000 if out & 0x8000 else out

    # T=8192: sub-knee unity; above it T + (a-T)>>1
    assert await dist_once(4096) == 4096, "sub-knee must pass at unity"
    assert await dist_once(16384) == 12288    # 8192 + 8192/2
    assert await dist_once(32767) == 20479    # 8192 + 24575/2 = soft ceiling
    assert await dist_once(-16384) == -12288  # sign reapplied
    assert await dist_once(-32768) == -20479  # mag sat 32767 -> shape -> negate


@cocotb.test()
async def test_fuzz_rectify(dut):
    """Fuzz stage (en[1]): full-wave rectifier + fixed clip at 16384 —
    the v1 chain value (v2.2: fuzz gets no param bit)."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0b0000_0100  # en[1] fuzz only (ui[2]); ui[7] = 0 (reserved)
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
    """Fuzz + dist stacked (en = 0x6, P0=0): rectify, clip at 16384,
    then the soft-knee shaper (T=8192, below the fuzz clip — the
    shaper always sees the clipped magnitude and shapes it). 10000
    pins the shaped branch reached without the fuzz clip."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0b0000_1100  # en[2:1] fuzz + dist
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
    """Trem stage (en[4]): stepped-gain triangle LFO as built (v1) —
    out = in >>> s, s = LFO gap >> 6 (gains 1, 1/2, 1/4, 1/8). v3 task 1
    params: rate = 2-bit k = {uio[4], uio[3]} = {P2, P1} with the OLD
    exp base 10 - k (k=0..3 -> half-period 2^(10-k) frames ->
    ~23.4/46.9/93.75/187.5 Hz; task 2 re-bases the ladder to the
    musical 2.9-23.4 Hz range). Depth span FIXED 255 (4 gain steps):
    the v2.2 depth bit lived on uio[5], which is the Rev B PSRAM MISO
    pin in the v3 map — span/rshift are constants now. The LFO counter
    value sampled by each output is the post-increment one; frame F's
    input is output in frame F+1 with count F+1 (validated against the
    v1 k=7 sequence). 23 exact values across all four k rates, probed
    at the gain-step boundary counts, plus a sign probe
    and a wrap/direction probe (v1 parity)."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0b0010_0000  # en[4] trem only (ui[5]); ui[7] = 0 (reserved)
    dut.uio_in.value = 0           # k=0 (uio[4:3] = 0) — set per config below
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    # (uio byte, probes) per rate k — v3: k = {uio[4], uio[3]}, so the
    # uio byte is k << 3. Span fixed 255 (rs 8), OLD exp base 10 - k.
    # Each probe is (post-strobe LFO count, input x, expected output):
    # out = x >>> (gap >> 6) with gap = ((cnt << 8) - cnt) >> exp
    # (falling) — counts chosen at the gain-step boundaries so the ramp
    # arithmetic is pinned exactly (gap 63/64, 127/128, 191/192; k=3's
    # 255*cnt >> 7 skips the even gaps, so its pairs pin the step
    # edges). Wrap probes sample post-wrap count 0, rising (max gap
    # 255 -> s=3).
    CONFIGS = [
        # k=0 (exp 10): boundary counts 257/258/515/772
        (0x00, [(257, 16384, 16384), (258, 16384, 8192),
                (515, 16384, 4096), (772, 16384, 2048)]),
        # k=1 (exp 9): boundaries 128/129/258/386, sign probe at 302,
        # wrap probe at 512 (rising starts at max gap 255)
        (0x08, [(128, 16384, 16384), (129, 16384, 8192),
                (258, 16384, 4096), (302, -16384, -4096),
                (386, 16384, 2048), (512, 16384, 2048)]),
        # k=2 (exp 8): boundaries 64/65/129/193, sign probe at 160,
        # wrap probe at 256 (post-wrap count 0, rising, gap 255)
        (0x10, [(64, 16384, 16384), (65, 16384, 8192),
                (129, 16384, 4096), (160, -16384, -4096),
                (193, 16384, 2048), (256, 16384, 2048)]),
        # k=3 (exp 7): boundaries 32/33 (gap 63->65), 64/65 (127->129),
        # 96/97 (191->193), wrap probe at 128 (count 0, rising, gap 255)
        (0x18, [(32, 16384, 16384), (33, 16384, 8192),
                (64, 16384, 8192), (65, 16384, 4096),
                (96, 16384, 4096), (97, 16384, 2048),
                (128, 16384, 2048)]),
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
    """Serial chain enable matrix (v3 task 1): ui[6:1] alone sets en
    (DIP-direct, no preset ROM). For each stage:
    ON applies its effect to known vectors (incl. -32768 — the output
    also proves every DOWNSTREAM stage passes bit-exact when disabled);
    with all en=0 the chain is bit-exact identity and +-32768 round-trip
    (Review Focus 3: no disabled stage may corrupt the sample). The Rev A
    identity-stub slots (comp en[0], phaser en[3], delay en[5]) must pass
    +-full-scale bit-exact when enabled — the enable itself corrupts
    nothing (comp gets its gain ladder in task 3; phaser/delay are Rev
    B). Fuzz + dist stacked feeds the shaper the clipped magnitude (both
    knees sit below the fuzz clip; dist is the v2.2 soft knee — v3 task
    5 replaces it — P0=0)."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    async def stage_once(en_val, word16):
        dut.ui_in.value = en_val << 1        # ui[6:1] = en; ui[7] = 0 (reserved)
        await _wait_left_start(dut)
        await i2s_send_and_capture(dut, (word16 & 0xFFFF) << 8)
        await _wait_left_start(dut)
        bits = await i2s_send_and_capture(dut, 0)
        out = tx_data_bits(bits)
        return out - 0x10000 if out & 0x8000 else out

    # trem first (needs the LFO advanced from reset: gap >> 6 = 2 at
    # post-strobe counts 600/603 on the k=0 / fixed-span-255 params)
    dut.ui_in.value = 0b0010_0000             # en[4] trem only
    await _wait_left_start(dut)
    cur = 1
    cur, out = await _trem_probe(dut, cur, 600, 16384)
    assert out == 4096, f"trem ON: 16384 >>> 2, got {out}"
    cur, out = await _trem_probe(dut, cur, 603, -32768)
    assert out == -8192, f"trem ON: -32768 >>> 2, got {out}"

    # fuzz ON: rectify + clip 16384 (-32768 folds and clips; 5000 below)
    assert await stage_once(0x2, -32768) == 16384
    assert await stage_once(0x2, 5000) == 5000

    # dist ON: soft-knee PWL (P0=0, T=8192); 1234 pins the fuzz-off
    # pass-through exact (sub-knee unity), 20000 the shaped branch
    assert await stage_once(0x4, -32768) == -20479
    assert await stage_once(0x4, 1234) == 1234
    assert await stage_once(0x4, 20000) == 14096

    # stub slots ON (comp en[0], phaser en[3], delay en[5] — Rev A
    # identity stubs): each must pass its input bit-exact when enabled,
    # through the stub slot AND every disabled stage (the old crush
    # vectors' role in the matrix; the crush slot itself is gone). All
    # three stubs stacked is identity too.
    assert await stage_once(0x1, -32768) == -32768    # comp stub
    assert await stage_once(0x1, 32767) == 32767
    assert await stage_once(0x8, -32768) == -32768    # phaser stub
    assert await stage_once(0x8, 32767) == 32767
    assert await stage_once(0x20, -32768) == -32768   # delay stub
    assert await stage_once(0x20, 32767) == 32767
    assert await stage_once(0x29, -32768) == -32768   # comp+phaser+delay
    assert await stage_once(0x29, 32767) == 32767

    # fuzz + dist stacked: rectify + clip 16384, then shape at the
    # 8192 knee (soft-knee dist replaces the v1 x3 tail)
    assert await stage_once(0x6, -20000) == 12288
    assert await stage_once(0x6, 5000) == 5000
    assert await stage_once(0x6, 20000) == 12288

    # all stages OFF: bit-exact identity, +-full-scale through
    assert await stage_once(0x0, 32767) == 32767
    assert await stage_once(0x0, -32768) == -32768


@cocotb.test()
async def test_clip_dp(dut):
    """dp clip LED (uo[7]), v2.2 task 6: peak-hold on output saturation.
    hit = (effect_sample == 0x7FFF) || (effect_sample == 0x8000)
    (0x7FFF = 0_111111111111111, 0x8000 = 1_000000000000000 — bitwise
    complements, so no single equality covers both), evaluated once per
    output frame on sample_valid — the same edge env clocks
    on, one clk after the strobe latched the frame's input, so the
    compare sees exactly the value the TX shift sends for the frame
    (en=0 here: effect_sample IS sample_in). Hold: a hit lights dp_q
    and reloads an 11-bit counter to 0; without a hit the counter runs
    to 2047 (~2048 frames ~ 43 ms at 48 kHz) and clears dp_q on the
    wrap edge; a new hit mid-hold restarts it (peak-hold).
    Full-scale is driven with en=0 (DIP all-off — digital clean): dist never
    emits full-scale (soft ceiling 20479) and fuzz clips at 16384, but the
    identity chain passes sample_in = 0x7FFF exactly. Phases, DIP all-off
    throughout:
      - 100 mid-level (0x1000) frames: dp stays dark (the LED is
        sticky for ~2048 frames, so one end-of-phase probe covers all
        100 frames);
      - ONE 0x7FFF frame: dp still dark right after the drive (the hit
        clocks at the frame's sample_valid edge, early in the right
        half), lit by the end of that frame; uo[6:0] (reserved, driven
        0) stay 0;
      - input returns to 0x1000: dp holds on its own — still lit 1000
        frames later, and at hold-frame 2040 (a half-length hold would
        already be dark; the clear comes at 2048);
      - cleared by clip-frame + 2048 + 64. Frames after the return
        frame advance in bulk (ui[0] idles low — silence, exactly as
        non-hitting as 0x1000; the hold is what's pinned)."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0     # en=0: DIP all-off -> identity (digital clean)
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    def dp():
        return int(dut.uo_out[7].value)

    def seg_ok():
        # uo[6:0] are reserved for v3 and driven 0: the dp must never
        # disturb them — PORT-level assert, active in GL too
        return (int(dut.uo_out.value) & 0x7F) == 0

    await _wait_left_start(dut)   # start of frame 0

    # 100 mid-level frames (frames 0..99): dp never lights
    for _ in range(100):
        await i2s_send_and_capture(dut, 0x1000 << 8)
        await ClockCycles(dut.clk, 256)
    await Timer(1, unit="ns")
    assert dp() == 0, "mid-level frames must never light the dp"
    assert seg_ok(), "uo[6:0] must stay 0 (reserved) with the dp dark"

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
    assert seg_ok(), "uo[6:0] must stay 0 during the clip (reserved)"

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
    assert seg_ok(), "uo[6:0] must still read 0 after the dp clears"
