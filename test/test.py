# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-FileCopyrightText: © 2026 Thomas McDonagh
# SPDX-License-Identifier: Apache-2.0
# Internal-state probes are RTL-only (guarded getattr — names may vanish
# in the flat GL netlist); port-level assertions carry the GL coverage.

import cocotb
import golden_model
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
    flipped (the k=1 CONFIG's count-4096 probe samples post-wrap count
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
# uio byte layout: params P0..P2 on uio[4:2] — P0=uio[2] (fuzz octave
# blend, live from v3 task 4 — the v2.2 dist-knee role is retired),
# P1=uio[3] + P2=uio[4] = trem rate bits k={uio[4],uio[3]}.
# uio[5] is reserved (Rev B PSRAM MISO): the v2.2 trem depth bit is gone —
# the trem span is a fixed constant now.

@cocotb.test()
async def test_comp(dut):
    """Comp stage (en[0]), v3 task 3: env-driven shift-ladder gain —
    comp_out = sat16(s0 <<< gain_sh), the ladder reading the shared env
    block (the retained v2.2 envelope follower on the PRE-CHAIN input
    magnitude; update semantics unchanged — attack (env+mag)>>>1,
    release env-(env>>>8)-(env!=0), one update per frame on
    sample_valid):
      env >= 24576  -> gain_sh 0 (x1, loud)
      16384..24575  -> gain_sh 1 (x2, mid)
      env  < 16384  -> gain_sh 2 (x4, quiet)
    decoded on env[14:13] — the top two bits of env's REACHABLE range
    (env tops out at 32766, env[15] never sets, so the quadrant size
    is 8192 and thresholds land at 16384 / 3*8192 = 24576; a literal
    env[15:14] slice would leave x1/x2 unreachable, pinning x4
    forever). TH = 16384 provisional (by-ear retune sanctioned). No
    param bit (fixed character).

    Frame timing (pinned, differs from the v2.2 crush latch): the
    output transmitted in frame N+1 is comp(x_N, env_N) — env AFTER
    frame N's own valid-edge update. comp is combinational into the
    TX load (frame N+1's first left-half clk, ~255 clk after frame
    N's valid edge), whereas the v2.2 crush latch registered on the
    same valid edge and read env one frame older. The probe word
    0x1000 (mag 4096) only ever RELEASES env at both probes (env
    32766 / 6721 both > 4096), so the pinned values are insensitive
    to that one-frame distinction.

    Phases (frames numbered from the first _wait_left_start):
      - 100 loud frames 0x7FFF (frames 0..99): env attacks 16383 ->
        ... -> 32766, sticky from frame 15 ((32766+32767)>>1 = 32766);
      - loud probe 0x1000 (frame 100), captured frame 101:
        comp(0x1000, env_100 = 32638) — release 32766-127-1, still in
        the x1 quadrant -> 0x1000;
      - 400 silent frames (frames 101..500; the capture frame above
        drove the first, ui[0] idling low): env releases to 6721
        (below 16384 from silent frame 176);
      - quiet probe 0x1000 (frame 501), captured frame 502:
        comp(0x1000, env_501 = 6694) — release 6721-26-1, below 16384
        -> x4 -> 0x4000. This is the port-level proof that env < 16384
        (no hierarchical probe — GL-visible);
      - stability (Review Focus 3): 50 identical mid frames 0x6000
        (frames 503..552), capturing each frame's output mid_outs[i]
        = output of frame 503+i = comp(x_502+i, env_502+i). env
        attacks 6667 -> 24575 by frame 517 and STICKS: mag 24576 >
        env 24575 keeps the attack branch and (24575+24576)>>1 =
        24575 — a sticky fixed point ONE LSB below the 24576 x1
        threshold, the maximally hunting-prone steady state. The last
        10 captures (frames 544..553, env = 24575 throughout) must be
        identical to each other (no per-frame rung hunting) and equal
        sat16(0x6000 <<< 1) = 0x7FFF — the x2 rung (a hunting x1 rung
        would emit 0x6000)."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0b0000_0010  # en[0] comp only (ui[1]); ui[7] = 0 (reserved)
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    await _wait_left_start(dut)                # start of frame 0

    # 100 loud frames (0x7FFF). RX only listens in left halves, so each
    # frame = one I2S drive (per-slot bits) + an exact 256-clk
    # right-half skip; ui[0] sits at 0 between drives.
    for _ in range(100):
        await i2s_send_and_capture(dut, 0x7FFF << 8)
        await ClockCycles(dut.clk, 256)

    # loud probe: 0x1000 in frame 100, captured frame 101 (the
    # one-frame I2S pipeline) — env_100 = 32638 -> x1 passthrough
    await i2s_send_and_capture(dut, 0x1000 << 8)   # frame 100
    await ClockCycles(dut.clk, 256)
    bits = await i2s_send_and_capture(dut, 0)      # frame 101 capture
    got = tx_data_bits(bits)
    assert got == 0x1000, f"loud probe: {got:#06x} != 0x1000 (x1)"

    # 400 silent frames (frame 101, the capture frame above, drove the
    # first): env 32766 -> 6721 (crosses below 16384 at silent frame 176)
    await ClockCycles(dut.clk, 256)                # rest of frame 101
    await ClockCycles(dut.clk, 512 * 399)          # frames 102..500

    # quiet probe: 0x1000 in frame 501, captured frame 502 — env_501 =
    # 6694 -> x4 = 0x4000 (no saturation: 0x4000 < 0x7FFF)
    await i2s_send_and_capture(dut, 0x1000 << 8)   # frame 501
    await ClockCycles(dut.clk, 256)
    bits = await i2s_send_and_capture(dut, 0)      # frame 502 capture
    got = tx_data_bits(bits)
    assert got == 0x4000, f"quiet probe: {got:#06x} != 0x4000 (x4)"

    # stability: 50 identical mid frames 0x6000 (frames 503..552).
    # mid_outs[i] = frame 503+i's output = comp(x_502+i, env_502+i):
    # i=0 echoes the quiet capture frame's 0 input; from i=1 the 0x6000
    # drives attack env 6667 -> 24575 (frame 517, sticky thereafter).
    await ClockCycles(dut.clk, 256)                # rest of frame 502
    mid_outs = []
    for _ in range(50):
        bits = await i2s_send_and_capture(dut, 0x6000 << 8)
        await ClockCycles(dut.clk, 256)
        out = tx_data_bits(bits)
        mid_outs.append(out - 0x10000 if out & 0x8000 else out)
    last10 = mid_outs[-10:]   # frames 544..553, env = 24575 throughout
    assert len(set(last10)) == 1, \
        f"steady mid input must not hunt rungs: last 10 outputs {last10}"
    assert last10[0] == 0x7FFF, \
        f"steady mid rung: {last10[0]:#06x} != 0x7FFF (sat16(0x6000 <<< 1), x2)"


@cocotb.test()
async def test_comp_decay(dut):
    """Shared-env pins for the comp task (v3 task 3): the env update
    semantics are UNCHANGED from v2.2 (attack (env+mag)>>>1, release
    env-(env>>>8)-(env!=0), async reset to 0, one update per frame on
    sample_valid) — pinned here so the comp wiring cannot silently
    disturb the block that task 4's fuzz bias also reads. Hierarchical
    probes are RTL-only (guarded getattr — names may vanish in the
    flat GL netlist; None voids the asserts; test_comp's port-level
    assertions carry the GL coverage):
      - env == 0 after reset;
      - 100 loud frames (0x7FFF) attack env to 32766 (sticky from
        frame 15) — assert env >= 16384;
      - long silence terminates at EXACTLY 0 — the release's 1-LSB
        floor (plain env-(env>>>8) stalls anywhere below 256). From
        32766 the recursion reaches 0 on the 1388th silent frame
        (frame 1487 here; below 256 the decay is env-1 per frame);
        1400 covers it with 12 frames of margin (v2.2 pattern: 1258
        frames from 19658)."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0b0000_0010  # en[0] comp only (ui[1]); ui[7] = 0 (reserved)
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    def env():
        # internal envelope reg — RTL-only probe (same guard as the
        # v2.2 autolofi sites); None voids the env asserts below
        e = getattr(dut.user_project, "env", None)
        return None if e is None else int(e.value)

    await _wait_left_start(dut)   # start of frame 0
    assert env() in (None, 0), "env must reset to 0"

    # 100 loud frames (0x7FFF): one drive + right-half skip per frame
    for _ in range(100):
        await i2s_send_and_capture(dut, 0x7FFF << 8)
        await ClockCycles(dut.clk, 256)
    assert env() is None or env() >= 16384, \
        "100 loud frames must attack env past 16384 (to 32766)"

    # 1400 silent frames (ui[0] idles low): release from 32766 hits
    # exact 0 on the 1388th update
    await ClockCycles(dut.clk, 512 * 1400)
    assert env() in (None, 0), "long silence must decay env to exactly 0"


@cocotb.test()
async def test_dist_diode(dut):
    """Dist stage (en[2]), v3 task 5: fixed drive x4 into a 3-piece diode
    knee. a = sat_mag(x); w = 4a (18-bit, no intermediate saturation);
    w <= 8192: out = w; w <= 32768: out = 8192 + ((w-8192) >> 2) (slope
    1/4, 14336 at w = 32768); else out = 14336 + ((w-32768) >> 4)
    (slope 1/16, max 20479 — sat16 never reached). Sign reapplied from
    x[15]. Fuzz is off, so the input is the raw sample. Exact points
    pin both seams (K1 at a=2048, K2 at a=8192, both sides) and the
    extremes; each is also asserted against golden_model.dist."""
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

    points = [
        (1000, 4000),      # linear x4 below K1
        (4096, 10240),     # w = 16384 -> 8192 + 8192/4
        (2047, 8188),      # just below K1
        (2048, 8192),      # K1 (w = 8192)
        (2049, 8193),      # just above K1: slope 1/4 (8192 + 4/4)
        (8191, 14335),     # w = 32764 -> 8192 + 24572/4
        (8192, 14336),     # K2 (w = 32768)
        (8193, 14336),     # w = 32772 -> 14336 + 4/16 (floor)
        (32767, 20479),    # w = 131068 -> 14336 + 98300/16
        (-32768, -20479),  # mag sat 32767, sign kept
        (-1000, -4000),    # sign reapplied, linear region
        (-16384, -16384),  # w = 65536 -> 14336 + 32768/16 = 16384
    ]
    for x, want in points:
        assert want == golden_model.dist(x), f"pin {x}: {want} != golden"
        got = await dist_once(x)
        assert got == want, f"dist({x}) = {got}, expected {want}"


@cocotb.test()
async def test_fuzz_asym(dut):
    """Fuzz stage (en[1]), v3 task 4: asymmetric clip + env bias on the
    stage's CHAIN input s1 — the mandatory task-3 routed re-coupling
    (v2.2's cell-saving trim read the PRE-CHAIN sample_in here, masking
    comp in every fuzz-on stack; fuzz now consumes comp's output, and
    dist below consumes fuzz's). Input x = s1, signed; provisional
    constants:
      a = sat_mag(x); t = x[15] ? sat16(13107 + (env >>> 2)) : 19660
      (the negative clip point relaxes with the input envelope —
      bias-starve dynamics; env is the shared PRE-CHAIN envelope, read
      post-update: the output captured in frame N+1 is fuzz(x_N, env_N),
      the same combinational-into-TX timing comp pins);
      shaped = a > t ? t : a; fuzz_core = x[15] ? -shaped : shaped.
    P0 = 0 throughout (the plain asym fuzz; the octave blend is
    test_fuzz_octave). Env-dependence is Review Focus 4: the SAME
    negative input under loud vs quiet env gives two distinct pinned
    values (t_neg moves with env). Phases (frames from the first
    _wait_left_start; env values model-derived from the task-3 env
    recursion — the model reproduces task-3's pinned 32766/32638/6721/
    6694 trajectory exactly):
      - 100 loud frames 0x7FFF (0..99): env 32766, sticky from frame 15;
      - corner -32768 (frame 100): env_100 = 32766 (attack from the
        sticky point with sat-mag 32767: (32766+32767)>>1 = 32766) ->
        t_neg = 13107 + 8191 = 21298, the MAX negative clip point ->
        -21298. sat-mag 32767 -> clip -> negate; shaped <= 21298, so
        the sat16 negate can never wrap: no +32768 anomaly;
      - loud -24576 (frame 102): env_102 = 32510 (releases 32766 ->
        32638 over the corner's capture frame, -> 32510 at the probe)
        -> t_neg = 13107 + 8127 = 21234 -> -21234;
      - +24576 (frame 104): +19660 — the positive clip point, env-free;
      - +1000 / -1000 (frames 106/108): sub-clip unity BOTH signs —
        the negative passes through NEGATIVE (the v2.2 rectifier this
        test replaces mapped -1000 -> +1000);
      - 401 silent frames (109..509): env releases to 6510;
      - quiet -24576 (frame 510, driven directly after the bulk advance
        — a _wait_left_start called exactly at a WS fall races its
        first lrclk read): env_510 = attack (6510+24576)>>1 = 15543
        -> t_neg = 13107 + 3885 = 16992 -> -16992 (vs -21234 loud on
        the same input — Review Focus 4's two distinct pins);
      - re-coupling probes on 0x1000 with env in the x4 quadrant:
        en=0x03 (frame 512, env 15421): comp x4 -> s1 = 0x4000 ->
        shaped = 16384 (sub-t_pos) -> +16384 — the fuzz stage receives
        the COMPED amplitude. PRE-FIX value: 4096 (fuzz read
        mag(sample_in); comp masked) — this assert fails there;
        en=0x05 (frame 514, env 15299): dist's fallback branch is
        mag(s1) now -> comp x4 -> 16384 -> diode knee dist(16384) =
        +16384 (pre-fix: the pre-chain 4096 -> dist(4096) = 10240)."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0b0000_0100  # en[1] fuzz only (ui[2]); ui[7] = 0 (reserved)
    dut.uio_in.value = 0           # P0 = 0: the plain asym fuzz
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    await _wait_left_start(dut)                # start of frame 0

    # 100 loud frames (0x7FFF): env attacks to 32766, sticky from frame 15
    for _ in range(100):
        await i2s_send_and_capture(dut, 0x7FFF << 8)
        await ClockCycles(dut.clk, 256)

    # corner probe (inline — the sim is at frame 100's start): -32768
    # with env 32766 -> t_neg 21298 (max) -> -21298
    await i2s_send_and_capture(dut, 0x8000 << 8)   # frame 100
    await _wait_left_start(dut)                    # frame 101
    bits = await i2s_send_and_capture(dut, 0)      # capture frame 101
    out = tx_data_bits(bits)
    got = out - 0x10000 if out & 0x8000 else out
    assert got == -21298, f"corner -32768: {got} != -21298 (env 32766)"

    async def fuzz_once(word16, en_val=0x02):
        # drive word16 in the next frame, capture the frame after
        # (stage_once pattern: en_val lets the re-coupling probes
        # switch the stack below)
        dut.ui_in.value = en_val << 1        # ui[6:1] = en; ui[7] = 0
        await _wait_left_start(dut)
        await i2s_send_and_capture(dut, (word16 & 0xFFFF) << 8)
        await _wait_left_start(dut)
        bits = await i2s_send_and_capture(dut, 0)
        out = tx_data_bits(bits)
        return out - 0x10000 if out & 0x8000 else out

    # loud -24576 (frame 102): env 32510 -> t_neg 21234 -> -21234
    got = await fuzz_once(-24576)
    assert got == -21234, f"loud -24576: {got} != -21234 (env 32510)"

    # +24576 (frame 104): positive clip point 19660 (env-free path)
    got = await fuzz_once(24576)
    assert got == 19660, f"+24576: {got} != 19660 (t_pos)"

    # sub-clip unity, both signs — the negative KEEPS its sign (the
    # v2.2 rectifier mapped -1000 -> +1000)
    got = await fuzz_once(1000)
    assert got == 1000, f"+1000: {got} != 1000 (sub-clip unity)"
    got = await fuzz_once(-1000)
    assert got == -1000, f"-1000: {got} != -1000 (sign kept, no rectify)"

    # 401 silent frames (capture frame 109 + 400 bulk frames 110..509):
    # env releases to 6510. The quiet probe drives DIRECTLY after the
    # bulk advance (task-3's pattern) — fuzz_once's leading
    # _wait_left_start, called exactly at a WS fall, races its first
    # lrclk read and can return a clk later, shifting the word a frame
    # (observed: -16992 via frame 510 either way, but only the direct
    # drive is deterministic).
    await ClockCycles(dut.clk, 256)             # rest of frame 109
    await ClockCycles(dut.clk, 512 * 400)       # frames 110..509

    # quiet -24576 (frame 510): env_510 = attack (6510 + 24576)>>1 =
    # 15543 -> t_neg = 13107 + 3885 = 16992 -> -16992. Same input as
    # the loud probe, distinct pin (Review Focus 4)
    await i2s_send_and_capture(dut, 0xA000 << 8)    # frame 510 (-24576)
    await _wait_left_start(dut)                     # frame 511
    bits = await i2s_send_and_capture(dut, 0)       # capture frame 511
    out = tx_data_bits(bits)
    got = out - 0x10000 if out & 0x8000 else out
    assert got == -16992, f"quiet -24576: {got} != -16992 (env 15543)"

    # re-coupling probe 1 (the mandatory fix): comp+fuzz, env 15421
    # (x4 quadrant) -> s1 = 0x4000 -> shaped = 16384 -> +16384.
    # PRE-FIX: 4096 (fuzz read the pre-chain mag) — this assert fails
    # on the v2.2-trim RTL: the probe's purpose
    got = await fuzz_once(0x1000, en_val=0x03)
    assert got == 16384, \
        f"comp->fuzz re-coupling: {got} != 16384 (pre-fix value 4096)"

    # re-coupling probe 2: comp+dist (fuzz off) — dist's fallback is
    # mag(s1): comp x4 -> 16384 -> dist(16384) = +16384
    # (pre-fix: the pre-chain 4096 -> dist(4096) = 10240)
    got = await fuzz_once(0x1000, en_val=0x05)
    assert got == golden_model.dist(16384) == 16384, \
        f"comp->dist re-coupling: {got} != 16384 (pre-fix value 10240)"


@cocotb.test()
async def test_fuzz_octave(dut):
    """Fuzz octave blend (en[1], v3 task 4): P0 = uio[2] is LIVE —
    fuzz_out = P0 ? sat16(shaped + mag(s1)) : fuzz_core, the 50/50
    blend with the full-wave-rectified copy. sat16 clamps UPPER only
    (shaped >= 0 and mag(s1) >= 0, so the blend is >= 0 under P0=1 in
    every reachable state). Every pinned value here is env-free by construction:
    sub-clip inputs shape to a (< both clip points for any env — the
    negative t_neg >= 13107), and the saturating pins saturate for any
    env (shaped >= 13107 -> shaped + 32767 > 32767). P0 = 1:
      +10000 -> sat16(10000 + 10000) = 20000;
      -10000 -> 20000 — the octave signature: the blend erases the
        sign (the plain core in test_fuzz_asym keeps it: -10000 ->
        -10000 under P0=0);
      +24576 -> sat16(19660 + 24576) = 32767 (upper rail);
      -32768 -> 32767 (shaped = t_neg(env) >= 13107, sat-mag 32767 ->
        saturated for any env).
    P0 = 0 contrast on the same words: -10000 -> -10000 (core),
    +24576 -> 19660."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0b0000_0100  # en[1] fuzz only (ui[2]); ui[7] = 0 (reserved)
    dut.uio_in.value = 0b0000_0100  # P0 = 1 (uio[2]); trem rate bits 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    async def oct_once(word16):
        await _wait_left_start(dut)
        await i2s_send_and_capture(dut, (word16 & 0xFFFF) << 8)
        await _wait_left_start(dut)
        bits = await i2s_send_and_capture(dut, 0)
        out = tx_data_bits(bits)
        return out - 0x10000 if out & 0x8000 else out

    # P0 = 1 (uio[2] set): the octave blend
    got = await oct_once(10000)
    assert got == 20000, f"P0=1 +10000: {got} != 20000 (blend 2a)"
    got = await oct_once(-10000)
    assert got == 20000, f"P0=1 -10000: {got} != 20000 (blend erases sign)"
    got = await oct_once(24576)
    assert got == 32767, f"P0=1 +24576: {got} != 32767 (sat16 upper rail)"
    got = await oct_once(-32768)
    assert got == 32767, f"P0=1 -32768: {got} != 32767 (sat for any env)"

    # P0 = 0 contrast on the same words: the core values
    dut.uio_in.value = 0
    got = await oct_once(-10000)
    assert got == -10000, f"P0=0 -10000: {got} != -10000 (core keeps sign)"
    got = await oct_once(24576)
    assert got == 19660, f"P0=0 +24576: {got} != 19660 (core clip)"


@cocotb.test()
async def test_fuzz_dist_chain(dut):
    """Fuzz + dist stacked (en = 0x6, P0=0): the task-4 asym fuzz feeds
    the v3 diode-knee shaper on the CHAIN value — dist consumes
    mag(s2) with s2 = fuzz_out (signed under P0=0; sign from s2[15]).
    Every expected value is golden_model.dist of the fuzz output. Env
    values are the probe frames' post-update env, model-derived from
    reset 0 (the first probe attacks to half its mag, then the capture
    frames release): -20000 frames the env-biased negative clip
    (env_0 = 10000 -> t_neg = 15607 -> fuzz -15607 -> dist -16189);
    5000 is below the fuzz clip (-> 11144); 10000 is shaped WITHOUT the
    fuzz clip (fuzz passes it at sub-t_pos unity -> 14788); 20000 clips
    at t_pos 19660 -> 17203; and a P0=1 stacking probe pins the
    octave-into-dist path on the direct |s2| shaper input: +10000 ->
    blend sat16(10000+10000) = 20000 (non-negative, so dist's sign
    input s2[15] reads 0) -> +17288."""
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

    assert await chain_once(-20000) == -16189  # env 10000, t_neg 15607 -> w 62428
    assert await chain_once(5000) == 11144     # below the fuzz clip: w 20000
    assert await chain_once(10000) == 14788    # fuzz passes 10000; w 40000
    assert await chain_once(20000) == 17203    # clip 19660; w 78640

    # P0=1 stacking probe (fix round): the octave blend INTO dist —
    # +10000 -> fuzz blend sat16(10000+10000) = 20000 (>= 0, so the
    # shaper's sign input s2[15] = 0) -> dist(20000) = +17288. A
    # shaper magnitude reading fuzz_shaped (10000) instead of the
    # blended output would emit 14788 — pins the octave-into-dist
    # path on the direct saturating |s2| (en[1]=1, P0=1 case).
    dut.uio_in.value = 0b0000_0100  # P0 = 1 (uio[2]); trem rate bits 0
    assert await chain_once(10000) == 17288    # blend 20000; w 80000


@cocotb.test()
async def test_tremolo(dut):
    """Trem stage (en[4]): stepped-gain triangle LFO as built (v1) —
    out = in >>> s, s = LFO gap >> 6 (gains 1, 1/2, 1/4, 1/8). v3 task 2
    rate ladder: rate = 2-bit k = {uio[4], uio[3]} = {P2, P1} with exp
    base 13 - k (k=0..3 -> half-period 2^(13-k) frames -> LFO
    full-cycle rates 2.93/5.86/11.72/23.44 Hz at 48 kHz — the musical
    2.9-23.4 Hz range from the listening session; the interim
    23.4-187.5 Hz v2.2-base ladder read as jitter). Depth span FIXED
    255 (4 gain steps): the v2.2 depth bit lived on uio[5], which is
    the Rev B PSRAM MISO pin in the v3 map — span/rshift are constants
    now. The LFO counter value sampled by each output is the
    post-increment one; frame F's input is output in frame F+1 with
    count F+1 (validated against the v1 k=7 sequence). 31 exact values
    across all four k rates, probed as boundary PAIRS at the gain-step
    band edges, plus a sign probe on the three fast rates and a
    wrap/direction probe on every rate (v1 parity)."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0b0010_0000  # en[4] trem only (ui[5]); ui[7] = 0 (reserved)
    dut.uio_in.value = 0           # k=0 (uio[4:3] = 0) — set per config below
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    # (uio byte, probes) per rate k — v3: k = {uio[4], uio[3]}, so the
    # uio byte is k << 3. Span fixed 255 (rs 8), exp base 13 - k.
    # Each probe is (post-strobe LFO count, input x, expected output):
    # out = x >>> (gap >> 6) with gap = ((cnt << 8) - cnt) >> exp
    # (falling) — counts chosen as boundary PAIRS: the last count of
    # one gain band and the first of the next (gap 63/64, 127/128,
    # 191/192; s = gap>>6 = 0/1/2/3), so the ramp arithmetic is pinned
    # exactly at every band edge (v2.2 lesson: boundary pairs, not end
    # states). Sign probe mid-band at gap 149 (s=2) on the fast rates;
    # wrap probes sample post-wrap count 0, rising (max gap 255 -> s=3).
    CONFIGS = [
        # k=0 (exp 13): boundaries 2056/2057, 4112/4113, 6168/6169,
        # wrap at 8192 (2.93 Hz full cycle)
        (0x00, [(2056, 16384, 16384), (2057, 16384, 8192),
                (4112, 16384, 8192), (4113, 16384, 4096),
                (6168, 16384, 4096), (6169, 16384, 2048),
                (8192, 16384, 2048)]),
        # k=1 (exp 12): boundaries 1028/1029, 2056/2057, 3084/3085,
        # sign probe at 2400, wrap at 4096 (5.86 Hz)
        (0x08, [(1028, 16384, 16384), (1029, 16384, 8192),
                (2056, 16384, 8192), (2057, 16384, 4096),
                (2400, -16384, -4096),
                (3084, 16384, 4096), (3085, 16384, 2048),
                (4096, 16384, 2048)]),
        # k=2 (exp 11): boundaries 514/515, 1028/1029, 1542/1543,
        # sign probe at 1200, wrap at 2048 (11.72 Hz)
        (0x10, [(514, 16384, 16384), (515, 16384, 8192),
                (1028, 16384, 8192), (1029, 16384, 4096),
                (1200, -16384, -4096),
                (1542, 16384, 4096), (1543, 16384, 2048),
                (2048, 16384, 2048)]),
        # k=3 (exp 10): boundaries 257/258, 514/515, 771/772 — the
        # same math as the task-1 interim k=0 (exp base 10), now the
        # ladder top; sign probe at 600, wrap at 1024 (23.44 Hz)
        (0x18, [(257, 16384, 16384), (258, 16384, 8192),
                (514, 16384, 8192), (515, 16384, 4096),
                (600, -16384, -4096),
                (771, 16384, 4096), (772, 16384, 2048),
                (1024, 16384, 2048)]),
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
    identity-stub slots (phaser en[3], delay en[5]) must pass
    +-full-scale bit-exact when enabled — the enable itself corrupts
    nothing (comp is live from task 3; phaser/delay are Rev B). Fuzz +
    dist stacked: the task-4 asym fuzz feeds the shaper mag(s2) (both
    fuzz clip points sit above the dist knees; dist is the v3 diode
    knee, P0=0)."""
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
    # post-strobe counts 5000/5003 on the k=0 / fixed-span-255 params —
    # mid-band of the task-2 ladder's slowest rate, exp base 13)
    dut.ui_in.value = 0b0010_0000             # en[4] trem only
    await _wait_left_start(dut)
    cur = 1
    cur, out = await _trem_probe(dut, cur, 5000, 16384)
    assert out == 4096, f"trem ON: 16384 >>> 2, got {out}"
    cur, out = await _trem_probe(dut, cur, 5003, -32768)
    assert out == -8192, f"trem ON: -32768 >>> 2, got {out}"

    # fuzz ON: the task-4 asym clip — -32768 sat-mags to 32767 and
    # clips at the env-biased negative point: env 30147 at the probe
    # (inherited from the trem phase's LFO drives above) -> t_neg =
    # 13107 + 7536 = 20643 -> -20643; 5000 passes below both clip
    # points at unity
    assert await stage_once(0x2, -32768) == -20643
    assert await stage_once(0x2, 5000) == 5000

    # dist ON: diode-knee PWL, fixed drive x4 (expected values from
    # golden_model.dist); 1234 the linear piece, 20000 the top piece
    assert await stage_once(0x4, -32768) == -20479
    assert await stage_once(0x4, 1234) == 4936
    assert await stage_once(0x4, 20000) == 17288

    # phaser en[3] and delay en[5] (Rev A identity stubs) must pass
    # +-full-scale bit-exact when enabled, through the slot AND every
    # disabled stage; comp en[0] is live, but +-full-scale saturates
    # through its gain, so it reads identity here. All three stacked
    # is identity too.
    assert await stage_once(0x1, -32768) == -32768    # comp (sat16 gain)
    assert await stage_once(0x1, 32767) == 32767
    assert await stage_once(0x8, -32768) == -32768    # phaser stub
    assert await stage_once(0x8, 32767) == 32767
    assert await stage_once(0x20, -32768) == -32768   # delay stub
    assert await stage_once(0x20, 32767) == 32767
    assert await stage_once(0x29, -32768) == -32768   # comp+phaser+delay
    assert await stage_once(0x29, 32767) == 32767

    # fuzz + dist stacked: the task-4 asym fuzz feeds the shaper
    # mag(s2) — -20000 passes the negative clip point UNCLIPPED at this
    # env (32376 -> t_neg = 21201 > 20000) and dist(-20000) = -17288;
    # 20000 clips at t_pos 19660 then shapes: dist(19660) = 17203
    assert await stage_once(0x6, -20000) == golden_model.dist(-20000) == -17288
    assert await stage_once(0x6, 5000) == golden_model.dist(5000) == 11144
    assert await stage_once(0x6, 20000) == golden_model.dist(19660) == 17203

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
    emits full-scale (soft ceiling 20479) and the plain fuzz clips below
    the rails (only the octave blend saturates, and fuzz is off here
    anyway), but the identity chain passes sample_in = 0x7FFF exactly. Phases, DIP all-off
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
