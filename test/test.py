# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-FileCopyrightText: © 2026 Thomas McDonagh
# SPDX-License-Identifier: Apache-2.0
# Internal-state probes are RTL-only (guarded getattr — names may vanish
# in the flat GL netlist); port-level assertions carry the GL coverage.

import math
import cocotb
from cocotb.handle import Force, Release
import golden_model
from golden_model import EN_COMP, EN_DIST, EN_PHASER, EN_TREM, EN_DELAY, EN_FREEZE  # noqa: F401
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, Edge, ReadOnly, RisingEdge, Timer, ValueChange, with_timeout
from cocotb.utils import get_sim_time


@cocotb.test()
async def test_skeleton_baseline(dut):
    """Task 1 baseline: reset state, pin wiring, and running frame clock."""
    dut._log.info("Start")

    # every DIP on (ui_in = 0xFF: ui[5:0] enables, ui[7:6] unused),
    # zero input: every stage maps 0 to 0, so DAC data stays low even
    # after the switches settle.
    await start(dut, 0xFF)

    # Reset state: DAC data low, uo idle — dp clip LED (uo[7]) low, flash
    # CS0 hold uo[6] = 1, uo[5] = 0, SPI idle (uo[1] = CS high, uo[2],
    # uo[0] = 0); uo[4] = LRCLK is masked (it runs)
    assert dut.uo_out[3].value == 0, "DAC data not low after reset"
    assert int(dut.uo_out.value) & 0xEF == 0x42, \
        "uo must read 0x42 at idle, LRCLK masked (dp dark, CS0 hold high, SPI CS idle high, rest 0)"

    # MCLK output enabled on uio[0] and following clk.
    # Sample away from clock edges (comb outputs need settle time).
    # fixed bits: uio[6] (BCLK) + uio[0] (MCLK) outputs, uio[7] (MISO)
    # + uio[1] (ADC data) inputs; uio[5:2] are the knob readers'
    # discharge drives (toggle; see test_pin_map)
    assert int(dut.uio_oe.value) & 0b1100_0011 == 0b0100_0001, \
        "uio_oe must enable uio[6] and uio[0] only (uio[7], uio[1] are inputs)"
    assert int(dut.uio_out.value) & 0b0011_1100 == 0, "uio_out[5:2] must be 0"
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

    # Frame clock is running: LRCLK (uo[4]) must toggle within one frame (512 clk)
    lr0 = dut.uo_out[4].value
    toggled = False
    for _ in range(600):
        await ClockCycles(dut.clk, 1)
        if dut.uo_out[4].value != lr0:
            toggled = True
            break
    assert toggled, "LRCLK never toggled — frame counter not running"

    # all enables on, zero input: DAC data still low
    await ClockCycles(dut.clk, 1)
    assert dut.uo_out[3].value == 0, "unexpected DAC output with zero input"


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
# knob_rc4 discharge phase per knob turn (Rev B4: 8192, was 1024, so the
# 100-220 nF caps of 10 kOhm pots empty fully)
KNOB_DIS = 8192
# one knob turn, worst case: discharge + the 65536-clk absent timeout
KNOB_TURN_MAX = KNOB_DIS + 65536 + 200


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
    """Wait for the next WS falling edge (LRCLK uo[4] 1->0) = start of a left
    (data) half-frame."""
    prev = int(dut.uo_out[4].value)
    while True:
        await ClockCycles(dut.clk, 1)
        await Timer(1, unit="ns")
        cur = int(dut.uo_out[4].value)
        if prev == 1 and cur == 0:
            return
        prev = cur


async def i2s_send_and_capture(dut, word24):
    """Drive one left-channel frame on uio[1] (tb adc_din; 24-bit word,
    slots 1..24) and capture the 32-slot left half from uo[3] (DAC).

    Returns the 32 captured bits (index = slot). Called right after a WS
    edge (start of left half)."""
    bits = []
    for slot in range(32):
        # Drive ADC data for this slot before its BCLK rising edge
        if 1 <= slot <= 24:
            bit = (word24 >> (24 - slot)) & 1
        else:
            bit = 0
        dut.adc_din.value = bit
        await _bclk_edge(dut, True)  # mid-slot: master samples here
        bits.append(int(dut.uo_out[3].value))
        await _bclk_edge(dut, False)  # slot boundary
    return bits


def tx_data_bits(bits):
    """Extract the 16 transmitted data bits from a captured frame
    (slots 1..16, MSB first)."""
    v = 0
    for k in range(16):
        v = (v << 1) | bits[1 + k]
    return v


# Switch debounce (v3 task 7): the enables (ui[5:0]) are 2-flop
# synchronised and only reach the chain after two agreeing 256-frame ticks
# (sw updates at strobe 512 from reset), so every test lets the switches
# settle before it probes.
SETTLE_FRAMES = 600

# Chain timing bound (Task 15a, serial multiplier): every chain step that
# uses the shared shift-add multiplier waits for it, so chain_done comes
# up to CHAIN_MAX_CLK clk after sample_valid (every effect on). The TX
# load of the result is at slot 0 of the next frame, 256 clk after it.
CHAIN_MAX_CLK = 200

# golden_model frame index of the frame start() returns at. The tb_audio /
# model frame 0 starts at reset release (lr_cnt = 0, WS already low), so
# the first WS fall after reset — start()'s "frame 0" — begins model frame
# 1. Stateful per-frame paths (the trem phase from EN_SETTLE) need the
# exact index: model inputs are [0] * START_FRAME + the driven words.
START_FRAME = SETTLE_FRAMES + 1


_S3 = {"v": None}    # latest dist-stage output seen by _s3_monitor (RTL only)


async def _s3_monitor(dut):
    """RTL-only probe of the dist-stage output (the s3 value the tone
    filter sees). Since Task 17a there is no s3 register: the running
    chain register cs holds it while step == 4, so sample cs there
    (event-driven on step changes; no-op on the flat GL netlist)."""
    _S3["v"] = None
    step, cs = _probe(dut, "step"), _probe(dut, "cs")
    if step is None or cs is None:
        return
    while True:
        await ValueChange(step)
        if int(step.value) == 4:
            await ReadOnly()
            _S3["v"] = int(cs.value)


def _s3_probe():
    """Latest dist-stage output (unsigned 16-bit) or None."""
    return _S3["v"]


async def start(dut, ui_in, uio_in=0, clock=True):
    """Clock, reset, drive the inputs, then wait SETTLE_FRAMES frames for
    the debounced switches to take effect and return aligned to a
    left-half start (frame SETTLE_FRAMES counted from the first left
    start after reset). clock=False re-resets inside a test that already
    started the clock."""
    if clock:
        cocotb.start_soon(
            Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2).start())
        cocotb.start_soon(_s3_monitor(dut))
    dut.ena.value = 1
    dut.ui_in.value = ui_in
    dut.uio_in.value = uio_in
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await _wait_left_start(dut)                              # frame 0
    await ClockCycles(dut.clk, 512 * SETTLE_FRAMES - 16)     # just before frame SETTLE_FRAMES
    await _wait_left_start(dut)                              # its left start


async def resettle(dut, ui_in):
    """Change the switches mid-test (no reset): from a left-half start
    +/- the previous capture, advance exactly SETTLE_FRAMES silent frames
    (ADC data low; the last drive left rx_shift at 0 — callers drove a zero
    capture frame) and return at a left-half start."""
    await _wait_left_start(dut)
    dut.ui_in.value = ui_in
    await ClockCycles(dut.clk, 512 * SETTLE_FRAMES - 16)
    await _wait_left_start(dut)


async def fast_frames(dut, words):
    """Low-overhead I2S driver for long runs: from a left-half start (as
    start()/_wait_left_start return, 1 ns after the WS-fall clk edge)
    drive one frame per 16-bit word on uio[1] (adc_din) and capture each
    frame's TX word from uo[3], using sim-time Timers instead of per-clk
    polling (~65 awaits/frame). Capture j is the result of input frame
    j-1 (one-frame TX latency). Returns signed captures, ending at the
    next left start (+1 ns)."""
    outs = []
    for w in words:
        w &= 0xFFFF
        v = 0
        for slot in range(32):
            dut.adc_din.value = (w >> (16 - slot)) & 1 if 1 <= slot <= 16 else 0
            await Timer(4 * CLK_PERIOD_PS, unit="ps")   # mid-slot
            if 1 <= slot <= 16:
                v = (v << 1) | int(dut.uo_out[3].value)
            await Timer(4 * CLK_PERIOD_PS, unit="ps")
        await Timer(256 * CLK_PERIOD_PS, unit="ps")     # right half
        outs.append(v - 0x10000 if v & 0x8000 else v)
    return outs


# RC knob pin model (v3 task 9): K1..K4 on uio[2..5]. While the reader
# drives its pin (uio_oe[2+idx] = 1, discharge) the pin reads 0; after the
# drive releases, the pin reads 1 exactly N = 64*v + 32 - SYNC_LAT clk
# later (at the falling clk edge closing clk N), so the reader's charge
# count lands mid-bin at 64*v + 32 and raw == v. SYNC_LAT = 2: the
# 2-flop synchronizer (the count itself starts at 0 on the release edge).
# v = None holds the pin at 0 (knob absent). Event-driven on uio_oe.
KNOB_SYNC_LAT = 2


class KnobPin:
    def __init__(self, dut, idx, v):
        self.dut, self.idx, self.v = dut, idx, v
        self.pin = dut.uio_in[2 + idx]
        self.task = cocotb.start_soon(self._run())

    def _oe(self):
        val = self.dut.uio_oe.value
        return (int(val) >> (2 + self.idx)) & 1 if val.is_resolvable else None

    async def _run(self):
        prev = self._oe()
        while True:
            await ValueChange(self.dut.uio_oe)
            cur = self._oe()
            if cur == 1:
                self.pin.value = 0
            elif cur == 0 and prev == 1 and self.v is not None:
                n = 64 * self.v + 32 - KNOB_SYNC_LAT
                await Timer(n * CLK_PERIOD_PS + CLK_PERIOD_PS // 2, unit="ps")
                cur = self._oe()
                if cur == 0:          # still charging (no reset meanwhile)
                    self.pin.value = 1
            prev = cur


def knob_pin(dut, idx, v):
    """Start the knob pin model for K(idx+1); set .v later to re-time."""
    return KnobPin(dut, idx, v)


async def knob_cycle(dut, idx):
    """Wait for one full charge phase of knob idx: uio_oe[2+idx] falls
    (its discharge ends), then the round-robin reader (knob_rc4) ends the
    measurement (pin seen high, or timeout) and starts the NEXT knob's
    discharge, so uio_oe[2 + (idx+1) % 4] rises. Returns the charge length
    in clk. Fails (instead of hanging) if no full cycle within one
    worst-case rotation (4 timeouts) plus 2 timeouts."""
    return await with_timeout(_knob_cycle(dut, idx), 6 * KNOB_TURN_MAX * CLK_PERIOD_PS, "ps")


async def _knob_cycle(dut, idx):
    def oe(i):
        val = dut.uio_oe.value
        return (int(val) >> (2 + i)) & 1 if val.is_resolvable else None
    nxt = (idx + 1) % 4
    while oe(idx) != 1:
        await ValueChange(dut.uio_oe)
    while oe(idx) != 0:
        await ValueChange(dut.uio_oe)
    t0 = get_sim_time(unit="ps")
    while oe(nxt) != 1:
        await ValueChange(dut.uio_oe)
    return round((get_sim_time(unit="ps") - t0) / CLK_PERIOD_PS)


def fast_knobs(dut, *idxs):
    """Pin models at v = 0 on the given knobs: each of their round-robin
    turns then lasts KNOB_DIS + 33 clk instead of a ~74k-clk timeout, so the
    knob under test comes round quickly (timing only: tests that use this
    never read those knobs' values)."""
    return [knob_pin(dut, i, 0) for i in idxs]


@cocotb.test()
async def test_bclk_lrclk_ratios(dut):
    """BCLK toggles every 4 clk (period 8); LRCLK toggles once per 256 clk."""
    await start(dut, 0)   # returns at a left-half start (LRCLK just fell)

    # Poll with quarter-period spacing so no edge race can hide a toggle.
    # One iteration = 1/4 clk period; 1620 iterations ~ 405 clk.
    bclk_rises = 0
    lr_toggles = 0
    prev_b = int(dut.uio_out[6].value)
    prev_l = int(dut.uo_out[4].value)
    for _ in range(1620):
        await Timer(10134, unit="ps")
        b = int(dut.uio_out[6].value)
        l = int(dut.uo_out[4].value)
        if b == 1 and prev_b == 0:
            bclk_rises += 1
        if l != prev_l:
            lr_toggles += 1
        prev_b, prev_l = b, l
    assert 49 <= bclk_rises <= 52, f"BCLK rises in 405 clk: {bclk_rises}"
    assert lr_toggles == 1, f"LRCLK toggles in 405 clk after a frame start: {lr_toggles}"


@cocotb.test()
async def test_i2s_tx_frame(dut):
    """Sample 0x55AA driven on ui[0] in frame N must appear MSB-first on
    uio[1] slots 1..16 in frame N+1; slot 0 low (I2S delay), slots 17..31 low."""
    await start(dut, 0)

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
    await start(dut, 0)  # en = 0 (identity chain; L=R mono contract)

    async def send_capture_both_halves(word16):
        """Drive one frame's left half with word16; capture BOTH halves.
        Must be called with the sim at a left-half start (the previous
        call's right-half loop ends exactly there)."""
        lhalf = []
        for slot in range(32):
            dut.adc_din.value = (1 <= slot <= 16) and ((word16 >> (16 - slot)) & 1) or 0
            await _bclk_edge(dut, True)
            lhalf.append(int(dut.uo_out[3].value))
            await _bclk_edge(dut, False)
        # right half: no drive needed, just capture slots 33..48
        rhalf = []
        for slot in range(32):
            await _bclk_edge(dut, True)
            rhalf.append(int(dut.uo_out[3].value))
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
    await start(dut, 0)  # en = 0: DIP all-off -> identity chain

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
    # bare reset (no switch settle: the I2S RX path doesn't need it) with
    # junk (ADC data uio[1] = 1) on the data line during reset
    cocotb.start_soon(
        Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2).start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    dut.adc_din.value = 1  # junk on the data line during reset
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    await _wait_left_start(dut)
    await i2s_send_and_capture(dut, 0)          # frame A: junk may flush
    await _wait_left_start(dut)
    bits = await i2s_send_and_capture(dut, 0)   # frame B: must be silent
    assert tx_data_bits(bits) == 0, \
        f"second frame after reset must be silent, got {tx_data_bits(bits):#06x}"
    assert int(dut.uo_out.value) & 0xE7 == 0x42, \
        "uo must read 0x42 after reset, LRCLK/DAC masked (dp dark, CS0 hold high, SPI CS idle high)"



# ui byte layout (Rev B3 final): ui[5:0] = DIP enables en[5:0] in chain
# order (comp, dist, phaser, trem, delay, freeze = golden_model.EN_*),
# debounced (see start()); ui[7:6] unused. No external data source is
# on any ui pin: the ADC data is uio[1] (tb adc_din), the PSRAM MISO uio[7].
# uio[5:2] are the RC knob pins K1..K4 (tests drive 0 = absent unless a
# knob_pin model runs).

@cocotb.test()
async def test_comp_ratio(dut):
    """Comp stage (en[0]), v3 task 10: a real compressor. State (golden
    Comp): cenv, a 16.8 smoothed envelope of the PRE-CHAIN magnitude
    (attack 1/32 of the gap, release cenv>>12 with a 1-LSB floor); lvl_q
    = 4*e + m2 of L = cenv>>8 (MSB index, two bits below it); gain target
    from the 1.5 dB table above the knee lvl_q 44; g slews 1/8 of the gap
    per frame. Output sat16((s0 * g) >>> 8).
    Constant +2048 (steady out A) and +16384 (steady out B), each from a
    fresh reset: both exactly golden frame-for-frame, B/A < 4 for an
    input ratio of 8, and the last 10 outputs of each run identical (no
    hunting). The golden model converges by frame 63 for 16384 (2048
    sits below the knee and is constant from frame 0), so a 200-frame
    window stands in for the plan's 2000 (CI also runs gate-level)."""
    n = 200
    steady = []
    for i, amp in enumerate((2048, 16384)):
        await start(dut, EN_COMP, clock=(i == 0))   # comp only (ui[0])
        got = await fast_frames(dut, [amp] * n + [0])
        want = golden_model.render([0] * START_FRAME + [amp] * n, 0x01)[START_FRAME:]
        # capture j carries input frame j-1's result
        assert got[1:] == want, f"comp {amp}: first mismatch at " \
            f"{next(j for j in range(n) if got[1 + j] != want[j])}"
        last10 = got[-10:]
        assert len(set(last10)) == 1, f"comp {amp} must settle: last 10 {last10}"
        steady.append(last10[0])
    a, b = steady
    assert (a, b) == (5736, 16384), f"steady outs {a}, {b}"
    assert b < 4 * a, f"B/A = {b / a:.3f} must be < 4 (input ratio 8)"


@cocotb.test()
async def test_comp_slew(dut):
    """Comp gain slew (v3 task 10): settle on +2048 (g stalls at 717, the
    upward slew's 1/8 step rounding to 0 within 8 of the 724 target),
    then step to +16384 for 200 frames: the output equals golden frame-
    for-frame, and the golden g trajectory moves by at most
    (|target - g| >> 3) + 1 per frame (the slew bound; the +1 is the
    arithmetic-shift rounding of a negative gap)."""
    pre, n = 100, 200
    await start(dut, EN_COMP)                     # comp only (ui[0])
    xs = [2048] * pre + [16384] * n
    got = await fast_frames(dut, xs + [0])
    want = golden_model.render([0] * START_FRAME + xs, 0x01)[START_FRAME:]
    assert got[1:] == want, \
        f"comp slew: first mismatch at {next(j for j in range(len(xs)) if got[1 + j] != want[j])}"
    st = golden_model.Comp()
    for _ in range(START_FRAME):
        st.step(0)
    for x in xs:
        g0, t = st.g, golden_model.comp_target(st.cenv)
        assert abs(st.step(x) - g0) <= (abs(t - g0) >> 3) + 1
    assert want[pre - 1] == 5736 and want[-1] == 16384


@cocotb.test()
async def test_comp_rails(dut):
    """Comp rails + clip LED with comp ON (v3 task 10). After the silent
    settle g = 717 (golden). (a) Clip LED: a ramp 8000, 9000, .. whose
    raw values never reach a rail; golden puts the first rail output
    (sat16(12000 * 717 >>> 8) = 32767) at ramp frame 4 — the rail is
    comp gain alone. uo[7] is dark before that frame's valid edge and
    lit by its end (hit taken at chain_done on the effect_sample the TX
    sends next frame), dark at the end of every earlier frame, and the
    next frame's capture is that rail value. (b) +32767 and -32768,
    each after 400 silent frames, give exactly 32767 / -32768 on the
    first loud frame (golden)."""
    await start(dut, EN_COMP)                     # comp only (ui[0])

    def dp():
        return int(dut.uo_out[7].value)

    ramp = [8000 + 1000 * j for j in range(8)]
    want = golden_model.render([0] * START_FRAME + ramp, 0x01)[START_FRAME:]
    k = next(j for j, v in enumerate(want) if v in (32767, -32768))
    assert k == 4 and want[k] == 32767 and all(abs(x) < 32767 for x in ramp)
    caps = []
    for j in range(k + 1):
        bits = await i2s_send_and_capture(dut, ramp[j] << 8)
        caps.append(_signed(tx_data_bits(bits)))
        await Timer(1, unit="ns")
        assert dp() == 0, f"LED lit before ramp frame {j}'s valid edge"
        await ClockCycles(dut.clk, 256)
        await Timer(1, unit="ns")
        assert dp() == (1 if j == k else 0), f"LED after ramp frame {j}: {dp()}"
    bits = await i2s_send_and_capture(dut, 0)
    caps.append(_signed(tx_data_bits(bits)))
    assert caps[1:] == want[:k + 1], f"ramp captures {caps[1:]} != {want[:k + 1]}"
    assert dp() == 1

    # (b) rails after 400 silent frames each
    xs = ramp[:k + 1] + [0]
    await ClockCycles(dut.clk, 256)                   # rest of the capture frame
    for word in (32767, -32768):
        await ClockCycles(dut.clk, 512 * 400)
        got = await fast_frames(dut, [word, 0])
        xs += [0] * 400 + [word, 0]
        want = golden_model.render([0] * START_FRAME + xs, 0x01)
        assert got[1] == want[-2] == word, f"rail {word}: {got[1]} (golden {want[-2]})"


@cocotb.test()
async def test_cenv_decay(dut):
    """Comp envelope release (v3 task 10), RTL-only cenv probe (guarded;
    None voids it on the flat GL netlist — test_comp_* carry GL): 100
    loud frames attack cenv, then 300 silent frames; the probe equals
    the golden cenv exactly at the end of the loud run and after every
    100 silent frames. Release = cenv - (cenv >> 12) - (cenv != 0);
    termination at exactly 0 (~34k frames from full scale, too long
    here) is proven by `golden_model.py --selftest` (wav_tool check)."""
    await start(dut, EN_COMP)                     # comp only (ui[0])
    probe = _probe(dut, "cenv")
    # RTL (where the chain's step reg is visible) must expose cenv
    assert probe is not None or _probe(dut, "step") is None, "RTL has no cenv"

    def cenv():
        return None if probe is None else int(probe.value)

    st = golden_model.Comp()
    for _ in range(START_FRAME):
        st.step(0)
    assert cenv() in (None, st.cenv) and st.cenv == 0
    await fast_frames(dut, [0x7FFF] * 100)
    for _ in range(100):
        st.step(0x7FFF)
    assert cenv() in (None, st.cenv), f"cenv after loud {cenv()} != {st.cenv}"
    for k in range(3):
        await ClockCycles(dut.clk, 512 * 100)
        for _ in range(100):
            st.step(0)
        assert cenv() in (None, st.cenv), f"cenv after {100 * (k + 1)} silent: {cenv()} != {st.cenv}"
    assert 0 < st.cenv < 8037707     # released, not yet at the floor
    if probe is not None:
        assert cenv() == 7469837


class Hist:
    """Frame history of a dirt-stage test (v3 task 11): dist outputs
    pass through the tone low-pass and the noise gate, which carry state
    across frames, so every probe is checked against golden_model.render
    of the whole history ([0] * START_FRAME + the driven frames) instead
    of an isolated-frame literal. The gate is opened beforehand by a
    loud priming run (L >= 256) so the outputs are non-zero."""

    def __init__(self, en):
        self.en = en
        self.xs = [0] * START_FRAME

    def add(self, x, n=1):
        self.xs += [x] * n

    def render(self, en=None):
        tap = []
        out = golden_model.render(self.xs, self.en if en is None else en,
                                  tap=tap)
        return out, tap


async def prime_gate(dut, hist, amp=20000, n=120):
    """Drive n loud frames (gate opens at ~62, gg full ~19 frames later);
    leaves the sim at a left-half start, as fast_frames does."""
    await fast_frames(dut, [amp] * n)
    hist.add(amp, n)


async def dirt_frame(dut, hist, word16, direct=False):
    """Drive word16 in one frame, then a zero capture frame (two frames,
    appended to hist). The captured output must equal golden for the
    word's frame (tone + gate included). Returns (stage, out, i): stage =
    the RTL s3 tap probe (the dist-stage output the tone filter sees) read
    CHAIN_MAX_CLK clk after the word's strobe, None where the probe is absent
    (flat GL netlist); i = the word's history index. direct=True skips
    the leading WS wait (right after start()/fast_frames, which already
    return at a left start)."""
    if not direct:
        await _wait_left_start(dut)
    await i2s_send_and_capture(dut, (word16 & 0xFFFF) << 8)
    await ClockCycles(dut.clk, CHAIN_MAX_CLK)
    p = _s3_probe()
    stage = None if p is None else _signed(p)
    await _wait_left_start(dut)
    out = _signed(tx_data_bits(await i2s_send_and_capture(dut, 0)))
    i = len(hist.xs)
    hist.add(word16)
    hist.add(0)
    want, tap = hist.render()
    assert out == want[i], f"word {word16}: out {out} != golden {want[i]}"
    assert stage in (None, tap[i]), f"word {word16}: s3 tap {stage} != golden {tap[i]}"
    return stage, out, i


HOLD_FRAMES = 48    # tone (kt 2) settles to within 1 LSB of a DC input


async def hold_frames(dut, hist, word16, n=HOLD_FRAMES, direct=True):
    """Gate-level-visible probe: drive word16 for n frames (a DC hold, so
    the tone low-pass settles and the open gate passes it), then check
    EVERY captured port word against golden_model.render of the whole
    history. Returns (port, stage, i, want, tap): the last captured port
    word and its golden index i (the hold's second-last frame, one-frame
    TX latency), the RTL s3 tap probe read after the last frame (the
    dist-stage output of the hold's last frame; None on GL, else asserted
    == tap[-1]), and the full golden want/tap lists. The gate must
    already be open (prime_gate). Frame-continuous with the previous
    prime_gate/hold_frames call (capture 0 is that call's last frame);
    direct=True expects the sim at a left start (as fast_frames leaves
    it — a second WS wait there can race), direct=False waits for one
    (after a dirt_frame, which ends mid-frame)."""
    if not direct:
        await _wait_left_start(dut)
    base = len(hist.xs)
    outs = await fast_frames(dut, [word16] * n)
    hist.add(word16, n)
    want, tap = hist.render()
    # capture j is the result of input frame base + j - 1
    for j, o in enumerate(outs):
        assert o == want[base - 1 + j], \
            f"hold {word16}: port {o} != golden {want[base - 1 + j]} (frame {base - 1 + j})"
    p = _s3_probe()
    stage = None if p is None else _signed(p)
    assert stage in (None, tap[-1]), f"hold {word16}: s3 tap {stage} != golden {tap[-1]}"
    return outs[-1], stage, base + n - 2, want, tap


@cocotb.test()
async def test_dist_diode(dut):
    """Dist stage (en[2]), v3 task 5: fixed drive x7.97 (Rev B5: 2041,
    Q4.8) into a 3-piece diode knee. a = sat_mag(x); w = (a * 2041) >> 8
    (18-bit, no intermediate saturation);
    w <= 8192: out = w; w <= 32768: out = 8192 + ((w-8192) >> 2) (slope
    1/4, 14336 at w = 32768); else out = 14336 + ((w-32768) >> 4)
    (slope 1/16, max 28615 — sat16 never reached). Sign reapplied from
    x[15]. Comp is off, so the input is the raw sample. Exact points
    pin both seams (K1 at a=2048, K2 at a=8192, both sides) and the
    extremes; each is also asserted against golden_model.dist. Since
    task 11 the pins read the RTL s3 tap probe (the dist-stage output,
    before tone + gate; guarded, GL carries the port check) and the port
    output is checked against golden after a loud run that opens the
    gate (Hist/dirt_frame)."""
    await start(dut, EN_DIST)  # dist only (ui[1])
    hist = Hist(EN_DIST)
    await prime_gate(dut, hist)

    async def dist_once(word16, first=False):
        # the stage value (RTL s3 tap probe; None on GL) is the pinned
        # dist output; the port output (through tone + gate) is checked
        # against golden inside dirt_frame
        stage, _, _ = await dirt_frame(dut, hist, word16, direct=first)
        return stage

    points = [
        (1000, 7972),      # linear x7.97 below K1
        (1027, 8187),      # just below K1 (w = 8187)
        (1028, 8192),      # just above K1 (w = 8195 -> 8192 + 3/4, floor)
        (4096, 14308),     # w = 32656 -> 8192 + 24464/4
        (4110, 14335),     # just below K2 (w = 32767)
        (4111, 14336),     # just above K2 (w = 32775 -> 14336 + 7/16)
        (32767, 28615),    # w = 261,248 -> 14336 + 228480/16
        (-32768, -28615),  # mag sat 32767, sign kept
        (-1000, -7972),    # sign reapplied, linear region
        (-16384, -20452),  # w = 130,624 -> 14336 + 97856/16
    ]
    for k, (x, want) in enumerate(points):
        assert want == golden_model.dist(x), f"pin {x}: {want} != golden"
        got = await dist_once(x, first=(k == 0))
        assert got in (None, want), f"dist({x}) = {got}, expected {want}"

    # Gate-level-visible seam pass (the pins above read the RTL s3 tap
    # probe; at the port a single frame is hidden by the tone low-pass).
    # A DC hold lets the tone settle and the open gate passes it. Every
    # frame of every hold is asserted against golden; the settled port
    # words of each seam pair (K1 1027/1028, K2 4110/4111) must differ,
    # so the seams are GL-visible.
    settled = {}
    for x in (1027, 1028, 4110, 4111):
        port, stage, i, want_out, tap = await hold_frames(
            dut, hist, x, direct=(x != 1027))
        assert tap[-1] == golden_model.dist(x)
        settled[x] = port
    assert settled[1027] != settled[1028], settled
    assert settled[4110] != settled[4111], settled


async def _drive_points(dut, points, k3=None, en=EN_DIST):
    """Fresh run (K3 = k3 pin model, None = absent): prime the gate, then
    for each x hold it and require the dist-stage output (RTL s3 tap
    probe, None on GL) and the settled port word to equal golden at the
    fixed drive; every frame is golden-checked inside hold_frames.
    Returns the list of (x, settled port, golden dist)."""
    if k3 is not None:
        knob_pin(dut, 2, k3)
    await start(dut, en)
    hist = Hist(en)
    await prime_gate(dut, hist)
    res = []
    for x in points:
        port, stage, i, want, tap = await hold_frames(dut, hist, x)
        d = golden_model.dist(x)
        assert tap[-1] == d and stage in (None, d), f"x={x}: stage {stage} != {d}"
        assert abs(port) <= 32767 and abs(d) <= 32767
        res.append((x, port, d))
    return res


@cocotb.test()
async def test_drive_fixed(dut):
    """Rev B5: the dist drive is fixed at 2041 (x7.97, the old K3 knob
    maximum) and K3 (now the speed knob) does not touch it: K3 absent and
    K3 = 0 both give the x7.97 points, through both knee seams and the
    rail (32767 -> 28615, inside +-32767; no wrap)."""
    assert golden_model.DRIVE == 2041
    pts = (1000, 4096, 32767)
    for k3 in (None, 0):
        res = await _drive_points(dut, pts, k3=k3)
        assert [d for _, _, d in res] == [7972, 14308, 28615], f"K3 {k3}"


def _probe(dut, *path):
    """Guarded RTL-only internal probe (None on the flat GL netlist)."""
    h = getattr(dut, "user_project", None)
    for name in path:
        h = getattr(h, name, None) if h is not None else None
    return h


async def _bare_reset(dut):
    """Reset without the switch settle (the knob reader needs none)."""
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1


@cocotb.test()
async def test_knob_reader(dut):
    """RC knob reader (knob_rc4 round robin, K1 on uio[2]; K2..K4 run
    v = 0 pin models so K1's turn comes round every ~33k clk): discharge
    KNOB_DIS clk with uio_oe[2] = 1, then charge until the pin reads 1; raw =
    cnt >> 6. Port level: every charge phase (uio_oe[2] fall -> the next
    knob's uio_oe[3] rise) lasts exactly 64*v + 33 clk (the pin
    model's N = 64v + 32 - SYNC_LAT, + SYNC_LAT synchronizer, + the
    detect edge), so uio_oe[2] toggles at the measurement rate; that
    length is set by the tb pin model, so it does NOT prove raw == v
    (only the RTL probe below does). Probe
    (RTL only): the first measurement loads value = raw = v (fast
    startup), and after 20 measurements value == knob_steady(v) with
    present = 1. Then the knob is pulled (pin held low): one timeout
    (a 65536-clk charge phase) clears present, value holds."""
    cocotb.start_soon(
        Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2).start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    kp = knob_pin(dut, 0, None)
    fast_knobs(dut, 1, 2, 3)
    kval = _probe(dut, "k1_value")      # knob_rc4 outputs, top-level wires
    kpres = _probe(dut, "k1_present")
    for v in (0, 77, 200):
        kp.v = v
        await _bare_reset(dut)
        for m in range(20):
            n = await knob_cycle(dut, 0)
            assert n == 64 * v + 33, f"v={v} meas {m}: charge {n} clk != {64 * v + 33}"
            if kval is not None and m == 0:
                await Timer(1, unit="ns")
                assert int(kval.value) == v, f"v={v}: first raw/value {int(kval.value)} != {v}"
        if kval is not None:
            await Timer(1, unit="ns")
            assert int(kval.value) == golden_model.knob_steady(v)
            assert int(kpres.value) == 1, f"v={v}: present must be 1"
    # absent: hold the pin low (set during the discharge phase)
    kp.v = None
    n = await knob_cycle(dut, 0)
    assert n == 65536, f"absent: charge phase {n} clk != 65536 (timeout)"
    if kval is not None:
        await Timer(1, unit="ns")
        assert int(kpres.value) == 0, "present must clear after a timeout"
        assert int(kval.value) == golden_model.knob_steady(200), "value must hold"


@cocotb.test()
async def test_knob4_reader(dut):
    """K4 on uio[5] (Rev B pin freeze): the same knob_rc4 reader as K1
    (K1..K3 run v = 0 pin models). Every charge phase lasts 64*v + 33 clk
    (uio_oe[5] fall -> uio_oe[2] rise, K1's turn); RTL probe: first
    value = v, steady value == knob_steady(v), present = 1 for v = 10 and
    200; pin held low -> one 65536-clk timeout clears present."""
    cocotb.start_soon(
        Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2).start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    kp = knob_pin(dut, 3, None)
    fast_knobs(dut, 0, 1, 2)
    kval = _probe(dut, "k4_value")
    kpres = _probe(dut, "k4_present")
    for v in (10, 200):
        kp.v = v
        await _bare_reset(dut)
        for m in range(20):
            n = await knob_cycle(dut, 3)
            assert n == 64 * v + 33, f"v={v} meas {m}: charge {n} clk != {64 * v + 33}"
            if kval is not None and m == 0:
                await Timer(1, unit="ns")
                assert int(kval.value) == v
        if kval is not None:
            await Timer(1, unit="ns")
            assert int(kval.value) == golden_model.knob_steady(v)
            assert int(kpres.value) == 1
    kp.v = None
    n = await knob_cycle(dut, 3)
    assert n == 65536, f"absent: charge phase {n} clk != 65536 (timeout)"
    if kval is not None:
        await Timer(1, unit="ns")
        assert int(kpres.value) == 0, "present must clear after a timeout"


@cocotb.test()
async def test_knob_round_robin(dut):
    """One shared reader (knob_rc4, Task 15a) measures K1, K2, K3, K4,
    K1, .. in turn: at any time at most one knob pin is driven (uio_oe
    [5:2] one-hot or 0), each turn discharges its pin for KNOB_DIS clk then
    releases it, and the measurement end (pin seen high: 64*v + 33 clk;
    absent: the 65536-clk timeout) starts the next knob's discharge on
    the same edge. K3 = 20, the rest absent (pins held low)."""
    cocotb.start_soon(
        Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2).start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    knob_pin(dut, 2, 20)
    await _bare_reset(dut)

    def pat():
        val = dut.uio_oe.value
        return (int(val) >> 2) & 0xF if val.is_resolvable else None

    async def record(n):
        ev = [(get_sim_time(unit="ps"), pat())]
        while len(ev) < n:
            await ValueChange(dut.uio_oe)
            cur = pat()
            if cur != ev[-1][1]:
                ev.append((get_sim_time(unit="ps"), cur))
        return ev

    ev = await with_timeout(record(10), 5 * KNOB_TURN_MAX * CLK_PERIOD_PS, "ps")
    pats = [p for _, p in ev]
    assert pats == [1, 0, 2, 0, 4, 0, 8, 0, 1, 0], f"turn order {pats}"
    lens = [round((b[0] - a[0]) / CLK_PERIOD_PS) for a, b in zip(ev, ev[1:])]
    D = KNOB_DIS
    want = [D, 65536, D, 65536, D, 64 * 20 + 33, D, 65536, D]
    # the first discharge counts from the reset release (+-1 clk of phase)
    assert abs(lens[0] - D) <= 1 and lens[1:] == want[1:], f"phase lengths {lens}"


def _signed(x):
    return x - 0x10000 if x & 0x8000 else x


@cocotb.test()
async def test_trem_depth(dut):
    """Rev B5: K2 sets the trem depth, gain = 256 - ((tri * v2) >> 8)
    (K2 absent keeps B4's 256 - tri/2 - tri/4, 75%; pinned by
    test_knob_absent_default). Constant 16384 input, K3 = 250 (inc 976,
    one LFO cycle ~4300 frames), 4400 frames per case, fresh reset each:
    K2 = 0 -> no trem (every word 16384); K2 = 255 -> == golden and the
    trough reaches gain 2 (word 128); K2 = 128 -> == golden, trough gain
    129 (word 8256)."""
    tri_max = 0xFF << 13                 # tri = 255 (rising half)
    assert golden_model.trem_gain(tri_max, None) == 66
    assert golden_model.trem_gain(tri_max, 255) == 2
    assert golden_model.trem_gain(tri_max, 128) == 129
    assert golden_model.trem_gain(tri_max, 0) == 256
    assert golden_model.trem_gain(0, 255) == 256
    kp2 = knob_pin(dut, 1, 0)
    knob_pin(dut, 2, 250)
    n = 4400
    for i, (v, lo) in enumerate(((0, 16384), (255, 128), (128, 8256))):
        kp2.v = v
        await start(dut, EN_TREM, clock=(i == 0))
        got = (await fast_frames(dut, [16384] * (n + 1)))[1:]
        want = golden_model.render([0] * START_FRAME + [16384] * n, EN_TREM,
                                   k2=v, k3=250)[START_FRAME:]
        first = _first_diff(got, want)
        assert first is None, f"K2 {v} frame {first}: got {got[first]} want {want[first]}"
        assert min(got) == lo, f"K2 {v}: trough {min(got)} != {lo}"
        if v == 0:
            assert set(got) == {16384}, "K2 = 0 must be no trem"
        step = max(abs(a - b) for a, b in zip(got, got[1:]))
        assert step <= TREM_MAX_STEP_16384, f"K2 {v}: max |dout| {step}"


@cocotb.test()
async def test_knob_absent_default(dut):
    """K3 absent (pin tied low): the trem phase steps by the default 437
    per frame (~5 Hz). Probe (RTL only): trem_p advances exactly 437 per
    frame over 5 consecutive frames (and equals 437 * frames since the
    enable reached the chain at EN_SETTLE). Port level: 20 consecutive
    TX words on a constant 16384 equal golden_model (k3 = None)."""
    await start(dut, EN_TREM)                 # trem only (ui[3]), uio_in = 0
    p = _probe(dut, "trem_p")
    if p is not None:
        ps = []
        for _ in range(6):
            ps.append(int(p.value))
            await ClockCycles(dut.clk, 500)
            await _wait_left_start(dut)
        assert ps[0] == golden_model.TREM_INC_ABSENT * (START_FRAME - golden_model.EN_SETTLE)
        d = [(b - a) & 0x3FFFFF for a, b in zip(ps, ps[1:])]
        assert d == [437] * 5, f"phase steps {d}"
    f0 = START_FRAME + (6 if p is not None else 0)
    want = golden_model.render([0] * f0 + [16384] * 21, EN_TREM)
    got = await fast_frames(dut, [16384] * 21)
    assert got[1:] == want[f0:f0 + 20], f"got {got[1:]} want {want[f0:f0 + 20]}"


# Smooth-trem step bound: gain = 256 - tri/2 - tri/4 moves by at most 2
# per tri LSB, and tri moves by at most 1 per frame for every inc <= 8192
# (K3 max inc 127 << 3 = 1016), so |dout| <= |x| * 2 / 256 = 128 at
# x = 16384 (the old stepped trem jumped by 8192).
TREM_MAX_STEP_16384 = 16384 * 2 // 256


@cocotb.test()
async def test_trem_smooth(dut):
    """Smooth trem at K3 = 128 (inc = 256): constant 16384 for 2000
    frames; the output equals golden_model frame-for-frame and never
    steps by more than TREM_MAX_STEP_16384."""
    knob_pin(dut, 2, 128)
    await start(dut, EN_TREM)
    n = 2000
    got = (await fast_frames(dut, [16384] * (n + 1)))[1:]
    want = golden_model.render([0] * START_FRAME + [16384] * n, EN_TREM, k3=128)[START_FRAME:]
    first = next((i for i in range(n) if got[i] != want[i]), None)
    assert first is None, f"frame {first}: got {got[first]} want {want[first]}"
    step = max(abs(a - b) for a, b in zip(got, got[1:]))
    assert step <= TREM_MAX_STEP_16384, f"max |dout| {step}"
    assert len(set(got)) > 20, "trem must move"


@cocotb.test()
async def test_trem_rate_change(dut):
    """K3 (speed) 128 -> 250 mid-run (pin model re-timed): the phase keeps
    accumulating (no reset/jump), so the output stays smooth across the
    change. Before the change the output equals golden_model (k3 = 128);
    across it max |dout| <= TREM_MAX_STEP_16384; probe (RTL only): the
    knob value has moved to within the hysteresis of 250. Port level
    (GL-visible): the post-change output must DIFFER from the golden
    render that keeps k3 = 128 (the rate really changed). Round robin
    (knob_rc4): K1, K4 run v = 0 pin models (en = trem only, so their
    values never reach the output) and K2 (trem depth since Rev B5) a
    v = 128 one (golden k2 = 128), giving one K3 reading per ~100
    frames (8192-clk discharges); the IIR needs 16 readings from 128 to
    249 (~1540 frames), so the post-change run is 1800 frames."""
    kp = knob_pin(dut, 2, 128)
    fast_knobs(dut, 0, 3)
    knob_pin(dut, 1, 128)     # K2 (Rev B5 trem depth) reaches the output: golden k2=128
    await start(dut, EN_TREM)
    pre = (await fast_frames(dut, [16384] * 201))[1:]
    want = golden_model.render([0] * START_FRAME + [16384] * 200, EN_TREM, k2=128, k3=128)[START_FRAME:]
    assert pre == want, "pre-change output must equal golden (k3=128)"
    kp.v = 250
    post = await fast_frames(dut, [16384] * 1800)
    seq = pre + post[1:]
    step = max(abs(a - b) for a, b in zip(seq, seq[1:]))
    assert step <= TREM_MAX_STEP_16384, f"max |dout| across the change {step}"
    keep = golden_model.render([0] * START_FRAME + [16384] * 2000, EN_TREM, k2=128, k3=128)[START_FRAME + 200:]
    ndiff = sum(1 for a, b in zip(post, keep) if a != b)
    assert ndiff > 200, f"post-change output tracks the k3=128 render ({ndiff} diffs): rate did not change"
    kval, kpres = _probe(dut, "k3_value"), _probe(dut, "k3_present")
    if kval is not None:
        kv = int(kval.value)
        assert int(kpres.value) == 1
        assert kv >= 248, f"knob value {kv} must track 250"


@cocotb.test()
async def test_chain_enables(dut):
    """Serial chain enable matrix (v3 task 1): the DIP enables alone set en
    (debounced, no preset ROM; every enable change settles 600 frames). For each stage:
    ON applies its effect to known vectors (incl. -32768 — the output
    also proves every DOWNSTREAM stage passes bit-exact when disabled);
    with all en=0 the chain is bit-exact identity and +-32768 round-trip
    (Review Focus 3: no disabled stage may corrupt the sample). The delay
    (no PSRAM fitted in this test: dry) must pass +-full-scale bit-exact
    when enabled — the enable itself corrupts nothing (comp is live from
    task 3; the phaser from Task 16, golden-checked). Since task 11 the
    dist-on pins are the stage output (RTL s3 tap probe, guarded) because the port word is then
    tone-filtered and gated; the dirt-off stacks still pin the port."""
    # trem first, on a fresh reset (K1 absent -> inc 437): the expected
    # words come from golden_model.render of the whole frame history
    await start(dut, EN_TREM)                 # trem only (ui[3])
    xs = [16384, -32768, 0]
    got = await fast_frames(dut, xs)
    want = golden_model.render([0] * START_FRAME + xs, EN_TREM)
    assert got[1] == want[START_FRAME] == (16384 * golden_model.trem_gain(
        437 * (START_FRAME + 1 - golden_model.EN_SETTLE))) >> 8, f"trem ON 16384: {got[1]}"
    assert got[2] == want[START_FRAME + 1], f"trem ON -32768: {got[2]} != {want[START_FRAME + 1]}"
    assert -32768 < got[2] < 0

    # phaser en[3] (Task 16) and comp+phaser+delay, each on a fresh reset:
    # the phaser phase pp (like the trem phase) counts frames from the
    # debounced enable, so the silent-settle shortcut of stage_once below
    # (which steps the golden Chain with the new enables for the whole
    # settle run) would misplace it; from reset the history is exact.
    # +-full-scale words equal golden render and are not identity (the
    # 50/50 dry/allpass mix).
    xs = [-32768, 32767, -32768, 0]
    for en_val in (EN_PHASER, EN_COMP | EN_PHASER | EN_DELAY):
        await start(dut, en_val, clock=False)
        got = await fast_frames(dut, xs)
        want = golden_model.render([0] * START_FRAME + xs, en_val)[START_FRAME:]
        assert got[1:] == want[:3], f"en {en_val:#x}: {got[1:]} != golden {want[:3]}"
        assert got[1:] != xs[:3], f"en {en_val:#x}: phaser must not be identity"

    # every other stage on a fresh reset: ref (golden Chain) mirrors
    # every frame driven (x, then the 0 capture frame) and the silent
    # settle after each enable change.
    await start(dut, EN_DIST, clock=False)       # dist only (ui[1])
    # ref = golden_model.Chain driven frame-by-frame (every frame sent,
    # and the silent settle after each enable change): its comp/tone/
    # gate state yields the golden port word of every probe (silent
    # frames output 0 in any enable setting, so the en split inside a
    # settle run is immaterial).
    ref = golden_model.Chain()
    for _ in range(START_FRAME):
        ref.step(0, 0)
    state = {"en": EN_DIST, "g": ref.cst.g}

    async def stage_once(en_val, word16, stage=False):
        """Drive word16, capture the next frame's port word, and assert
        it equals golden's output for the frame (Chain history: tone,
        gate, comp, every stage). stage=True (a dirt-on stack, task 11:
        the port word is tone/gate filtered and gated) returns instead
        the RTL s3 tap probe — the dist stage output the tone filter
        sees (cs sampled at step 4 by _s3_monitor, after CHAIN_MAX_CLK clk) — or None where the probe
        is absent (flat GL netlist; there the golden port assertion is
        the check — the gate is cold on these single frames, so the port
        word is golden's, mostly 0: the port-level tone/gate coverage is
        test_tone_lpf / test_gate_hysteresis / test_chain_latency and the
        DC-hold probes of test_dist_diode / test_drive_*)."""
        if en_val != state["en"]:
            await resettle(dut, en_val)          # ui[5:0] = en; debounced
            for _ in range(SETTLE_FRAMES):
                ref.step(0, en_val)
            state["en"] = en_val
        await _wait_left_start(dut)
        await i2s_send_and_capture(dut, (word16 & 0xFFFF) << 8)
        probe = None
        if stage:
            await ClockCycles(dut.clk, CHAIN_MAX_CLK)
            p = _s3_probe()
            probe = None if p is None else _signed(p)
        want, s3 = ref.step(word16, en_val)      # the x frame
        state["g"] = ref.cst.g                   # comp gain for the x frame
        await _wait_left_start(dut)
        bits = await i2s_send_and_capture(dut, 0)
        ref.step(0, en_val)                      # the capture frame
        out = tx_data_bits(bits)
        out = out - 0x10000 if out & 0x8000 else out
        assert out == want, f"en {en_val:#x} word {word16}: port {out} != golden {want}"
        assert probe in (None, s3), f"en {en_val:#x} word {word16}: s3 tap {probe} != golden {s3}"
        return probe if stage else out

    # dist ON: diode-knee PWL, fixed drive x7.97 (Rev B5; expected values
    # from golden_model.dist); 1234 the slope-1/4 piece, 20000 the top piece
    for x, want in ((-32768, -28615), (1234, 8603), (20000, 22253)):
        assert golden_model.dist(x) == want
        assert await stage_once(EN_DIST, x, stage=True) in (None, want)

    # delay (no PSRAM fitted here: dry) must pass +-full-scale
    # bit-exact when enabled, through the slot AND every disabled stage;
    # comp en[0] is live: its value is golden comp at the frame's gain g
    # (after the silences here g >= 256, so +-full-scale saturates and
    # reads identity). (The phaser is pinned above, from reset.)
    for x in (-32768, 32767):                         # comp (sat16 gain)
        got = await stage_once(EN_COMP, x)
        assert got == golden_model.comp(x, state["g"]) == x, f"comp {x}: {got} (g {state['g']})"
    assert await stage_once(EN_DELAY, -32768) == -32768   # delay, no PSRAM: dry
    assert await stage_once(EN_DELAY, 32767) == 32767

    # all stages OFF: bit-exact identity, +-full-scale through
    assert await stage_once(0x0, 32767) == 32767
    assert await stage_once(0x0, -32768) == -32768


@cocotb.test()
async def test_enable_bounce(dut):
    """Switch debounce (v3 task 7): a burst of contact bounce on an
    enable followed by a stable level must produce exactly ONE clean
    transition on the output — never a glitch back. Dist (ui[1]) on a
    constant 30000 input: off -> 30000 (identity), on -> the tone
    low-pass of golden dist(30000) = 27236 (top knee piece, drive x7.97;
    Rev B5: 30000 because x7.97 boosts inputs below ~24.5k), which
    settles from 30000 down to golden's steady value over a few
    frames (task 11; the gate is open on the loud constant).
    The debounce samples every 256 frames and needs two agreeing ticks,
    so the switch toggles every 5 frames for 200 frames (at most one
    tick inside the burst), then holds; the engage output must start at
    30000, only ever FALL (a bounce glitch back would rise), and end at
    the steady value, its first change within 600 frames of the hold.
    The release repeats the burst toward 0: exactly one steady -> 30000
    transition (dirt off is identity at once)."""
    await start(dut, 0)

    off_val = 30000
    assert golden_model.dist(30000) == 27236
    tg = golden_model.ToneGate()         # tone: settled on 30000, then the dist output
    for _ in range(300):
        tg.tone(30000)
    for _ in range(300):
        on_val = tg.tone(golden_model.dist(30000))
    assert on_val in (27236, 27237)      # approached from above (1-LSB floor possible)
    sw_on = EN_DIST

    async def frame(ui):
        dut.ui_in.value = ui                       # between frames
        bits = await i2s_send_and_capture(dut, off_val << 8)
        await ClockCycles(dut.clk, 256)
        out = tx_data_bits(bits)
        return out - 0x10000 if out & 0x8000 else out

    async def bounce_then_hold(level):
        outs = []
        for i in range(200):                       # the burst: toggle every 5 frames
            outs.append(await frame(sw_on if ((i // 5) & 1) == 0 else 0))
        hold_start = len(outs)
        for _ in range(600):                       # held at its final level
            outs.append(await frame(sw_on if level else 0))
        return outs, hold_start

    # prime: let the constant input reach the output
    for _ in range(3):
        await frame(0)

    # engage: bounce (toggle every 5 frames, 200 frames), then hold on
    outs, hold_start = await bounce_then_hold(1)
    trans = [i for i in range(1, len(outs)) if outs[i] != outs[i - 1]]
    assert outs[0] == off_val, "must start clean"
    assert all(a >= b for a, b in zip(outs, outs[1:])), "engage output rose again (glitch back)"
    assert trans and outs[-1] == on_val, f"engage must settle at {on_val}, got {outs[-1]}"
    assert trans[0] - hold_start <= 600, f"engage began {trans[0] - hold_start} frames after the hold"
    # the tone one-pole (kt 2) crosses the 30000 -> 27236 gap within 40 frames
    assert all(outs[i] == on_val for i in range(trans[0] + 40, len(outs))), "engage must settle within 40 frames"

    # release: bounce toward 0 starting from level 1, then hold 0
    outs = []
    for i in range(200):
        outs.append(await frame(0 if ((i // 5) & 1) == 0 else sw_on))
    hold_start = len(outs)
    for _ in range(600):
        outs.append(await frame(0))
    assert set(outs) <= {off_val, on_val}, f"unexpected output values {set(outs)}"
    trans = [i for i in range(1, len(outs)) if outs[i] != outs[i - 1]]
    assert outs[0] == on_val, "release phase must start distorted"
    assert len(trans) == 1, f"release must switch exactly once, transitions at {trans}"
    assert outs[trans[0]] == off_val and outs[-1] == off_val
    assert trans[0] - hold_start <= 600, f"release settled {trans[0] - hold_start} frames after the hold"


@cocotb.test()
async def test_pin_map(dut):
    """Rev B3 final pin map (DIP-safe): uio_oe = {0,1,k4,k3,k2,k1,0,1}
    (uio[6] BCLK + uio[0] MCLK outputs; uio[7] PSRAM MISO and uio[1] ADC
    data are inputs at EVERY clk — Review Focus 4: an input pin driven
    into the ADC/PSRAM output would fight it; uio[5:2] are the knob pins
    K4..K1, driven low only while the round-robin reader discharges that
    pin: K1 alone in reset and for the first KNOB_DIS (8192) clk after it),
    uio_out[5:2] = 0, uo = {dp 0, CS0 hold 1, 0, LRCLK, DAC, MOSI 0,
    CE# 1, SCK 0} at idle."""
    cocotb.start_soon(
        Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2).start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    # asserted straight away in reset too — the idle level must not depend on state
    await Timer(1, unit="ns")
    assert dut.uio_oe.value == 0b0100_0101, f"uio_oe in reset: {dut.uio_oe.value}"
    assert int(dut.uio_out.value) & 0b0011_1100 == 0, "uio_out[5:2] must be 0 in reset"
    assert int(dut.uo_out.value) & 0xE7 == 0x42, "uo must idle dp=0 CS0=1 uo5=0 SCK=0 CS=1 MOSI=0 in reset"
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, KNOB_DIS - 4)
    await Timer(1, unit="ns")
    assert dut.uio_oe.value == 0b0100_0101, f"K1 alone must discharge for {KNOB_DIS} clk"
    await ClockCycles(dut.clk, 10)
    await Timer(1, unit="ns")
    assert dut.uio_oe.value == 0b0100_0001, "K1 must release after the discharge"
    assert int(dut.uio_out.value) & 0b0011_1100 == 0, "uio_out[5:2] must be 0"
    # every knob phase (4 discharges + charges, all knobs absent: the
    # 65536-clk timeout each), sampled every 64 clk: uio[7] and uio[1]
    # never driven, uio[6] and uio[0] always driven
    for _ in range(4 * KNOB_TURN_MAX // 64):
        await ClockCycles(dut.clk, 64)
        oe = int(dut.uio_oe.value)
        assert oe & 0b1000_0010 == 0, f"uio[7]/uio[1] driven: uio_oe {oe:08b}"
        assert oe & 0b0100_0001 == 0b0100_0001, f"uio[6]/uio[0] not driven: uio_oe {oe:08b}"
    await start(dut, 0, clock=False)
    assert int(dut.uio_oe.value) & 0b1100_0011 == 0b0100_0001, f"uio_oe: {dut.uio_oe.value}"
    assert int(dut.uo_out.value) & 0xE7 == 0x42, \
        f"uo must idle 0x42 (LRCLK/DAC masked), got {int(dut.uo_out.value):08b}"


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
    emits full-scale (soft ceiling 20479), but the identity chain passes sample_in = 0x7FFF exactly. Phases, DIP all-off
    throughout:
      - 100 mid-level (0x1000) frames: dp stays dark (the LED is
        sticky for ~2048 frames, so one end-of-phase probe covers all
        100 frames);
      - ONE 0x7FFF frame: dp still dark right after the drive (the hit
        clocks at the frame's sample_valid edge, early in the right
        half), lit by the end of that frame; uo[6:5] and the SPI
        pins (idle: CS0 hold 1, uo[5] 0, CS high, SCK/MOSI 0) stay idle;
      - input returns to 0x1000: dp holds on its own — still lit 1000
        frames later, and at hold-frame 2040 (a half-length hold would
        already be dark; the clear comes at 2048);
      - cleared by clip-frame + 2048 + 64. Frames after the return
        frame advance in bulk (ui[0] idles low — silence, exactly as
        non-hitting as 0x1000; the hold is what's pinned)."""
    await start(dut, 0)     # en=0: DIP all-off -> identity (digital clean)

    def dp():
        return int(dut.uo_out[7].value)

    def seg_ok():
        # uo[6:5] (CS0 hold 1, 0) and the SPI idle (CS high on uo[1],
        # SCK/MOSI 0) — LRCLK/DAC (uo[4:3]) masked: the dp must never
        # disturb them — PORT-level assert, active in GL too
        return (int(dut.uo_out.value) & 0x67) == 0x42

    # 100 mid-level frames (frames 0..99): dp never lights
    for _ in range(100):
        await i2s_send_and_capture(dut, 0x1000 << 8)
        await ClockCycles(dut.clk, 256)
    await Timer(1, unit="ns")
    assert dp() == 0, "mid-level frames must never light the dp"
    assert seg_ok(), "uo idle bits must stay 0x42 with the dp dark"

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
    assert seg_ok(), "uo idle bits must stay 0x42 during the clip"

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
    assert seg_ok(), "uo idle bits must still read 0x42 after the dp clears"


@cocotb.test()
async def test_chain_latency(dut):
    """Task 8 (sequenced chain): pins the chain's frame-level latency
    and the clip-LED timing across the combinational -> 6-step refactor.
    (a) en = comp + dist + trem: three known frames then a zero frame; each
    TX value (captured one frame after its input, mono contract) equals
    golden_model.render of the same input history — the silent settle
    frames are rendered too so env and the trem LFO line up. (b) en = 0:
    a 0x7FFF frame lights uo[7] within CHAIN_MAX_CLK clk of the frame's
    strobe (not before it), well before the frame's TX load of the
    NEXT frame, and a mid-level frame in the same position does not.
    (c) RTL probe, every DIP on (ui_in = 0xFF): chain_done
    pulses once per frame, at most CHAIN_MAX_CLK clk after sample_valid
    (Task 15a: the multiplier steps wait for the serial multiplier)."""
    ins = [12000, -20000, 30000, 0]
    en = EN_COMP | EN_DIST | EN_TREM
    await start(dut, en)
    xs = [0] * START_FRAME + ins
    want = golden_model.render(xs, en)
    for j, x in enumerate(ins):
        bits = await i2s_send_and_capture(dut, (x & 0xFFFF) << 8)
        await _wait_left_start(dut)
        out = tx_data_bits(bits)
        out = out - 0x10000 if out & 0x8000 else out
        # capture j (model frame START_FRAME + j) carries frame (.. + j - 1)'s result
        exp = want[START_FRAME + j - 1]
        assert out == exp, f"frame {j}: got {out}, golden {exp}"

    # (b) LED timing, identity chain
    await start(dut, 0)
    await i2s_send_and_capture(dut, 0x1000 << 8)
    await ClockCycles(dut.clk, CHAIN_MAX_CLK)
    await Timer(1, unit="ns")
    assert int(dut.uo_out[7].value) == 0, "mid-level frame must not light the LED"
    await _wait_left_start(dut)
    await i2s_send_and_capture(dut, 0x7FFF << 8)
    await Timer(1, unit="ns")
    assert int(dut.uo_out[7].value) == 0, "LED lit before the clip frame's valid edge"
    await ClockCycles(dut.clk, CHAIN_MAX_CLK)
    await Timer(1, unit="ns")
    assert int(dut.uo_out[7].value) == 1, \
        f"LED must light within {CHAIN_MAX_CLK} clk of the strobe"

    # (c) chain_done bound, every effect on (RTL probe)
    sv, cd = _probe(dut, "sample_valid"), _probe(dut, "chain_done")
    if sv is not None:
        await start(dut, 0xFF, clock=False)     # every DIP on
        for _ in range(3):
            while int(sv.value) != 1:
                await ClockCycles(dut.clk, 1)
            n = 0
            while int(cd.value) != 1:
                await ClockCycles(dut.clk, 1)
                n += 1
                assert n <= CHAIN_MAX_CLK, f"chain_done > {CHAIN_MAX_CLK} clk after sample_valid"
            dut._log.info(f"chain_done {n} clk after sample_valid")
            await ClockCycles(dut.clk, 1)
            assert int(cd.value) == 0, "chain_done must be a 1-clk pulse"


@cocotb.test()
async def test_tone_lpf(dut):
    """Post-dirt tone (v3 task 11), dist on, K1 absent (kt = 2): the
    one-pole low-pass ytone += ((s3 << 4) - ytone) >>> 2. An alternating
    +-8000 input (Nyquist; dist at x7.97 maps it to +-16274) is attenuated
    to |out| < 3000 (golden: 2325 once the gate is open), and a DC input
    of 8000 settles to golden's 16273 (the floor leaves the 16.4 state a
    few LSB under dist's 16274). Each run is exact against golden for
    every frame, from a fresh reset (the gate opens within ~60 frames)."""
    n = 300
    for i, (name, xs) in enumerate((
            ("nyquist", [8000 if j % 2 == 0 else -8000 for j in range(n)]),
            ("dc", [8000] * n))):
        await start(dut, EN_DIST, clock=(i == 0))   # dist only
        got = await fast_frames(dut, xs + [0])
        want = golden_model.render([0] * START_FRAME + xs, EN_DIST)[START_FRAME:]
        assert got[1:] == want, f"{name}: first mismatch at " \
            f"{next(j for j in range(n) if got[1 + j] != want[j])}"
        if name == "nyquist":
            assert max(abs(v) for v in got[-50:]) < 3000, f"nyquist {got[-6:]}"
            assert got[-1] == -2325
        else:
            assert len(set(got[-10:])) == 1 and got[-1] == 16273, f"dc {got[-10:]}"


@cocotb.test()
async def test_gate_hysteresis(dut):
    """Noise gate (v3 task 11), dist on, K2 absent. Key L = cenv >> 8 of
    the compressor: opens at L >= 256, closes at L < 128. Steps (frame
    counts from golden's ToneGate, the probe on `gate_open` is RTL-only and
    guarded; the output carries the GL coverage):
      1. DC 300 until the gate opens: closed after 61 frames, open after
         the 62nd (L reaches 256 on the pre-edge cenv).
      2. DC 192 for 1000 frames: L decays from 258 to 200 but never
         below 128, so `open` stays 1 (the hysteresis; DC 192 from reset
         would never open the gate at all).
      3. Silence until the close event (1747 frames): open = 1 one frame
         before, 0 at it.
      4. From the close event, DC 100 for 300 frames: gg falls 256 -> 0
         one per frame, so the output (tone * gg >> 8, tone -> 400) is
         still 1 at post-close frame 254 and exactly 0 from frame 255
         on, i.e. after exactly 256 frames (golden); the whole run is
         exact against golden."""
    await start(dut, EN_DIST)                     # dist only
    probe = _probe(dut, "gate_open")
    assert probe is not None or _probe(dut, "step") is None, "RTL has no gate_open"

    def is_open():
        return None if probe is None else int(probe.value)

    ref = golden_model.ToneGate()
    cs = golden_model.Comp()
    hist = [0] * START_FRAME

    def drive(x, k):
        for _ in range(k):
            ref.gate(cs.cenv)
            cs.step(x)
        hist.extend([x] * k)

    assert is_open() in (None, 0)

    # 1. open
    drive(300, 61)
    assert ref.open == 0
    got = await fast_frames(dut, [300] * 61)
    assert is_open() in (None, 0), "gate open one frame early"
    drive(300, 1)
    assert ref.open == 1
    got += await fast_frames(dut, [300])
    assert is_open() in (None, 1), "gate did not open at L >= 256"
    # 2. hysteresis hold
    drive(192, 1000)
    assert ref.open == 1 and 128 <= cs.cenv >> 8 == 200
    got += await fast_frames(dut, [192] * 1000)
    assert is_open() in (None, 1), "gate closed during the L ~ 200 hold"
    # 3. silence until the close event
    n_close = 0
    while ref.open:
        drive(0, 1)
        n_close += 1
    assert n_close == 1747
    await ClockCycles(dut.clk, 512 * (n_close - 1))
    assert is_open() in (None, 1), "gate closed a frame early"
    await ClockCycles(dut.clk, 512)
    assert is_open() in (None, 0), "gate did not close at L < 128"
    # 4. gg decay, observed on the output
    drive(100, 300)
    got_dc = await fast_frames(dut, [100] * 300 + [0])
    want = golden_model.render(hist, EN_DIST)
    tail = want[-300:]
    assert tail[254] > 0 and tail[255] == 0 and not any(tail[255:]), "golden gg decay"
    assert got_dc[1:] == tail, "decay run differs from golden at " \
        f"{next(j for j in range(300) if got_dc[1 + j] != tail[j])}"
    # the loud phases' captures agree with golden too (capture j = frame j-1)
    assert got[1:] == want[START_FRAME:START_FRAME + len(got) - 1]


@cocotb.test()
async def test_dirt_off_identity(dut):
    """Tone/gate act only when dist is on (v3 task 11). With trem
    only (en[4]) the chain output equals golden render bit-exactly,
    +-full-scale and -32768 included, and equals the render with the
    tone/gate stage skipped (tgb): the 16.4 tone state runs every frame
    but never reaches the output. With phaser only (Task 16) the words
    equal golden render (the stub's old identity no longer holds)."""
    xs = [32767, -32768, 1000, -1, 0, -32768, 12345, 0]
    await start(dut, EN_TREM)                     # trem only
    got = await fast_frames(dut, xs + [0])
    want = golden_model.render([0] * START_FRAME + xs, EN_TREM)[START_FRAME:]
    assert want == golden_model.render([0] * START_FRAME + xs, EN_TREM, tgb=True)[START_FRAME:]
    assert got[1:] == want, f"trem only: {got[1:]} != {want}"
    await start(dut, EN_PHASER, clock=False)        # phaser only
    got = await fast_frames(dut, xs + [0])
    want = golden_model.render([0] * START_FRAME + xs, EN_PHASER)[START_FRAME:]
    assert got[1:] == want, f"phaser only: {got[1:]} != {want}"


def _sweep(n, f0, f1, amp):
    """Exponential sine sweep f0 -> f1 Hz over n frames at amplitude amp."""
    import math
    out, phi = [], 0.0
    for i in range(n):
        f = f0 * (f1 / f0) ** (i / n)
        phi += 2 * math.pi * f / 48000
        out.append(int(round(amp * math.sin(phi))))
    return out


def _first_diff(got, want):
    return next((i for i in range(min(len(got), len(want))) if got[i] != want[i]), None)


@cocotb.test()
async def test_phaser_identity_off(dut):
    """en[3] = 0: the phaser slot passes s4 bit-exactly. (a) en = 0: a
    sweep plus +-full-scale words come out unchanged (s4 = the input),
    while the stage states still run (RTL probe ph_y3: the last stage's
    output state is non-zero, the phaser is warm). (b) dist on (the
    tone/gate in): the port words equal golden render, i.e. s4."""
    xs = _sweep(300, 200, 4000, 8000) + [32767, -32768, 32767, -32768, 0]
    await start(dut, 0)
    got = await fast_frames(dut, xs + [0])
    assert got[1:] == xs, f"en=0 not identity at frame {_first_diff(got[1:], xs)}"
    y3 = _probe(dut, "ph_y3")
    if y3 is not None:
        ref = golden_model.Chain()
        for x in [0] * START_FRAME + xs + [0]:
            ref.step(x, 0)
        # warm: the state saw the input although en[3] is off
        assert ref.ph.y1[3] != 0
        assert _signed(int(y3.value)) == ref.ph.y1[3], \
            f"ph_y3 {_signed(int(y3.value))} != golden {ref.ph.y1[3]}"
    await start(dut, EN_DIST, clock=False)
    xs2 = _sweep(200, 200, 4000, 8000)
    got = await fast_frames(dut, xs2 + [0])
    want = golden_model.render([0] * START_FRAME + xs2, EN_DIST)[START_FRAME:]
    assert got[1:] == want, f"dist, phaser off: frame {_first_diff(got[1:], want)}"


@cocotb.test()
async def test_phaser_coef(dut):
    """phaser_coef: pp = 0 -> 3840 (d = 256, lowest notches), 2^20 -> 3328,
    2^21 - 1 -> 2049 (golden). RTL probe: phaser_c equals golden
    phaser_coef(phaser_p) at those three phases (deposited into the
    phase register just after a valid edge) and on 5 live frames."""
    assert golden_model.phaser_coef(0) == 3840
    assert golden_model.phaser_coef(1 << 20) == 3328
    assert golden_model.phaser_coef((1 << 21) - 1) == 2049
    assert golden_model.phaser_coef(1 << 21 | 0x1FF) == golden_model.phaser_coef((1 << 21) - 1)
    await start(dut, EN_PHASER)                       # phaser, K1 absent
    pp, pc, sv = _probe(dut, "phaser_p"), _probe(dut, "phaser_c"), _probe(dut, "sample_valid")
    if pp is None:
        return
    for _ in range(5):
        await fast_frames(dut, [1000])
        c, p = int(pc.value), int(pp.value)
        assert c == golden_model.phaser_coef(p), f"pp {p}: c {c} != {golden_model.phaser_coef(p)}"
    for p, want in ((0, 3840), (1 << 20, 3328), ((1 << 21) - 1, 2049)):
        while int(sv.value) != 1:
            await ClockCycles(dut.clk, 1)
        await ClockCycles(dut.clk, 1)
        pp.value = p
        await Timer(1, unit="ns")
        assert int(pc.value) == want, f"pp {p}: c {int(pc.value)} != {want}"
        await ClockCycles(dut.clk, 1)
        await Timer(1, unit="ns")
        assert int(pp.value) == p and int(pc.value) == want


@cocotb.test()
async def test_phaser_sweep(dut):
    """Phaser on, K3 = 250 (inc 976): a 200 Hz - 4 kHz exponential sweep at
    amplitude 8000 for 4000 frames; every port word equals golden."""
    knob_pin(dut, 2, 250)
    await start(dut, EN_PHASER)
    xs = _sweep(4000, 200, 4000, 8000)
    got = (await fast_frames(dut, xs + [0]))[1:]
    want = golden_model.render([0] * START_FRAME + xs, EN_PHASER, k3=250)[START_FRAME:]
    first = _first_diff(got, want)
    assert first is None, f"frame {first}: got {got[first]} want {want[first]}"
    assert got != xs and len(set(got)) > 1000, "phaser must change the signal"


@cocotb.test()
async def test_phaser_rails(dut):
    """Phaser on, K3 absent: a +-32767 square at 100 Hz (240 frames per
    half) for 2000 frames equals golden every frame, and never wraps sign
    against it (the sat16 stage sums). RTL probe: the phase steps by
    PHASER_INC_ABSENT = 87 per frame."""
    await start(dut, EN_PHASER)
    xs = [32767 if (i // 240) % 2 == 0 else -32767 for i in range(2000)]
    got = (await fast_frames(dut, xs + [0]))[1:]
    want = golden_model.render([0] * START_FRAME + xs, EN_PHASER)[START_FRAME:]
    wraps = [i for i, (g, w) in enumerate(zip(got, want)) if (g < 0) != (w < 0) and g and w]
    assert not wraps, f"sign wraps vs golden at frames {wraps[:5]}"
    first = _first_diff(got, want)
    assert first is None, f"frame {first}: got {got[first]} want {want[first]}"
    assert max(want) > 30000 and min(want) < -30000, "rails test must reach the rails"
    pp = _probe(dut, "phaser_p")
    if pp is not None:
        ps = []
        for _ in range(4):
            ps.append(int(pp.value))
            await fast_frames(dut, [0])
        d = [(b - a) & 0x3FFFFF for a, b in zip(ps, ps[1:])]
        assert d == [golden_model.PHASER_INC_ABSENT] * 3, f"phase steps {d}"


@cocotb.test()
async def test_phaser_rate_k3(dut):
    """K3 = 250 sets the phaser rate (shared with trem): RTL probe, the
    phaser phase advances by phaser_inc(250) = 976 per frame over 5 frames
    and equals 976 * frames since EN_SETTLE; port level, 20 words of a
    constant-ish input equal golden (k3 = 250)."""
    assert golden_model.phaser_inc(250) == golden_model.trem_inc(250) == 976
    assert golden_model.phaser_inc(None) == golden_model.PHASER_INC_ABSENT == 87
    knob_pin(dut, 2, 250)
    await start(dut, EN_PHASER)
    pp = _probe(dut, "phaser_p")
    nf = 0
    if pp is not None:
        ps = []
        for _ in range(6):
            ps.append(int(pp.value))
            await ClockCycles(dut.clk, 500)
            await _wait_left_start(dut)
        nf = 6
        assert ps[0] == 976 * (START_FRAME - golden_model.EN_SETTLE), f"phase {ps[0]}"
        d = [(b - a) & 0x3FFFFF for a, b in zip(ps, ps[1:])]
        assert d == [976] * 5, f"phase steps {d}"
    f0 = START_FRAME + nf
    xs = _sweep(20, 500, 600, 12000)
    want = golden_model.render([0] * f0 + xs, EN_PHASER, k3=250)[f0:]
    got = (await fast_frames(dut, xs + [0]))[1:]
    assert got == want, f"got {got} want {want}"


@cocotb.test()
async def test_mult_corners(dut):
    """Serial multiplier in isolation (tb.v's testbench-only mult_tb
    instance, RTL only): p == a * b exactly, done 12 clk after start and
    1 clk wide, for a in {65535, -65535, -32768, -1, 0, 1} x b in {0, 1, 3840, 4095}
    (+-65535 = the largest |y1 - s| the phaser stages feed it; 3840 = the
    largest phaser coefficient)."""
    if not hasattr(dut, "mt_a"):
        return
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_PS, unit="ps").start())
    dut.mt_start.value = 0
    dut.mt_a.value = 0
    dut.mt_b.value = 0
    await _bare_reset(dut)
    await ClockCycles(dut.clk, 2)
    for a in (65535, -65535, -32768, -1, 0, 1):
        for b in (0, 1, 3840, 4095):
            await Timer(1, unit="ns")
            dut.mt_a.value = a & 0x1FFFF
            dut.mt_b.value = b
            dut.mt_start.value = 1
            await ClockCycles(dut.clk, 1)
            await Timer(1, unit="ns")
            dut.mt_start.value = 0
            dut.mt_a.value = 0x15555               # operands are captured: scramble
            dut.mt_b.value = 0xAAA
            n = 0
            while int(dut.mt_done.value) != 1:
                await ClockCycles(dut.clk, 1)
                await Timer(1, unit="ns")
                n += 1
                assert n <= 20, "mult done never came"
            v = int(dut.mt_p.value)
            v = v - (1 << 29) if v & (1 << 28) else v
            assert n == 12, f"latency {n} (a {a}, b {b})"
            assert v == a * b, f"{a} * {b}: p {v}"
            await ClockCycles(dut.clk, 1)
            await Timer(1, unit="ns")
            assert int(dut.mt_done.value) == 0, "done must be a 1-clk pulse"
            v2 = int(dut.mt_p.value)
            assert (v2 - (1 << 29) if v2 & (1 << 28) else v2) == a * b, "p must hold after done"



# ---------------------------------------------------------------------
# PSRAM SPI engine (Rev B3, Task 21, src/psram_spi.v) against the
# behavioral APS6404L model test/psram_model.v (tb: psram_en = fitted,
# psram_so_dly_ps = SO delay, psram_err = protocol error). Bus: uo[0] SCK
# = clk/2, uo[1] CE#, uo[2] MOSI, MISO on the dedicated uio[7] (tb: SO
# while the model drives it, else ui_in[7]). SpiMon decodes
# every CE# low window at the pins (RTL and GL): MOSI and MISO are taken at
# each SCK rise (MISO is exact there for so_dly 0: the model launches on
# the fall). Internal probes (ok, phase, rdata, dly_widx, sw) are RTL-only.
# ---------------------------------------------------------------------

PSRAM_TCE_MAX_CLK = 49 * 2 + 2          # CE# low per transaction <= 49 SCK + 2 clk
PSRAM_PU_PS = 150_000_000               # >= 150 us after power-up before the first command
PROBE_W = [(0x010000, 0xA5C3), (0x010002, 0x5A3C)]


class SpiMon:
    def __init__(self, dut):
        self.dut = dut
        self.txns = []
        self.cur = None
        self.t_rise = None
        self.min_gap_clk = None
        cocotb.start_soon(self._run())

    async def _run(self):
        # prev starts at the idle bus level (CE# high, SCK/MOSI low), not
        # None: the monitor may be created while the first transaction is
        # already falling CE#, and a None prev would swallow it (the
        # 0x66 reset-enable would be missing from txns).
        prev = 0b10
        while True:
            await ValueChange(self.dut.uo_out)
            v = self.dut.uo_out.value
            if not v.is_resolvable:
                continue
            v = int(v)
            now = get_sim_time(unit="ps")
            if prev is not None:
                if prev & 2 and not v & 2:            # CE# fall
                    if self.t_rise is not None:
                        g = (now - self.t_rise) / CLK_PERIOD_PS
                        self.min_gap_clk = g if self.min_gap_clk is None else min(self.min_gap_clk, g)
                    self.cur = {"t0": now, "mosi": [], "miso": []}
                elif not prev & 2 and v & 2 and self.cur is not None:   # CE# rise
                    c = self.cur
                    c["clk"] = round((now - c["t0"]) / CLK_PERIOD_PS)
                    r = _probe(self.dut, "psram", "rdata")
                    c["rdata"] = int(r.value) if r is not None else None
                    self.txns.append(self._decode(c))
                    self.cur = None
                    self.t_rise = now
                elif not v & 2 and v & 1 and not prev & 1 and self.cur is not None:  # SCK rise
                    self.cur["mosi"].append(prev >> 2 & 1)
                    m = self.dut.miso_pin.value
                    self.cur["miso"].append(int(m) if m.is_resolvable else 0)
            prev = v

    @staticmethod
    def _decode(c):
        b, q = c["mosi"], c["miso"]
        word = lambda bits: int("".join(map(str, bits)), 2) if bits else None
        c["n"] = len(b)
        c["cmd"] = word(b[:8])
        c["addr"] = word(b[8:32])
        c["data"] = word((b if c["cmd"] == 0x02 else q)[32:48])
        return c

    def summary(self, txns=None):
        return [(f"{t['cmd']:02x}", t["n"], None if t["addr"] is None else f"{t['addr']:06x}",
                 None if t["data"] is None else f"{t['data']:04x}", t["clk"])
                for t in (self.txns if txns is None else txns)]


def _probe_seq(phases):
    """Expected (cmd, nbits, addr, data) of the power-up probe: reset
    enable, reset, the two pattern writes, then one read pair per tried
    sample phase."""
    seq = [(0x66, 8, None, None), (0x99, 8, None, None)]
    seq += [(0x02, 48, a, d) for a, d in PROBE_W]
    seq += [(0x03, 48, a, d) for a, d in PROBE_W] * phases
    return seq


def _key(t, data=True):
    return (t["cmd"], t["n"], t["addr"], t["data"] if data else None)


async def psram_start(dut, ui=0, present=True, so_dly_ps=0, uio=0):
    """Clock + bare reset with the PSRAM model fitted or not; returns
    (SpiMon, reset-release time ps)."""
    cocotb.start_soon(
        Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2).start())
    dut.psram_en.value = int(present)
    dut.psram_so_dly_ps.value = so_dly_ps
    dut.ena.value = 1
    dut.ui_in.value = ui
    dut.uio_in.value = uio
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    mon = SpiMon(dut)
    dut.rst_n.value = 1
    return mon, get_sim_time(unit="ps")


def psram_off(dut):
    dut.psram_en.value = 0
    dut.psram_so_dly_ps.value = 0


def _check_bus(dut, mon):
    """Every transaction: 8 or 48 SCK, CE# low <= 49 SCK + 2 clk, CE# high
    >= 2 clk between transactions, no model protocol error."""
    for t in mon.txns:
        assert t["n"] in (8, 48), f"transaction of {t['n']} SCK: {mon.summary([t])}"
        assert t["clk"] <= PSRAM_TCE_MAX_CLK, f"CE# low {t['clk']} clk > {PSRAM_TCE_MAX_CLK}"
    assert mon.min_gap_clk is None or mon.min_gap_clk >= 2, f"CE# high gap {mon.min_gap_clk} clk"
    assert int(dut.psram_err.value) == 0, f"model protocol error code {int(dut.psram_err_code.value)}"


def _go_pairs(txns):
    """(write, read) pairs of the go traffic (non-probe addresses)."""
    t = [x for x in txns if x["n"] == 48 and x["addr"] < 0x010000]
    pairs = []
    for i, w in enumerate(t):
        if w["cmd"] != 0x02:
            continue                 # a read whose write fell before this window
        if i + 1 < len(t):
            assert t[i + 1]["cmd"] == 0x03, "go write not followed by its read"
            pairs.append((w, t[i + 1]))
    return pairs


def _ok(dut, mon=None):
    o = _probe(dut, "psram", "ok")
    return int(o.value) if o is not None else None


@cocotb.test()
async def test_psram_probe_ok(dut):
    """Model fitted, SO delay 0: >= 150 us after reset the engine sends
    0x66, 0x99, writes 0xA5C3 @ 0x010000 and 0x5A3C @ 0x010002, reads both
    back with sample phase 0 (one read pair), sets ok within 20 frames and
    then runs one write + one read per frame (go). CE# low 8 / 48 SCK
    within the 49 SCK + 2 clk bound, model protocol flag clear."""
    try:
        mon, t_rst = await psram_start(dut)
        await ClockCycles(dut.clk, 20 * 512)
        assert mon.txns, "no PSRAM traffic within 20 frames"
        assert mon.txns[0]["t0"] - t_rst >= PSRAM_PU_PS, \
            f"first command {(mon.txns[0]['t0'] - t_rst) / 1e6:.1f} us after reset (< 150 us)"
        seq = [_key(t) for t in mon.txns[:6]]
        assert seq == _probe_seq(1), f"probe sequence {mon.summary(mon.txns[:6])}"
        assert len(_go_pairs(mon.txns[6:])) >= 2, f"no go traffic after the probe: {mon.summary()}"
        mem = dut.psram.mem
        assert [int(mem[0x10000 + i].value) for i in range(4)] == [0xA5, 0xC3, 0x5A, 0x3C]
        if _ok(dut) is not None:
            assert _ok(dut) == 1 and int(_probe(dut, "psram", "phase").value) == 0
        _check_bus(dut, mon)
        dut._log.info(f"probe: {mon.summary(mon.txns[:8])}")
    finally:
        psram_off(dut)


@cocotb.test()
async def test_psram_probe_phase1(dut):
    """SO delayed 45 ns (> half an SCK, 40.7 ns): the phase-0 read pair
    fails, the reads repeat with phase 1 (sampled at the SCK fall) and
    pass: ok within 20 frames, phase = 1, go traffic follows."""
    try:
        mon, _ = await psram_start(dut, so_dly_ps=45_000)
        await ClockCycles(dut.clk, 20 * 512)
        seq = [_key(t, data=False) for t in mon.txns[:8]]
        want = [(c, n, a, None) for c, n, a, _ in _probe_seq(2)]
        assert seq == want, f"probe sequence {mon.summary(mon.txns[:8])}"
        assert len(_go_pairs(mon.txns[8:])) >= 2, f"no go traffic after the probe: {mon.summary()}"
        if _ok(dut) is not None:
            assert _ok(dut) == 1 and int(_probe(dut, "psram", "phase").value) == 1
        _check_bus(dut, mon)
    finally:
        psram_off(dut)


@cocotb.test()
async def test_psram_absent(dut):
    """No PSRAM: uio[7] (the MISO) is just the tb's drive level, so the
    probe reads 0x0000 (low) or 0xFFFF (high) and fails with both phases:
    ok = 0, no go traffic. With the delay switch (ui[4]) on, the debounced en[4] rises at
    the debounce (strobe 512) and reprobes once, which fails again.
    CE# then stays high."""
    try:
        for sw in (0, 1):
            # ui[4] = the delay enable (drives the reprobe), uio[7] = the
            # MISO level the absent-model probe reads back
            mon, _ = await psram_start(dut, ui=sw << 4, present=False, uio=sw << 7)
            await ClockCycles(dut.clk, 30 * 512)
            got = [_key(t) for t in mon.txns]
            rd = 0xFFFF if sw else 0x0000
            want = [(c, n, a, d if c != 0x03 else rd) for c, n, a, d in _probe_seq(2)]
            assert got == want, f"sw {sw}: {mon.summary()}"
            if sw:
                await ClockCycles(dut.clk, 600 * 512)
                more = mon.txns[len(want):]
                assert more, "en[4] rising must reprobe"
                assert [_key(t) for t in more[-6:]] == want[-6:], f"reprobe {mon.summary(more)}"
                assert not [t for t in more if t["n"] == 48 and t["addr"] < 0x010000], "go traffic"
            n = len(mon.txns)
            await ClockCycles(dut.clk, 100 * 512)
            assert len(mon.txns) == n and int(dut.uo_out.value) >> 1 & 1, \
                f"sw {sw}: CE# activity after the failed probe"
            if _ok(dut) is not None:
                assert _ok(dut) == 0
            _check_bus(dut, mon)
    finally:
        psram_off(dut)


@cocotb.test()
async def test_psram_status_pin(dut):
    """Rev B5 status pin: uo[5] = PSRAM detected (the engine's ok; seg f
    on the demoboard). Model fitted: 0 from reset through the probe (it
    starts after 16 frames), 1 within 20 frames and held while the go
    traffic runs. No PSRAM (MISO low and high): 0 for 30 frames."""
    try:
        mon, _ = await psram_start(dut)
        for f in range(16):
            await ClockCycles(dut.clk, 512)
            assert int(dut.uo_out[5].value) == 0, f"uo[5] high at frame {f + 1}, before the probe"
        await ClockCycles(dut.clk, 4 * 512)
        for f in range(10):
            assert int(dut.uo_out[5].value) == 1, f"uo[5] low {f} frames after the probe passed"
            if _ok(dut) is not None:
                assert _ok(dut) == 1
            await ClockCycles(dut.clk, 512)
        assert _go_pairs(mon.txns), "no go traffic while uo[5] is high"
    finally:
        psram_off(dut)
    for miso in (0, 1):
        await psram_start(dut, present=False, uio=miso << 7)
        for f in range(30):
            await ClockCycles(dut.clk, 512)
            assert int(dut.uo_out[5].value) == 0, f"uo[5] high with no PSRAM (MISO {miso}), frame {f + 1}"


@cocotb.test()
async def test_psram_rw(dut):
    """Write / read round trip through go (Task 21 wiring, wdata now
    quant12'd by the Task 22 delay: waddr = the ring write index dly_widx,
    raddr = (widx + 1 - dly) mod 32768). Effects off (identity). RTL:
    dly_len is forced to 1 so raddr = widx and every go reads back the
    word it just wrote. GL (no force): dly = 16800, so the read lands
    16799 words behind the write and must return the model's word there. Three addresses (RTL: dly_widx set to 0x0005, 0x2AAA,
    0x7FFE so the third round covers the top word and the 15-bit wrap; GL:
    consecutive). Checked at the pins (write data, read address, read data
    on SO), in the model's memory, and (RTL) in rdata."""
    try:
        mon, _ = await psram_start(dut)
        await ClockCycles(dut.clk, 22 * 512)
        await _wait_left_start(dut)
        wp = _probe(dut, "dly_widx")
        dl = _probe(dut, "dly_len")
        if dl is not None:
            dl.value = Force(1)
        # quant12-invariant words (Task 22: the engine writes quant12(effect_sample))
        words = [0x1230, 0xBEF0, 0x7000]
        rounds = []
        for k, (base, w) in enumerate(zip((0x0005, 0x2AAA, 0x7FFE), words)):
            if wp is not None:
                wp.value = base
            n0 = len(mon.txns)
            await fast_frames(dut, [w, 0, 0])
            rounds.append(mon.txns[n0:])
        for w, txns in zip(words, rounds):
            pairs = _go_pairs(txns)
            wr = [p for p in pairs if p[0]["data"] == w]
            assert wr, f"{w:#06x} never written: {mon.summary(txns)}"
            a = wr[0][0]["addr"]
            rd = wr[0][1]                   # the same go's read (write, then read)
            assert rd is not None, "no read after the write"
            mem = dut.psram.mem
            assert (int(mem[a].value) << 8 | int(mem[a + 1].value)) == w
            ra = a if dl is not None else (a + 2 * (1 - golden_model.DELAY_ABSENT)) % 0x10000
            want = w if dl is not None else (int(mem[ra].value) << 8 | int(mem[ra + 1].value))
            assert rd["addr"] == ra, f"read {rd['addr']:#08x} != {ra:#08x} (write {a:#08x})"
            assert rd["data"] == want, f"SO read {rd['data']:#06x} != {want:#06x}"
            if rd["rdata"] is not None:
                assert rd["rdata"] == want, f"rdata {rd['rdata']:#06x} != {want:#06x}"
            dut._log.info(f"round trip {w:#06x} @ byte {a:#08x}")
        if wp is not None:
            addrs = [_go_pairs(t)[[p[0]["data"] for p in _go_pairs(t)].index(w)][0]["addr"]
                     for w, t in zip(words, rounds)]
            assert addrs == [0x000C, 0x5556, 0xFFFE], [hex(x) for x in addrs]
        _check_bus(dut, mon)
    finally:
        dl = _probe(dut, "dly_len")
        if dl is not None:
            dl.value = Release()
        psram_off(dut)


@cocotb.test()
async def test_delay_enable_switch(dut):
    """ui[4] = the delay enable, a plain debounced switch (Rev B3 final;
    the MISO is uio[7]). With the PSRAM fitted and running (all-ones
    samples written and read back on uio[7]), switching ui[4] on must
    reach en[4] at the debounce (strobe 512) and must NOT reprobe the
    PSRAM (Rev B3 final: the reprobe fires only while no chip is
    detected, ok = 0 — test_psram_absent; a working chip keeps its ring),
    and go traffic keeps running; off must not reprobe either. The SO traffic on uio[7] must never reach the enables (it
    is not wired to the debouncer at all)."""
    try:
        mon, _ = await psram_start(dut)
        await ClockCycles(dut.clk, 22 * 512)
        await _wait_left_start(dut)
        sw = _probe(dut, "sw")
        en5 = []

        async def run(level, word, n):
            dut.ui_in.value = level << 4
            for _ in range(n):
                await fast_frames(dut, [word])
                if sw is not None:
                    en5.append(int(sw.value) >> 4 & 1)

        def probes():
            return [t for t in mon.txns if t["cmd"] == 0x02 and t["addr"] == 0x010000]

        await run(0, 0xFFFF, 600)
        assert len(probes()) == 1, "reprobe with the switch off"
        assert not any(en5), "en[4] on with the switch off"
        k = len(en5)
        await run(1, 0x0000, 700)
        assert len(probes()) == 1, f"switch on with a working chip must not reprobe ({len(probes())})"
        if sw is not None:
            on = en5[k:]
            first = on.index(1)
            assert all(on[first:]), "en[4] fell with the switch on"
        last = mon.txns[-6:]
        assert len(_go_pairs(last)) >= 2 and last[-1]["t0"] > probes()[-1]["t0"], "go traffic with the delay on"
        k = len(en5)
        await run(0, 0xFFFF, 700)
        assert len(probes()) == 1, "switch off must not reprobe"
        if sw is not None:
            off = en5[k:]
            first = off.index(0)
            assert not any(off[first:]), "en[4] rose with the switch off"
        _check_bus(dut, mon)
    finally:
        psram_off(dut)


# ---------------------------------------------------------------------
# Delay DSP (Rev B3, Task 22; golden_model.Delay). The ring lives in the
# external PSRAM (the engine writes quant12(out) at widx and reads
# (widx + 1 - dly) mod 32768 after chain_done of every frame). dly slews
# +/-1 frame toward the settled K4 target from EN_SETTLE (absent 16800).
# rd (the previous frame's read) is gated by ok and filled; filled after
# FILL_FRAMES writes. The golden models the testbench PSRAM's LFSR fill
# so pre-fill reads are bit-exact too. The FILL_FRAMES tests need the
# Makefile override (make -B FILL_FRAMES=64): the GL netlist has no
# parameters and keeps 32768 (the wav_tool check covers the delay there).
# ---------------------------------------------------------------------


def _fill_param(dut):
    """The RTL FILL_FRAMES parameter (None on the flat GL netlist).
    tb.v declares FILL_FRAMES = 64 even under GL_TEST (it only skips the
    override there), so the GL case is detected from the DUT itself: the
    flat netlist has no internal dly_len and keeps FILL_FRAMES = 32768."""
    if _probe(dut, "dly_len") is None:
        return None
    try:
        return int(dut.FILL_FRAMES.value)
    except AttributeError:
        return None


async def _delay_hist(dut, hist, xs, compare=True):
    """Drive xs one frame each (from a left-half start) against the
    golden Chain history; returns the outputs (capture j = the result of
    input frame base + j - 1, as fast_frames does) and the golden list."""
    base = len(hist.xs)
    outs = await fast_frames(dut, xs)
    hist.xs += list(xs)
    want = golden_model.render(hist.xs, hist.en, k2=hist.k2, k4=hist.k4,
                               fill=hist.fill, psram=hist.psram)
    for j, o in enumerate(outs if compare else []):
        assert o == want[base - 1 + j], \
            f"frame {base - 1 + j}: out {o} != golden {want[base - 1 + j]}"
    return outs, want


class DelayHist:
    """Frame history of a delay test: the golden Chain (with the Delay
    stage) over [0] * START_FRAME + the driven frames."""

    def __init__(self, en, k4=None, fill=32768, psram=True, k2=None):
        self.en, self.k4, self.fill, self.psram, self.k2 = en, k4, fill, psram, k2
        self.xs = [0] * START_FRAME


async def _dly_w_frames(dut, hist, xs):
    """_delay_hist one frame at a time, also returning the ring write
    word (RTL probe dly_w, held from step 12 to the next frame's step 12)
    after each frame."""
    ws = []
    for x in xs:
        await _delay_hist(dut, hist, [x])
        ws.append(int(_probe(dut, "dly_w").value))
    return ws


@cocotb.test()
async def test_echo_level(dut):
    """Rev B5: K2 sets the echo level, out = sat16(dry + ((lp * v2) >> 8))
    (K2 absent: B4's ~0.39 mix). Needs make -B FILL_FRAMES=64: the ring is
    then `filled` at frame ~81 and every read returns the PSRAM model's
    full-scale LFSR fill (dly is ~16800, nothing written that far back),
    so lp is busy every frame. Per K2 in (0, 128, 255, absent), fresh
    reset: 200 zero frames then a +-32767 square (100 frames): every word
    == golden; K2 = 0 -> output == dry exactly; and (golden) the K2 = 255
    echo is >= 2.4x louder than the absent one; no wrap at the rails."""
    kp2 = knob_pin(dut, 1, None)
    try:
        dut.psram_en.value = 1
        dut.psram_so_dly_ps.value = 0
        fill = None
        sums = {}
        sq = [32767 if (i // 10) % 2 == 0 else -32767 for i in range(100)]
        for i, v in enumerate((0, 128, 255, None)):
            kp2.v = v
            await start(dut, EN_DELAY, clock=(i == 0))
            fill = _fill_param(dut)
            if fill != 64:
                return              # plain suite / GL: the wav_tool delay_k2* modes cover it
            hist = DelayHist(EN_DELAY, fill=fill, k2=v)
            outs, want = await _delay_hist(dut, hist, [0] * 200 + sq)
            echo = outs[1:201]
            if v == 0:
                assert outs[1:] == [0] * 200 + sq[:99], "K2 = 0 must be dry only"
            else:
                assert any(echo), f"K2 {v}: no echo (lp never moved)"
            assert all(-32768 <= o <= 32767 for o in outs)
            sums[v] = sum(abs(o) for o in echo)
        assert sums[255] >= 2.4 * sums[None], f"level 255 vs absent {sums}"
    finally:
        kp2.v = None                # leave K2 absent for later tests
        dut.psram_en.value = 0       # and the PSRAM unfitted (the tb default)


@cocotb.test()
async def test_echo_level_feedback_fixed(dut):
    """Rev B5: the level only scales what is heard; the ring write word
    (repeats) stays B4's quant12(sat16(dry + ~0.39 lp)). RTL probe dly_w
    after each of 120 frames (FILL_FRAMES=64, ring reading the LFSR fill):
    K2 = 0 and K2 absent write identical words (non-zero), while the
    outputs differ (dry vs echo). Then freeze on at K2 = 0: the output is
    dry (silent here) but the ring still recirculates (dly_w = quant12 of
    the word just read, non-zero)."""
    kp2 = knob_pin(dut, 1, None)
    try:
        dut.psram_en.value = 1
        dut.psram_so_dly_ps.value = 0
        words = {}
        for i, v in enumerate((0, None)):
            kp2.v = v
            await start(dut, EN_DELAY, clock=(i == 0))
            fill = _fill_param(dut)
            if fill != 64:
                return
            hist = DelayHist(EN_DELAY, fill=fill, k2=v)
            words[v] = await _dly_w_frames(dut, hist, [0] * 120)
        assert words[0] == words[None], "K2 changed the ring write word (repeats)"
        assert any(words[0]), "ring write words all zero: the test does not bite"
        kp2.v = 0
        await start(dut, EN_DELAY | EN_FREEZE, clock=False)
        hist = DelayHist(EN_DELAY | EN_FREEZE, fill=64, k2=0)
        ws = await _dly_w_frames(dut, hist, [0] * 120)
        assert any(ws), "frozen ring must still recirculate at K2 = 0"
    finally:
        kp2.v = None                # leave K2 absent for later tests
        dut.psram_en.value = 0       # and the PSRAM unfitted (the tb default)


@cocotb.test()
async def test_delay_off_identity(dut):
    """Delay off, PSRAM fitted and running: bit-exact dry output while the
    ring fills with quant12(x) behind the scenes (the output is the
    input). GL-safe (no parameter override needed)."""
    await start(dut, 0)
    hist = DelayHist(0)
    xs = [0, 16384, -24576, 0, 12345, -12345] * 30
    await _delay_hist(dut, hist, xs)


@cocotb.test()
async def test_delay_no_psram(dut):
    """Delay on, no PSRAM chip (tb psram_en = 0): the probe fails, ok stays
    0, no ring traffic: bit-exact dry output for 5000 frames."""
    dut.psram_en.value = 0          # absent BEFORE reset: the probe runs at frame 16
    dut.psram_so_dly_ps.value = 0
    await start(dut, EN_DELAY)
    hist = DelayHist(EN_DELAY, psram=False)
    xs = [0, 16384, -24576, 0, 12345, -12345] * 400
    await _delay_hist(dut, hist, xs)


@cocotb.test()
async def test_delay_echo(dut):
    """Impulse into the enabled delay (EN_DELAY, K4 absent, PSRAM fitted).
    Output == golden every frame (the golden models the PSRAM's LFSR fill
    for the pre-echo reads). FILL_FRAMES must be overridden to 64
    (make -B FILL_FRAMES=64): `filled` then happens at frame ~80 instead
    of ~32784, so the impulse echo (~16800 frames later) is reachable in
    sim. The echo frame is pinned purely in the golden: the same sequence
    with the impulse zeroed first differs exactly where the echo lands
    (the impulse word is read dly-1 frames after it was written).
    With the default FILL_FRAMES (the plain suite run) only the pre-fill
    window is checked; the full-riff bit-exact echo is the wav_tool
    check's delay modes at the real FILL_FRAMES."""
    dut.psram_en.value = 1          # fitted before start: the probe runs at frame 16
    dut.psram_so_dly_ps.value = 0
    await start(dut, EN_DELAY)    # the delay enable (ui[4])
    fill = _fill_param(dut)
    if fill is None:
        return                      # GL netlist: the check covers the delay
    n_in = 40 + fill
    hist = DelayHist(EN_DELAY, fill=fill)
    xs = [0] * 40 + [16384] + [0] * (n_in - 41)
    await _delay_hist(dut, hist, xs)
    if fill >= 32768:
        return                      # short window only (the suite run)
    # where the golden puts the echo (pure model, no extra RTL run)
    tail = [0] * (golden_model.DELAY_ABSENT + 200)
    want = golden_model.render([0] * START_FRAME + xs + tail, EN_DELAY, fill=fill)
    xs0 = list(xs)
    xs0[40] = 0
    want0 = golden_model.render([0] * START_FRAME + xs0 + tail, EN_DELAY, fill=fill)
    imp = START_FRAME + 40          # the direct impulse frame (dry path)
    first = next((j for j in range(imp + 1, len(want)) if want[j] != want0[j]), None)
    assert first is not None, "no impulse echo in the golden"
    # first is an absolute render frame; the impulse sits at START_FRAME+40
    at = first - START_FRAME - 40
    assert abs(at - (golden_model.DELAY_ABSENT - 1)) <= 2, \
        f"echo lands {at} frames after the impulse, expected ~{golden_model.DELAY_ABSENT - 1}"


@cocotb.test()
async def test_delay_time_slew(dut):
    """K4 = 10 from reset: dly holds at 16800 until EN_SETTLE, then slews
    exactly -1 frame per frame toward target(10) = 2072, matching
    golden_model.Delay's dly; then K4 moves to 200 mid-run and dly still
    moves at most 1 frame per frame (RTL-only: the dly probe; the GL
    check covers the slew via the delay_k4* modes)."""
    dp = _probe(dut, "dly_len")
    if dp is None:
        return                      # GL netlist: no internal probe
    dut.psram_en.value = 1
    dut.psram_so_dly_ps.value = 0
    kp = knob_pin(dut, 3, 10)       # K4 = uio[5], v 10
    await start(dut, EN_DELAY)
    hist = DelayHist(EN_DELAY, k4=10, fill=_fill_param(dut))

    def gold_dly(n):
        d = golden_model.Delay(10, hist.fill)
        for _ in range(n):
            d.step(0, True)
        return d.dly

    await _delay_hist(dut, hist, [0] * 560)
    d0 = int(dp.value)
    want = [gold_dly(len(hist.xs) + k) for k in (-1, 0, 1)]
    dut._log.info(f"dly {d0} after {len(hist.xs)} frames; golden n-1/n/n+1 = {want}")
    assert d0 == want[1], f"dly {d0} != golden {want[1]} after {len(hist.xs)} frames"
    assert d0 < golden_model.DELAY_ABSENT, "dly never left the reset value"
    # the slew: 40 more frames move it exactly -40
    await _delay_hist(dut, hist, [0] * 40)
    d1 = int(dp.value)
    assert d0 - d1 == 40, f"slew must be -1/frame: {d0} -> {d1} over 40 frames"
    # knob move mid-run (the golden keeps k4 = 10, so outputs are not
    # compared from here): the slew stays bounded to 1 frame per frame
    kp.v = 200
    prev = d1
    for _ in range(10):
        await _delay_hist(dut, hist, [0] * 40, compare=False)
        cur = int(dp.value)
        assert abs(cur - prev) <= 40, f"slew exceeded 1 frame/frame: {prev} -> {cur}"
        prev = cur


@cocotb.test()
async def test_all_dips_on(dut):
    """Review Focus 1 (Rev B3 final, DIP-safe pin map): every DIP on
    (ui_in = 0xFF) with the PSRAM and the ADC wired on uio — no external
    data source shares a ui pin, so the switches cannot fight anything:
    the PSRAM still probes OK and every frame equals the golden chain
    with all six enables (comp, dist, phaser, trem, delay, freeze) in
    force, ui[7:6] ignored."""
    dut.psram_en.value = 1
    dut.psram_so_dly_ps.value = 0
    try:
        await start(dut, 0xFF)
        ok = _ok(dut)
        assert ok in (None, 1), "PSRAM must probe OK with every DIP on"
        fill = _fill_param(dut)
        hist = DelayHist(0x3F, fill=32768 if fill is None else fill)
        xs = [(i * 2731) % 40000 - 20000 for i in range(64)]
        await _delay_hist(dut, hist, xs)
        assert int(dut.psram_err.value) == 0, "PSRAM protocol error with every DIP on"
    finally:
        psram_off(dut)


@cocotb.test()
async def test_chain_before_next_frame(dut):
    """Rev B3 final (Task 3) guard: the chain reads the ADC shift register
    rx_shift directly (no sample_in copy), which is only valid while
    rx_shift holds still — it shifts in the LEFT half (lr_cnt[8] == 0).
    So with every DIP on (the longest chain), every chain_done pulse in
    32 frames must land in the RIGHT half of the frame its sample was
    latched in (lr_cnt[8] == 1, i.e. after the latch at lr_cnt 256 and
    before the frame wraps at 511). RTL probe; returns under GL."""
    lr, cd = _probe(dut, "lr_cnt"), _probe(dut, "chain_done")
    if lr is None:
        return
    await start(dut, 0xFF)
    seen = 0
    for _ in range(32 * 512):
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(cd.value) == 1:
            seen += 1
            v = int(lr.value)
            assert v >= 256, f"chain_done at lr_cnt {v}: in the left half (rx_shift moving)"
    assert seen >= 31, f"only {seen} chain_done pulses in 32 frames"


def _s16(v):
    return v - 0x10000 if v & 0x8000 else v


async def _freeze_case(dut, en):
    """PSRAM fitted, switches en from reset (freeze on): 200 frames equal
    the golden chain, and every go write word is quant12 of the word the
    PREVIOUS go read back (the loop recirculates; no new audio enters).
    The SPI monitor attaches after the settle, at a left-half start: a
    transaction already in flight there is partial (n < 48) and dropped by
    _go_pairs."""
    dut.psram_en.value = 1
    dut.psram_so_dly_ps.value = 0
    try:
        await start(dut, en)
        mon = SpiMon(dut)
        fill = _fill_param(dut)
        hist = DelayHist(en, fill=32768 if fill is None else fill)
        xs = [(i * 3001) % 40000 - 20000 for i in range(200)]
        if fill is not None:
            await _delay_hist(dut, hist, xs)
        else:
            await fast_frames(dut, xs)              # GL: the write-path check below is the test
        pairs = _go_pairs(mon.txns)
        assert len(pairs) >= 150, f"only {len(pairs)} go pairs"
        # RTL (FILL_FRAMES 64): the ring is filled from frame ~81, so the
        # frozen write is quant12 of the previous read. GL (the netlist
        # keeps 32768): not yet filled in this window, rd = 0, so every
        # frozen write is 0 — still no new (non-zero) audio enters.
        filled = fill is not None
        for (_, r0), (w1, _) in zip(pairs, pairs[1:]):
            want = (golden_model.quant12(_s16(r0["data"])) & 0xFFFF) if filled else 0
            assert w1["data"] == want, \
                f"frozen write {w1['data']:#06x} != quant12(prev read {r0['data']:#06x}) {want:#06x}"
        return len(pairs)
    finally:
        psram_off(dut)


@cocotb.test()
async def test_freeze_holds(dut):
    """Delay + freeze (ui[4] + ui[5]): the ring write word is the word just
    read, so the loop holds; output == golden (dry + the frozen echo)."""
    await _freeze_case(dut, EN_DELAY | EN_FREEZE)


@cocotb.test()
async def test_freeze_while_bypassed(dut):
    """Review Focus 2: freeze with the delay OFF still holds the ring (the
    write word is the read word) while the output is the dry signal, so
    a loop frozen while bypassed is intact when the delay comes back on
    (the audio check's freeze_bypass mode plays it)."""
    await _freeze_case(dut, EN_FREEZE)


@cocotb.test()
async def test_freeze_holds_time(dut):
    """Final-review fix: the delay time HOLDS while frozen (turning K4 with
    freeze on would otherwise re-slew the read pointer through a ring that
    only recirculates, smearing the loop into one held word). K4 = 10 from
    reset with delay + freeze on: dly stays at 16800 (the reset value)
    after EN_SETTLE instead of slewing toward 2072, like golden Delay; the
    outputs equal golden. RTL-only (the dly probe)."""
    dp = _probe(dut, "dly_len")
    if dp is None:
        return                      # GL netlist: no internal probe
    dut.psram_en.value = 1
    dut.psram_so_dly_ps.value = 0
    try:
        knob_pin(dut, 3, 10)        # K4 = uio[5], v 10
        en = EN_DELAY | EN_FREEZE
        await start(dut, en)
        hist = DelayHist(en, k4=10, fill=_fill_param(dut))
        await _delay_hist(dut, hist, [0] * 100)
        d = golden_model.Delay(10, hist.fill)
        for _ in range(len(hist.xs)):
            d.step(0, True, True)
        assert d.dly == golden_model.DELAY_ABSENT, f"golden dly {d.dly} moved while frozen"
        assert int(dp.value) == golden_model.DELAY_ABSENT, f"dly {int(dp.value)} moved while frozen"
    finally:
        psram_off(dut)
