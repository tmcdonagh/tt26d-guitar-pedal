# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-FileCopyrightText: © 2026 Thomas McDonagh
# SPDX-License-Identifier: Apache-2.0
# Internal-state probes are RTL-only (guarded getattr — names may vanish
# in the flat GL netlist); port-level assertions carry the GL coverage.

import cocotb
import golden_model
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, ReadOnly, Timer, ValueChange, with_timeout
from cocotb.utils import get_sim_time


@cocotb.test()
async def test_skeleton_baseline(dut):
    """Task 1 baseline: reset state, pin wiring, and running frame clock."""
    dut._log.info("Start")

    # DIP enables all on (en = 0x3F, ui[6:1]), ui[7] = 0 (octave off),
    # zero input: every stage maps 0 to 0 (the stub slots are identity),
    # so DAC data stays low even after the switches settle.
    await start(dut, 0b0111_1110)

    # Reset state: DAC data low, uo idle — dp clip LED (uo[7]) low, SPI
    # idle (uo[1] = CS high), uo[6:3] and uo[2], uo[0] driven 0
    assert dut.uio_out[1].value == 0, "DAC data not low after reset"
    assert int(dut.uo_out.value) == 0x02, \
        "uo must read 0x02 at idle (dp dark, SPI CS idle high, rest 0)"

    # MCLK output enabled on uio[0] and following clk.
    # Sample away from clock edges (comb outputs need settle time).
    # fixed bits: uio[7:6] + uio[1:0] outputs, uio[5] input; uio[4:2] are
    # the knob readers' discharge drives (toggle; see test_pin_map)
    assert int(dut.uio_oe.value) & 0b1110_0011 == 0b1100_0011, \
        "uio_oe must enable uio[7:6] and uio[1:0], uio[5] an input"
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

    # Frame clock is running: LRCLK (uio[7]) must toggle within one frame (512 clk)
    lr0 = dut.uio_out[7].value
    toggled = False
    for _ in range(600):
        await ClockCycles(dut.clk, 1)
        if dut.uio_out[7].value != lr0:
            toggled = True
            break
    assert toggled, "LRCLK never toggled — frame counter not running"

    # all enables on, zero input: DAC data still low
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


# Switch debounce (v3 task 7): the enables and octave (ui[7:1]) are 2-flop
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
    (ui[0] low; the last drive left rx_shift at 0 — callers drove a zero
    capture frame) and return at a left-half start."""
    await _wait_left_start(dut)
    dut.ui_in.value = ui_in
    await ClockCycles(dut.clk, 512 * SETTLE_FRAMES - 16)
    await _wait_left_start(dut)


async def fast_frames(dut, words):
    """Low-overhead I2S driver for long runs: from a left-half start (as
    start()/_wait_left_start return, 1 ns after the WS-fall clk edge)
    drive one frame per 16-bit word on ui[0] and capture each frame's
    TX word from uio[1], using sim-time Timers instead of per-clk
    polling (~65 awaits/frame). Capture j is the result of input frame
    j-1 (one-frame TX latency). Returns signed captures, ending at the
    next left start (+1 ns)."""
    outs = []
    for w in words:
        w &= 0xFFFF
        v = 0
        for slot in range(32):
            dut.ui_in[0].value = (w >> (16 - slot)) & 1 if 1 <= slot <= 16 else 0
            await Timer(4 * CLK_PERIOD_PS, unit="ps")   # mid-slot
            if 1 <= slot <= 16:
                v = (v << 1) | int(dut.uio_out[1].value)
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
    return await with_timeout(_knob_cycle(dut, idx), 6 * 67000 * CLK_PERIOD_PS, "ps")


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
    turns then lasts 1024 + 33 clk instead of a 66.6k-clk timeout, so the
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
    # junk (ui[0] = 1) on the data line during reset
    cocotb.start_soon(
        Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2).start())
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
    assert int(dut.uo_out.value) == 0x02, \
        "uo must read 0x02 after reset (dp dark, SPI CS idle high, rest 0)"



# ui byte layout (v3 task 7): ui[0]=I2S data (driven by the helper),
# ui[5:1]=DIP effect enables en[4:0] (en[0]=comp .. en[4]=trem, bit
# order = chain order), ui[6] = delay enable + PSRAM MISO (the delay slot
# is an identity stub), ui[7] = fuzz octave. All of ui[7:1] are debounced (see start()).
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
        await start(dut, 0b0000_0010, clock=(i == 0))   # en[0] comp only
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
    await start(dut, 0b0000_0010)                     # en[0] comp only
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
    await start(dut, 0b0000_0010)                     # en[0] comp only

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
    await start(dut, 0b0000_0010)                     # en[0] comp only
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
    """Frame history of a dirt-stage test (v3 task 11): fuzz/dist outputs
    pass through the tone low-pass and the noise gate, which carry state
    across frames, so every probe is checked against golden_model.render
    of the whole history ([0] * START_FRAME + the driven frames) instead
    of an isolated-frame literal. The gate is opened beforehand by a
    loud priming run (L >= 256) so the outputs are non-zero."""

    def __init__(self, en, oct_sw=0, k3=None):
        self.en, self.oct, self.k3 = en, oct_sw, k3
        self.xs = [0] * START_FRAME

    def add(self, x, n=1):
        self.xs += [x] * n

    def render(self, en=None):
        tap = []
        out = golden_model.render(self.xs, self.en if en is None else en,
                                  oct=self.oct, k3=self.k3, tap=tap)
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
    """Dist stage (en[2]), v3 task 5: fixed drive x4 into a 3-piece diode
    knee. a = sat_mag(x); w = 4a (18-bit, no intermediate saturation);
    w <= 8192: out = w; w <= 32768: out = 8192 + ((w-8192) >> 2) (slope
    1/4, 14336 at w = 32768); else out = 14336 + ((w-32768) >> 4)
    (slope 1/16, max 20479 — sat16 never reached). Sign reapplied from
    x[15]. Fuzz is off, so the input is the raw sample. Exact points
    pin both seams (K1 at a=2048, K2 at a=8192, both sides) and the
    extremes; each is also asserted against golden_model.dist. Since
    task 11 the pins read the RTL s3 tap probe (the dist-stage output,
    before tone + gate; guarded, GL carries the port check) and the port
    output is checked against golden after a loud run that opens the
    gate (Hist/dirt_frame)."""
    await start(dut, 0b0000_1000)  # en[2] dist only (ui[3]); octave off
    hist = Hist(0x04)
    await prime_gate(dut, hist)

    async def dist_once(word16, first=False):
        # the stage value (RTL s3 tap probe; None on GL) is the pinned
        # dist output; the port output (through tone + gate) is checked
        # against golden inside dirt_frame
        stage, _, _ = await dirt_frame(dut, hist, word16, direct=first)
        return stage

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
    for k, (x, want) in enumerate(points):
        assert want == golden_model.dist(x), f"pin {x}: {want} != golden"
        got = await dist_once(x, first=(k == 0))
        assert got in (None, want), f"dist({x}) = {got}, expected {want}"

    # Gate-level-visible seam pass (the pins above read the RTL s3 tap
    # probe; at the port a single frame is hidden by the tone low-pass).
    # A DC hold lets the tone settle (the >>> floor leaves the 16.4
    # state up to 3/16 LSB under 16*s3, so the port reads s3 or s3-1)
    # and the open gate passes it. Golden settled port words: 2047/2048/
    # 2049 -> 8188/8191/8192 (the K1 seam, three distinct words) and
    # 8191/8192/8193 -> 14334/14335/14335 (K2 low side distinct; 8192 vs
    # 8193 are equal in dist itself). Every frame of every hold is
    # asserted against golden, so these distinctions are GL-visible.
    settled = {}
    for x in (2047, 2048, 2049, 8191, 8192, 8193):
        port, stage, i, want_out, tap = await hold_frames(
            dut, hist, x, direct=(x != 2047))
        assert tap[-1] == golden_model.dist(x)
        settled[x] = port
    assert len({settled[2047], settled[2048], settled[2049]}) == 3, settled
    assert settled[8191] != settled[8192], settled


async def _drive_points(dut, k3, points, en=0x04):
    """Fresh run with K3 = k3 (None = absent): prime the gate, then for
    each x hold it and require the dist-stage output (RTL s3 tap probe,
    None on GL) and the settled port word to equal golden at the K3
    drive; every frame is golden-checked inside hold_frames. Returns the
    list of (x, settled port, golden dist)."""
    if k3 is not None:
        knob_pin(dut, 2, k3)
    await start(dut, en << 1)
    kval, kpres = _probe(dut, "k3_value"), _probe(dut, "k3_present")
    if kval is not None:
        assert int(kpres.value) == (0 if k3 is None else 1)
        if k3 is not None:
            assert int(kval.value) == golden_model.knob_steady(k3)
    hist = Hist(en, k3=k3)
    await prime_gate(dut, hist)
    dq = golden_model.drive_q8(k3)
    res = []
    for x in points:
        port, stage, i, want, tap = await hold_frames(dut, hist, x)
        d = golden_model.dist(x, dq)
        assert tap[-1] == d and stage in (None, d), f"x={x}: stage {stage} != {d}"
        assert abs(port) <= 32767 and abs(d) <= 32767
        res.append((x, port, d))
    return res


@cocotb.test()
async def test_drive_default(dut):
    """K3 absent: drive = 1024 (x4) — every test_dist_diode point is
    unchanged (that test reruns the pins in the same suite; here the
    golden default is pinned and K3 reads absent). Pins at the port after
    a DC hold: three points across the knee pieces."""
    assert golden_model.drive_q8(None) == 1024
    pts = (1000, 4096, 32767)
    res = await _drive_points(dut, None, pts)
    assert [d for _, _, d in res] == [golden_model.dist(x) for x in pts]
    assert [d for _, _, d in res] == [4000, 10240, 20479]


@cocotb.test()
async def test_drive_rails(dut):
    """K3 = 255: drive = 256 + 7*255 = 2041 (x7.97). Full-scale inputs:
    w = 261,248 (18 bits, no pre-knee saturation) -> top piece
    14336 + (261248 - 32768)/16 = 28615, inside +-32767 for both rails
    (the -32768 input sat-mags to 32767 first); no wrap."""
    assert golden_model.drive_q8(255) == 2041
    res = await _drive_points(dut, 255, (32767, -32768, 1000))
    assert [d for _, _, d in res] == [28615, -28615, 7972]
    assert abs(res[0][1]) <= 32767 and abs(res[1][1]) <= 32767


@cocotb.test()
async def test_drive_mid(dut):
    """K3 = 128: drive = 256 + 7*128 = 1152 (x4.5). Three points, one
    per knee piece (+ a negative): 1000 -> 4500 (linear); 8000 ->
    w 36000 -> 14538 (top piece); -20000 -> w 90000 -> -17913."""
    assert golden_model.drive_q8(128) == 1152
    res = await _drive_points(dut, 128, (1000, 8000, -20000))
    assert [d for _, _, d in res] == [4500, 14538, -17913]


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
      post-update: the output captured in frame N+1 is fuzz(x_N, env_N));
      shaped = a > t ? t : a; fuzz_core = x[15] ? -shaped : shaped.
    octave off throughout (the plain asym fuzz; the octave blend is
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
      - re-coupling probes on 0x1000, each from a fresh reset (comp
        gain golden g = 717 after the silent settle; task 10's slewed
        compressor): en=0x03: s1 = 11472 -> fuzz passes it -> +11472
        — the fuzz stage receives the COMPED amplitude (PRE-FIX value
        4096: fuzz read mag(sample_in); comp masked); en=0x05: dist
        reads a direct saturating |s2| (= s1 with fuzz off) ->
        dist(11472) = +15156 (pre-fix: dist(4096) = 10240).
    Since task 11 every pinned value is the fuzz/dist STAGE output read
    from the RTL s3 tap probe (before tone + gate; guarded, None on GL),
    and the port output is checked against golden_model.render of the
    whole frame history (Hist/dirt_frame); the 100 loud frames open
    the gate for the main run, the fresh-reset probes (gate closed)
    pin the stage only."""
    # en[1] fuzz only (ui[2]); octave off (ui[7] = 0): the plain asym fuzz
    await start(dut, 0b0000_0100)              # at the start of frame 0 (env 0)

    # 100 loud frames (0x7FFF): env attacks to 32766, sticky from frame 15
    hist = Hist(0x02)                          # tone + gate follow the fuzz
    for _ in range(100):
        await i2s_send_and_capture(dut, 0x7FFF << 8)
        await ClockCycles(dut.clk, 256)
    hist.add(0x7FFF, 100)

    # corner probe (inline — the sim is at frame 100's start): -32768
    # with env 32766 -> t_neg 21298 (max) -> -21298
    got, _, _ = await dirt_frame(dut, hist, -32768, direct=True)  # frames 100, 101
    assert got in (None, -21298), f"corner -32768: {got} != -21298 (env 32766)"

    async def fuzz_once(word16):
        # drive word16 in the next frame, capture the frame after; the
        # pinned value is the fuzz-stage output (RTL s3 tap probe, None on
        # GL), the port output through tone + gate is golden-checked
        # (the switches are set by start() — debounced)
        return (await dirt_frame(dut, hist, word16))[0]

    # loud -24576 (frame 102): env 32510 -> t_neg 21234 -> -21234
    got = await fuzz_once(-24576)
    assert got in (None, -21234), f"loud -24576: {got} != -21234 (env 32510)"

    # +24576 (frame 104): positive clip point 19660 (env-free path)
    got = await fuzz_once(24576)
    assert got in (None, 19660), f"+24576: {got} != 19660 (t_pos)"

    # sub-clip unity, both signs — the negative KEEPS its sign (the
    # v2.2 rectifier mapped -1000 -> +1000)
    got = await fuzz_once(1000)
    assert got in (None, 1000), f"+1000: {got} != 1000 (sub-clip unity)"
    got = await fuzz_once(-1000)
    assert got in (None, -1000), f"-1000: {got} != -1000 (sign kept, no rectify)"

    # 401 silent frames (capture frame 109 + 400 bulk frames 110..509):
    # env releases to 6510. The quiet probe drives DIRECTLY after the
    # bulk advance (task-3's pattern) — fuzz_once's leading
    # _wait_left_start, called exactly at a WS fall, races its first
    # lrclk read and can return a clk later, shifting the word a frame
    # (observed: -16992 via frame 510 either way, but only the direct
    # drive is deterministic).
    await ClockCycles(dut.clk, 256)             # rest of frame 109
    await ClockCycles(dut.clk, 512 * 400)       # frames 110..509
    hist.add(0, 400)

    # quiet -24576 (frame 510): env_510 = attack (6510 + 24576)>>1 =
    # 15543 -> t_neg = 13107 + 3885 = 16992 -> -16992. Same input as
    # the loud probe, distinct pin (Review Focus 4)
    got, _, _ = await dirt_frame(dut, hist, -24576, direct=True)  # frames 510, 511
    assert got in (None, -16992), f"quiet -24576: {got} != -16992 (env 15543)"

    # re-coupling probes (the mandatory fix): each on a fresh reset, so
    # the comp gain is golden's post-settle g = 717 (cenv 0). comp+fuzz:
    # s1 = sat16(4096 * 717 >>> 8) = 11472 (sub-t_pos) -> +11472. PRE-FIX:
    # 4096 (fuzz read the pre-chain mag) — the probe's purpose
    async def chain_fresh(en_val, word16):
        # the gate is still closed on a fresh reset, so the pinned value
        # is the stage output (RTL s3 tap; None on GL), not the port word
        await start(dut, en_val << 1, clock=False)
        h = Hist(en_val)
        stage, _, _ = await dirt_frame(dut, h, word16, direct=True)
        return stage, h

    async def recoupled_at_port(h, pre_fix_en):
        # Gate-level-visible follow-up on the same run: a loud DC pre-roll
        # opens the gate, then a DC hold of 0x1000 settles the tone, and
        # the port word is asserted against golden on every frame. The
        # re-coupling is visible at the port: the same history rendered
        # with comp OFF (pre_fix_en: what a stage reading the pre-chain
        # sample would produce) settles to a different word.
        await _wait_left_start(dut)
        await prime_gate(dut, h)
        port, _, i, want, _ = await hold_frames(dut, h, 0x1000)
        alt, _ = h.render(pre_fix_en)
        assert port == want[i] != alt[i], f"re-coupling at port: {port} vs pre-fix {alt[i]}"

    got, h = await chain_fresh(0x03, 0x1000)
    assert got in (None, 11472), \
        f"comp->fuzz re-coupling: {got} != 11472 (pre-fix value 4096)"
    await recoupled_at_port(h, 0x02)

    # re-coupling probe 2: comp+dist (fuzz off) — dist reads |s2| = |s1|:
    # 11472 -> dist(11472) = +15156 (pre-fix: dist(4096) = 10240)
    got, h = await chain_fresh(0x05, 0x1000)
    assert got in (None, 15156) and golden_model.dist(11472) == 15156, \
        f"comp->dist re-coupling: {got} != 15156 (pre-fix value 10240)"
    await recoupled_at_port(h, 0x04)


@cocotb.test()
async def test_fuzz_octave(dut):
    """Fuzz octave blend (en[1], v3 task 4; octave switch on ui[7] since
    task 7): fuzz_out = oct ? sat16(shaped + mag(s1)) : fuzz_core, the
    50/50 blend with the full-wave-rectified copy. sat16 clamps UPPER
    only (shaped >= 0 and mag(s1) >= 0, so the blend is >= 0 with the
    octave on in every reachable state). Every pinned value here is
    env-free by construction: sub-clip inputs shape to a (< both clip
    points for any env, since the negative t_neg >= 13107), and the
    saturating pins saturate for any env (shaped >= 13107 ->
    shaped + 32767 > 32767). Octave on (ui[7] = 1):
      +10000 -> sat16(10000 + 10000) = 20000;
      -10000 -> 20000 — the octave signature: the blend erases the
        sign (the plain core in test_fuzz_asym keeps it: -10000 ->
        -10000 octave off);
      +24576 -> sat16(19660 + 24576) = 32767 (upper rail);
      -32768 -> 32767 (shaped = t_neg(env) >= 13107, sat-mag 32767 ->
        saturated for any env).
    Octave-off contrast on the same words (a fresh run): -10000 ->
    -10000 (core), +24576 -> 19660. Pins read the RTL s3 tap probe (the
    fuzz output before tone + gate, guarded); ports are golden-checked."""
    # en[1] fuzz only (ui[2]) + octave on (ui[7])
    await start(dut, 0b1000_0100)
    hist = Hist(0x02, 1)
    await prime_gate(dut, hist)

    async def oct_once(word16, first=False):
        # stage value = fuzz output (RTL s3 tap probe, None on GL); the
        # port word is golden-checked through tone + gate in dirt_frame
        return (await dirt_frame(dut, hist, word16, direct=first))[0]

    # octave on (ui[7]): the octave blend
    got = await oct_once(10000, first=True)
    assert got in (None, 20000), f"oct +10000: {got} != 20000 (blend 2a)"
    got = await oct_once(-10000)
    assert got in (None, 20000), f"oct -10000: {got} != 20000 (blend erases sign)"
    got = await oct_once(24576)
    assert got in (None, 32767), f"oct +24576: {got} != 32767 (sat16 upper rail)"
    got = await oct_once(-32768)
    assert got in (None, 32767), f"oct -32768: {got} != 32767 (sat for any env)"

    # octave off contrast on the same words: the core values (a fresh
    # run: the golden history carries one constant switch setting)
    await start(dut, 0b0000_0100, clock=False)
    hist = Hist(0x02, 0)
    await prime_gate(dut, hist)
    got = await oct_once(-10000, first=True)
    assert got in (None, -10000), f"core -10000: {got} != -10000 (core keeps sign)"
    got = await oct_once(24576)
    assert got in (None, 19660), f"core +24576: {got} != 19660 (core clip)"


@cocotb.test()
async def test_fuzz_dist_chain(dut):
    """Fuzz + dist stacked (en = 0x6, octave off): the task-4 asym fuzz feeds
    the v3 diode-knee shaper on the CHAIN value — dist consumes
    mag(s2) with s2 = fuzz_out (signed, octave off; sign from s2[15]).
    Every expected value is golden_model.dist of the fuzz output; the
    pins are the RTL s3 tap stage output (guarded), ports golden-checked
    after a priming run that opens the gate (task 11). Env values are
    the probe frames' post-update env, model-derived: -20000 frames the env-biased negative clip
    (after the priming env 9999 attacks to 14999 -> t_neg = 16856 ->
    fuzz -16856 -> dist -16502);
    5000 is below the fuzz clip (-> 11144); 10000 is shaped WITHOUT the
    fuzz clip (fuzz passes it at sub-t_pos unity -> 14788); 20000 clips
    at t_pos 19660 -> 17203; and an octave-on stacking probe pins the
    octave-into-dist path on the direct |s2| shaper input: +10000 ->
    blend sat16(10000+10000) = 20000 (non-negative, so dist's sign
    input s2[15] reads 0) -> +17288."""
    await start(dut, 0b0000_1100)  # en[2:1] fuzz + dist, octave off
    hist = Hist(0x06)
    await prime_gate(dut, hist, amp=10000)   # env 10000, gate open

    async def chain_once(word16, first=False):
        # the dist-stage output (RTL s3 tap probe, None on GL); the port
        # word is golden-checked through tone + gate in dirt_frame
        return (await dirt_frame(dut, hist, word16, direct=first))[0]

    # -20000 frames the env-biased negative clip: env 9999 after the
    # priming (the sticky point under a constant 10000), attack to 14999
    # -> t_neg 16856 -> fuzz -16856 -> dist -16502
    assert golden_model.dist(golden_model.fuzz(-20000, 14999, 0)) == -16502
    assert await chain_once(-20000, first=True) in (None, -16502)
    assert await chain_once(5000) in (None, 11144)    # below the fuzz clip: w 20000
    assert await chain_once(10000) in (None, 14788)   # fuzz passes 10000; w 40000
    assert await chain_once(20000) in (None, 17203)   # clip 19660; w 78640

    # octave-on stacking probe (fix round): the octave blend INTO dist —
    # +10000 -> fuzz blend sat16(10000+10000) = 20000 (>= 0, so the
    # shaper's sign input s2[15] = 0) -> dist(20000) = +17288. A
    # shaper magnitude reading fuzz_shaped (10000) instead of the
    # blended output would emit 14788 — pins the octave-into-dist
    # path on the direct saturating |s2| (en[1]=1, octave-on case).
    await start(dut, 0b1000_1100, clock=False)  # octave on (ui[7]), fresh run
    hist = Hist(0x06, 1)
    await prime_gate(dut, hist, amp=10000)
    assert await chain_once(10000, first=True) in (None, 17288)  # blend 20000; w 80000


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
    v = 0 pin models so K1's turn comes round every ~4.2k clk): discharge
    1024 clk with uio_oe[2] = 1, then charge until the pin reads 1; raw =
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
    [5:2] one-hot or 0), each turn discharges its pin for 1024 clk then
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

    ev = await with_timeout(record(10), 5 * 67000 * CLK_PERIOD_PS, "ps")
    pats = [p for _, p in ev]
    assert pats == [1, 0, 2, 0, 4, 0, 8, 0, 1, 0], f"turn order {pats}"
    lens = [round((b[0] - a[0]) / CLK_PERIOD_PS) for a, b in zip(ev, ev[1:])]
    want = [1024, 65536, 1024, 65536, 1024, 64 * 20 + 33, 1024, 65536, 1024]
    # the first discharge counts from the reset release (+-1 clk of phase)
    assert abs(lens[0] - 1024) <= 1 and lens[1:] == want[1:], f"phase lengths {lens}"


def _signed(x):
    return x - 0x10000 if x & 0x8000 else x


@cocotb.test()
async def test_knob_absent_default(dut):
    """K1 absent (pin tied low): the trem phase steps by the default 437
    per frame (~5 Hz). Probe (RTL only): trem_p advances exactly 437 per
    frame over 5 consecutive frames (and equals 437 * frames since the
    enable reached the chain at EN_SETTLE). Port level: 20 consecutive
    TX words on a constant 16384 equal golden_model (k1 = None)."""
    await start(dut, 0x10 << 1)               # en[4] trem only, uio_in = 0
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
    want = golden_model.render([0] * f0 + [16384] * 21, 0x10)
    got = await fast_frames(dut, [16384] * 21)
    assert got[1:] == want[f0:f0 + 20], f"got {got[1:]} want {want[f0:f0 + 20]}"


# Smooth-trem step bound: gain = 256 - tri/2 - tri/4 moves by at most 2
# per tri LSB, and tri moves by at most 1 per frame for every inc <= 8192
# (K1 max inc 127 << 3 = 1016), so |dout| <= |x| * 2 / 256 = 128 at
# x = 16384 (the old stepped trem jumped by 8192).
TREM_MAX_STEP_16384 = 16384 * 2 // 256


@cocotb.test()
async def test_trem_smooth(dut):
    """Smooth trem at K1 = 128 (inc = 256): constant 16384 for 2000
    frames; the output equals golden_model frame-for-frame and never
    steps by more than TREM_MAX_STEP_16384."""
    knob_pin(dut, 0, 128)
    await start(dut, 0x10 << 1)
    n = 2000
    got = (await fast_frames(dut, [16384] * (n + 1)))[1:]
    want = golden_model.render([0] * START_FRAME + [16384] * n, 0x10, k1=128)[START_FRAME:]
    first = next((i for i in range(n) if got[i] != want[i]), None)
    assert first is None, f"frame {first}: got {got[first]} want {want[first]}"
    step = max(abs(a - b) for a, b in zip(got, got[1:]))
    assert step <= TREM_MAX_STEP_16384, f"max |dout| {step}"
    assert len(set(got)) > 20, "trem must move"


@cocotb.test()
async def test_trem_rate_change(dut):
    """K1 128 -> 250 mid-run (pin model re-timed): the phase keeps
    accumulating (no reset/jump), so the output stays smooth across the
    change. Before the change the output equals golden_model (k1 = 128);
    across it max |dout| <= TREM_MAX_STEP_16384; probe (RTL only): the
    knob value has moved to within the hysteresis of 250. Port level
    (GL-visible): the post-change output must DIFFER from the golden
    render that keeps k1 = 128 (the rate really changed). Round robin
    (knob_rc4): K2..K4 run v = 0 pin models (en = trem only, so their
    values never reach the output), giving one K1 reading per ~40
    frames; the IIR needs 16 readings from 128 to 249, so the post-change
    run is 900 frames."""
    kp = knob_pin(dut, 0, 128)
    fast_knobs(dut, 1, 2, 3)
    await start(dut, 0x10 << 1)
    pre = (await fast_frames(dut, [16384] * 201))[1:]
    want = golden_model.render([0] * START_FRAME + [16384] * 200, 0x10, k1=128)[START_FRAME:]
    assert pre == want, "pre-change output must equal golden (k1=128)"
    kp.v = 250
    post = await fast_frames(dut, [16384] * 900)
    seq = pre + post[1:]
    step = max(abs(a - b) for a, b in zip(seq, seq[1:]))
    assert step <= TREM_MAX_STEP_16384, f"max |dout| across the change {step}"
    keep = golden_model.render([0] * START_FRAME + [16384] * 1100, 0x10, k1=128)[START_FRAME + 200:]
    ndiff = sum(1 for a, b in zip(post, keep) if a != b)
    assert ndiff > 200, f"post-change output tracks the k1=128 render ({ndiff} diffs): rate did not change"
    kval, kpres = _probe(dut, "k1_value"), _probe(dut, "k1_present")
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
    (Review Focus 3: no disabled stage may corrupt the sample). The Rev A
    identity-stub slot (delay en[5]) must pass +-full-scale bit-exact
    when enabled — the enable itself corrupts nothing (comp is live from
    task 3; the phaser en[3] from Task 16, golden-checked). Fuzz +
    dist stacked: the task-4 asym fuzz feeds the shaper mag(s2) (both
    fuzz clip points sit above the dist knees; dist is the v3 diode
    knee, octave off). Since task 11 the fuzz/dist-on pins are the stage
    output (RTL s3 tap probe, guarded) because the port word is then
    tone-filtered and gated; the dirt-off stacks still pin the port."""
    # trem first, on a fresh reset (K1 absent -> inc 437): the expected
    # words come from golden_model.render of the whole frame history
    await start(dut, 0b0010_0000)             # en[4] trem only (ui[5])
    xs = [16384, -32768, 0]
    got = await fast_frames(dut, xs)
    want = golden_model.render([0] * START_FRAME + xs, 0x10)
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
    for en_val in (0x08, 0x29):
        await start(dut, en_val << 1, clock=False)
        got = await fast_frames(dut, xs)
        want = golden_model.render([0] * START_FRAME + xs, en_val)[START_FRAME:]
        assert got[1:] == want[:3], f"en {en_val:#x}: {got[1:]} != golden {want[:3]}"
        assert got[1:] != xs[:3], f"en {en_val:#x}: phaser must not be identity"

    # every other stage on a fresh reset (env = 0 after the silent
    # settle) so the env-dependent pins can be derived from the golden
    # model's Env: ref (golden Chain) mirrors every frame driven (x, then the 0
    # capture frame) and the silent settle after each enable change.
    await start(dut, 0b0000_0100, clock=False)   # en[1] fuzz only (ui[2])
    # ref = golden_model.Chain driven frame-by-frame (every frame sent,
    # and the silent settle after each enable change): its env/comp/tone/
    # gate state yields the env-dependent pins AND the golden port word
    # of every probe (silent frames output 0 in any enable setting, so
    # the en split inside a settle run is immaterial).
    ref = golden_model.Chain()
    for _ in range(START_FRAME):
        ref.step(0, 0)
    state = {"en": 0x02, "e": 0, "g": ref.cst.g}

    async def stage_once(en_val, word16, stage=False):
        """Drive word16, capture the next frame's port word, and assert
        it equals golden's output for the frame (Chain history: tone,
        gate, comp, every stage). stage=True (a dirt-on stack, task 11:
        the port word is tone/gate filtered and gated) returns instead
        the RTL s3 tap probe — the fuzz/dist stage output the tone filter
        sees (cs sampled at step 4 by _s3_monitor, after CHAIN_MAX_CLK clk) — or None where the probe
        is absent (flat GL netlist; there the golden port assertion is
        the check — the gate is cold on these single frames, so the port
        word is golden's, mostly 0: the port-level tone/gate coverage is
        test_tone_lpf / test_gate_hysteresis / test_chain_latency and the
        DC-hold probes of test_dist_diode / test_fuzz_asym /
        test_drive_*)."""
        if en_val != state["en"]:
            await resettle(dut, en_val << 1)     # ui[6:1] = en; debounced
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
        state["e"] = ref.env.v                   # env after the x frame
        state["g"] = ref.cst.g                   # comp gain for the x frame
        await _wait_left_start(dut)
        bits = await i2s_send_and_capture(dut, 0)
        ref.step(0, en_val)                      # the capture frame
        out = tx_data_bits(bits)
        out = out - 0x10000 if out & 0x8000 else out
        assert out == want, f"en {en_val:#x} word {word16}: port {out} != golden {want}"
        assert probe in (None, s3), f"en {en_val:#x} word {word16}: s3 tap {probe} != golden {s3}"
        return probe if stage else out

    # fuzz ON: the task-4 asym clip — -32768 sat-mags to 32767 and
    # clips at the env-biased negative point (env from the golden
    # model: 16383 after the first attack -> t_neg 17202); 5000 passes
    # below both clip points at unity
    got = await stage_once(0x2, -32768, stage=True)
    assert golden_model.fuzz(-32768, state["e"], 0) == -17202
    assert got in (None, -17202)
    got = await stage_once(0x2, 5000, stage=True)
    assert golden_model.fuzz(5000, state["e"], 0) == 5000
    assert got in (None, 5000)

    # dist ON: diode-knee PWL, fixed drive x4 (expected values from
    # golden_model.dist); 1234 the linear piece, 20000 the top piece
    for x, want in ((-32768, -20479), (1234, 4936), (20000, 17288)):
        assert golden_model.dist(x) == want
        assert await stage_once(0x4, x, stage=True) in (None, want)

    # delay en[5] (identity stub until Task 22) must pass +-full-scale
    # bit-exact when enabled, through the slot AND every disabled stage;
    # comp en[0] is live: its value is golden comp at the frame's gain g
    # (after the silences here g >= 256, so +-full-scale saturates and
    # reads identity). (The phaser is pinned above, from reset.)
    for x in (-32768, 32767):                         # comp (sat16 gain)
        got = await stage_once(0x1, x)
        assert got == golden_model.comp(x, state["g"]) == x, f"comp {x}: {got} (g {state['g']})"
    assert await stage_once(0x20, -32768) == -32768   # delay stub
    assert await stage_once(0x20, 32767) == 32767

    # fuzz + dist stacked: the task-4 asym fuzz feeds the shaper mag(s2).
    # (input, fuzz-stage output): -20000 clips at the env-biased negative
    # point (t_neg = 15891 at this env, below 20000); 5000 is below both
    # clip points (unity); 20000 clips at t_pos 19660. The fuzz column is a
    # literal cross-checked against golden fuzz, so the clip intent is
    # asserted; the RTL must equal golden dist of that value.
    for x, fuzzed in ((-20000, -15891), (5000, 5000), (20000, 19660)):
        got = await stage_once(0x6, x, stage=True)
        assert golden_model.fuzz(x, state["e"], 0) == fuzzed
        want = golden_model.dist(fuzzed)
        assert got in (None, want), f"fuzz+dist({x}): {got} != {want}"

    # all stages OFF: bit-exact identity, +-full-scale through
    assert await stage_once(0x0, 32767) == 32767
    assert await stage_once(0x0, -32768) == -32768


@cocotb.test()
async def test_enable_bounce(dut):
    """Switch debounce (v3 task 7): a burst of contact bounce on an
    enable followed by a stable level must produce exactly ONE clean
    transition on the output — never a glitch back. Fuzz (ui[2]) on a
    constant 20000 input: off -> 20000 (identity), on -> the tone
    low-pass of golden fuzz(20000) = 19660 (positive clip point,
    env-free), which settles from ~20000 down to golden's steady value
    over a few frames (task 11; the gate is open on the loud constant).
    The debounce samples every 256 frames and needs two agreeing ticks,
    so the switch toggles every 5 frames for 200 frames (at most one
    tick inside the burst), then holds; the engage output must start at
    20000, only ever fall (a bounce glitch back would rise), and end at
    the steady value, its first change within 600 frames of the hold.
    The release repeats the burst toward 0: exactly one steady -> 20000
    transition (dirt off is identity at once)."""
    await start(dut, 0)

    off_val = 20000
    assert golden_model.fuzz(20000, 0, 0) == 19660
    tg = golden_model.ToneGate()         # tone: settled on 20000, then 19660
    for _ in range(300):
        tg.tone(20000)
    for _ in range(300):
        on_val = tg.tone(19660)
    assert on_val == 19660               # approached from above: exact

    async def frame(ui):
        dut.ui_in.value = ui                       # between frames: bit0 low
        bits = await i2s_send_and_capture(dut, off_val << 8)
        await ClockCycles(dut.clk, 256)
        out = tx_data_bits(bits)
        return out - 0x10000 if out & 0x8000 else out

    async def bounce_then_hold(level):
        outs = []
        for i in range(200):                       # the burst: toggle every 5 frames
            outs.append(await frame(0b100 if ((i // 5) & 1) == 0 else 0))
        hold_start = len(outs)
        for _ in range(600):                       # held at its final level
            outs.append(await frame(0b100 if level else 0))
        return outs, hold_start

    # prime: let the constant input reach the output
    for _ in range(3):
        await frame(0)

    # engage: bounce (toggle every 5 frames, 200 frames), then hold on
    outs, hold_start = await bounce_then_hold(1)
    trans = [i for i in range(1, len(outs)) if outs[i] != outs[i - 1]]
    assert outs[0] == off_val, "must start un-fuzzed"
    assert all(b <= a for a, b in zip(outs, outs[1:])), "engage output rose again (glitch back)"
    assert trans and outs[-1] == on_val, f"engage must settle at {on_val}, got {outs[-1]}"
    assert trans[0] - hold_start <= 600, f"engage began {trans[0] - hold_start} frames after the hold"
    assert all(outs[i] == on_val for i in range(trans[0] + 20, len(outs))), "engage must settle within 20 frames"

    # release: bounce toward 0 starting from level 1, then hold 0
    outs = []
    for i in range(200):
        outs.append(await frame(0 if ((i // 5) & 1) == 0 else 0b100))
    hold_start = len(outs)
    for _ in range(600):
        outs.append(await frame(0))
    assert set(outs) <= {off_val, on_val}, f"unexpected output values {set(outs)}"
    trans = [i for i in range(1, len(outs)) if outs[i] != outs[i - 1]]
    assert outs[0] == on_val, "release phase must start fuzzed"
    assert len(trans) == 1, f"release must switch exactly once, transitions at {trans}"
    assert outs[trans[0]] == off_val and outs[-1] == off_val
    assert trans[0] - hold_start <= 600, f"release settled {trans[0] - hold_start} frames after the hold"


@cocotb.test()
async def test_pin_map(dut):
    """Rev B pin map at reset and idle: uio_oe = {1,1,k4,k3,k2,k1,1,1}
    (uio[7:6] + uio[1:0] outputs; uio[5:2] are the knob pins K4..K1,
    driven low only while the round-robin reader discharges that pin:
    K1 alone in reset and for the first 1024 clk after it, the others
    released), uio_out[5:2] = 0,
    uo[3:0] = 0b0010 (SPI SCK low, CS high, MOSI low, 595 RCLK low)
    with the dp LED and uo[6:4] dark."""
    cocotb.start_soon(
        Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2).start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    # asserted straight away in reset too — the idle level must not depend on state
    await Timer(1, unit="ns")
    assert dut.uio_oe.value == 0b1100_0111, f"uio_oe in reset: {dut.uio_oe.value}"
    assert int(dut.uio_out.value) & 0b0011_1100 == 0, "uio_out[5:2] must be 0 in reset"
    assert int(dut.uo_out.value) & 0x07 == 0b010, "uo[2:0] must idle SCK=0 CS=1 MOSI=0 in reset"
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 1020)
    await Timer(1, unit="ns")
    assert dut.uio_oe.value == 0b1100_0111, "K1 alone must discharge for 1024 clk"
    await ClockCycles(dut.clk, 10)
    await Timer(1, unit="ns")
    assert dut.uio_oe.value == 0b1100_0011, "K1 must release after the discharge"
    assert int(dut.uio_out.value) & 0b0011_1100 == 0, "uio_out[5:2] must be 0"
    await start(dut, 0, clock=False)
    assert int(dut.uio_oe.value) & 0b1100_0011 == 0b1100_0011, f"uio_oe: {dut.uio_oe.value}"
    assert int(dut.uo_out.value) & 0x07 == 0b010, \
        f"uo[2:0] must idle SCK=0 CS=1 MOSI=0, got {int(dut.uo_out.value) & 7:03b}"
    assert int(dut.uo_out.value) == 0x02, "uo[7:3] must be 0 at idle (dp dark, 595 RCLK low)"
    assert (int(dut.uo_out.value) >> 3) & 1 == 0, "uo[3] (595 RCLK) must idle low"


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
        half), lit by the end of that frame; uo[6:0] (reserved, idle:
        SPI CS high on uo[1], rest 0) stay 0x02;
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
        # uo[6:0] are reserved and idle (SPI CS high on uo[1], rest 0):
        # the dp must never disturb them — PORT-level assert, active in GL too
        return (int(dut.uo_out.value) & 0x7F) == 0x02

    # 100 mid-level frames (frames 0..99): dp never lights
    for _ in range(100):
        await i2s_send_and_capture(dut, 0x1000 << 8)
        await ClockCycles(dut.clk, 256)
    await Timer(1, unit="ns")
    assert dp() == 0, "mid-level frames must never light the dp"
    assert seg_ok(), "uo[6:0] must stay idle (0x02) with the dp dark"

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
    assert seg_ok(), "uo[6:0] must stay idle (0x02) during the clip"

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
    assert seg_ok(), "uo[6:0] must still read idle (0x02) after the dp clears"


@cocotb.test()
async def test_chain_latency(dut):
    """Task 8 (sequenced chain): pins the chain's frame-level latency
    and the clip-LED timing across the combinational -> 6-step refactor.
    (a) en = 0x17 (comp + fuzz + dist + trem; ui_in = 0x2E): three known frames then a zero frame; each
    TX value (captured one frame after its input, mono contract) equals
    golden_model.render of the same input history — the silent settle
    frames are rendered too so env and the trem LFO line up. (b) en = 0:
    a 0x7FFF frame lights uo[7] within CHAIN_MAX_CLK clk of the frame's
    strobe (not before it), well before the frame's TX load of the
    NEXT frame, and a mid-level frame in the same position does not.
    (c) RTL probe, every effect on (en = 0x3F, octave on): chain_done
    pulses once per frame, at most CHAIN_MAX_CLK clk after sample_valid
    (Task 15a: the multiplier steps wait for the serial multiplier)."""
    ins = [12000, -20000, 30000, 0]
    await start(dut, 0x17 << 1)
    xs = [0] * START_FRAME + ins
    want = golden_model.render(xs, 0x17)
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
        await start(dut, 0xFE, clock=False)     # en = 0x3F, octave on
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
    """Post-dirt tone (v3 task 11), dist on, K2 absent (kt = 2): the
    one-pole low-pass ytone += ((s3 << 4) - ytone) >>> 2. An alternating
    +-8000 input (Nyquist; dist maps it to +-14144) is attenuated to
    |out| < 3000 (golden: 2021 once the gate is open), and a DC input of
    8000 settles to golden's 14143 (the floor leaves the 16.4 state a
    few LSB under dist's 14144). Each run is exact against golden for
    every frame, from a fresh reset (the gate opens within ~60 frames)."""
    n = 300
    for i, (name, xs) in enumerate((
            ("nyquist", [8000 if j % 2 == 0 else -8000 for j in range(n)]),
            ("dc", [8000] * n))):
        await start(dut, 0b0000_1000, clock=(i == 0))   # en[2] dist only
        got = await fast_frames(dut, xs + [0])
        want = golden_model.render([0] * START_FRAME + xs, 0x04)[START_FRAME:]
        assert got[1:] == want, f"{name}: first mismatch at " \
            f"{next(j for j in range(n) if got[1 + j] != want[j])}"
        if name == "nyquist":
            assert max(abs(v) for v in got[-50:]) < 3000, f"nyquist {got[-6:]}"
            assert got[-1] == -2021
        else:
            assert len(set(got[-10:])) == 1 and got[-1] == 14143, f"dc {got[-10:]}"


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
    await start(dut, 0b0000_1000)                     # en[2] dist only
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
    want = golden_model.render(hist, 0x04)
    tail = want[-300:]
    assert tail[254] > 0 and tail[255] == 0 and not any(tail[255:]), "golden gg decay"
    assert got_dc[1:] == tail, "decay run differs from golden at " \
        f"{next(j for j in range(300) if got_dc[1 + j] != tail[j])}"
    # the loud phases' captures agree with golden too (capture j = frame j-1)
    assert got[1:] == want[START_FRAME:START_FRAME + len(got) - 1]


@cocotb.test()
async def test_dirt_off_identity(dut):
    """Tone/gate act only when fuzz or dist is on (v3 task 11). With trem
    only (en[4]) the chain output equals golden render bit-exactly,
    +-full-scale and -32768 included, and equals the render with the
    tone/gate stage skipped (tgb): the 16.4 tone state runs every frame
    but never reaches the output. With phaser only (Task 16) the words
    equal golden render (the stub's old identity no longer holds)."""
    xs = [32767, -32768, 1000, -1, 0, -32768, 12345, 0]
    await start(dut, 0b0010_0000)                     # en[4] trem only
    got = await fast_frames(dut, xs + [0])
    want = golden_model.render([0] * START_FRAME + xs, 0x10)[START_FRAME:]
    assert want == golden_model.render([0] * START_FRAME + xs, 0x10, tgb=True)[START_FRAME:]
    assert got[1:] == want, f"trem only: {got[1:]} != {want}"
    await start(dut, 0b0001_0000, clock=False)        # en[3] phaser only
    got = await fast_frames(dut, xs + [0])
    want = golden_model.render([0] * START_FRAME + xs, 0x08)[START_FRAME:]
    assert got[1:] == want, f"phaser only: {got[1:]} != {want}"


@cocotb.test()
async def test_env_decay(dut):
    """Fast envelope release (routed from task 10's review, RTL-only
    guarded probe): 100 full-scale frames attack env to 32766, then
    1388 silent frames bring it to exactly 0 (the 1-LSB release floor,
    golden Env): still non-zero one frame earlier, 0 at frame 1388."""
    await start(dut, 0b0000_0010)                     # en[0] comp only
    probe = _probe(dut, "env")
    assert probe is not None or _probe(dut, "step") is None, "RTL has no env"
    ref = golden_model.Env()
    for _ in range(START_FRAME):
        ref.step(0)
    for _ in range(100):
        ref.step(0x7FFF)
    assert ref.v == 32766
    n = 0
    while ref.v:
        ref.step(0)
        n += 1
    assert n == 1388
    await fast_frames(dut, [0x7FFF] * 100)
    if probe is not None:
        assert int(probe.value) == 32766
    await ClockCycles(dut.clk, 512 * (n - 1))
    if probe is not None:
        assert int(probe.value) == 1, f"env one frame before the floor: {int(probe.value)}"
    await ClockCycles(dut.clk, 512)
    if probe is not None:
        assert int(probe.value) == 0, f"env after {n} silent frames: {int(probe.value)}"


# ---------------------------------------------------------------------
# Phaser (Rev B1, Task 16): 4 first-order allpass stages on the shared
# multiplier (chain steps 6..9), 50/50 mix at step 10, own 22-bit phase
# (RTL phaser_p, golden Chain.pp) at the K1 rate (shared with trem; 87
# with K1 absent). Expected values come from golden_model.
# ---------------------------------------------------------------------
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
    output state is non-zero, the phaser is warm). (b) en = 0x02 (fuzz,
    the gate in): the port words equal golden render, i.e. s4."""
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
    await start(dut, 0x02 << 1, clock=False)
    xs2 = _sweep(200, 200, 4000, 8000)
    got = await fast_frames(dut, xs2 + [0])
    want = golden_model.render([0] * START_FRAME + xs2, 0x02)[START_FRAME:]
    assert got[1:] == want, f"fuzz, phaser off: frame {_first_diff(got[1:], want)}"


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
    await start(dut, 0x08 << 1)                       # en[3], K1 absent
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
    """Phaser on, K1 = 250 (inc 976): a 200 Hz - 4 kHz exponential sweep at
    amplitude 8000 for 4000 frames; every port word equals golden."""
    knob_pin(dut, 0, 250)
    await start(dut, 0x08 << 1)
    xs = _sweep(4000, 200, 4000, 8000)
    got = (await fast_frames(dut, xs + [0]))[1:]
    want = golden_model.render([0] * START_FRAME + xs, 0x08, k1=250)[START_FRAME:]
    first = _first_diff(got, want)
    assert first is None, f"frame {first}: got {got[first]} want {want[first]}"
    assert got != xs and len(set(got)) > 1000, "phaser must change the signal"


@cocotb.test()
async def test_phaser_rails(dut):
    """Phaser on, K1 absent: a +-32767 square at 100 Hz (240 frames per
    half) for 2000 frames equals golden every frame, and never wraps sign
    against it (the sat16 stage sums). RTL probe: the phase steps by
    PHASER_INC_ABSENT = 87 per frame."""
    await start(dut, 0x08 << 1)
    xs = [32767 if (i // 240) % 2 == 0 else -32767 for i in range(2000)]
    got = (await fast_frames(dut, xs + [0]))[1:]
    want = golden_model.render([0] * START_FRAME + xs, 0x08)[START_FRAME:]
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
async def test_phaser_rate_k1(dut):
    """K1 = 250 sets the phaser rate (shared with trem): RTL probe, the
    phaser phase advances by phaser_inc(250) = 976 per frame over 5 frames
    and equals 976 * frames since EN_SETTLE; port level, 20 words of a
    constant-ish input equal golden (k1 = 250)."""
    assert golden_model.phaser_inc(250) == golden_model.trem_inc(250) == 976
    assert golden_model.phaser_inc(None) == golden_model.PHASER_INC_ABSENT == 87
    knob_pin(dut, 0, 250)
    await start(dut, 0x08 << 1)
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
    want = golden_model.render([0] * f0 + xs, 0x08, k1=250)[f0:]
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
