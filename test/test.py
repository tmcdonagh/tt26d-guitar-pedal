# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-FileCopyrightText: © 2026 Tom McDonagh
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

    # Reset state: DAC data low, LED bar dark
    assert dut.uo_out[0].value == 0, "DAC data not low after reset"
    assert ((int(dut.uo_out.value) >> 1) & 0x1F) == 0, "LED bar not dark after reset"

    # MCLK output enabled on uio[0] and following clk.
    # Sample away from clock edges (comb outputs need settle time).
    assert dut.uio_oe.value == 0b0000_0001, "uio_oe must enable only uio[0]"
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

    # Frame clock is running: LRCLK (uo[7]) must toggle within one frame (512 clk)
    start = dut.uo_out[7].value
    toggled = False
    for _ in range(600):
        await ClockCycles(dut.clk, 1)
        if dut.uo_out[7].value != start:
            toggled = True
            break
    assert toggled, "LRCLK never toggled — frame counter not running"

    # Mode select bits reach the mux (bypass stub returns sample_in=0)
    dut.ui_in.value = 0b0000_1110  # mode 7, params 0
    await ClockCycles(dut.clk, 1)
    assert dut.uo_out[0].value == 0, "unexpected DAC output in skeleton"


# ---------------------------------------------------------------------
# I2S helpers (used by all frame-level tests).
#
# Framing: 512 clk per frame; BCLK (uo[6]) rises mid-slot (clk cycles
# 8k+4) and falls at slot boundaries (8k). Slots 0..31 = left half (WS
# low), 32..63 = right half. Data bits ride slots 1..16, MSB first,
# after the one-slot I2S delay. ADC data in on ui[0], DAC data out on
# uo[0]. All sampling uses a settle Timer to avoid edge races.
# ---------------------------------------------------------------------

CLK_PERIOD_PS = 40534


async def _bclk_edge(dut, want_rising, settle_ns=1):
    """Wait for the next BCLK edge (uo[6] transition), return after settle."""
    prev = int(dut.uo_out[6].value)
    while True:
        await ClockCycles(dut.clk, 1)
        await Timer(settle_ns, unit="ns")
        cur = int(dut.uo_out[6].value)
        if cur != prev:
            if (cur == 1) == want_rising:
                return
            prev = cur
        else:
            prev = cur


async def _wait_left_start(dut):
    """Wait for the next WS falling edge (LRCLK 1->0) = start of a left
    (data) half-frame."""
    prev = int(dut.uo_out[7].value)
    while True:
        await ClockCycles(dut.clk, 1)
        await Timer(1, unit="ns")
        cur = int(dut.uo_out[7].value)
        if prev == 1 and cur == 0:
            return
        prev = cur


async def i2s_send_and_capture(dut, word24):
    """Drive one left-channel frame on ui[0] (24-bit word, slots 1..24)
    and capture the 32-slot left half of the same frame from uo[0].

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
        bits.append(int(dut.uo_out[0].value))
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
    prev_b = int(dut.uo_out[6].value)
    prev_l = int(dut.uo_out[7].value)
    for _ in range(1620):
        await Timer(10134, unit="ps")
        b = int(dut.uo_out[6].value)
        l = int(dut.uo_out[7].value)
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
    uo[0] slots 1..16 in frame N+1; slot 0 low (I2S delay), slots 17..31 low."""
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
    frame with a clean (zero) input must be silent, LED bar dark."""
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
    assert ((int(dut.uo_out.value) >> 1) & 0x1F) == 0, "LED bar must be dark after reset"


@cocotb.test()
async def test_led_meter(dut):
    """LED bar = max(|sample|>>11 clamped to 15, bar-1), updated once per
    frame. Sequence: full-scale -> 15; decay by 1 per sample; 16 zeros
    reach 0; small sample re-lights to its level (max rule)."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    async def send_and_read_bar(word16):
        await _wait_left_start(dut)
        await i2s_send_and_capture(dut, word16 << 8)
        await ClockCycles(dut.clk, 4)
        await Timer(1, unit="ns")
        return (int(dut.uo_out.value) >> 1) & 0x1F

    assert await send_and_read_bar(0x8000) == 15, "|-32768| saturates to bar 15"
    bar_decay = await send_and_read_bar(0x0000)
    assert bar_decay == 14, f"one zero frame should decay to 14, got {bar_decay}"
    for _ in range(16):
        bar = await send_and_read_bar(0x0000)
    assert bar == 0, f"16 zero frames must decay to 0, got {bar}"
    assert await send_and_read_bar(0x0FFF) == 1, "|4095|>>11 = bar 1"


# ui byte layout: ui[0]=I2S data (driven by helper), ui[3:1]=mode,
# ui[5:4]=bit-depth sel, ui[7:6]=decimation sel

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
async def test_ring_mod_square(dut):
    """Mode 3: output = input * (+/-1) alternating with an LFO whose
    half-period is 2^(10-k) samples (k = ui[6:4]). DC +1000 in, k=2
    (half-period 256 frames): sign flips every 256 frames, amplitude
    exactly +/-1000."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0b0010_011_0  # mode 3, k=2 (half-period 256)
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    outs = []
    for _ in range(600):
        await _wait_left_start(dut)
        outs.append(tx_data_bits(await i2s_send_and_capture(dut, 1000 << 8)))

    signed = [o - 0x10000 if o & 0x8000 else o for o in outs]
    # outs[0] is the partial reset frame (may be 0) — same as Task 3 ruling
    assert set(signed[1:]) <= {1000, -1000}, \
        f"ring mod output must be +/-1000, saw {[hex(o) for o in outs[:8]]}..."
    assert signed[31] == signed[201], "sign must be stable within one phase"
    assert signed[31] == -signed[287], "sign must flip after one half-period (256)"
    assert signed[31] == signed[543], "sign must return after a full period (512)"


@cocotb.test()
async def test_distortion_gain(dut):
    """Mode 2: out = sat16(in * G), G = 1 + ui[5:4] (1..5)."""
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
    # G=5 (ui[5:4]=11): 20000*5 saturates
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
    """Mode 4: out = (in * f) >> 8 via the serial multiplier; f = triangle
    255<->min over 2*2^(10-k) frames. k=7 (ramp=8 frames), min=0.
    The multiplier uses f of the post-increment LFO counter; with the
    partial reset frame consuming count 0, the expected output sequence
    (DC 16384 in) is exactly:"""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0b11101000  # mode 4 (ui[3:1]=100), k=7 (ui[7:5]=111), min=0 (ui[4]=0)
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    def ffall(c): return 255 - ((c * 255) >> 3)
    def frise(c): return (c * 255) >> 3
    exp_outs = [0,                                   # partial frame: input 0
                (16384 * ffall(2)) >> 8, (16384 * ffall(3)) >> 8,
                (16384 * ffall(4)) >> 8, (16384 * ffall(5)) >> 8,
                (16384 * ffall(6)) >> 8, (16384 * ffall(7)) >> 8,
                0,                                   # count wraps, rising f(0)=0
                (16384 * frise(1)) >> 8, (16384 * frise(2)) >> 8,
                (16384 * frise(3)) >> 8, (16384 * frise(4)) >> 8,
                (16384 * frise(5)) >> 8, (16384 * frise(6)) >> 8,
                (16384 * frise(7)) >> 8,
                (16384 * ffall(0)) >> 8]             # wraps again: f=255

    outs = []
    for _ in range(len(exp_outs)):
        await _wait_left_start(dut)
        outs.append(tx_data_bits(await i2s_send_and_capture(dut, 16384 << 8)))
    assert outs == exp_outs, \
        f"tremolo sequence mismatch: got {[hex(o) for o in outs]} exp {exp_outs}"

    # sign preservation: negative DC processed two frames after the sequence
    # ends, where the deterministic f cycle is at ffall(2) = 192
    await _wait_left_start(dut)
    await i2s_send_and_capture(dut, ((-16384) & 0xFFFF) << 8)
    await _wait_left_start(dut)
    out_neg = tx_data_bits(await i2s_send_and_capture(dut, 0))
    out_neg = out_neg - 0x10000 if out_neg & 0x8000 else out_neg
    assert out_neg == -((16384 * ffall(2)) >> 8), f"negative DC: got {out_neg}"


@cocotb.test()
async def test_tone(dut):
    """Mode 7: internal square tone, half-period = 55 << k frames
    (k = ui[6:4]), levels +/-16383, ADC input ignored. Phase after
    mid-frame reset is deterministic within +/-1 frame, so assert one
    exact sign change per window and exact amplitude."""
    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0b000_111_0  # mode 7 (ui[3:1]=111), k=0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    async def capture(n):
        vals = []
        for _ in range(n):
            await _wait_left_start(dut)
            v = tx_data_bits(await i2s_send_and_capture(dut, 0x5151 << 8))
            vals.append(v - 0x10000 if v & 0x8000 else v)
        return vals

    # k=0: half-period 55 frames -> exactly one flip in an 80-frame window
    vals = await capture(80)
    assert set(vals) == {16383, -16383}, f"tone levels wrong: {set(vals)}"
    flips = sum(1 for a, b in zip(vals, vals[1:]) if (a > 0) != (b > 0))
    assert flips == 1, f"k=0 half-period 55: expected 1 flip in 80 frames, got {flips}"

    # k=2: half-period 220 -> exactly one flip in a 250-frame window
    dut.ui_in.value = 0b0_010_111_0  # mode 7 (ui[3:1]=111), k=2 (ui[6:4]=010)
    vals = await capture(250)
    assert set(vals) == {16383, -16383}, f"tone levels wrong (k=2): {set(vals)}"
    flips = sum(1 for a, b in zip(vals, vals[1:]) if (a > 0) != (b > 0))
    assert flips == 1, f"k=2 half-period 220: expected 1 flip in 250 frames, got {flips}"
