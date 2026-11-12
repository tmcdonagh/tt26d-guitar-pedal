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
