# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-FileCopyrightText: © 2026 Tom McDonagh
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles


@cocotb.test()
async def test_skeleton_baseline(dut):
    """Task 1 baseline: reset state, pin wiring, and running frame clock."""
    dut._log.info("Start")

    clock = Clock(dut.clk, 40.535, unit="ns")  # 24.576 MHz
    cocotb.start_soon(clock.start())

    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    # Reset state: DAC data low, LED bar dark
    assert dut.uo_out[0].value == 0, "DAC data not low after reset"
    assert dut.uo_out[5:1].value == 0, "LED bar not dark after reset"

    # MCLK output enabled on uio[0] and following clk
    assert dut.uio_oe.value == 0b0000_0001, "uio_oe must enable only uio[0]"
    for _ in range(4):
        v = dut.uio_out[0].value
        await ClockCycles(dut.clk, 1)
        assert dut.uio_out[0].value != v, "MCLK must follow clk"

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
