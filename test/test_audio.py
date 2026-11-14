# SPDX-FileCopyrightText: © 2026 Tom McDonagh
# SPDX-License-Identifier: Apache-2.0
#
# Audio listening utility: streams a 16-bit 48 kHz mono WAV through the
# design's effect path and writes one WAV per mode to test/audio_out/.
#
# This drives the internal sample_in/effect_sample signals directly
# (one 512-clk batch per sample) — the I2S pin serialization is already
# verified bit-exact by test.py; this tool is for evaluating the DSP by ear.
#
# Run (from test/):
#   PATH=/home/tom/dev/.venv-tt/bin:$PATH make COCOTB_TEST_MODULES=test_audio
#
# Input: audio_in.wav (or $AUDIO_IN). Outputs: audio_out/<mode>.wav

import array
import os
import wave

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, Timer

from test import CLK_PERIOD_PS

SR = 48000
IN_WAV = os.environ.get("AUDIO_IN", "audio_in.wav")
OUT_DIR = os.path.join(os.path.dirname(__file__), "audio_out")

# ui byte: ui[7:4] = params, ui[3:1] = mode. Param values chosen as
# sensible defaults; tweak and re-run to tune by ear.
MODES = [
    ("bypass", 0, 0x0),
    ("fuzz",   1, 0x0),   # T=24576
    ("dist1",  2, 0x1),   # G=2
    ("dist2",  2, 0x2),   # G=3
    ("dist3",  2, 0x3),   # G=4
    ("ring",   3, 0x0),   # slowest LFO (k=0)
    ("trem",   4, 0x0),   # k=7, min=0 (full steps)
    ("crush8", 5, 0x2),   # N=8, no decimation
    ("chain",  6, 0x0),
]


def load_wav(path):
    """Return (list of int16 samples, sample_rate). Mono-mixes, converts
    bit depths, and naive-linear-resamples if the rate differs."""
    w = wave.open(path, "rb")
    n, ch, sw, fr = w.getnframes(), w.getnchannels(), w.getsampwidth(), w.getframerate()
    raw = w.readframes(n)
    w.close()
    a = array.array("h")
    if sw != 2:
        raise SystemExit(f"unsupported sample width {sw*8} bits; convert to s16 first")
    a.frombytes(raw)
    if ch > 1:
        a = array.array("h", (sum(a[i : i + ch]) // ch for i in range(0, len(a), ch)))
    if fr != SR:
        # naive linear resample (phone recordings are 44.1k/48k; 48k needs none)
        out = []
        for i in range(0, len(a) - ch, ch * fr // SR or 1):
            out.append(a[i])
        a = array.array("h", out)
    return a, fr


def write_wav(path, samples):
    w = wave.open(path, "wb")
    w.setnchannels(1)
    w.setsampwidth(2)
    w.setframerate(SR)
    w.writeframes(array.array("h", samples).tobytes())
    w.close()


@cocotb.test()
async def test_audio_through(dut):
    """Stream the input WAV through every mode; write one WAV per mode."""
    dut._log.info(f"audio in: {IN_WAV}")
    samples, fr = load_wav(IN_WAV)
    dut._log.info(f"{len(samples)} samples at {fr} Hz "
                  f"({len(samples) / SR:.1f} s), {len(MODES)} modes")

    clock = Clock(dut.clk, CLK_PERIOD_PS, unit="ps", period_high=CLK_PERIOD_PS // 2)
    cocotb.start_soon(clock.start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    # align our 512-clk batches to the frame counter phase (lr_cnt==0 after reset)
    await ClockCycles(dut.clk, 512 - 10)
    await Timer(1, unit="ns")

    up = dut.user_project
    os.makedirs(OUT_DIR, exist_ok=True)

    for name, mode, params in MODES:
        dut.ui_in.value = (params << 4) | (mode << 1)
        out = array.array("h")
        for s in samples:
            up.sample_in.value = s & 0xFFFF
            await ClockCycles(dut.clk, 512)
            v = int(up.effect_sample.value)
            out.append(v - 0x10000 if v & 0x8000 else v)
        path = os.path.join(OUT_DIR, f"{name}.wav")
        write_wav(path, out)
        dut._log.info(f"wrote {path}")
