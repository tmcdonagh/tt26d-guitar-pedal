#!/usr/bin/env python3
# SPDX-FileCopyrightText: © 2026 Thomas McDonagh
# SPDX-License-Identifier: Apache-2.0
"""Audio listening pass: WAV -> hex -> simulator -> hex -> WAV, all modes.

Simulator: verilator --binary (fast, default) or icarus (make SIM=icarus).
Usage (from test/):
    python3 wav_tool.py run          # convert in, run all modes, convert out
    python3 wav_tool.py wav2hex
    python3 wav_tool.py hex2wav
"""
import array
import os
import subprocess
import sys
import wave

SR = 48000
HERE = os.path.dirname(os.path.abspath(__file__))
IN_WAV = os.path.join(HERE, "audio_in.wav")
IN_HEX = os.path.join(HERE, "audio_in.hex")
OUT_DIR = os.path.join(HERE, "audio_out")

# name, mode, params
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


def wav2hex():
    w = wave.open(IN_WAV, "rb")
    n, ch, sw, fr = w.getnframes(), w.getnchannels(), w.getsampwidth(), w.getframerate()
    a = array.array("h")
    a.frombytes(w.readframes(n))
    w.close()
    assert sw == 2 and fr == SR, f"convert input first: ffmpeg -i in.m4a -ar 48000 -ac 1 -sample_fmt s16 {IN_WAV}"
    if ch > 1:
        a = array.array("h", (sum(a[i:i + ch]) // ch for i in range(0, len(a), ch)))
    with open(IN_HEX, "w") as f:
        f.write("\n".join("%04x" % (s & 0xFFFF) for s in a))
    return len(a)


def hex2wav(name):
    # sample lines are $display'ed to stdout, captured in the .log file
    vals = []
    with open(os.path.join(OUT_DIR, f"{name}.log")) as f:
        for line in f:
            s = line.strip()
            if len(s) == 4 and all(c in "0123456789abcdefABCDEF" for c in s):
                vals.append(int(s, 16))
    samples = array.array("h", (v - 0x10000 if v & 0x8000 else v for v in vals))
    path = os.path.join(OUT_DIR, f"{name}.wav")
    w = wave.open(path, "wb")
    w.setnchannels(1)
    w.setsampwidth(2)
    w.setframerate(SR)
    w.writeframes(samples.tobytes())
    w.close()
    return path, len(samples)


def run_mode(name, mode, params, n, sim):
    out = os.path.join(OUT_DIR, f"{name}.hex")
    if sim == "verilator":
        exe = os.path.join(HERE, "obj_dir", "sim_audio_vl")
        cmd = [exe, f"+MODE={mode}", f"+PARAMS={params}", f"+N={n}", f"+OUT={out}"]
    else:
        cmd = ["vvp", "sim_audio.vvp", f"+MODE={mode}", f"+PARAMS={params}",
               f"+N={n}", f"+OUT={out}"]
    log = open(os.path.join(OUT_DIR, f"{name}.log"), "w")
    return subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, cwd=HERE)


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else "run"
    sim = os.environ.get("SIM", "verilator")
    os.makedirs(OUT_DIR, exist_ok=True)
    if what == "wav2hex":
        print(wav2hex(), "samples")
    elif what == "hex2wav":
        for name, _, _ in MODES:
            p, n = hex2wav(name)
            print(f"{p}: {n} samples")
    elif what == "run":
        n = wav2hex()
        procs = [run_mode(name, mode, params, n, sim) for name, mode, params in MODES]
        for p in procs:
            p.wait()
        for name, _, _ in MODES:
            path, cnt = hex2wav(name)
            print(f"{path}: {cnt} samples")
    else:
        raise SystemExit(__doc__)


if __name__ == "__main__":
    main()
