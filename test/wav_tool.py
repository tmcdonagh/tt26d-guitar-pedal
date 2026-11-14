#!/usr/bin/env python3
# SPDX-FileCopyrightText: © 2026 Thomas McDonagh
# SPDX-License-Identifier: Apache-2.0
"""Audio listening pass: WAV -> hex -> simulator -> hex -> WAV, all modes.

Simulator: verilator --binary (fast, default), icarus (make SIM=icarus), or
the gate-level netlist (SIM=gl — needs PDK_ROOT and test/gate_level_netlist.v;
see gl_wav_tool.md; outputs go to audio_out_gl/ and must be bit-exact vs the
RTL audio_out/ baseline, checked by `python3 wav_tool.py gl-parity`).
Usage (from test/):
    python3 wav_tool.py run          # convert in, run all modes, convert out
    python3 wav_tool.py wav2hex
    python3 wav_tool.py hex2wav
    python3 wav_tool.py gl-cmd       # print the GL iverilog command (dry run)
    python3 wav_tool.py gl-parity    # bit-exact GL-vs-RTL WAV check
"""
import array
import os
import re
import shlex
import subprocess
import sys
import wave

SR = 48000
HERE = os.path.dirname(os.path.abspath(__file__))
IN_WAV = os.path.join(HERE, "audio_in.wav")
IN_HEX = os.path.join(HERE, "audio_in.hex")
OUT_DIR = os.path.join(HERE, "audio_out")
OUT_DIR_GL = os.path.join(HERE, "audio_out_gl")  # netlist outputs; audio_out/ = RTL parity baseline
GL_VVP = os.path.join(HERE, "sim_audio_gl.vvp")

# name, en (ui[7:4], bit order = chain order en[0]=fuzz..en[3]=trem),
# params (P3..P0 on uio[5:2])
MODES = [
    ("bypass",   0x0, 0x0),
    ("fuzz",     0x1, 0x0),   # fixed clip 16384
    ("dist",     0x2, 0x0),   # soft-knee PWL, P0=0 (T=8192)
    ("distX",    0x2, 0x1),   # soft-knee PWL, P0=1 (T=4096)
    ("crush",    0x4, 0x0),   # auto-lofi, P1=0 base: N=6 loud / N=4 quiet
    ("crushQ",   0x4, 0x2),   # auto-lofi, P1=1 base: N=8 loud / N=6 quiet
    ("trem",     0x8, 0x0),   # k=0 (~23.4 Hz), span 255 (4 steps)
    ("tremS",    0x8, 0xC),   # k=1 (~46.9 Hz), span 127 (2 steps)
    ("fuzzdist", 0x3, 0x0),   # rectify + clip 16384, then soft knee
    ("all4",     0xF, 0x0),   # full stack
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


def hex2wav(name, out_dir=OUT_DIR):
    # sample lines are $display'ed to stdout, captured in the .log file
    vals = []
    with open(os.path.join(out_dir, f"{name}.log")) as f:
        for line in f:
            s = line.strip()
            if len(s) == 4 and all(c in "0123456789abcdefABCDEF" for c in s):
                vals.append(int(s, 16))
    samples = array.array("h", (v - 0x10000 if v & 0x8000 else v for v in vals))
    path = os.path.join(out_dir, f"{name}.wav")
    w = wave.open(path, "wb")
    w.setnchannels(1)
    w.setsampwidth(2)
    w.setframerate(SR)
    w.writeframes(samples.tobytes())
    w.close()
    return path, len(samples)


def gl_cmd():
    """iverilog command for the gate-level audio build — mirrors test/Makefile
    GATES=yes exactly: same define set, same sky130 cell sources, same order
    (primitives.v, sky130_fd_sc_hd.v, gate_level_netlist.v)."""
    lib = os.path.join(os.environ.get("PDK_ROOT", ""),
                       "sky130A", "libs.ref", "sky130_fd_sc_hd", "verilog")
    return ["iverilog", "-g2012", "-s", "tb_audio_gl",
            "-DGL_TEST", "-DFUNCTIONAL", "-DUSE_POWER_PINS", "-DSIM",
            "-DUNIT_DELAY=#1", "-o", GL_VVP,
            os.path.join(HERE, "tb_audio_gl.v"),
            os.path.join(lib, "primitives.v"),
            os.path.join(lib, "sky130_fd_sc_hd.v"),
            os.path.join(HERE, "gate_level_netlist.v")]


GLVL_LIB = os.path.join(HERE, "obj_dir_gl", "sky130_fd_sc_hd_glv.v")


def glvl_make_lib():
    """Write a Verilator-compatible copy of the cell library: the _pp$PG$N
    UDP instantiations skip NOTIFIER with an empty positional argument
    (`, ,`), which Verilator 5.032 rejects. 1'bx is exact for our stub
    models (NOTIFIER is ignored under FUNCTIONAL) and for the real UDP
    (constant input -> no table row fires)."""
    lib = os.path.join(os.environ.get("PDK_ROOT", ""),
                       "sky130A", "libs.ref", "sky130_fd_sc_hd", "verilog",
                       "sky130_fd_sc_hd.v")
    os.makedirs(os.path.dirname(GLVL_LIB), exist_ok=True)
    with open(lib) as f, open(GLVL_LIB, "w") as out:
        for line in f:
            if "sky130_fd_sc_hd__udp_" in line:
                line = re.sub(r",\s*,", ", 1'bx,", line)
            # Verilator parses `#1 inst` (the UNIT_DELAY expansion) as a
            # parameter override, not an instance delay — strip it; zero
            # delay on cell instances is exact for this tb's sampling
            # scheme (outputs only move on clk edges).
            line = line.replace("`UNIT_DELAY ", "")
            out.write(line)


def glvl_cmd():
    """Verilator gate-level audio build — the fast GL path (iverilog GL is
    ~7 h/mode on a 1x1 netlist; Verilator compiles the same FUNCTIONAL cell
    models to C++, typically 10-100x faster, and is required for 2x2-sized
    netlists). primitives.v (UDP models) is omitted: with -DFUNCTIONAL the
    cells come from the behavioral models in the transformed library copy,
    and the UDPs themselves come from glvl_stub.v."""
    out = os.path.join(HERE, "obj_dir_gl")
    return ["verilator", "--binary", "--timing", "-j", "16",
            "-Wno-fatal", "-o", "sim_audio_glv",
            "--Mdir", out,
            "-DGL_TEST", "-DFUNCTIONAL", "-DUSE_POWER_PINS", "-DSIM",
            "-DUNIT_DELAY=#1", "--top-module", "tb_audio_gl",
            os.path.join(HERE, "tb_audio_gl.v"),
            os.path.join(HERE, "glvl_stub.v"),
            GLVL_LIB,
            os.path.join(HERE, "gate_level_netlist.v")]


def glvl_build():
    netlist = os.path.join(HERE, "gate_level_netlist.v")
    if not os.path.exists(netlist):
        raise SystemExit(f"missing {netlist} — copy it from the GDS run artifacts (gl_wav_tool.md)")
    if not os.environ.get("PDK_ROOT"):
        raise SystemExit("PDK_ROOT must point at the sky130A PDK (see gl_wav_tool.md)")
    glvl_make_lib()
    subprocess.check_call(glvl_cmd(), cwd=HERE)


def gl_build():
    netlist = os.path.join(HERE, "gate_level_netlist.v")
    if not os.path.exists(netlist):
        raise SystemExit(f"missing {netlist} — copy it from the GDS run artifacts (gl_wav_tool.md)")
    if not os.environ.get("PDK_ROOT"):
        raise SystemExit("PDK_ROOT must point at the sky130A PDK (see gl_wav_tool.md)")
    subprocess.check_call(gl_cmd(), cwd=HERE)


def gl_parity():
    """Parity gate: every GL mode's .wav must be bit-exact vs the RTL .wav."""
    bad = []
    for name, _, _ in MODES:
        with wave.open(os.path.join(OUT_DIR, f"{name}.wav")) as a, \
             wave.open(os.path.join(OUT_DIR_GL, f"{name}.wav")) as b:
            ok = ((a.getnchannels(), a.getsampwidth(), a.getframerate(), a.getnframes())
                  == (b.getnchannels(), b.getsampwidth(), b.getframerate(), b.getnframes())
                  and a.readframes(a.getnframes()) == b.readframes(b.getnframes()))
        print(f"{name}: {'PASS' if ok else 'FAIL'}")
        if not ok:
            bad.append(name)
    if bad:
        raise SystemExit(f"parity FAIL: {bad} — systematic debugging before any 'close enough' conclusion")


def run_mode(name, en, params, n, sim):
    out = os.path.join(OUT_DIR_GL if sim.startswith("gl") else OUT_DIR, f"{name}.hex")
    if sim == "gl":
        cmd = ["vvp", GL_VVP, f"+EN={en}", f"+PARAMS={params}",
               f"+N={n}", f"+OUT={out}"]
    elif sim == "glvl":
        exe = os.path.join(HERE, "obj_dir_gl", "sim_audio_glv")
        cmd = [exe, f"+EN={en}", f"+PARAMS={params}", f"+N={n}", f"+OUT={out}"]
    elif sim == "verilator":
        exe = os.path.join(HERE, "obj_dir", "sim_audio_vl")
        cmd = [exe, f"+EN={en}", f"+PARAMS={params}", f"+N={n}", f"+OUT={out}"]
    else:
        cmd = ["vvp", "sim_audio.vvp", f"+EN={en}", f"+PARAMS={params}",
               f"+N={n}", f"+OUT={out}"]
    log = open(os.path.join(OUT_DIR_GL if sim.startswith("gl") else OUT_DIR, f"{name}.log"), "w")
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
    elif what == "gl-cmd":
        print(shlex.join(gl_cmd()))
    elif what == "glvl-cmd":
        print(shlex.join(glvl_cmd()))
    elif what == "gl-build":
        gl_build()
    elif what == "glvl-build":
        glvl_build()
    elif what == "gl-parity":
        gl_parity()
    elif what == "run":
        out_dir = OUT_DIR_GL if sim.startswith("gl") else OUT_DIR
        os.makedirs(out_dir, exist_ok=True)
        if sim == "gl":
            gl_build()
        elif sim == "glvl":
            glvl_build()
        n = wav2hex()
        procs = [run_mode(name, en, params, n, sim) for name, en, params in MODES]
        for p in procs:
            p.wait()
        for name, _, _ in MODES:
            path, cnt = hex2wav(name, out_dir)
            print(f"{path}: {cnt} samples")
    else:
        raise SystemExit(__doc__)


if __name__ == "__main__":
    main()
