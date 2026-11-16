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
    python3 wav_tool.py check        # ~6 s excerpt: Verilator vs golden model
    python3 wav_tool.py model        # full-riff golden renders -> audio_model/
"""
import array
import os
import re
import hashlib
import shlex
import struct
import subprocess
import sys
import tempfile
import wave

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # works under python3 -I
import golden_model  # noqa: E402

SR = 48000
HERE = os.path.dirname(os.path.abspath(__file__))
IN_WAV = os.path.join(HERE, "audio_in.wav")
IN_HEX = os.path.join(HERE, "audio_in.hex")
OUT_DIR = os.path.join(HERE, "audio_out")
OUT_DIR_GL = os.path.join(HERE, "audio_out_gl")  # netlist outputs; audio_out/ = RTL parity baseline
GL_VVP = os.path.join(HERE, "sim_audio_gl.vvp")
EXCERPT_HEX = os.path.join(HERE, "audio_excerpt.hex")
MODEL_DIR = os.path.join(HERE, "audio_model")
VL_EXE = os.path.join(HERE, "obj_dir", "sim_audio_vl")

# 6 s window of the riff (2452800 = 51.1 s in), picked by script: first
# 4800-frame-aligned start whose first second holds a frame with |x| >= 24000
# (measured peak 30570) and which holds a stretch where a 2400-frame moving
# max of |x| stays < 600 for >= 12000 frames (measured 16558 frames, 0.345 s).
EXCERPT_START = 2452800
EXCERPT_FRAMES = 288000

# sha256 of little-endian int16 bytes of
# golden_model.render(excerpt, en=0x02, tgb=True)[1024:], computed at 2063810
# behavior. Fuzz-sound regression: any change to the fuzz voicing breaks it.
FUZZ_BASELINE_SHA256 = "daadb2a3c1a6f891dd9cb47bd24bdfe0381babf752e514d2ed75e4484f3fe94c"

# name, en (ui[6:1], bit order = chain order en[0]=comp..en[5]=delay;
# +EN range 0..63), oct (ui[7] fuzz octave switch; +OCT 0/1). The switches
# are debounced in the DUT: golden_model.EN_SETTLE frames render as off.
MODES = [
    ("bypass",   0x0,  0),  # all stages off — digital clean identity
    ("comp",     0x01, 0),  # comp: smoothed env, 1.5 dB gain table, slewed gain
    ("fuzz",     0x02, 0),  # asym clip, env-biased negative point
    ("fuzzoct",  0x02, 1),  # fuzz + octave blend
    ("dist",     0x04, 0),  # diode-knee PWL, fixed drive x4
    ("trem",     0x10, 0),  # smooth trem, K1 absent (inc 437, ~5 Hz)
    ("fuzzdist", 0x06, 0),  # fuzz clip, then diode knee
    ("all4",     0x17, 0),  # comp+fuzz+dist+trem (phaser/delay excluded — Rev B stubs)
]
# check modes: name, en, oct, k1 (knob K1 value 0..255 for the tb's RC pin
# model, None = absent). Every listening mode runs K1-absent; the knob
# modes pin the trem rate path (inc = (64 + k1[5:0]) << k1[7:6]).
CHECK_MODES = [(name, en, oct_sw, None) for name, en, oct_sw in MODES] + [
    ("trem_k1lo",  0x10, 0, 8),    # inc 72  (~0.82 Hz)
    ("trem_k1mid", 0x10, 0, 128),  # inc 256 (~2.93 Hz)
    ("trem_k1hi",  0x10, 0, 250),  # inc 976 (~11.2 Hz)
]
K1_RE = re.compile(r"^K1:(\d+) (\d+) (\d+)$")


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


SAMPLE_RE = re.compile(r"^S:([0-9a-fA-F]{4})$")


def parse_samples(log_path):
    """Signed samples from a sim log: only the tb's prefixed "S:xxxx"
    lines count (other $display output and sim chatter are ignored)."""
    vals = []
    with open(log_path) as f:
        for line in f:
            m = SAMPLE_RE.match(line.strip())
            if m:
                v = int(m.group(1), 16)
                vals.append(v - 0x10000 if v & 0x8000 else v)
    return vals


def parse_k1_probe(log_path):
    """(value, present, moved) from the tb's "K1:" settle-probe line."""
    with open(log_path) as f:
        for line in f:
            m = K1_RE.match(line.strip())
            if m:
                return tuple(int(g) for g in m.groups())
    return None


def hex2wav(name, out_dir=OUT_DIR):
    # sample lines are $display'ed to stdout, captured in the .log file
    samples = array.array("h", parse_samples(os.path.join(out_dir, f"{name}.log")))
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


def verilator_cmd(exe, mdir, top, flags, sources):
    """Shared Verilator --binary command (RTL and gate-level builds)."""
    return (["verilator", "--binary", "--timing", "-j", "16", "-o", exe,
             "--Mdir", mdir, "--top-module", top] + flags + sources)


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
    return verilator_cmd(
        "sim_audio_glv", os.path.join(HERE, "obj_dir_gl"), "tb_audio_gl",
        ["-Wno-fatal", "-DGL_TEST", "-DFUNCTIONAL", "-DUSE_POWER_PINS",
         "-DSIM", "-DUNIT_DELAY=#1"],
        [os.path.join(HERE, "tb_audio_gl.v"),
         os.path.join(HERE, "glvl_stub.v"),
         GLVL_LIB,
         os.path.join(HERE, "gate_level_netlist.v")])


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


def read_wav(path):
    with wave.open(path, "rb") as w:
        a = array.array("h")
        a.frombytes(w.readframes(w.getnframes()))
    return list(a)


def vl_build():
    src = os.path.join(HERE, "..", "src")
    subprocess.check_call(
        verilator_cmd("sim_audio_vl", os.path.join(HERE, "obj_dir"), "tb_audio",
                      ["-Wno-WIDTHEXPAND", "-Wno-WIDTHTRUNC"],
                      [os.path.join(HERE, "tb_audio.v")]
                      + sorted(os.path.join(src, f) for f in os.listdir(src)
                               if f.endswith(".v"))),
        cwd=HERE, stdout=subprocess.DEVNULL)


def check():
    """Fast gate: Verilator sim vs golden model on the excerpt, bit-exact."""
    full = read_wav(IN_WAV)
    ex = full[EXCERPT_START:EXCERPT_START + EXCERPT_FRAMES]
    assert len(ex) == EXCERPT_FRAMES, "excerpt runs past end of audio_in.wav"
    vl_build()
    with open(EXCERPT_HEX, "w") as f:
        f.write("\n".join("%04x" % (s & 0xFFFF) for s in ex))
    with tempfile.TemporaryDirectory() as tmp:
        procs = {}
        for name, en, oct_sw, k1 in CHECK_MODES:
            with open(os.path.join(tmp, f"{name}.log"), "w") as log:
                procs[name] = subprocess.Popen(
                    [VL_EXE, f"+EN={en}", f"+OCT={oct_sw}", f"+N={EXCERPT_FRAMES}",
                     f"+IN={EXCERPT_HEX}", f"+K1={-1 if k1 is None else k1}"],
                    stdout=log, stderr=subprocess.STDOUT, cwd=HERE)
        rcs = {name: p.wait() for name, p in procs.items()}
        bad = False
        for name, en, oct_sw, k1 in CHECK_MODES:
            if rcs[name] != 0:
                bad = True
                print(f"{name}: FAIL (sim exited with return code {rcs[name]})")
                continue
            log_path = os.path.join(tmp, f"{name}.log")
            # the model applies knob_steady(k1) from frame 0: exact only if
            # the RTL K1 reading is settled and still from EN_SETTLE - 1 on
            probe = parse_k1_probe(log_path)
            want_probe = (0 if k1 is None else golden_model.knob_steady(k1),
                          0 if k1 is None else 1, 0)
            if probe != want_probe:
                bad = True
                print(f"{name}: FAIL (K1 probe value/present/moved {probe} != {want_probe})")
                continue
            want = golden_model.render(ex, en, oct=oct_sw, k1=k1)
            got = parse_samples(log_path)
            first = next((i for i in range(min(len(got), len(want))) if got[i] != want[i]), None)
            if first is None and len(got) != len(want):
                first = min(len(got), len(want))
            if first is None:
                print(f"{name}: PASS")
            else:
                bad = True
                g = got[first] if first < len(got) else None
                print(f"{name}: FAIL (first mismatch frame {first}: got {g}/want {want[first] if first < len(want) else None})")
    # golden-model selftest (cenv release terminates at exactly 0 — too
    # slow to show in cocotb)
    st = subprocess.run([sys.executable, "-I", os.path.join(HERE, "golden_model.py"),
                         "--selftest"], capture_output=True, text=True)
    if st.returncode == 0:
        print(st.stdout.strip())
    else:
        bad = True
        print(f"selftest: FAIL (exit {st.returncode}) {st.stdout.strip()} {st.stderr.strip()}")
    fz = golden_model.render(ex, en=0x02, tgb=True)[1024:]
    digest = hashlib.sha256(struct.pack(f"<{len(fz)}h", *fz)).hexdigest()
    if digest == FUZZ_BASELINE_SHA256:
        print("fuzz-regression: PASS")
    else:
        bad = True
        print(f"fuzz-regression: FAIL (sha256 {digest} != baseline {FUZZ_BASELINE_SHA256})")
    if bad:
        raise SystemExit(1)


def model():
    """Full-riff golden renders to audio_model/<mode>.wav (for listening)."""
    os.makedirs(MODEL_DIR, exist_ok=True)
    full = read_wav(IN_WAV)
    for name, en, oct_sw in MODES:
        out = golden_model.render(full, en, oct=oct_sw)
        path = os.path.join(MODEL_DIR, f"{name}.wav")
        with wave.open(path, "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(SR)
            w.writeframes(array.array("h", out).tobytes())
        print(f"{path}: {len(out)} samples")


def run_mode(name, en, oct_sw, n, sim):
    out_dir = OUT_DIR_GL if sim.startswith("gl") else OUT_DIR
    args = [f"+EN={en}", f"+OCT={oct_sw}", f"+N={n}"]
    if sim == "gl":
        cmd = ["vvp", GL_VVP] + args
    elif sim == "glvl":
        cmd = [os.path.join(HERE, "obj_dir_gl", "sim_audio_glv")] + args
    elif sim == "verilator":
        cmd = [VL_EXE] + args
    else:
        cmd = ["vvp", "sim_audio.vvp"] + args
    with open(os.path.join(out_dir, f"{name}.log"), "w") as log:
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
    elif what == "check":
        check()
    elif what == "model":
        model()
    elif what == "run":
        out_dir = OUT_DIR_GL if sim.startswith("gl") else OUT_DIR
        os.makedirs(out_dir, exist_ok=True)
        if sim == "gl":
            gl_build()
        elif sim == "glvl":
            glvl_build()
        n = wav2hex()
        procs = [run_mode(name, en, oct_sw, n, sim) for name, en, oct_sw in MODES]
        for p in procs:
            p.wait()
        for name, _, _ in MODES:
            path, cnt = hex2wav(name, out_dir)
            print(f"{path}: {cnt} samples")
    else:
        raise SystemExit(__doc__)


if __name__ == "__main__":
    main()
