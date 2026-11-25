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
                                     # (SIM=glvl: same, on the gate-level netlist)
    python3 wav_tool.py model        # full-riff golden renders -> audio_model/
    python3 wav_tool.py model-parity # bit-exact audio_out/ (run) vs audio_model/ (model)
    python3 wav_tool.py render <label>  # RTL full riff, every mode -> ../renders/YYYY-MM-DD-<label>/
                                     # (<mode>.wav + README.md; refuses an existing folder)
Modes = MODES (= CHECK_MODES: listening modes plus the K1/K2/K3 knob modes).
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
LISTEN_MODES = [
    ("bypass",   0x0,  0),  # all stages off — digital clean identity
    ("comp",     0x01, 0),  # comp: smoothed env, 1.5 dB gain table, slewed gain
    ("fuzz",     0x02, 0),  # asym clip, env-biased negative point
    ("fuzzoct",  0x02, 1),  # fuzz + octave blend
    ("dist",     0x04, 0),  # diode-knee PWL, fixed drive x4
    ("trem",     0x10, 0),  # smooth trem, K1 absent (inc 437, ~5 Hz)
    ("fuzzdist", 0x06, 0),  # fuzz clip, then diode knee
    ("all4",     0x17, 0),  # comp+fuzz+dist+trem (the Rev A stack, unchanged)
    ("all5",     0x1F, 0),  # comp+fuzz+dist+phaser+trem (Rev B1)
]
# check modes: name, en, oct, k1, k2, k3, k4 (knob K1..K4 values 0..255 for the
# tb's RC pin model, None = absent). Every listening mode runs both
# knobs absent; the K1 modes pin the trem/phaser rate path (inc = (64 +
# k1[5:0]) << k1[7:6]), the K2 modes the tone shift (kt = 4 - k2[7:6]:
# k2=0 -> kt 4 dark, k2=255 -> kt 1 bright).
CHECK_MODES = [(name, en, oct_sw, None, None, None, None) for name, en, oct_sw in LISTEN_MODES] + [
    ("trem_k1lo",  0x10, 0, 8, None, None, None),    # inc 72  (~0.82 Hz)
    ("trem_k1mid", 0x10, 0, 128, None, None, None),  # inc 256 (~2.93 Hz)
    ("trem_k1hi",  0x10, 0, 250, None, None, None),  # inc 976 (~11.2 Hz)
    ("fuzz_k2dark",   0x02, 0, None, 0, None, None),    # fuzz + tone kt 4 (darkest)
    ("fuzz_k2bright", 0x02, 0, None, 255, None, None),  # fuzz + tone kt 1 (brightest)
    ("dist_k3lo",  0x04, 0, None, None, 0, None),    # dist drive x1 (256)
    ("dist_k3hi",  0x04, 0, None, None, 255, None),  # dist drive x7.97 (2041)
    ("phaser",      0x08, 0, None, None, None, None),  # 4-stage allpass, K1 absent (inc 87, ~1.0 Hz)
    ("phaser_k1lo", 0x08, 0, 8, None, None, None),     # inc 72  (~0.82 Hz)
    ("phaser_k1hi", 0x08, 0, 250, None, None, None),   # inc 976 (~11.2 Hz)
    ("phasertrem",  0x18, 0, None, None, None, None),  # phaser into trem
    ("fuzzphaser",  0x0A, 0, None, None, None, None),  # fuzz (+ tone/gate) into the phaser
]
MODES = CHECK_MODES   # `run`/`model`/parity cover every mode, knob modes included
K1_RE = re.compile(r"^K1:(\d+) (\d+) (\d+)$")
K2_RE = re.compile(r"^K2:(\d+) (\d+) (\d+)$")
K3_RE = re.compile(r"^K3:(\d+) (\d+) (\d+)$")
K4_RE = re.compile(r"^K4:(\d+) (\d+) (\d+)$")


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


def parse_k1_probe(log_path, regex=K1_RE):
    """(value, present, moved) from the tb's "K1:" (K2:/K3: via regex)
    settle-probe line."""
    with open(log_path) as f:
        for line in f:
            m = regex.match(line.strip())
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
    with open(netlist, errors="replace") as f:
        if "crush_hold" in f.read():
            raise SystemExit(
                f"STALE NETLIST: {netlist} contains the old v2.2 `crush_hold` net — it is a "
                "1x1 netlist of a previous design, not this 2x2 one. Replace it with the "
                "`<top>.v` from the GDS workflow's tt_submission artifact (gl_wav_tool.md).")
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
    for name, *_ in MODES:
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


def probes_ok(log_path, k1, k2, k3, k4, name):
    """RTL-only K1..K4 settle probes (the model applies knob_steady(k)
    from frame 0: exact only if the RTL reading is settled and unmoved)."""
    for tag, regex, k in (("K1", K1_RE, k1), ("K2", K2_RE, k2), ("K3", K3_RE, k3),
                           ("K4", K4_RE, k4)):
        probe = parse_k1_probe(log_path, regex)
        want = (0 if k is None else golden_model.knob_steady(k),
                0 if k is None else 1, 0)
        if probe != want:
            print(f"{name}: FAIL ({tag} probe value/present/moved {probe} != {want})")
            return False
    return True


def check():
    """Fast gate: Verilator sim vs golden model on the excerpt, bit-exact."""
    gl = os.environ.get("SIM", "verilator") == "glvl"
    full = read_wav(IN_WAV)
    ex = full[EXCERPT_START:EXCERPT_START + EXCERPT_FRAMES]
    assert len(ex) == EXCERPT_FRAMES, "excerpt runs past end of audio_in.wav"
    if gl:
        glvl_build()   # also refuses a stale netlist
        exe = os.path.join(HERE, "obj_dir_gl", "sim_audio_glv")
        os.makedirs(OUT_DIR_GL, exist_ok=True)
    else:
        vl_build()
        exe = VL_EXE
    with open(EXCERPT_HEX, "w") as f:
        f.write("\n".join("%04x" % (s & 0xFFFF) for s in ex))
    with tempfile.TemporaryDirectory() as tmp:
        if gl:
            tmp = OUT_DIR_GL   # netlist logs land in audio_out_gl/<mode>.log
        procs = {}
        for name, en, oct_sw, k1, k2, k3, k4 in CHECK_MODES:
            with open(os.path.join(tmp, f"{name}.log"), "w") as log:
                procs[name] = subprocess.Popen(
                    [exe, f"+EN={en}", f"+OCT={oct_sw}", f"+N={EXCERPT_FRAMES}",
                     f"+IN={EXCERPT_HEX}", f"+K1={-1 if k1 is None else k1}",
                     f"+K2={-1 if k2 is None else k2}",
                     f"+K3={-1 if k3 is None else k3}",
                     f"+K4={-1 if k4 is None else k4}"],
                    stdout=log, stderr=subprocess.STDOUT, cwd=HERE)
        rcs = {name: p.wait() for name, p in procs.items()}
        bad = False
        for name, en, oct_sw, k1, k2, k3, k4 in CHECK_MODES:
            if rcs[name] != 0:
                bad = True
                print(f"{name}: FAIL (sim exited with return code {rcs[name]})")
                continue
            log_path = os.path.join(tmp, f"{name}.log")
            # The K1/K2/K3 settle probes are RTL-only: tb_audio_gl.v does not
            # print them. The RTL `check` proves them and the netlist is
            # meant to be equivalent, so GL mode makes only the bit-exact
            # sample comparison below.
            if not gl and not probes_ok(log_path, k1, k2, k3, k4, name):
                bad = True
                continue
            want = golden_model.render(ex, en, oct=oct_sw, k1=k1, k2=k2, k3=k3, k4=k4)
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
    for name, en, oct_sw, k1, k2, k3, k4 in MODES:
        out = golden_model.render(full, en, oct=oct_sw, k1=k1, k2=k2, k3=k3, k4=k4)
        path = os.path.join(MODEL_DIR, f"{name}.wav")
        with wave.open(path, "wb") as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(SR)
            w.writeframes(array.array("h", out).tobytes())
        print(f"{path}: {len(out)} samples")


def model_parity():
    """Bit-exact check: every full-riff sim WAV in audio_out/ (SIM=...
    outputs: audio_out_gl/ with SIM=gl*) vs the golden render in audio_model/
    (`model` first)."""
    out_dir = OUT_DIR_GL if os.environ.get("SIM", "verilator").startswith("gl") else OUT_DIR
    bad = []
    for name, *_ in MODES:
        a = read_wav(os.path.join(out_dir, f"{name}.wav"))
        b = read_wav(os.path.join(MODEL_DIR, f"{name}.wav"))
        first = next((i for i, (x, y) in enumerate(zip(a, b)) if x != y), None)
        ok = first is None and len(a) == len(b)
        print(f"{name}: {'PASS' if ok else 'FAIL'} ({len(a)} vs {len(b)} samples"
              + ("" if ok else f", first diff at {first}") + ")")
        if not ok:
            bad.append(name)
    if bad:
        raise SystemExit(f"model-parity FAIL: {bad}")


def run_mode(name, en, oct_sw, k1, k2, k3, k4, n, sim):
    out_dir = OUT_DIR_GL if sim.startswith("gl") else OUT_DIR
    args = [f"+EN={en}", f"+OCT={oct_sw}", f"+N={n}",
            f"+K1={-1 if k1 is None else k1}", f"+K2={-1 if k2 is None else k2}",
            f"+K3={-1 if k3 is None else k3}",
            f"+K4={-1 if k4 is None else k4}"]
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


RENDER_JOBS = 16   # max parallel simulator processes for `render`


def render(label):
    """Dated full-riff RTL renders: ../renders/YYYY-MM-DD-<label>/<mode>.wav
    for every MODES entry, plus a README.md (modes, base commit, git diff
    --stat, command). Refuses to overwrite an existing folder."""
    import datetime
    import shutil
    if not re.fullmatch(r"[A-Za-z0-9._-]+", label):
        raise SystemExit("render: label must match [A-Za-z0-9._-]+")
    repo = os.path.abspath(os.path.join(HERE, ".."))
    today = datetime.date.today().isoformat()   # once: the folder and README must agree
    out = os.path.join(repo, "renders", f"{today}-{label}")
    if os.path.exists(out):
        raise SystemExit(f"render: {out} already exists; refusing to overwrite")

    def git(*a):
        r = subprocess.run(["git", "-C", repo] + list(a), capture_output=True, text=True)
        return r.stdout.strip() if r.returncode == 0 else f"(git {a[0]} failed)"

    vl_build()
    n = wav2hex()
    os.makedirs(out)
    logs = tempfile.mkdtemp(prefix="render_")
    try:
        pending, running, failed = list(MODES), [], []
        while pending or running:
            while pending and len(running) < RENDER_JOBS:
                name, en, oct_sw, *knobs = pending.pop(0)
                args = [f"+EN={en}", f"+OCT={oct_sw}", f"+N={n}"] + [
                    f"+K{i + 1}={-1 if k is None else k}" for i, k in enumerate(knobs)]
                lf = open(os.path.join(logs, f"{name}.log"), "w")
                running.append((name, lf, subprocess.Popen(
                    [VL_EXE] + args, stdout=lf, stderr=subprocess.STDOUT, cwd=HERE)))
            name, lf, p = running.pop(0)
            rc = p.wait()
            lf.close()
            if rc != 0:
                failed.append((name, rc))
        if failed:
            raise SystemExit("render FAIL: simulator exited non-zero: "
                             + ", ".join(f"{m} (rc {rc})" for m, rc in failed))
        rows = []
        for name, en, oct_sw, *knobs in MODES:
            samples = array.array("h", parse_samples(os.path.join(logs, f"{name}.log")))
            if len(samples) != n:
                raise SystemExit(f"render FAIL: {name}: {len(samples)} samples, expected {n}")
            with wave.open(os.path.join(out, f"{name}.wav"), "wb") as w:
                w.setnchannels(1)
                w.setsampwidth(2)
                w.setframerate(SR)
                w.writeframes(samples.tobytes())
            kn = ", ".join(f"K{i + 1}=" + ("absent" if k is None else str(k))
                           for i, k in enumerate(knobs)) or "-"
            rows.append(f"| {name} | 0x{en:02X} | {oct_sw} | {kn} |")
            print(f"{os.path.join(out, name + '.wav')}: {len(samples)} samples")
    except BaseException:
        shutil.rmtree(out, ignore_errors=True)   # never leave a partial dated folder
        raise
    finally:
        shutil.rmtree(logs, ignore_errors=True)
    cmd = "python3 -I wav_tool.py render " + label
    with open(os.path.join(out, "README.md"), "w") as f:
        f.write(f"# Render {os.path.basename(out)}\n\n"
                f"RTL (Verilator) full riff, {n} samples ({n / SR:.1f} s) per mode.\n\n"
                f"- Base commit: `{git('rev-parse', 'HEAD')}`\n"
                f"- Command (from `test/`): `{cmd}`\n"
                f"- Date: {today}\n\n"
                "## Modes\n\n| name | en (ui[6:1]) | oct (ui[7]) | knobs |\n|---|---|---|---|\n"
                + "\n".join(rows)
                + "\n\n## git diff --stat HEAD (working tree vs base commit)\n\n```\n"
                + git("diff", "--stat", "HEAD") + "\n```\n")
    print(f"render: {len(MODES)} modes -> {out}")


def main():
    what = sys.argv[1] if len(sys.argv) > 1 else "run"
    sim = os.environ.get("SIM", "verilator")
    os.makedirs(OUT_DIR, exist_ok=True)
    if what == "wav2hex":
        print(wav2hex(), "samples")
    elif what == "hex2wav":
        for name, *_ in MODES:
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
    elif what == "model-parity":
        model_parity()
    elif what == "render":
        if len(sys.argv) != 3:
            raise SystemExit("usage: wav_tool.py render <label>")
        render(sys.argv[2])
    elif what == "run":
        out_dir = OUT_DIR_GL if sim.startswith("gl") else OUT_DIR
        os.makedirs(out_dir, exist_ok=True)
        if sim == "gl":
            gl_build()
        elif sim == "glvl":
            glvl_build()
        n = wav2hex()
        procs = [run_mode(name, en, oct_sw, k1, k2, k3, k4, n, sim)
                 for name, en, oct_sw, k1, k2, k3, k4 in MODES]
        rcs = [p.wait() for p in procs]
        failed = [(m[0], rc) for m, rc in zip(MODES, rcs) if rc != 0]
        if failed:
            raise SystemExit("run FAIL: simulator exited non-zero: "
                             + ", ".join(f"{n} (rc {rc})" for n, rc in failed))
        for name, *_ in MODES:
            path, cnt = hex2wav(name, out_dir)
            print(f"{path}: {cnt} samples")
    else:
        raise SystemExit(__doc__)


if __name__ == "__main__":
    main()
