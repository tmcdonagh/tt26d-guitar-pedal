# Gate-level WAV parity runbook (v2.2 task 7)

Once the GDS CI run is green, prove the netlist produces the same audio as
the RTL: run every `wav_tool.py` mode against the real `gate_level_netlist.v`
and require **bit-exact** outputs vs the RTL `.wav` files in `audio_out/`
(plan Global Constraints, netlist parity gate). Sampling instants are
input-edge-determined, so no tolerance is expected.

## Prerequisites

- `gate_level_netlist.v` in `test/` — download the `tt_submission` artifact
  from the green GDS workflow run (or copy from
  `runs/*/results/final/verilog/gl/` of a local harden) and `cp` it to
  `test/gate_level_netlist.v`, the same name the Makefile's `GATES=yes` path
  uses.
- `PDK_ROOT` exported, pointing at a sky130A PDK install (the same one the
  GDS/gl_test CI uses; locally e.g. via volare/ciel).
- iverilog + the RTL baseline: first run `python3 wav_tool.py run` (verilator,
  default SIM) so `audio_out/*.wav` is fresh RTL evidence.

## Modes (SIM env var)

- `SIM=glvl` — **Verilator GL: the default for parity work** (minutes per
  full 10-mode pass). Compiles the same FUNCTIONAL cell models to C++.
  Requires two build-time pieces created for this (see `glvl_stub.v` and
  `wav_tool.py glvl_make_lib()`): behavioral models for the primitives.v
  UDPs (Verilator cannot compile UDPs; all 17 transcribed from their
  tables, power-guarded per the `_pp$PG$N` semantics), and a transformed
  library copy (strips `#1` instance delays, which Verilator parses as
  parameter overrides; rewrites skipped NOTIFIER positional args as `1'bx`).
- `SIM=gl` — iverilog GL: same defines/sources as the Makefile `GATES=yes`
  path, correct but slow (~7 h/mode on a 1×1 netlist). Fallback only.

## 1. The compile command

`test/tb_audio_gl.v` is `tb_audio.v` with the GL power-pin instantiation from
`test/tb.v`. The compile mirrors the Makefile's `GATES=yes` define set and the
sky130 cell source list **exactly** (same order):

```sh
export PDK_ROOT=/path/to/pdk   # contains sky130A/libs.ref/sky130_fd_sc_hd/...
cd test
iverilog -g2012 -s tb_audio_gl -DGL_TEST -DFUNCTIONAL -DUSE_POWER_PINS -DSIM \
  -D'UNIT_DELAY=#1' -o sim_audio_gl.vvp tb_audio_gl.v \
  $PDK_ROOT/sky130A/libs.ref/sky130_fd_sc_hd/verilog/primitives.v \
  $PDK_ROOT/sky130A/libs.ref/sky130_fd_sc_hd/verilog/sky130_fd_sc_hd.v \
  gate_level_netlist.v
```

Print it without running anything (dry run): `python3 wav_tool.py gl-cmd`.

## 2. Run all 10 modes

Everything below is wrapped in `wav_tool.py`:

```sh
cd test
export PDK_ROOT=/path/to/pdk
SIM=gl python3 wav_tool.py run       # builds sim_audio_gl.vvp, runs all modes,
                                     # writes audio_out_gl/{mode}.{log,wav}
```

Modes run in parallel (10 `vvp` processes), same as the RTL pass. Manual
equivalent for one mode:

```sh
vvp sim_audio_gl.vvp +EN=2 +PARAMS=1 +N=2818047 +OUT=audio_out_gl/distX.hex \
  > audio_out_gl/distX.log
```

(see `wav_tool.py` `MODES` for every mode's `en`/`params` pair).

## 3. The parity check — bit-exact, no tolerance

```sh
python3 wav_tool.py gl-parity
```

compares each `audio_out_gl/{mode}.wav` payload byte-for-byte against
`audio_out/{mode}.wav` (header and frame count must match too) and prints
PASS/FAIL per mode. Manual equivalent (wave-payload diff):

```sh
python3 - <<'EOF'
import wave
for m in "bypass fuzz dist distX crush crushQ trem tremS fuzzdist all4".split():
    a = wave.open(f"audio_out/{m}.wav"); b = wave.open(f"audio_out_gl/{m}.wav")
    print(m, a.readframes(a.getnframes()) == b.readframes(b.getnframes()))
EOF
```

**On any mismatch: systematic debugging before any "close enough" conclusion.**
The gate is bit-exact; "audibly identical" or "a few LSBs off" is a FAIL.
Likely mismatch sources, in order: stale RTL baseline (re-run step 0), stale
netlist vs current `src/`, X-propagation from a floating input in the harness,
or a real synthesis bug (that's the point of the gate).

## Known risks / notes

- Output capture is port-based: `tb_audio_gl.v` reassembles the DAC sample
  from `uio_out[1]` (slots 1–16, mid-slot) — the RTL's internal
  `effect_sample` alias does NOT survive netlist flattening (proven by the
  first GL compile; opt_clean collapses it). The tb handles the one-frame
  DAC lag internally (flush frame + print alignment), so GL logs stay
  index-aligned with the RTL logs and `gl-parity` compares 1:1.
- The netlist is flat (sub-module internals exist only as escaped net
  names); only top-level names are probeable.
- `UNIT_DELAY=#1` gives every cell a unit delay — if deep paths ever
  misbehave (X / parity FAIL only under GL), check path depth vs the 40.5 ns
  clock period; the CI `gl_test` job shares this risk and is the cross-check.
- GL outputs live in `audio_out_gl/` so the RTL evidence in `audio_out/` is
  never overwritten.
