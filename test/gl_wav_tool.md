# Gate-level WAV parity runbook (2x2 design)

Once the GDS CI run is green, prove the netlist produces the same audio as
the golden model: run the `wav_tool.py check` excerpt against the real
`gate_level_netlist.v` and require **bit-exact** output for every mode.
Sampling instants are input-edge-determined, so no tolerance is expected.

> **WARNING: the `test/gate_level_netlist.v` currently in the tree is a STALE
> v2.2 1x1 netlist (it still contains the old `crush_hold` net). It does NOT
> match this design and MUST be replaced before any gate-level run.**
> `wav_tool.py glvl-build` (and therefore `SIM=glvl check`/`run`) refuses it
> with a "STALE NETLIST" error. The file is untracked; replace it, do not
> commit it.

## Getting the netlist

1. Take the green GDS workflow run on the 2x2 design and download its
   artifact (named `tt_submission` or similar).
2. Inside it, take the gate-level Verilog `<top>.v`
   (`tt_um_tmcdonagh_guitar_pedal.v`; a local harden has it under
   `runs/*/results/final/verilog/gl/`).
3. Copy it to `test/gate_level_netlist.v` (the name the Makefile's
   `GATES=yes` path also uses).

## Prerequisites

- The fresh `test/gate_level_netlist.v` above.
- `PDK_ROOT` pointing at a sky130A PDK install (e.g. `~/.volare-pdk`; the
  same PDK the GDS/gl_test CI uses).
- Verilator (the default GL path); iverilog only for the slow `SIM=gl`
  fallback.

## Parity check (the gate)

```sh
cd test
PDK_ROOT=~/.volare-pdk SIM=glvl python3 wav_tool.py check
```

This builds the Verilator gate-level binary (`tb_audio_gl.v` + `glvl_stub.v`
+ a transformed sky130 cell library + the netlist), runs the ~6 s excerpt
through every `CHECK_MODES` entry (15 modes), and compares each bit-exactly
against `golden_model.py`, plus return codes, lengths, the fuzz sha and the
golden-model selftest. Logs go to `audio_out_gl/`. The K1/K2/K3 settle
probes are skipped: the GL tb does not print them, and the plain RTL
`python3 -I wav_tool.py check` already proves them (the netlist is meant to
be equivalent).

## Optional: full riff

```sh
PDK_ROOT=~/.volare-pdk SIM=glvl python3 wav_tool.py run          # writes audio_out_gl/{mode}.{log,wav}
python3 wav_tool.py model                                         # golden renders -> audio_model/
PDK_ROOT=~/.volare-pdk SIM=glvl python3 wav_tool.py model-parity  # GL vs golden, bit-exact
```

(`python3 wav_tool.py gl-parity` instead compares `audio_out_gl/` against the
RTL `audio_out/` from a plain `python3 wav_tool.py run`.)

## Testbench interface

`tb_audio_gl.v` plusargs: `+EN` (ui[6:1] enables, 0..63), `+OCT` (ui[7] fuzz
octave, 0/1), `+N` (frames), `+IN` (input hex, default `audio_in.hex`),
`+K1`, `+K2`, `+K3` (knob raw value 0..255 through the RC pin model, -1 =
absent). Manual run of one mode:

```sh
obj_dir_gl/sim_audio_glv +EN=2 +OCT=1 +N=288000 +IN=audio_excerpt.hex \
  +K1=-1 +K2=-1 +K3=-1 > audio_out_gl/fuzzoct.log
```

Print the iverilog (`SIM=gl`, ~7 h/mode on a 1x1 netlist, fallback only) or
Verilator commands without running: `python3 wav_tool.py gl-cmd` /
`glvl-cmd`.

**On any mismatch: systematic debugging before any "close enough"
conclusion.** The gate is bit-exact. Likely sources, in order: stale
netlist vs current `src/`, X-propagation from a floating input in the
harness, or a real synthesis bug (that is the point of the gate).

## Known risks / notes

- Output capture is port-based: `tb_audio_gl.v` reassembles the DAC sample
  from `uio_out[1]` (slots 1-16, mid-slot); the RTL's internal
  `effect_sample` alias does not survive netlist flattening. The tb handles
  the one-frame DAC lag (flush frame + print alignment), so GL logs stay
  index-aligned with the RTL logs.
- The netlist is flat; only top-level names are probeable.
- `UNIT_DELAY=#1` gives every cell a unit delay; if deep paths misbehave
  (X / parity FAIL only under GL), check path depth vs the 40.5 ns clock.
- Verilator needs behavioral models for the primitives.v UDPs
  (`glvl_stub.v`) and a transformed cell library copy (`glvl_make_lib()`
  strips `#1` instance delays and rewrites skipped NOTIFIER args).
- GL outputs live in `audio_out_gl/`, so the RTL evidence in `audio_out/` is
  never overwritten.
- **CI note:** the cocotb gate-level job (`gl_test`) may take 0.5-2 h on the
  2x2 netlist. If it nears the job time limit, the largest tests are
  `test_chain_enables`, `test_gate_hysteresis`, `test_clip_dp`,
  `test_fuzz_asym`, `test_trem_smooth` and `test_env_decay`.
