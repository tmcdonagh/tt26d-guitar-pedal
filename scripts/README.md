# scripts/

- `harden_local.sh <label> [--force] [--no-glvl] [--gl-test] [--tiles WxH]` -- local
  LibreLane harden + precheck + gate-level check of the working tree, no
  GitHub Actions. Copies the tree to `~/tools/harden/<label>/` and writes
  `SUMMARY.md` there (utilization, slack per corner, precheck PASS/FAIL,
  cell count, gate-level result). Exit status is non-zero if any step failed.
  About 26 min end to end (14 harden, 1 precheck, 10 glvl check).
- Dated full-riff listening renders: `cd test && python3 -I wav_tool.py render <label>`
  -> `renders/YYYY-MM-DD-<label>/` (git-excluded).

Setup (tools, versions, Docker image, PDK hashes), how to read `SUMMARY.md`
and the gf180 (Wafer.Space) deltas are in the archive doc
`ai-dev-archive/projects/tinytapeout-sky130/specs/2x2-pedalboard/local-gds-flow.md`.

Flags: `--force` (replace an existing run dir), `--no-glvl` (skip the glvl
check), `--gl-test` (also run `make -B GATES=yes`; needs `TESTS=N PASS=N FAIL=0`
in its log).
`--tiles WxH` hardens at another tile size (for example `--tiles 3x2` for the
Rev B2 option, which does not fit 2x2): it rewrites `tiles:` in the COPY's
`info.yaml` before `--create-user-config` (the repo's is never touched) and
records the value in `SUMMARY.md` (`- Tiles:`).
