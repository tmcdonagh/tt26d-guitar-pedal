#!/usr/bin/env bash
# SPDX-FileCopyrightText: © 2026 Thomas McDonagh
# SPDX-License-Identifier: Apache-2.0
#
# Local Tiny Tapeout hardening: copy the working tree, harden with LibreLane
# (dockerized, via tt-support-tools), run precheck, optionally gate-level
# checks, and write SUMMARY.md. No GitHub Actions needed.
# Full write-up: ai-dev-archive .../specs/2x2-pedalboard/local-gds-flow.md
#
# Usage: scripts/harden_local.sh <label> [--force] [--no-glvl] [--gl-test]
#   <label>     run dir is ~/tools/harden/<label>/ (must not exist, see --force)
#   --force     delete an existing ~/tools/harden/<label>/ first
#   --no-glvl   skip the fast Verilator gate-level check (~10 min)
#   --gl-test   also run the slow iverilog cocotb gate-level test (>30 min)
# Env overrides: TOOLS (~/tools), HARDEN_PDK / PRECHECK_PDK (ciel hashes).
set -euo pipefail

TOOLS="${TOOLS:-$HOME/tools}"
HARDEN_PDK="${HARDEN_PDK:-8afc8346a57fe1ab7934ba5a6056ea8b43078e71}"    # ciel, auto-fetched by tt_tool --harden
PRECHECK_PDK="${PRECHECK_PDK:-0536d02d875c8f67dd7cca3902ac457e62f20005}" # pinned by precheck/action.yml
TOP=tt_um_tmcdonagh_guitar_pedal
CVENV="${CVENV:-/home/tom/dev/.venv-tt}"   # cocotb venv for the gate-level make test
CIEL="$HOME/.ciel/ciel/sky130/versions"

label="" force=0 do_glvl=1 do_gltest=0
for a in "$@"; do
  case "$a" in
    --force) force=1 ;;
    --no-glvl) do_glvl=0 ;;
    --gl-test) do_gltest=1 ;;
    -h|--help) sed -n 2,16p "$0"; exit 0 ;;
    -*) echo "unknown option: $a" >&2; exit 2 ;;
    *) [ -z "$label" ] && label="$a" || { echo "one label only" >&2; exit 2; } ;;
  esac
done
[ -n "$label" ] || { echo "usage: $0 <label> [--force] [--no-glvl] [--gl-test]" >&2; exit 2; }
[[ "$label" =~ ^[A-Za-z0-9._-]+$ ]] || { echo "bad label: $label" >&2; exit 2; }

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="$TOOLS/harden/$label"
[ -d "$TOOLS/tt" ] && [ -f "$TOOLS/venv-ll/bin/activate" ] || { echo "missing $TOOLS/tt or $TOOLS/venv-ll" >&2; exit 2; }
if [ -e "$DEST" ]; then
  [ "$force" = 1 ] || { echo "$DEST exists; use --force to replace it" >&2; exit 2; }
  rm -rf "$DEST"
fi

PYSYS="$(command -v python3)"   # system python for wav_tool (before the venv shadows it)
START=$SECONDS
FAILS=0
LAST_OK=0
STEPS=()   # "NAME|PASS/FAIL/SKIPPED|seconds|note"

say() { printf '\n==== [%s +%ds] %s\n' "$(date +%H:%M:%S)" $((SECONDS - START)) "$*"; }

# step <name> <logfile> <cmd...>: run, log, record. Never aborts the script.
step() {
  local name="$1" log="$2"; shift 2
  local t0=$SECONDS rc=0
  LAST_OK=0
  say "$name"
  "$@" >"$log" 2>&1 || rc=$?
  local dt=$((SECONDS - t0))
  if [ "$rc" = 0 ]; then
    LAST_OK=1; STEPS+=("$name|PASS|$dt|"); say "$name: PASS (${dt}s)"
  else
    STEPS+=("$name|FAIL|$dt|exit $rc, see $(basename "$log")"); FAILS=$((FAILS + 1))
    say "$name: FAIL rc=$rc (${dt}s); tail of $log:"; tail -n 15 "$log" || true
  fi
  return 0
}
skip() { STEPS+=("$1|SKIPPED|0|$2"); say "$1: SKIPPED ($2)"; }

# gl_results_ok <log>: needs a cocotb "TESTS=N PASS=N FAIL=0" line (N equal, > 0)
gl_results_ok() {
  "$PYSYS" -I - "$1" <<'PY' || { echo "gl_results_ok: no passing TESTS=N PASS=N FAIL=0 line in $1" >&2; return 1; }
import re, sys
for m in re.finditer(r"TESTS=(\d+) PASS=(\d+) FAIL=(\d+)", open(sys.argv[1], errors="replace").read()):
    if int(m[1]) > 0 and m[1] == m[2] and m[3] == "0":
        sys.exit(0)
sys.exit(1)
PY
}

# --- 1. copy the working tree -------------------------------------------------
say "rsync $REPO -> $DEST"
mkdir -p "$DEST"
rsync -a \
  --exclude='/runs/' --exclude='/build/' --exclude='/renders/' --exclude='/tt_submission/' \
  --exclude='/test/obj_dir*' --exclude='/test/sim_build' --exclude='/test/audio_out*' \
  --exclude='/test/audio_model*' --exclude='/test/*.fst' --exclude='/test/*.fst.hier' \
  --exclude='/test/audio_in.hex' --exclude='/test/audio_excerpt.hex' --exclude='/GuitarSample1.m4a' --exclude='__pycache__' --exclude='/.superpowers/' \
  "$REPO"/ "$DEST"/
ln -sfn "$TOOLS/tt" "$DEST/tt"
cd "$DEST"
BASE_COMMIT="$(git -C "$REPO" rev-parse --short HEAD 2>/dev/null || echo unknown)"
DIRTY="$(git -C "$REPO" status --porcelain 2>/dev/null | wc -l)"

# --- 2. harden ----------------------------------------------------------------
set +u; source "$TOOLS/venv-ll/bin/activate"; set -u
harden_ok=0
step "create-user-config" harden.log ./tt/tt_tool.py --create-user-config
if [ "$LAST_OK" = 1 ]; then
  step "harden" harden_run.log ./tt/tt_tool.py --harden
  cat harden_run.log >>harden.log; rm -f harden_run.log
  harden_ok=$LAST_OK
fi
if [ "$harden_ok" = 1 ]; then
  step "print-warnings"      warnings.log ./tt/tt_tool.py --print-warnings
  step "print-stats"         stats.log    ./tt/tt_tool.py --print-stats
  step "print-cell-category" cells.log    ./tt/tt_tool.py --print-cell-category
  step "create-tt-submission" submission.log ./tt/tt_tool.py --create-tt-submission
else
  skip "post-harden steps" "harden did not succeed"
fi

# --- 3. precheck (docker-wrapped magic/klayout from the LibreLane image) -------
GDS="$DEST/tt_submission/$TOP.gds"
precheck_status="NOT RUN (harden/submission failed)"
if [ -f "$GDS" ]; then
  if [ ! -x "$TOOLS/bin-precheck/magic" ] || [ ! -d "$CIEL/$PRECHECK_PDK" ]; then
    precheck_status="NOT RUN (missing $TOOLS/bin-precheck wrappers or PDK $PRECHECK_PDK in $CIEL)"
    skip "precheck" "$precheck_status"
  else
    pc() { ( cd "$TOOLS/tt/precheck" && PDK_ROOT="$CIEL/$PRECHECK_PDK" PDK=sky130A \
        PATH="$TOOLS/bin-precheck:$PATH" python precheck.py --gds "$GDS" --tech sky130A ); }
    # the reports dir is shared between runs: clear stale results first, copy only fresh ones
    rm -f "$TOOLS/tt/precheck/reports/results.md" "$TOOLS/tt/precheck/reports/results.xml"
    step "precheck" precheck.log pc
    for r in results.md results.xml; do
      [ -f "$TOOLS/tt/precheck/reports/$r" ] && cp "$TOOLS/tt/precheck/reports/$r" "precheck_$r"
    done
    if [ "$LAST_OK" = 1 ] && grep -q "Precheck passed" precheck.log; then
      precheck_status=PASS
    else
      precheck_status="FAIL (see precheck.log)"
    fi
  fi
fi

# --- 4. gate-level netlist + checks ---------------------------------------------
if [ -f "tt_submission/$TOP.v" ]; then
  cp "tt_submission/$TOP.v" gate_level_netlist.v
  cp "tt_submission/$TOP.v" test/gate_level_netlist.v
  # audio_in.wav is git-ignored but rsync copied it from the working tree
  [ -f test/audio_in.wav ] || STEPS+=("audio_in.wav|FAIL|0|missing test/audio_in.wav")
  if [ "$do_glvl" = 1 ]; then
    glvl() { ( cd test && PDK_ROOT="$CIEL/$HARDEN_PDK" SIM=glvl "$PYSYS" -I wav_tool.py check ); }
    step "glvl-check" glvl_check.log glvl
  else
    skip "glvl-check" "--no-glvl"
  fi
  if [ "$do_gltest" = 1 ]; then
    gltest() {
      ( cd test && PDK_ROOT="$CIEL/$HARDEN_PDK" PATH="$CVENV/bin:$PATH" make -B GATES=yes ) || return 1
      gl_results_ok gl_test.log
    }
    step "gl-test" gl_test.log gltest
  else
    skip "gl-test" "not requested (--gl-test)"
  fi
else
  skip "gate-level steps" "no tt_submission/$TOP.v"
fi

# --- 5. SUMMARY.md --------------------------------------------------------------
say "writing SUMMARY.md"
STEPS_FILE="$(mktemp)"; printf '%s\n' "${STEPS[@]}" >"$STEPS_FILE"
LABEL="$label" SRC="$REPO" BASE="$BASE_COMMIT" DIRTY="$DIRTY" PCS="$precheck_status" TOTAL=$((SECONDS - START)) \
  STEPS_FILE="$STEPS_FILE" "$PYSYS" -I - <<'PY' >SUMMARY.md
import json, os, re
e = os.environ
def sec(s): return f"{int(s)//60}m{int(s)%60:02d}s"
out = [f"# Local harden summary: {e['LABEL']}", "",
       f"- Source: {e['SRC']} (base commit {e['BASE']}, {e['DIRTY']} dirty paths in the working tree)",
       f"- Total wall time: {sec(int(e['TOTAL']))}", ""]
fails = []
out += ["## Steps", "", "| step | result | time | note |", "|---|---|---|---|"]
for line in open(e["STEPS_FILE"]):
    n, r, t, note = (line.rstrip("\n").split("|") + ["", "", "", ""])[:4]
    if not n: continue
    out.append(f"| {n} | {r} | {sec(int(t or 0))} | {note} |")
    if r == "FAIL": fails.append(f"FAIL: {n} {note}")
out += ["", "## Metrics (runs/wokwi/final/metrics.json)", ""]
try:
    m = json.load(open("runs/wokwi/final/metrics.json"))
    g = m.get
    out += [f"- Utilization: {100*g('design__instance__utilization', float('nan')):.2f} %",
            f"- Stdcell area: {g('design__instance__area__stdcell')} um^2 "
            f"(fill: {g('design__instance__area__class:fill_cell')} um^2)",
            f"- Cell count: {g('design__instance__count')} total, "
            f"{g('design__instance__count__stdcell')} stdcell, "
            f"{g('design__instance__count__class:sequential_cell')} sequential",
            f"- Worst setup slack (all corners): {g('timing__setup__ws')} ns; "
            f"worst hold slack: {g('timing__hold__ws')} ns",
            f"- Violations: setup {g('timing__setup_vio__count')}, hold {g('timing__hold_vio__count')}, "
            f"route DRC {g('route__drc_errors')}, LVS {g('design__lvs_error__count')}", "",
            "| corner | setup WS (ns) | hold WS (ns) |", "|---|---|---|"]
    for k in sorted(m):
        c = re.fullmatch(r"timing__setup__ws__corner:(.+)", k)
        if c:
            out.append(f"| {c[1]} | {m[k]:.3f} | {m.get('timing__hold__ws__corner:' + c[1], float('nan')):.3f} |")
    for key in ("route__drc_errors", "design__lvs_error__count", "timing__setup_vio__count", "timing__hold_vio__count"):
        v = g(key)
        if v not in (0, None): fails.append(f"FAIL: metric {key} = {v}")
except Exception as ex:
    out.append(f"- metrics unavailable: {ex}")
    fails.append("FAIL: no metrics.json (harden did not complete)")
pcs = e["PCS"]
out += ["", "## Precheck", "", f"- precheck: {pcs}"]
if pcs.startswith("FAIL"): fails.append("FAIL: precheck")
def count(log):
    try: t = open(log).read()
    except OSError: return None
    return len(re.findall(r": PASS\b", t)), len(re.findall(r": FAIL\b", t)), t
for log, title in (("glvl_check.log", "Gate-level check (SIM=glvl wav_tool.py check)"),):
    c = count(log)
    if c:
        out += ["", f"## {title}", "", f"- {c[0]} PASS, {c[1]} FAIL (see {log})"]
        if c[1] or not c[0]: fails.append(f"FAIL: {title}")
        out += ["", "```"] + [l for l in c[2].splitlines() if re.search(r": (PASS|FAIL)", l)] + ["```"]
out += ["", "## Overall", ""] + (fails or [("PASS (precheck NOT RUN)" if pcs.startswith("NOT RUN") else "PASS") + ": every step that ran succeeded."])
print("\n".join(out))
PY
rm -f "$STEPS_FILE"
cat SUMMARY.md
if [ "$FAILS" -ne 0 ] || grep -q '^FAIL' SUMMARY.md; then
  say "DONE with failures ($FAILS failed steps); see $DEST/SUMMARY.md"; exit 1
fi
say "DONE ok in $((SECONDS - START))s; $DEST/SUMMARY.md"
