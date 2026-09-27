#!/usr/bin/env bash
# run-unsteered-recheck.sh -- rerun the thesis-quoted campaign cells with the gNB
# uniformly unsteered (2026-09-18).
#
# WHY: in the campaign the gNB applies the steering vector only while the UAV
# remains on the cell that registered it at initial attachment, so each arm's
# beam state depends on when its FIRST handover happens ("opening-window
# exposure"). A3 arms lose the window at 0.658 s on 13-14 of 20 seeds; RL arms
# cannot hand over before ~1.203 s and on some seeds keep it for most of the
# episode (seed 10: 36.4 s = 71 %). Comparisons across different handover
# behaviour therefore mix the intended variable with a beam-regime difference.
# This run removes that everywhere by making the whole episode unsteered.
#
# NO INJECTION NEEDED (since 2026-09-18 the scenario default is quasiomni, see
# nr-rl-handover-scenario.cc): the canonical configs are passed unchanged and
# each seed's meta.yaml is checked afterwards to report quasiomni. Run on the
# UNPATCHED build (contrib/nr @ nr-v5.1-local, a75668ae): `directpath` means
# something different in the patched build, but `quasiomni` is patch-invariant.
# The UE-side method difference is irrelevant (1x1 element).
#
# Output: results-unsteered/<eval-name>/<tag>/seed_N, a sibling of results/ inside
# this example dir (same convention as run-canonical-wave*.sh), so
# sync-thesis-data.py can be pointed at it.
#
# Usage:
#   bash campaigns/run-unsteered-recheck.sh [--dry-run] [--tags a,b] [--only eval]
#       [--jobs-a3 10] [--jobs-rl 6] [--drop-load-matched] [--keep-bw30-probe]
# Resumable: a config whose quoted cells are all complete is skipped.
set -uo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"              # campaigns/
ROOT="$(cd "$HERE/.." && pwd)"                      # example dir: results* live here
NS3ROOT="$(cd "$HERE/../../../../.." && pwd)"       # ns-3-dev root
cd "$ROOT" || exit 1
OUT_ROOT="$ROOT/results-unsteered"
export NS3_HOME="$NS3ROOT"
export PATH="$NS3ROOT:$PATH"
LOGDIR="campaigns/unsteered-logs"
MANIFEST="/home/nisaak/uzh/mt/Master-Thesis-Overleaf/EvaluationData/manifest.csv"
DRY=""; TAGS=""; ONLY=""; JOBS_A3=10; JOBS_RL=6; TIMEOUT=2400

# Cells excluded from the rerun because the final thesis discussion does not
# quote them (checked 2026-09-18 against Chapter/*.tex and the tables registry):
#   - a3-extremes-alt: unused data (tex-results-tables.md).
#   - *-m5t256 (tuned A3): the tuned arm was dropped 2026-09-16; one baseline only.
#   - mtx-ul h5-15_*: the 5-15 m band was dropped from the matrix (resultsdiscussion.tex).
#   - capacity-probe-canonical: the offered-load argument is carried by the A3
#     runs; this also removes bw50-n1, the only 50 MHz cell.
# Recorded here so the rerun stays reproducible and the omissions are explicit.
declare -A DROP_TAGS
DROP_TAGS[mtx-ul]="h5-15_s5,h5-15_s10,h5-15_s20"
DROPPED_EVALS="a3-extremes-alt nr-rl-handover-a3-baseline-30mhz-tcp-ul-m5t256 nr-rl-handover-a3-baseline-30mhz-quic-ul-m5t256 capacity-probe-canonical"
while [ $# -gt 0 ]; do
  case "$1" in
    --dry-run) DRY="--dry-run"; shift ;;
    --tags) TAGS="$2"; shift 2 ;;
    --only) ONLY="$2"; shift 2 ;;
    --jobs-a3) JOBS_A3="$2"; shift 2 ;;
    --jobs-rl) JOBS_RL="$2"; shift 2 ;;
    --drop-load-matched) DROPPED_EVALS="$DROPPED_EVALS transport-comparison-load-matched"; shift ;;
    --keep-bw30-probe) DROPPED_EVALS="${DROPPED_EVALS//capacity-probe-canonical/}"
                       DROP_TAGS[capacity-probe-canonical]="bw10-n1,bw50-n1"; shift ;;
    *) echo "unknown arg: $1"; exit 2 ;;
  esac
done
mkdir -p "$OUT_ROOT" "$LOGDIR"

# RL cells need `run-agent` on PATH (poetry venv). Add it when the caller has not
# activated the environment, so a launch cannot die halfway through on RL cells.
VENV_BIN="/home/nisaak/.cache/pypoetry/virtualenvs/ns-defiance-ZPBkiHWi-py3.10/bin"
if ! command -v run-agent >/dev/null 2>&1 && [ -x "$VENV_BIN/run-agent" ]; then
  export PATH="$VENV_BIN:$PATH"
fi
if ! command -v run-agent >/dev/null 2>&1 && [ -z "$ONLY" ] && [ -z "$DRY" ]; then
  echo "WARNING: run-agent not on PATH; RL cells will fail (A3 cells are fine)"
fi

# Config order: the claim-bearing core pairs first, then the rest of A3, then RL.
# The two beta-1.0 QUIC cells (added 2026-09-18) are not in the campaign
# manifest yet, so they are seeded here with their own cell/seed declaration.
python3 - "$MANIFEST" <<'PY' > /tmp/unsteered-order.txt
import csv, os, sys
rows = list(csv.DictReader(open(sys.argv[1])))
evals = {}
for r in rows:
    e = r['evaluation']
    v = evals.setdefault(e, {'mode': r['mode'], 'runs': 0, 'ns': set()})
    try: v['runs'] += int(float(r['n_seeds']))
    except Exception: pass
    v['ns'].add(r['n_seeds'])
# Cells that exist as configs but have no campaign results yet (new matrix legs).
for e in ('agent-eval-ul-no-if-quic-beta1', 'agent-eval-ul-if-quic-beta1'):
    evals.setdefault(e, {'mode': 'rl', 'runs': 20, 'ns': {'20'}})
base = 'evaluation-scenarios'
def path(e):
    for sub in ('baseline', 'rl'):
        p = f'{base}/{sub}/{e}.yaml'
        if os.path.exists(p): return p
    return None
priority = ['nr-rl-handover-a3-baseline-30mhz-tcp-ul',
            'agent-eval-ul-no-if-tcp',
            'agent-eval-ul-if-tcp',
            'nr-rl-handover-a3-baseline-30mhz-quic-ul',
            'agent-eval-ul-no-if-quic',
            'agent-eval-ul-if-quic',
            'agent-eval-ul-no-if-quic-beta1',
            'agent-eval-ul-if-quic-beta1']
a3 = [e for e in evals if evals[e]['mode'] != 'rl']
rl = [e for e in evals if evals[e]['mode'] == 'rl']
def order(lst):
    return sorted(lst, key=lambda e: (priority.index(e) if e in priority else len(priority), e))
# Priority cells first, ACROSS modes (they are the claim-bearing pairs), then the
# remaining A3 cells, then the remaining RL cells.
for e in order([x for x in evals if x in priority]) + order([x for x in a3 if x not in priority]) \
         + order([x for x in rl if x not in priority]):
    p = path(e)
    if p: print(f"{evals[e]['mode']}\t{e}\t{p}\t{evals[e]['runs']}\t{','.join(sorted(evals[e]['ns']))}")
PY

total_runs=0; total_cells=0; total_bad=0; total_checked=0
while IFS=$'\t' read -r mode name cfg runs ns; do
  [ -n "$ONLY" ] && [ "$ONLY" != "$name" ] && continue
  case " $DROPPED_EVALS " in *" $name "*) echo "DROP (not quoted in the thesis): $name"; continue ;; esac
  [ -z "$cfg" ] && { echo "SKIP (no config): $name"; continue; }

  # Tag filtering: run only the cells the thesis quotes.
  mapfile -t all_tags < <(grep -oE 'tag: "?[A-Za-z0-9_.-]+' "$cfg" | sed 's/^tag: "\?//')
  drop="${DROP_TAGS[$name]:-}"
  keep=()
  for t in "${all_tags[@]}"; do
    [[ ",$drop," == *",$t,"* ]] && continue
    keep+=("$t")
  done
  tag_flag=""
  if [ "${#keep[@]}" -lt "${#all_tags[@]}" ]; then
    tag_flag="--tags $(IFS=,; echo "${keep[*]}")"
    echo "   (dropping tags: $drop)"
  fi
  [ -n "$TAGS" ] && tag_flag="--tags $TAGS"

  # Completion: a cell counts as done when every seed has finished, successful or
  # not. Use run-info.yaml, which run-evaluations.py writes *after* the seed
  # process returns (meta.yaml and the per-seed CSVs appear at simulation setup,
  # so an interrupted seed would otherwise look complete). Failures are reported
  # but do not hold the config back: a deterministic crash
  # (docs/BUGS-ISSUES.md #29) could otherwise never be clean, which would force a
  # full re-run of that config on every resume.
  want="${ns%%,*}"
  done_cells=0
  for t in "${keep[@]}"; do
    attempted=$(ls "$OUT_ROOT/$name/$t"/seed_*/run-info.yaml 2>/dev/null | wc -l)
    bad=$(grep -l "error:" "$OUT_ROOT/$name/$t"/seed_*/run-info.yaml 2>/dev/null | wc -l)
    [ "$attempted" -ge "$want" ] && done_cells=$((done_cells + 1))
    [ "$bad" -gt 0 ] && echo "   note: $t has $bad failed seed(s); the analyzer excludes them"
  done
  if [ "$done_cells" -ge "${#keep[@]}" ] && [ -z "$DRY" ]; then
    echo "SKIP (complete): $name"; continue
  fi

  # Guard: nothing in this run may set a method other than the quasiomni default.
  if grep -qE '^\s*beamformingMethod:' "$cfg"; then
    echo "ERROR: $cfg sets beamformingMethod; this rerun is unsteered-only."; exit 1
  fi
  cfg_abs="$PWD/$cfg"
  jobs=$JOBS_A3; [ "$mode" = "rl" ] && jobs=$JOBS_RL
  quoted=${#keep[@]}
  echo "== $name ($mode, $quoted cells, $runs runs declared, jobs=$jobs)"
  ( cd "$ROOT" && python3 "$ROOT/run-evaluations.py" "$cfg_abs" \
        --jobs "$jobs" --analyze --timeout "$TIMEOUT" --output "$OUT_ROOT/$name" $tag_flag $DRY ) \
      2>&1 | tee "$LOGDIR/$name.log" | tail -2

  # Verify the effective beamforming method from the run metadata.
  bad=0; checked=0
  for t in "${keep[@]}"; do
    for m in "$OUT_ROOT/$name/$t"/seed_*/meta.yaml; do
      [ -f "$m" ] || continue
      checked=$((checked + 1))
      grep -qE '^\s*beamformingMethod:\s*quasiomni' "$m" || { bad=$((bad + 1)); echo "   !! not quasiomni: $m"; }
    done
  done
  [ "$checked" -gt 0 ] && echo "   method check: $((checked - bad))/$checked seeds report quasiomni"
  total_bad=$((total_bad + bad)); total_checked=$((total_checked + checked))

  # authoritative run count: quoted cells x seeds per cell
  if [[ "$ns" == *,* ]]; then
    kept_runs=0
    for k in ${ns//,/ }; do kept_runs=$((kept_runs + k * ${#keep[@]})); done
  else
    kept_runs=$(( ${#keep[@]} * ns ))
  fi
  total_runs=$((total_runs + kept_runs))
  total_cells=$((total_cells + ${#keep[@]}))
done < /tmp/unsteered-order.txt
echo "=== done. quoted cells: $total_cells ; expected runs: $total_runs ; output: $OUT_ROOT"
echo "=== method check: $((total_checked - total_bad))/$total_checked seeds report quasiomni"
[ "$total_bad" -gt 0 ] && { echo "!!! $total_bad seeds did not report quasiomni"; exit 1; }
exit 0
