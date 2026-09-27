#!/usr/bin/env bash
# run-native-cp-ladder.sh -- exploratory: the in-regime native cell evaluated on
# later checkpoints of PPO_2026-09-21_12-07-15 instead of the selected best
# checkpoint_000052 (after iteration 53). Tests whether the handover churn seen
# at cp52 persists as the policy entropy falls.
#
#   cp99  = after iteration 100 (last checkpoint; entropy 0.194)
#   cp89  = after iteration 90  (lowest entropy in the second half; 0.110)
#
# Output goes to output/ (gitignored scratch), NOT results-unsteered/, so the
# thesis dataset stays untouched.
#
# Usage: bash campaigns/run-native-cp-ladder.sh [jobs]      (default 5)
set -uo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
NS3ROOT="$(cd "$HERE/../../../../.." && pwd)"
cd "$ROOT" || exit 1
export NS3_HOME="$NS3ROOT"
export PATH="$NS3ROOT:$PATH"
VENV=/home/nisaak/.cache/pypoetry/virtualenvs/ns-defiance-ZPBkiHWi-py3.10/bin
[ -x "$VENV/run-agent" ] && export PATH="$VENV:$PATH"
JOBS="${1:-5}"
RUN=PPO_2026-09-21_12-07-15
TRIAL=PPO_defiance_3873e_00000_0_2026-09-21_12-07-15
CFG=evaluation-scenarios/rl/agent-eval-ul-if-tcp-beta5-fixed-native.yaml

for cp in 000099 000089; do
    out="output/agent-eval-ul-if-tcp-beta5-fixed-native-cp${cp#0000}"
    ckpt="$HOME/ray_results/$RUN/$TRIAL/checkpoint_$cp"
    echo "== checkpoint_$cp -> $out (jobs=$JOBS) $(date -Is)"
    python3 run-evaluations.py "$CFG" \
        --checkpoint "$ckpt" --jobs "$JOBS" --analyze --output "$out"
    echo "== checkpoint_$cp done rc=$? $(date -Is)"
done
echo "=== ladder done $(date -Is)"
