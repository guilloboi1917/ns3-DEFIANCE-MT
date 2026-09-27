#!/usr/bin/env bash
# run-fixed-arm-cells.sh -- evaluate the CORRECTED clean-regime arm
# (PPO_2026-09-20_13-59-15) in the three remaining cells:
#   agent-eval-ul-if-tcp-beta5-fixed-transfer zero-shot transfer into interference (TCP)
#   agent-eval-ul-no-if-quic-beta5-fixed       clean regime over QUIC
#   agent-eval-ul-if-quic-beta5-fixed          interfered regime over QUIC
# Companion of the already-run clean-TCP cell
# (agent-eval-ul-no-if-tcp-beta5-fixed).
#
# RENAMED 2026-09-22: the interfered TCP cell gained the -transfer suffix so it
# cannot collide with the in-regime arm's native cell. See
# campaigns/run-native-arm-cells.sh for the native counterpart.
#
# Output goes to results-unsteered/<eval>/, so this arm lives in the same tree as
# the unsteered rerun it is compared against (the A3 comparators are unaffected
# by the RL-side interface fixes).
#
# Usage: bash campaigns/run-fixed-arm-cells.sh [jobs]      (default 5)
set -uo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"                 # example dir (results* live here)
NS3ROOT="$(cd "$HERE/../../../../.." && pwd)"  # ns-3-dev
cd "$ROOT" || exit 1
export NS3_HOME="$NS3ROOT"
export PATH="$NS3ROOT:$PATH"
VENV=/home/nisaak/.cache/pypoetry/virtualenvs/ns-defiance-ZPBkiHWi-py3.10/bin
[ -x "$VENV/run-agent" ] && export PATH="$VENV:$PATH"
JOBS="${1:-5}"

for cfg in agent-eval-ul-if-tcp-beta5-fixed-transfer \
           agent-eval-ul-no-if-quic-beta5-fixed \
           agent-eval-ul-if-quic-beta5-fixed; do
    echo "== $cfg (jobs=$JOBS) $(date -Is)"
    python3 run-evaluations.py "evaluation-scenarios/rl/$cfg.yaml" \
        --jobs "$JOBS" --analyze --output "results-unsteered/$cfg"
done
echo "=== all cells done $(date -Is)"
