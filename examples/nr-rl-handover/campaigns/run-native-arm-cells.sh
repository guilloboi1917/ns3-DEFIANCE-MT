#!/usr/bin/env bash
# run-native-arm-cells.sh -- evaluate the CORRECTED in-regime arm
# (PPO_2026-09-21_12-07-15, trained with four aerial interferers) in the
# interfered cells:
#   agent-eval-ul-if-tcp-beta5-fixed-native  native interfered TCP
#
# Counterpart of campaigns/run-fixed-arm-cells.sh (the clean-trained arm). Only
# the interfered TCP cell is run for now; the in-regime QUIC cell is deferred.
#
# Output goes to results-unsteered/<eval>/, next to the transfer cell it is
# compared against (the A3 comparators are unaffected by the RL-side fixes).
#
# Usage: bash campaigns/run-native-arm-cells.sh [jobs]      (default 5)
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

for cfg in agent-eval-ul-if-tcp-beta5-fixed-native; do
    echo "== $cfg (jobs=$JOBS) $(date -Is)"
    python3 run-evaluations.py "evaluation-scenarios/rl/$cfg.yaml" \
        --jobs "$JOBS" --analyze --output "results-unsteered/$cfg"
done
echo "=== all cells done $(date -Is)"
