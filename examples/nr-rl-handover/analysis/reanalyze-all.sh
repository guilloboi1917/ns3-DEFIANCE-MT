#!/usr/bin/env bash
# Re-derive raw.csv / aggregate.csv for every result cell with the current
# analyzer. Needed after a metric-definition change (the goodput denominator is
# now the offered window, sim_time - appStartS, see analyze-evaluations.py).
# Analysis only: no simulations are run.
#
# Usage: ./analysis/reanalyze-all.sh

set -u -o pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
cd "$ROOT"

n=0
for d in results/*/*/; do
    [[ -d "$d" ]] || continue
    case "$d" in
        results/_archive*|results/_logs*) continue ;;
    esac
    ls "$d"seed_* >/dev/null 2>&1 || continue
    if python3 analysis/analyze-evaluations.py "$d" >/dev/null; then
        n=$((n + 1))
    else
        echo "WARNING: analyze failed for $d"
    fi
done
echo "re-analyzed $n cells"
