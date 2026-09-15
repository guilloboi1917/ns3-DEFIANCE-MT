#!/usr/bin/env python3
"""Infer the best checkpoint of an RLlib run directory.

Checkpoint index == iteration - 1 (verified; warns otherwise). Picks the argmax
of a trailing-mean smoothed training curve (--window), preferring valid greedy
eval returns when at least two eval points exist.

Usage: python3 training/best-checkpoint.py <run-dir> [--all] [--window 5]
"""

import argparse
import glob
import hashlib
import json
import os
import sys

WINDOW = 3
MIN_ITER = 1  # ignore the first iterations (policy still cold) when selecting


def md5(path: str) -> str:
    h = hashlib.md5()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def smooth(vals: list[float], window: int) -> list[float]:
    out = []
    acc = 0.0
    for i, v in enumerate(vals):
        acc += v
        if i >= window:
            acc -= vals[i - window]
        out.append(acc / min(i + 1, window))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("run_dir", help="ray_results/<RUN>/ directory")
    ap.add_argument("--all", action="store_true", help="print every iteration")
    ap.add_argument("--window", type=int, default=WINDOW)
    ap.add_argument("--min-iter", type=int, default=MIN_ITER,
                    help="ignore iterations below this when selecting")
    ap.add_argument("--suffix", default="policy_state.pkl",
                    help="policy state file name inside checkpoints")
    args = ap.parse_args()

    run = os.path.abspath(args.run_dir)
    if not os.path.isdir(run):
        print(f"[ERROR] not a directory: {run}")
        return 1

    # Locate the trial dir (one level down) and result.json.
    trials = [d for d in glob.glob(run + "/*/") if os.path.isfile(os.path.join(d, "result.json"))]
    if not trials:
        if os.path.isfile(os.path.join(run, "result.json")):
            trials = [run]
        else:
            print(f"[ERROR] no trial with result.json under {run}")
            return 1
    trial = trials[0]

    rows = [json.loads(l) for l in open(os.path.join(trial, "result.json"))]
    iters = [(r["training_iteration"],
              r.get("env_runners", {}).get("episode_return_mean"),
              r.get("evaluation", {}).get("episode_return_mean"))
             for r in rows]
    n = len(iters)
    if n == 0:
        print("[ERROR] empty result.json")
        return 1

    # Checkpoint dirs + index<->iteration mapping.
    ckpts = sorted(glob.glob(os.path.join(trial, "checkpoint_*")))
    ckpt_idx = {int(p.rsplit("_", 1)[1]): p for p in ckpts}
    if not ckpt_idx:
        print(f"[WARN] no checkpoint_* dirs under {trial}")
    if ckpt_idx and (max(ckpt_idx) != n - 1 or len(ckpt_idx) != n):
        print(f"[WARN] checkpoint count ({len(ckpt_idx)}, max idx "
              f"{max(ckpt_idx)}) != iterations ({n}); index==iteration-1 "
              "mapping may be wrong (checkpoint_frequency set?)")

    def ckpt_path(iteration: int) -> str:
        # checkpoint index == iteration - 1
        idx = iteration - 1
        p = ckpt_idx.get(idx)
        return p or "(none)"

    # Candidates: eval-based (>=2 valid eval points, after min_iter) else training.
    def valid_eval(it: tuple) -> bool:
        itn, tr, ev = it
        return itn >= args.min_iter and ev is not None

    eval_pts = [it for it in iters if valid_eval(it)]
    if len(eval_pts) >= 2:
        best_iter = max(eval_pts, key=lambda it: it[2])[0]
        basis = "evaluation return (explore=False)"
    else:
        if eval_pts:
            print(f"[WARN] only {len(eval_pts)} valid eval point(s) (ray 2.55.1 "
                  "drops eval results after iter 1 in some setups); using the "
                  "training curve")
        cand = [(itn, tr) for itn, tr, _ in iters if itn >= args.min_iter and tr is not None]
        if not cand:
            print("[ERROR] no usable returns")
            return 1
        sm = smooth([v for _, v in cand], args.window)
        best_iter = cand[max(range(len(cand)), key=lambda i: sm[i])][0]
        basis = f"training return, trailing-mean {args.window}-window"

    # Map run-level best_checkpoint dir onto an iteration via policy weights.
    bc_dir = os.path.join(run, "best_checkpoint")
    bc_iter = None
    if os.path.isdir(bc_dir):
        bc_files = glob.glob(os.path.join(bc_dir, "policies", "*", args.suffix))
        if bc_files:
            bc_hash = md5(bc_files[0])
            for idx, p in ckpt_idx.items():
                hit = glob.glob(os.path.join(p, "policies", "*", args.suffix))
                if hit and md5(hit[0]) == bc_hash:
                    bc_iter = idx + 1
                    break

    print(f"Run: {run}")
    print(f"Trial: {trial}")
    print(f"Iterations: {n} | checkpoints: {len(ckpt_idx)}")
    print(f"Selection basis: {basis}\n")
    print(f"{'iter':>4} {'train_ret':>9} {'eval_ret':>9}  checkpoint")
    for itn, tr, ev in iters:
        if args.all or itn == best_iter or itn == n or (bc_iter and itn == bc_iter):
            mark = ""
            if itn == best_iter:
                mark = "  <-- recommended"
            elif bc_iter and itn == bc_iter:
                mark = "  <-- run best_checkpoint/"
            elif itn == n:
                mark = "  (final)"
            tr_s = f"{tr:>9.2f}" if tr is not None else "      NaN"
            ev_s = f"{ev:>9.2f}" if ev is not None else "      -"
            print(f"{itn:>4} {tr_s} {ev_s}  {os.path.basename(ckpt_path(itn))}{mark}")

    print(f"\nRecommended checkpoint: {ckpt_path(best_iter)}")
    print(f"Final checkpoint:       {ckpt_path(n)}")
    if bc_iter:
        print(f"run best_checkpoint/ resolves to iteration {bc_iter} "
              f"({os.path.basename(ckpt_path(bc_iter))})")
    else:
        print("run best_checkpoint/: present but weights not matched to any "
              "checkpoint (different policy layout)")
    print("\nNext: run-evaluations.py <matrix.yaml> --jobs 4   "
          "(behavioural KPI selection; checkpoint is set in the YAML)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
