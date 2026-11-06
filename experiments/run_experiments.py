"""Batch experiments for the report (owner: M5). Uses the headless simulator.

    python experiments/run_experiments.py            # all experiments (~5 min on 8+ cores)
    python experiments/run_experiments.py --quick    # 5 seeds only

E1  proposed vs Bug2 vs APF, arc vs pivot turns, 20 random fields (Monte-Carlo)
E2  obstacle density sweep (0 ... 16 obstacles), all three methods
E3  boom (tool) width sweep 1.0 ... 1.4 m: coverage vs overlap (re-uses E1 trajectories)
E4  heading-PID tuning: lateral step response for three gain sets

Outputs: results/*.csv and report/figures/*.png
"""

import argparse
import csv
import json
import os
import sys
from multiprocessing import Pool

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from agri.coverage import CoverageGrid                    # noqa: E402
from agri.field import random_field     # noqa: E402
from agri.navigation import PathTracker                  # noqa: E402
from agri.planner import Path        # noqa: E402
from sim.headless_sim import DiffDriveSim, run_episode   # noqa: E402

RES = os.path.join(ROOT, "results")
FIG = os.path.join(ROOT, "report", "figures")
TOOL_WIDTHS = [1.0, 1.1, 1.2, 1.3, 1.4]
METHODS = ("proposed", "bug2", "apf")


# ---------------------------------------------------------------------- workers
def _e1_job(args):
    seed, method, turn = args
    field = random_field(seed)
    s, traj, agent = run_episode(field, method, turn, seed=seed)
    # E3: recompute coverage for other boom widths from the same trajectory
    widths = {}
    for tw in TOOL_WIDTHS:
        g = CoverageGrid(field, tw)
        prev = None
        for r in traj:
            if r[6] and prev is not None:
                g.mark(prev, (r[1], r[2]), r[8])
            prev = (r[1], r[2]) if r[6] else None
        g._flush()
        st = g.stats()
        widths[tw] = (st["coverage_pct"], st["overlap_pct"])
    s["widths"] = widths
    return s


def _e2_job(args):
    seed, method, n_obs = args
    field = random_field(1000 + seed, n_stones=(n_obs + 1) // 2, n_poles=n_obs // 2)
    s, _, _ = run_episode(field, method, "arc", seed=seed)
    s["n_obs"] = n_obs
    return s


# ---------------------------------------------------------------------- helpers
def write_csv(path, rows, keys):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)


def mean_std(rows, key):
    v = np.array([r[key] for r in rows], float)
    return float(v.mean()), float(v.std(ddof=1)) if len(v) > 1 else 0.0


# ---------------------------------------------------------------------- E4: PID tuning
def step_response(kp, kd, ki, offset=0.4, t_end=12.0, dt=0.032):
    """Robot starts `offset` m beside a straight lane; returns t, cross-track error."""
    pts = np.column_stack([np.arange(-10, 30, 0.2), np.zeros(200)])
    path = Path(pts, np.zeros(len(pts), int))
    tr = PathTracker(path, kp=kp, kd=kd, ki=ki)
    robot = DiffDriveSim((-9.0, offset, 0.0), slip_std=0.0)
    pose = (-9.0, offset, 0.0)
    ts, es, ws = [], [], []
    t = 0.0
    while t < t_end:
        v, w = tr.update(pose, dt)
        pose = robot.step(v, w, dt, [])
        ts.append(t)
        es.append(pose[1])
        ws.append(w)
        t += dt
    return np.array(ts), np.array(es), np.array(ws)


def step_metrics(t, e, offset):
    y = e / offset                      # normalised: 1 -> 0
    rise = next((ti for ti, yi in zip(t, y) if yi <= 0.1), float("nan"))   # 90 % rise
    overshoot = max(0.0, -y.min()) * 100
    settle = float("nan")
    band = np.abs(y) > 0.05
    if band.any():
        last = np.where(band)[0][-1]
        settle = t[min(last + 1, len(t) - 1)]
    return rise, overshoot, settle


# ---------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--workers", type=int, default=min(16, os.cpu_count() or 4))
    args = ap.parse_args()
    os.makedirs(RES, exist_ok=True)
    os.makedirs(FIG, exist_ok=True)
    seeds = list(range(1, 6 if args.quick else 21))

    # ---------------- E1 + E3
    jobs = [(s, m, t) for s in seeds for m in METHODS for t in ("arc", "pivot")]
    print("E1: %d runs ..." % len(jobs))
    with Pool(args.workers) as pool:
        e1 = pool.map(_e1_job, jobs)
    keys = ["seed", "method", "turn", "completed", "coverage_pct", "overlap_pct", "missed_m2",
            "time_s", "path_m", "detours", "skips", "reflex_stops", "collisions",
            "min_clearance_m", "xte_rms_cm", "xte_max_cm", "mapped_obstacles", "obstacles"]
    write_csv(os.path.join(RES, "e1_monte_carlo.csv"), e1, keys)

    summary = {}
    for m in METHODS:
        for t in ("arc", "pivot"):
            rows = [r for r in e1 if r["method"] == m and r["turn"] == t]
            d = {"n": len(rows), "completed": sum(r["completed"] for r in rows),
                 "collisions_total": sum(r["collisions"] for r in rows),
                 "runs_with_skips": sum(r["skips"] > 0 for r in rows)}
            for k in ("coverage_pct", "overlap_pct", "missed_m2", "time_s", "path_m",
                      "min_clearance_m", "xte_rms_cm", "skips", "reflex_stops"):
                d[k] = mean_std(rows, k)
            d["coverage_min"] = min(r["coverage_pct"] for r in rows)
            d["clearance_min"] = min(r["min_clearance_m"] for r in rows)
            summary["%s_%s" % (m, t)] = d
    # E3 summary (proposed / arc)
    rows = [r for r in e1 if r["method"] == "proposed" and r["turn"] == "arc"]
    e3 = []
    for tw in TOOL_WIDTHS:
        cov = np.array([r["widths"][tw][0] for r in rows])
        ov = np.array([r["widths"][tw][1] for r in rows])
        e3.append({"tool_width": tw, "coverage_mean": cov.mean(), "coverage_std": cov.std(ddof=1),
                   "overlap_mean": ov.mean(), "overlap_std": ov.std(ddof=1)})
    write_csv(os.path.join(RES, "e3_tool_width.csv"), e3, list(e3[0].keys()))
    summary["e3"] = e3

    # ---------------- E2
    densities = [0, 4, 8, 12, 16]
    jobs = [(s, m, n) for s in seeds[:10] for m in METHODS for n in densities]
    print("E2: %d runs ..." % len(jobs))
    with Pool(args.workers) as pool:
        e2 = pool.map(_e2_job, jobs)
    write_csv(os.path.join(RES, "e2_density.csv"), e2, ["n_obs"] + keys)
    summary["e2"] = {}
    for m in METHODS:
        summary["e2"][m] = []
        for n in densities:
            rr = [r for r in e2 if r["method"] == m and r["n_obs"] == n]
            summary["e2"][m].append({"n_obs": n, "coverage": mean_std(rr, "coverage_pct"),
                                     "time": mean_std(rr, "time_s"),
                                     "clearance": mean_std(rr, "min_clearance_m"),
                                     "skips": mean_std(rr, "skips")})

    # ---------------- E4
    gains = [("Kp=1.0, Kd=0.00", 1.0, 0.0, 0.0), ("Kp=2.2, Kd=0.12 (used)", 2.2, 0.12, 0.05),
             ("Kp=4.5, Kd=0.00", 4.5, 0.0, 0.0)]
    e4 = []
    curves = []
    for name, kp, kd, ki in gains:
        t, e, w = step_response(kp, kd, ki)
        rise, ov, settle = step_metrics(t, e, 0.4)
        e4.append({"gains": name, "kp": kp, "kd": kd, "ki": ki, "rise_s": rise,
                   "overshoot_pct": ov, "settling_s": settle})
        curves.append((name, t, e, w))
    write_csv(os.path.join(RES, "e4_pid_tuning.csv"), e4, list(e4[0].keys()))
    summary["e4"] = e4

    with open(os.path.join(RES, "experiment_summary.json"), "w") as f:
        json.dump(summary, f, indent=2, default=float)
    np.save(os.path.join(RES, "e4_curves.npy"), np.array(curves, dtype=object), allow_pickle=True)
    for k, v in summary.items():
        if k.startswith("e"):
            continue
        print("%-15s done %2d/%d  cov %.2f+-%.2f (min %.2f)  time %.0f+-%.0f  clr %.3f (min %.3f)  xte %.1f  skips-runs %d"
              % (k, v["completed"], v["n"], v["coverage_pct"][0], v["coverage_pct"][1], v["coverage_min"],
                 v["time_s"][0], v["time_s"][1], v["min_clearance_m"][0], v["clearance_min"],
                 v["xte_rms_cm"][0], v["runs_with_skips"]))
    print("E3", e3)
    print("E4", e4)


if __name__ == "__main__":
    main()
