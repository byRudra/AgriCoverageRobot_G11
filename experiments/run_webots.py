"""Run the Webots world in batch (fast, no rendering) for each method / turn combination.

Owner: M5 (experiments). Results go to results/webots_<method>_<turn>/.

    python experiments/run_webots.py                      # all four combinations
    python experiments/run_webots.py --method proposed --turn arc
    python experiments/run_webots.py --webots "D:/Webots/msys64/mingw64/bin/webots.exe"
"""

import argparse
import json
import os
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_WEBOTS = r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"


def run(webots, world, method, turn, timeout=1800):
    env = dict(os.environ, AGRI_METHOD=method, AGRI_TURN=turn, AGRI_QUIT="1", AGRI_NODASH="1",
               AGRI_RUN="webots_%s_%s" % (method, turn))
    cmd = [webots, "--mode=fast", "--no-rendering", "--minimize", "--batch", "--stdout", "--stderr", world]
    t0 = time.time()
    p = subprocess.run(cmd, env=env, cwd=os.path.dirname(world), stdout=subprocess.PIPE,
                       stderr=subprocess.STDOUT, timeout=timeout, text=True, errors="replace")
    for line in p.stdout.splitlines():
        if line.startswith("[agri]") or "Traceback" in line or "Error:" in line:
            print("   ", line)
    summ = os.path.join(ROOT, "results", "webots_%s_%s" % (method, turn), "summary.json")
    if os.path.exists(summ) and os.path.getmtime(summ) > t0:
        with open(summ) as f:
            return json.load(f)
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--webots", default=os.environ.get("WEBOTS_EXE", DEFAULT_WEBOTS))
    ap.add_argument("--world", default=os.path.join(ROOT, "worlds", "crop_field.wbt"))
    ap.add_argument("--method", choices=["proposed", "bug2", "apf"])
    ap.add_argument("--turn", choices=["arc", "pivot"])
    args = ap.parse_args()
    methods = [args.method] if args.method else ["proposed", "bug2", "apf"]
    turns = [args.turn] if args.turn else ["arc", "pivot"]
    rows = []
    for m in methods:
        for t in turns:
            print("== Webots run: %s / %s" % (m, t))
            s = run(args.webots, args.world, m, t)
            if s is None:
                print("   run failed (no summary written)")
                continue
            rows.append(s)
            print("   coverage %.2f %% | time %.1f s | path %.1f m | detours %d | collisions %d | min clearance %.3f m"
                  % (s["coverage_pct"], s["time_s"], s["path_m"], s["detours"], s["collisions"], s["min_clearance_m"]))
    out = os.path.join(ROOT, "results", "webots_summary.json")
    with open(out, "w") as f:
        json.dump(rows, f, indent=2)
    print("saved", out)


if __name__ == "__main__":
    sys.exit(main())
