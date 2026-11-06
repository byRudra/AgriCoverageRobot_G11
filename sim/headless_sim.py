"""Headless 2-D simulator for fast, repeatable batch experiments.

It models the same Pioneer 3-AT (skid-steer, wheel radius 0.11 m, 6.4 rad/s wheel limit)
and the same 180-ray lidar used in the Webots world, and runs the identical
`agri.agent.CoverageAgent`. Webots is used for the 3-D demo and validation runs;
this simulator is used for the 20-seed statistics in the report.

Usage:
    python sim/headless_sim.py --method proposed --turn arc --seed 11 --plot
"""

import argparse
import json
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from agri.agent import CoverageAgent, lidar_angles   # noqa: E402
from agri.field import Field, default_field, random_field   # noqa: E402

WHEEL_R = 0.11
WHEEL_MAX = 6.4                 # rad/s
TRACK_EFF = 0.394 * 1.4         # effective track of a skid-steer base (slip) [m]
LIDAR_X = 0.145                 # lidar mounted 14.5 cm ahead of the centre
ROBOT_R = 0.30


class DiffDriveSim:
    """Unicycle with wheel saturation, first-order actuator lag and slip noise."""

    def __init__(self, pose, tau=0.12, slip_std=0.02, seed=0):
        self.x, self.y, self.th = pose
        self.v = 0.0
        self.w = 0.0
        self.tau = tau
        self.slip_std = slip_std
        self.rng = np.random.default_rng(seed)

    def step(self, v_cmd, w_cmd, dt, obstacles):
        vmax = WHEEL_R * WHEEL_MAX
        vl = v_cmd - w_cmd * TRACK_EFF / 2
        vr = v_cmd + w_cmd * TRACK_EFF / 2
        s = max(1.0, abs(vl) / vmax, abs(vr) / vmax)      # keep the curvature when saturating
        vl, vr = vl / s, vr / s
        v_t = (vl + vr) / 2
        w_t = (vr - vl) / TRACK_EFF
        a = dt / (self.tau + dt)
        self.v += a * (v_t - self.v)
        self.w += a * (w_t - self.w)
        v = self.v * (1 + self.rng.normal(0, self.slip_std))
        w = self.w * (1 + self.rng.normal(0, self.slip_std))
        nx = self.x + v * math.cos(self.th) * dt
        ny = self.y + v * math.sin(self.th) * dt
        # rigid obstacles: the robot cannot move into a stone or pole
        blocked = any(math.hypot(nx - o.x, ny - o.y) < o.r + ROBOT_R * 0.9 for o in obstacles)
        if not blocked:
            self.x, self.y = nx, ny
        self.th = math.atan2(math.sin(self.th + w * dt), math.cos(self.th + w * dt))
        return (self.x, self.y, self.th)


class LidarSim:
    """Single-layer 2-D lidar: ray / circle intersections, Gaussian range noise."""

    def __init__(self, n=180, fov=math.pi, max_range=10.0, noise=0.01, seed=0):
        self.angles = lidar_angles(n, fov)
        self.max_range = max_range
        self.noise = noise
        self.rng = np.random.default_rng(seed + 1000)

    def scan(self, pose, obstacles):
        x, y, th = pose
        ox = x + LIDAR_X * math.cos(th)
        oy = y + LIDAR_X * math.sin(th)
        a = th + self.angles
        dx, dy = np.cos(a), np.sin(a)
        r = np.full(len(a), np.inf)
        for o in obstacles:
            # stones look a bit smaller at lidar height than their ground footprint
            rad = o.r * (0.78 if o.kind == "stone" else 1.0)
            fx, fy = ox - o.x, oy - o.y
            b = fx * dx + fy * dy
            c = fx * fx + fy * fy - rad * rad
            disc = b * b - c
            m = disc >= 0
            t = -b - np.sqrt(np.where(m, disc, 0))
            hit = m & (t > 0)
            r = np.where(hit & (t < r), t, r)
        r = r + self.rng.normal(0, self.noise, len(r))
        r[r > self.max_range] = np.inf
        return r


def run_episode(field, method="proposed", turn="arc", tool_width=None, seed=0,
                dt=0.032, t_max=1500.0, record_every=5):
    agent = CoverageAgent(field, method=method, turn=turn, tool_width=tool_width)
    robot = DiffDriveSim(field.start, seed=seed)
    lidar = LidarSim(seed=seed)
    pose = field.start
    traj = []
    k = 0
    while agent.metrics.time < t_max:
        ranges = lidar.scan(pose, field.obstacles)
        cmd = agent.step(pose, ranges, dt)
        if agent.done:
            break
        pose = robot.step(cmd.v, cmd.w, dt, field.obstacles)
        if k % record_every == 0:
            traj.append((agent.metrics.time, pose[0], pose[1], pose[2], cmd.v, cmd.w,
                         int(cmd.spray), cmd.state, int(agent.path.lane[agent.tracker.idx])))
        k += 1
    s = agent.summary()
    s["seed"] = field.seed
    return s, traj, agent


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--method", default="proposed", choices=["proposed", "bug2", "apf"])
    ap.add_argument("--turn", default="arc", choices=["arc", "pivot"])
    ap.add_argument("--seed", type=int, default=None, help="random field seed (default: demo field)")
    ap.add_argument("--field", default=None, help="field JSON file")
    ap.add_argument("--tool-width", type=float, default=None)
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()

    if args.field:
        field = Field.load(args.field)
    elif args.seed is not None:
        field = random_field(args.seed)
    else:
        field = default_field()
    s, traj, agent = run_episode(field, args.method, args.turn, args.tool_width)
    print(json.dumps(s, indent=2))
    if args.plot:
        from experiments.plotting import plot_run
        out = os.path.join(ROOT, "results", "headless_%s_%s.png" % (args.method, args.turn))
        plot_run(field, agent, traj, out, title="%s / %s" % (args.method, args.turn))
        print("saved", out)


if __name__ == "__main__":
    main()
