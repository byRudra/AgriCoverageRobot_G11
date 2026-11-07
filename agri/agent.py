"""Layered coverage agent used by both the Webots controller and the headless simulator.

Three-layer (declarative - procedural - reflexive) architecture:

    Declarative  boustrophedon coverage plan for the whole field          (planner)
    Procedural   path tracking + obstacle mapping + detour / APF steering (navigation, avoidance)
    Reflexive    emergency stop and back-off when the front clearance is too small

Call `step(pose, ranges, dt)` once per control period. It returns a Command with the
forward speed v [m/s], yaw rate w [rad/s], spray on/off and a short state string.
"""

import math
from dataclasses import dataclass

import numpy as np

from .avoidance import (Bug2Follower, GeometricDetour, ObstacleMap, ReactiveAPF,
                        front_clearance, remove_isolated)
from .coverage import CoverageGrid, RunMetrics
from .navigation import PathTracker
from .planner import plan_coverage_path


@dataclass
class Command:
    v: float
    w: float
    spray: bool
    state: str


def lidar_angles(n=180, fov=math.pi):
    """Ray angles in the robot frame, index 0 = leftmost ray (+fov/2), CCW positive."""
    step = fov / n
    return fov / 2.0 - step * (np.arange(n) + 0.5)


class CoverageAgent:
    STOP_DIST = 0.40     # reflex: front clearance (from the lidar) that triggers a stop
    SLOW_DIST = 2.0      # speed scheduling starts below this front clearance

    def __init__(self, field, method="proposed", turn="arc", tool_width=None,
                 v_max=0.5, angles=None, max_range=8.0, coverage_res=0.05,
                 lidar_offset=0.145):
        self.field = field
        self.lidar_offset = lidar_offset   # lidar is mounted ahead of the robot centre [m]
        self.method = method
        self.turn = turn
        self.angles = lidar_angles() if angles is None else np.asarray(angles)
        self.path = plan_coverage_path(field, turn=turn)
        self.planned_path = self.path.copy()
        self.tracker = PathTracker(self.path, v_max=v_max)
        self.omap = ObstacleMap(field, max_range=max_range,
                                ang_res=abs(self.angles[1] - self.angles[0]))
        if method == "proposed":
            self.avoider = GeometricDetour(field)
        elif method == "apf":
            self.avoider = ReactiveAPF()
        elif method == "bug2":
            self.avoider = Bug2Follower(field)
        else:
            raise ValueError("method must be 'proposed', 'bug2' or 'apf'")
        self.coverage = CoverageGrid(field, tool_width, res=coverage_res)
        self.metrics = RunMetrics(field)
        self.state = "START"
        self.reflex_count = 0
        self._backoff = 0.0
        self._prev_pos = None
        self.done = False

    # ------------------------------------------------------------------
    def spraying(self, pose):
        lane = self.path.lane[self.tracker.idx]
        return bool(lane >= 0 and self.field.in_crop(pose[0], pose[1]))

    def step(self, pose, ranges, dt):
        x, y, th = pose
        ranges = remove_isolated(ranges)        # drop single-ray ghost returns

        # ---- perception (all methods build the map, for the dashboard and logs)
        lidar_pose = (x + self.lidar_offset * math.cos(th),
                      y + self.lidar_offset * math.sin(th), th)
        self.omap.update(ranges, self.angles, lidar_pose)
        d_front = front_clearance(ranges, self.angles)

        if self.done:
            return Command(0.0, 0.0, False, "DONE")

        # ---- reflexive layer
        if self._backoff > 0.0:
            self._backoff -= dt
            cmd = Command(-0.12, 0.0, False, "REFLEX")
            self._account(pose, dt, False)
            return cmd
        if d_front < self.STOP_DIST:
            self.reflex_count += 1
            self._backoff = 0.8
            self.state = "REFLEX"
            self._account(pose, dt, False)
            return Command(0.0, 0.0, False, "REFLEX")

        # ---- procedural layer
        speed_scale = float(np.clip((d_front - self.STOP_DIST) /
                                    (self.SLOW_DIST - self.STOP_DIST), 0.35, 1.0))
        override = None
        if isinstance(self.avoider, GeometricDetour):
            self.avoider.update(self.tracker, self.omap, pose)
        else:
            self.tracker.target_point(np.array([x, y]))
            override = self.avoider.heading(self.tracker, ranges, self.angles, lidar_pose, dt)

        v, w = self.tracker.update(pose, dt, speed_scale, override)
        if self.tracker.done:
            self.done = True
            self.state = "DONE"
            self.coverage._flush()
            return Command(0.0, 0.0, False, "DONE")

        spray = self.spraying(pose)
        on_detour = bool(self.path.detour[self.tracker.idx]) or override is not None
        if self.tracker.pivoting:
            self.state = "PIVOT"
        elif on_detour:
            self.state = "AVOID"
        elif self.path.lane[self.tracker.idx] >= 0:
            self.state = "LANE %d" % (self.path.lane[self.tracker.idx] + 1)
        else:
            self.state = "TURN"
        self._account(pose, dt, spray, on_detour)
        return Command(v, w, spray, self.state)

    def _account(self, pose, dt, spray, on_detour=False):
        x, y, _ = pose
        lane = int(self.path.lane[self.tracker.idx])
        if spray and self._prev_pos is not None:
            self.coverage.mark(self._prev_pos, (x, y), lane)
        self._prev_pos = (x, y)
        xte = None
        if lane >= 0 and not on_detour and self.field.in_crop(x, y):
            xte = y - self.field.lanes()[lane]
        self.metrics.update(pose, dt, xte, spray)

    # ------------------------------------------------------------------
    def summary(self):
        s = {"method": self.method, "turn": self.turn,
             "tool_width": self.coverage.tool_width,
             "obstacles": len(self.field.obstacles),
             "detours": self.avoider.n_detours, "skips": self.avoider.n_skips,
             "reflex_stops": self.reflex_count,
             "mapped_obstacles": len(self.omap.confirmed()),
             "completed": self.done}
        s.update(self.metrics.summary())
        s.update(self.coverage.stats())
        return s
