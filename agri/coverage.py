"""Coverage accounting, spray gating and run metrics.

Owner: M5 (coverage logging and analysis).

The crop area is rasterised into square cells (default 5 cm). While the sprayer is on,
every cell whose centre lies within tool_width/2 of the boom's swept segment is marked.
Cells are counted per pass (one pass = one lane, including its detours), so overlap is
the share of sprayed area that received two or more passes.

Obstacle footprints (ground truth) are excluded from the coverable area. They are used
only for evaluation; the robot itself never sees ground truth.
"""

import math

import numpy as np


class CoverageGrid:
    def __init__(self, field, tool_width=None, res=0.05):
        self.f = field
        self.res = res
        self.tool_width = tool_width or field.tool_width
        self.nx = int(round(field.width / res))
        self.ny = int(round(field.height / res))
        xs = field.x_min + (np.arange(self.nx) + 0.5) * res
        ys = field.y_min + (np.arange(self.ny) + 0.5) * res
        self.X, self.Y = np.meshgrid(xs, ys)          # shape (ny, nx)
        self.free = np.ones((self.ny, self.nx), bool)
        for o in field.obstacles:
            self.free &= (self.X - o.x) ** 2 + (self.Y - o.y) ** 2 > o.r ** 2
        self.count = np.zeros((self.ny, self.nx), np.int16)
        self._pass = np.zeros((self.ny, self.nx), bool)
        self._pass_id = None

    def _flush(self):
        self.count += self._pass
        self._pass[:] = False

    def mark(self, p0, p1, pass_id):
        """Mark the capsule of radius tool_width/2 around segment p0 -> p1."""
        if pass_id != self._pass_id:
            self._flush()
            self._pass_id = pass_id
        h = self.tool_width / 2.0
        f, res = self.f, self.res
        x0 = min(p0[0], p1[0]) - h
        x1 = max(p0[0], p1[0]) + h
        y0 = min(p0[1], p1[1]) - h
        y1 = max(p0[1], p1[1]) + h
        i0 = max(0, int((x0 - f.x_min) / res))
        i1 = min(self.nx, int((x1 - f.x_min) / res) + 1)
        j0 = max(0, int((y0 - f.y_min) / res))
        j1 = min(self.ny, int((y1 - f.y_min) / res) + 1)
        if i0 >= i1 or j0 >= j1:
            return
        X = self.X[j0:j1, i0:i1]
        Y = self.Y[j0:j1, i0:i1]
        ax, ay = p0
        dx, dy = p1[0] - ax, p1[1] - ay
        l2 = dx * dx + dy * dy
        if l2 < 1e-12:
            d2 = (X - ax) ** 2 + (Y - ay) ** 2
        else:
            t = np.clip(((X - ax) * dx + (Y - ay) * dy) / l2, 0.0, 1.0)
            d2 = (X - ax - t * dx) ** 2 + (Y - ay - t * dy) ** 2
        self._pass[j0:j1, i0:i1] |= d2 <= h * h

    def covered_mask(self):
        return (self.count + self._pass) > 0

    def stats(self):
        cnt = self.count + self._pass
        free = self.free
        covered = (cnt > 0) & free
        n_free = int(free.sum())
        n_cov = int(covered.sum())
        n_over = int(((cnt > 1) & free).sum())
        return {
            "coverage_pct": 100.0 * n_cov / max(1, n_free),
            "overlap_pct": 100.0 * n_over / max(1, n_cov),
            "covered_m2": n_cov * self.res ** 2,
            "missed_m2": (n_free - n_cov) * self.res ** 2,
            "coverable_m2": n_free * self.res ** 2,
        }


class RunMetrics:
    """Accumulates path length, time, clearance, collisions and tracking error."""

    def __init__(self, field, robot_radius=0.30):
        self.f = field
        self.robot_radius = robot_radius
        self.time = 0.0
        self.path_len = 0.0
        self.min_clearance = float("inf")
        self.collisions = 0
        self._in_contact = False
        self.xte = []
        self.prev = None
        self.spray_time = 0.0

    def update(self, pose, dt, xte=None, spraying=False):
        x, y, _ = pose
        self.time += dt
        if self.prev is not None:
            self.path_len += math.hypot(x - self.prev[0], y - self.prev[1])
        self.prev = (x, y)
        if spraying:
            self.spray_time += dt
        if xte is not None:
            self.xte.append(abs(xte))
        c = min((o.distance_to(x, y) - self.robot_radius for o in self.f.obstacles),
                default=float("inf"))
        self.min_clearance = min(self.min_clearance, c)
        contact = c < 0.0
        if contact and not self._in_contact:
            self.collisions += 1
        self._in_contact = contact

    def summary(self):
        xte = np.array(self.xte) if self.xte else np.zeros(1)
        return {
            "time_s": self.time,
            "path_m": self.path_len,
            "min_clearance_m": self.min_clearance,
            "collisions": self.collisions,
            "xte_mean_cm": 100.0 * float(xte.mean()),
            "xte_max_cm": 100.0 * float(xte.max()),
            "xte_rms_cm": 100.0 * float(np.sqrt((xte ** 2).mean())),
        }
