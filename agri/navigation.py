"""Path tracking: pure-pursuit target selection + PID heading controller.

Owner: M3 (navigation and control).

Control law (unicycle model, v forward speed, w yaw rate):

    e_theta = wrap(atan2(y_t - y, x_t - x) - theta)        heading error to look-ahead target
    w       = Kp e + Ki integral(e) + Kd de/dt             PID, output limited to +/- w_max
    v       = v_max * s_obs * s_turn * max(0, cos e)       speed scheduling
    if |e| > pivot_threshold:  v = 0                        rotate in place (skid steer)

Speed scheduling (s_obs, s_turn) slows the robot near obstacles and in headland turns,
which reduces overshoot and keeps the boom on the row.
"""

import math

import numpy as np

from .geometry import wrap_angle


class PID:
    def __init__(self, kp, ki=0.0, kd=0.0, out_limit=None, i_limit=None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit = out_limit
        self.i_limit = i_limit
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev = None

    def step(self, err, dt):
        self.integral += err * dt
        if self.i_limit is not None:
            self.integral = max(-self.i_limit, min(self.i_limit, self.integral))
        deriv = 0.0 if self.prev is None or dt <= 0 else (err - self.prev) / dt
        self.prev = err
        u = self.kp * err + self.ki * self.integral + self.kd * deriv
        if self.out_limit is not None:
            u = max(-self.out_limit, min(self.out_limit, u))
        return u


class PathTracker:
    """Follows a `planner.Path` and keeps a monotonic progress index."""

    def __init__(self, path, v_max=0.5, w_max=1.6, lookahead=0.6,
                 corner_lookahead=0.25, detour_lookahead=0.35, pivot_threshold=math.radians(55),
                 kp=2.2, ki=0.05, kd=0.12, v_turn=0.3):
        self.path = path
        self.v_max = v_max
        self.w_max = w_max
        self.lookahead = lookahead
        self.corner_lookahead = corner_lookahead
        self.detour_lookahead = detour_lookahead   # short look-ahead: pure pursuit cuts arcs
        self.pivot_threshold = pivot_threshold
        self.v_turn = v_turn
        self.pid = PID(kp, ki, kd, out_limit=w_max, i_limit=0.5)
        self.idx = 0
        self.pivoting = False
        self._pivot_dir = 1.0
        self.done = False
        self.heading_error = 0.0
        self.target = path.pts[0]

    # ------------------------------------------------------------------ helpers
    def _nearest(self, pos, window_m=2.5):
        pts = self.path.pts
        n_win = int(window_m / 0.2) + 2
        j1 = min(len(pts), self.idx + n_win)
        d = np.linalg.norm(pts[self.idx:j1] - pos, axis=1)
        return self.idx + int(np.argmin(d))

    def _near_corner(self, i, n=4):
        c = self.path.corner
        return bool(c[max(0, i - 1):min(len(c), i + n)].any())

    def cross_track_error(self, pos):
        """Signed distance to the local path segment (left positive)."""
        pts = self.path.pts
        i = min(self.idx, len(pts) - 2)
        a, b = pts[i], pts[i + 1]
        t = b - a
        nrm = np.linalg.norm(t)
        if nrm < 1e-9:
            return 0.0
        return float(np.cross(t / nrm, pos - a))

    def skip_ahead(self, metres):
        self.idx = min(len(self.path) - 1, self.idx + int(metres / 0.2))

    # ------------------------------------------------------------------ main
    def target_point(self, pos):
        self.idx = self._nearest(pos)
        if self._near_corner(self.idx):
            L = self.corner_lookahead
        elif self.path.detour[self.idx:self.idx + 4].any():
            L = self.detour_lookahead
        else:
            L = self.lookahead
        pts = self.path.pts
        j = self.idx
        while j < len(pts) - 1 and np.linalg.norm(pts[j] - pos) < L:
            if self.path.corner[j] and j > self.idx:
                break   # never cut a pivot corner
            j += 1
        self.target = pts[j]
        return self.target

    def update(self, pose, dt, speed_scale=1.0, heading_override=None):
        """Return (v, w). heading_override lets an avoidance layer steer instead."""
        x, y, th = pose
        pos = np.array([x, y])
        tgt = self.target_point(pos)
        if self.idx >= len(self.path) - 2 and np.linalg.norm(self.path.pts[-1] - pos) < 0.3:
            self.done = True
            return 0.0, 0.0

        if heading_override is None:
            desired = math.atan2(tgt[1] - y, tgt[0] - x)
        else:
            desired = heading_override
        e = wrap_angle(desired - th)
        self.heading_error = e

        # hysteresis: start pivoting above threshold, stop when well aligned.
        # While an avoidance layer steers, keep rolling (APF needs forward motion).
        threshold = self.pivot_threshold if heading_override is None else math.radians(120)
        if abs(e) > threshold:
            if not self.pivoting:
                self._pivot_dir = math.copysign(1.0, e)   # lock the turn direction
            self.pivoting = True
        elif abs(e) < math.radians(15):
            self.pivoting = False

        w = self.pid.step(e, dt)
        if self.pivoting:
            # near +/-180 deg the sign of e is ambiguous: keep the locked direction
            if abs(e) < math.radians(150):
                self._pivot_dir = math.copysign(1.0, e)
            return 0.0, self._pivot_dir * max(abs(w), 0.6)

        s_turn = 1.0
        if self.path.lane[self.idx] < 0 or self._near_corner(self.idx, 6):
            s_turn = self.v_turn / self.v_max
        if heading_override is None:
            v = self.v_max * speed_scale * s_turn * max(0.0, math.cos(e))
        else:
            v = self.v_max * speed_scale * s_turn * max(0.25, math.cos(e))
        return v, w
