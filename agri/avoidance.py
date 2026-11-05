"""Lidar perception and obstacle avoidance.

Owner: M4 (perception and obstacle avoidance).

Pipeline for every lidar scan
    1. ranges -> points in the world frame (using the robot pose)
    2. split the scan into clusters wherever consecutive points jump > `gap`
    3. fit a circle to each cluster (chord width + push the centroid away from the robot)
    4. fuse the circles into a persistent ObstacleMap (confirmed after `min_hits` scans)

Two avoidance strategies are compared in the report:

    GeometricDetour  (proposed)  every confirmed obstacle that blocks the planned lane is
                     cut out of the path and replaced by a circular arc of radius
                     R = r_obs + robot_radius + margin on the free side. The arc ends on the
                     same lane, so the boustrophedon pattern is kept and no strip is missed.

    ReactiveAPF      (baseline)  classic artificial potential field: the look-ahead target
                     attracts, every lidar point closer than d0 repels. The resulting
                     force direction is used as the heading command.

Both use the same reflex layer (emergency stop and back-off when something is too close).
"""

import math

import numpy as np

from .geometry import segment_circle_entry, wrap_angle


# ====================================================================== perception
def filter_low_returns(ranges, angles, roll, pitch, lidar_height=0.35, min_height=0.27):
    """Remove returns that hit below min_height (crop tops, ground) when the body tilts.

    Hit height of a horizontal ray at bearing a after roll/pitch (Z-Y-X convention,
    positive pitch = nose down):  z = h - r sin(p) cos(a) + r cos(p) sin(r_) sin(a)
    """
    r = np.asarray(ranges, float).copy()
    a = np.asarray(angles, float)
    with np.errstate(invalid="ignore"):
        z = lidar_height + r * (-math.sin(pitch) * np.cos(a) + math.cos(pitch) * math.sin(roll) * np.sin(a))
    r[np.isfinite(r) & (z < min_height)] = np.inf
    return r


def remove_isolated(ranges, rel_tol=0.06, abs_tol=0.08):
    """Drop 'mixed pixel' ghost returns: a ray is kept only if a neighbouring ray
    reports a similar range. Real stones and poles always span two or more rays
    inside the planning horizon; edge ghosts are single, isolated returns."""
    r = np.asarray(ranges, float)
    fin = np.isfinite(r)
    tol = abs_tol + rel_tol * np.where(fin, r, 0.0)
    left = np.zeros_like(fin)
    right = np.zeros_like(fin)
    with np.errstate(invalid="ignore"):
        left[1:] = fin[1:] & fin[:-1] & (np.abs(r[1:] - r[:-1]) < tol[1:])
        right[:-1] = fin[:-1] & fin[1:] & (np.abs(r[:-1] - r[1:]) < tol[:-1])
    out = r.copy()
    out[fin & ~(left | right)] = np.inf
    return out


def scan_to_points(ranges, angles, pose, max_range):
    """Convert a scan to world-frame points. Returns (points Nx2, mask of valid rays)."""
    r = np.asarray(ranges, float)
    valid = np.isfinite(r) & (r > 0.05) & (r < max_range * 0.98)
    x, y, th = pose
    a = th + np.asarray(angles, float)
    pts = np.column_stack([x + r * np.cos(a), y + r * np.sin(a)])
    return pts, valid


def cluster_scan(pts, valid, gap=0.3):
    """Split consecutive valid points into clusters (lists of index arrays)."""
    clusters, cur = [], []
    prev = None
    for i in range(len(pts)):
        if not valid[i]:
            if cur:
                clusters.append(cur)
            cur, prev = [], None
            continue
        if prev is not None and np.linalg.norm(pts[i] - pts[prev]) > gap:
            clusters.append(cur)
            cur = []
        cur.append(i)
        prev = i
    if cur:
        clusters.append(cur)
    return [np.array(c) for c in clusters]


def fit_circle(cluster_pts, robot_xy, ang_res, rng):
    """Approximate a cluster by a circle (cx, cy, r)."""
    p = cluster_pts
    width = np.linalg.norm(p[-1] - p[0]) + ang_res * rng   # add one ray footprint
    r = max(0.05, 0.5 * width)
    c = p.mean(axis=0)
    away = c - robot_xy
    n = np.linalg.norm(away)
    if n > 1e-6:
        c = c + away / n * r * 0.8
    return float(c[0]), float(c[1]), float(r)


class MappedObstacle:
    _next_id = 0

    def __init__(self, x, y, r):
        self.x, self.y, self.r = x, y, r
        self.r_max = r
        self.hits = 1
        self.id = MappedObstacle._next_id
        MappedObstacle._next_id += 1

    def update(self, x, y, r, alpha=0.2):
        self.x += alpha * (x - self.x)
        self.y += alpha * (y - self.y)
        self.r += alpha * (r - self.r)
        self.r_max = max(self.r_max, self.r)
        self.hits += 1

    @property
    def radius(self):
        """Conservative radius used for planning (largest filtered estimate)."""
        return max(self.r, self.r_max)


class ObstacleMap:
    """Persistent map of circular obstacles built from lidar clusters."""

    def __init__(self, field, max_range=8.0, min_hits=8, gap=0.3, ang_res=math.pi / 180):
        self.field = field
        self.max_range = max_range
        self.min_hits = min_hits
        self.gap = gap
        self.ang_res = ang_res
        self.obstacles = []
        self.last_points = np.zeros((0, 2))

    def update(self, ranges, angles, pose):
        pts, valid = scan_to_points(ranges, angles, pose, self.max_range)
        # keep only points on the field soil (trees and buildings outside are ignored)
        for i in np.where(valid)[0]:
            if not self.field.in_soil(pts[i, 0], pts[i, 1]):
                valid[i] = False
        self.last_points = pts[valid]
        robot_xy = np.array(pose[:2])
        rng = np.where(valid, np.asarray(ranges, float), 0.0)
        n = len(pts)
        for idx in cluster_scan(pts, valid, self.gap):
            if idx[0] == 0 or idx[-1] == n - 1:
                continue                    # cut by the edge of the field of view
            cx, cy, r = fit_circle(pts[idx], robot_xy, self.ang_res, float(rng[idx].mean()))
            best, bd = None, 1e9
            for o in self.obstacles:
                d = math.hypot(o.x - cx, o.y - cy)
                if d < max(0.5, o.r + r + 0.1) and d < bd:
                    best, bd = o, d
            if best is None:
                self.obstacles.append(MappedObstacle(cx, cy, r))
            else:
                best.update(cx, cy, r)
        self._merge()

    def _merge(self):
        """Merge estimates whose circles overlap (one physical obstacle seen as two)."""
        obs = sorted(self.obstacles, key=lambda o: -o.hits)
        kept = []
        for o in obs:
            host = None
            for k in kept:
                if math.hypot(k.x - o.x, k.y - o.y) < k.radius + o.radius + 0.05:
                    host = k
                    break
            if host is None:
                kept.append(o)
                continue
            if o.hits >= self.min_hits:
                # enclosing circle of both estimates
                d = math.hypot(host.x - o.x, host.y - o.y)
                R = max(host.radius, (d + host.radius + o.radius) / 2.0)
                if R > host.radius and d > 1e-6:
                    t = (R - host.radius) / d
                    host.x += (o.x - host.x) * t
                    host.y += (o.y - host.y) * t
                    host.r = host.r_max = R
            host.hits += o.hits
        self.obstacles = kept

    def confirmed(self):
        return [o for o in self.obstacles if o.hits >= self.min_hits]


def front_clearance(ranges, angles, half_angle=math.radians(30)):
    r = np.asarray(ranges, float)
    a = np.asarray(angles, float)
    m = (np.abs(a) <= half_angle) & np.isfinite(r) & (r > 0.05)
    return float(r[m].min()) if m.any() else float("inf")


# ====================================================================== proposed method
class GeometricDetour:
    """Splices circular detours around mapped obstacles into the coverage path."""

    name = "proposed"

    def __init__(self, field, robot_radius=0.32, margin=0.30, horizon=5.0, ds=0.2):
        self.field = field
        self.robot_radius = robot_radius
        self.margin = margin
        self.horizon = horizon
        self.ds = ds
        self.handled = {}       # obstacle id -> inflated radius used for its detour
        self.n_detours = 0
        self.n_skips = 0

    def inflated(self, o):
        return o.radius + self.robot_radius + self.margin

    def _arc_ok(self, arc, others):
        f = self.field
        for p in arc:
            if not f.in_soil(p[0], p[1], margin=self.robot_radius):
                return False
            for o in others:
                if math.hypot(p[0] - o.x, p[1] - o.y) < o.radius + self.robot_radius:
                    return False
        return True

    def _make_arc(self, c, R, pe, px, ccw):
        a0 = math.atan2(pe[1] - c[1], pe[0] - c[0])
        a1 = math.atan2(px[1] - c[1], px[0] - c[0])
        if ccw:
            while a1 <= a0:
                a1 += 2 * math.pi
        else:
            while a1 >= a0:
                a1 -= 2 * math.pi
        n = max(3, int(math.ceil(abs(a1 - a0) * R / self.ds)))
        a = np.linspace(a0, a1, n + 1)
        return np.column_stack([c[0] + R * np.cos(a), c[1] + R * np.sin(a)])

    def update(self, tracker, omap, pose):
        """Check the path ahead against every confirmed obstacle; splice detours.

        Runs every control step, so an obstacle seen early (e.g. during a headland turn)
        is handled as soon as the path ahead reaches it, and a detour is re-planned
        automatically if the obstacle estimate grows.
        """
        conf = omap.confirmed()
        for o in conf:
            R = self.inflated(o)
            d_robot = math.hypot(o.x - pose[0], o.y - pose[1])
            if d_robot > self.horizon + R or d_robot < R:
                continue        # too far, or already inside the safety circle (reflex layer)
            others = [q for q in conf if q is not o]
            if self._splice(tracker, (o.x, o.y), R, others) and o.id not in self.handled:
                self.handled[o.id] = R
                self.n_detours += 1

    def _splice(self, tracker, c, R, others):
        path = tracker.path
        pts = path.pts
        i_start = tracker.idx + 2
        i_end = min(len(pts), i_start + int(self.horizon / self.ds) + int(2 * R / self.ds) + 4)
        d = np.linalg.norm(pts[i_start:i_end] - np.asarray(c), axis=1)
        inside = np.where(d < R - 0.08)[0]      # tolerance avoids re-planning on jitter
        if len(inside) == 0:
            return False                    # path already clear of this obstacle
        i_in = i_start + inside[0]
        while i_in - 1 > tracker.idx and np.linalg.norm(pts[i_in - 1] - c) < R:
            i_in -= 1
        if np.linalg.norm(pts[i_in - 1] - c) < R:
            return False        # no valid entry point ahead of the robot
        i_out = i_in
        while i_out + 1 < len(pts) and np.linalg.norm(pts[i_out + 1] - c) < R:
            i_out += 1
        if i_in == 0 or i_out + 1 >= len(pts):
            return False
        t = segment_circle_entry(pts[i_in - 1], pts[i_in], c, R)
        pe = pts[i_in - 1] + (pts[i_in] - pts[i_in - 1]) * (t if t is not None else 0.0)
        t = segment_circle_entry(pts[i_out + 1], pts[i_out], c, R)
        px = pts[i_out + 1] + (pts[i_out] - pts[i_out + 1]) * (t if t is not None else 0.0)

        candidates = []
        for ccw in (True, False):
            arc = self._make_arc(c, R, pe, px, ccw)
            if self._arc_ok(arc, others):
                candidates.append(arc)
        lane_id = int(path.lane[i_in])
        if not candidates:
            # no safe side: drop the blocked part of the lane (recorded as a skip)
            path.splice(i_in, i_out + 1, np.array([pe, px]), lane_id)
            self.n_skips += 1
            return True
        arc = min(candidates, key=len)      # shorter side
        path.splice(i_in, i_out + 1, arc, lane_id)
        return True


# ====================================================================== baseline method
class ReactiveAPF:
    """Artificial potential field with a vortex (tangential) term.

    F = F_att + F_rep + F_vortex
        F_att    = unit vector to the look-ahead target on the planned lane
        F_rep    = k_rep (1/d - 1/d0) / d^2 * (-u)       for the closest lidar return (d < d0)
        F_vortex = |F_rep| * (u rotated +/-90 deg)       slides the robot around the obstacle
                                                         (the usual fix for APF local minima)
    The heading of F replaces the path-tracking heading while an obstacle is within d0.
    """

    name = "apf"

    def __init__(self, d0=1.5, k_rep=0.4, stall_time=6.0):
        self.d0 = d0
        self.k_rep = k_rep
        self.stall_time = stall_time
        self.n_detours = 0
        self.n_skips = 0
        self._active = False
        self._side = 0.0
        self._idle = 0.0
        self._last_heading = None
        self._stall = 0.0
        self._last_idx = 0

    def heading(self, tracker, ranges, angles, pose, dt):
        """Return a heading override (world frame) or None when no obstacle is near."""
        x, y, th = pose
        tgt = tracker.target
        att = np.array([tgt[0] - x, tgt[1] - y])
        n = np.linalg.norm(att)
        att = att / n if n > 1e-6 else np.array([math.cos(th), math.sin(th)])
        r = np.asarray(ranges, float)
        ang = np.asarray(angles, float)
        m = np.isfinite(r) & (r > 0.05) & (r < self.d0) & (np.abs(ang) < math.radians(80))
        if not m.any():
            self._idle += dt
            if self._idle > 1.0:              # obstacle left behind: release the side lock
                self._active = False
                self._stall = 0.0
            # keep the last avoidance heading for a moment (the obstacle may just have
            # left the field of view while the robot turns)
            return self._last_heading if self._active else None
        self._idle = 0.0
        i = np.where(m)[0][np.argmin(r[m])]
        d, a_rel = r[i], ang[i]
        if not self._active:
            self.n_detours += 1
            # choose the passing side once (hysteresis) to avoid left/right chattering
            self._side = -1.0 if a_rel >= 0 else 1.0   # obstacle on the left -> pass right
        self._active = True
        u = np.array([math.cos(th + a_rel), math.sin(th + a_rel)])
        mag = self.k_rep * (1.0 / d - 1.0 / self.d0) / d ** 2
        tangent = np.array([-u[1], u[0]]) * self._side
        force = att - mag * u + mag * tangent
        # stall detection (no progress along the path): give up this part of the lane
        if tracker.idx <= self._last_idx + 1:
            self._stall += dt
        else:
            self._stall = 0.0
            self._last_idx = tracker.idx
        if self._stall > self.stall_time:
            self._skip_past(tracker, np.array([x, y]) + d * u)
            self.n_skips += 1
            self._stall = 0.0
        self._last_heading = math.atan2(force[1], force[0])
        return self._last_heading

    @staticmethod
    def _skip_past(tracker, obs_pt, clear=1.2):
        """Move the path index just beyond the blocking obstacle (part of the lane is lost)."""
        pts = tracker.path.pts
        j0 = tracker.idx
        j1 = min(len(pts), j0 + 60)
        j = j0 + int(np.argmin(np.linalg.norm(pts[j0:j1] - obs_pt, axis=1)))
        while j < len(pts) - 1 and np.linalg.norm(pts[j] - obs_pt) < clear:
            j += 1
        tracker.idx = j


# ====================================================================== baseline 2: Bug2
class Bug2Follower:
    """Bug2 boundary following with the current lane as the M-line.

    NORMAL  follow the lane with pure pursuit.
    FOLLOW  triggered when the front clearance drops below d_trigger. The robot keeps the
            obstacle on one side at distance d_wall (wall-following P-law on the closest
            lidar return) until it is back on the M-line (|cross-track| < leave_tol) at a
            point further along the lane than the hit point; then it leaves the boundary.
    """

    name = "bug2"

    def __init__(self, field, d_trigger=0.9, d_wall=0.75, k_wall=1.2, leave_tol=0.15,
                 stall_time=10.0):
        self.field = field
        self.d_trigger = d_trigger
        self.d_wall = d_wall
        self.k_wall = k_wall
        self.leave_tol = leave_tol
        self.stall_time = stall_time
        self.n_detours = 0
        self.n_skips = 0
        self.following = False
        self._side = 1.0
        self._hit = None
        self._lane_y = None
        self._dir = 1.0
        self._t = 0.0
        self._prev_off = None

    def heading(self, tracker, ranges, angles, pose, dt):
        x, y, th = pose
        r = np.asarray(ranges, float)
        a = np.asarray(angles, float)
        fin = np.isfinite(r) & (r > 0.05)
        lane = int(tracker.path.lane[tracker.idx])
        if not self.following:
            front = fin & (np.abs(a) < math.radians(40)) & (r < self.d_trigger)
            if not front.any() or lane < 0:
                return None
            i = np.where(front)[0][np.argmin(r[front])]
            # obstacle on the left -> keep it on the left (turn right) and vice versa
            self._side = 1.0 if a[i] >= 0 else -1.0
            self.following = True
            self._hit = (x, y)
            self._lane_y = self.field.lanes()[lane]
            tgt = tracker.target
            self._dir = 1.0 if tgt[0] >= x else -1.0      # lane direction along x
            self._t = 0.0
            self._prev_off = None
            self.n_detours += 1
        self._t += dt
        # leave condition: M-line reached (or crossed) at a point past the hit point
        off = y - self._lane_y
        crossed = self._prev_off is not None and off * self._prev_off < 0
        self._prev_off = off
        progressed = (x - self._hit[0]) * self._dir > 0.6
        if progressed and (abs(off) < self.leave_tol or crossed):
            self.following = False
            return None
        if self._t > self.stall_time * 4:                 # give up this obstacle
            self.following = False
            self.n_skips += 1
            ReactiveAPF._skip_past(tracker, np.array([x, y]) + np.array([self._dir, 0.0]))
            return None
        near = fin & (np.abs(a) < math.radians(89))
        if near.any() and r[near].min() < self.d_wall * 1.8:
            i = np.where(near)[0][np.argmin(r[near])]
            d, ab = r[i], th + a[i]
            # tangent direction keeping the obstacle on self._side, P-correction on distance
            tangent = ab - self._side * math.pi / 2
            corr = self.k_wall * (d - self.d_wall)
            return wrap_angle(tangent + self._side * max(-0.8, min(0.8, corr)))
        # obstacle behind / out of view: head back to the M-line at 45 deg
        to_lane = math.copysign(1.0, self._lane_y - y)
        base = 0.0 if self._dir > 0 else math.pi
        return wrap_angle(base + to_lane * self._dir * math.radians(45))
