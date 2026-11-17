"""Boustrophedon (ox-plough) coverage path planner with headland turns.

Owner: M2 (coverage path planning).

The field is covered lane by lane in alternating directions. Between two lanes the
robot turns in the headland, using one of two manoeuvres:

    "arc"   - a semicircular U-turn of radius lane_spacing / 2 (continuous motion)
    "pivot" - drive to the lane end, rotate 90 deg in place, side-step, rotate again

The result is a dense `Path` (points every `ds` metres) with one tag per point:
    lane[i]   >= 0  index of the lane the point belongs to (spraying allowed)
              == -1 transit or headland turn (sprayer off)
    corner[i]       True at pivot vertices (tracker reduces its look-ahead there)
    detour[i]       True for points inserted later by the obstacle-avoidance layer
"""

import math

import numpy as np

from .geometry import polyline_length, resample_polyline


class Path:
    def __init__(self, pts, lane, corner=None, detour=None):
        self.pts = np.asarray(pts, float)
        self.lane = np.asarray(lane, int)
        n = len(self.pts)
        self.corner = np.zeros(n, bool) if corner is None else np.asarray(corner, bool)
        self.detour = np.zeros(n, bool) if detour is None else np.asarray(detour, bool)

    def __len__(self):
        return len(self.pts)

    @property
    def length(self):
        return polyline_length(self.pts)

    def splice(self, i0, i1, new_pts, lane_id, detour=True):
        """Replace points [i0, i1) with new_pts (used by the detour planner)."""
        new_pts = np.asarray(new_pts, float).reshape(-1, 2)
        if len(new_pts) > 1:      # drop duplicate consecutive points
            keep = np.ones(len(new_pts), bool)
            keep[1:] = np.linalg.norm(np.diff(new_pts, axis=0), axis=1) > 1e-3
            new_pts = new_pts[keep]
        k = len(new_pts)
        self.pts = np.vstack([self.pts[:i0], new_pts, self.pts[i1:]])
        self.lane = np.concatenate([self.lane[:i0], np.full(k, lane_id), self.lane[i1:]])
        self.corner = np.concatenate([self.corner[:i0], np.zeros(k, bool), self.corner[i1:]])
        self.detour = np.concatenate([self.detour[:i0], np.full(k, detour), self.detour[i1:]])
        # keep the path free of repeated points (the tracker needs distinct points)
        keep = np.ones(len(self.pts), bool)
        keep[1:] = np.linalg.norm(np.diff(self.pts, axis=0), axis=1) > 1e-3
        if not keep.all():
            self.pts, self.lane = self.pts[keep], self.lane[keep]
            self.corner, self.detour = self.corner[keep], self.detour[keep]

    def copy(self):
        return Path(self.pts.copy(), self.lane.copy(), self.corner.copy(), self.detour.copy())


def _arc(cx, cy, r, a0, a1, ds):
    n = max(4, int(math.ceil(abs(a1 - a0) * r / ds)))
    a = np.linspace(a0, a1, n + 1)
    return np.column_stack([cx + r * np.cos(a), cy + r * np.sin(a)])


def plan_coverage_path(field, turn="arc", spacing=None, ds=0.2, overshoot=0.3):
    """Plan the full boustrophedon path from the start pose over all lanes.

    overshoot: how far beyond the crop edge each lane is extended so the boom
    covers the row ends completely before the robot turns.
    """
    if turn not in ("arc", "pivot"):
        raise ValueError("turn must be 'arc' or 'pivot'")
    lanes = field.lanes(spacing)
    xa = field.x_min - overshoot
    xb = field.x_max + overshoot
    sx, sy, _ = field.start

    pts, tags, corners = [], [], []

    def add(segment, lane_id, corner_last=False):
        seg = np.asarray(segment, float)
        if pts:
            seg = seg[1:] if np.allclose(seg[0], pts[-1]) else seg
        for i, p in enumerate(seg):
            pts.append(p)
            tags.append(lane_id)
            corners.append(corner_last and i == len(seg) - 1)

    # transit from the start pose to the beginning of lane 0
    add(resample_polyline([(sx, sy), (xa, lanes[0])], ds), -1)

    for k, y in enumerate(lanes):
        forward = (k % 2 == 0)
        x0, x1 = (xa, xb) if forward else (xb, xa)
        add(resample_polyline([(x0, y), (x1, y)], ds), k, corner_last=(turn == "pivot"))
        if k == len(lanes) - 1:
            break
        y_next = lanes[k + 1]
        if turn == "arc":
            r = (y_next - y) / 2.0
            cy = y + r
            if forward:      # bulge towards +x: angles -90 deg -> +90 deg
                seg = _arc(x1, cy, r, -math.pi / 2, math.pi / 2, ds)
            else:            # bulge towards -x: angles -90 deg -> -270 deg
                seg = _arc(x1, cy, r, -math.pi / 2, -3 * math.pi / 2, ds)
            add(seg, -1)
        else:
            add(resample_polyline([(x1, y), (x1, y_next)], ds), -1, corner_last=True)

    return Path(pts, tags, corners)


def path_statistics(path):
    """Length on lanes, length in turns/transit, number of lanes."""
    p = path.pts
    seg = np.linalg.norm(np.diff(p, axis=0), axis=1)
    on_lane = (path.lane[1:] >= 0) & (path.lane[:-1] >= 0)
    return {
        "total_m": float(seg.sum()),
        "lane_m": float(seg[on_lane].sum()),
        "turn_m": float(seg[~on_lane].sum()),
        "lanes": int(path.lane.max() + 1),
    }
