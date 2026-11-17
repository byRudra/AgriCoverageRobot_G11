"""Small 2-D geometry helpers shared by all modules."""

import math

import numpy as np


def wrap_angle(a):
    """Wrap an angle to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def point_segment_distance(p, a, b):
    """Distance from point p to segment ab (all 2-tuples / arrays)."""
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    l2 = dx * dx + dy * dy
    if l2 < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / l2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def segment_circle_entry(a, b, c, r):
    """Parameter t in [0, 1] where segment a->b first enters circle (c, r), or None."""
    a = np.asarray(a, float)
    d = np.asarray(b, float) - a
    f = a - np.asarray(c, float)
    A = d @ d
    if A < 1e-12:
        return None
    B = 2.0 * (f @ d)
    C = f @ f - r * r
    disc = B * B - 4 * A * C
    if disc < 0:
        return None
    s = math.sqrt(disc)
    for t in ((-B - s) / (2 * A), (-B + s) / (2 * A)):
        if 0.0 <= t <= 1.0:
            return t
    return None


def polyline_length(pts):
    pts = np.asarray(pts, float)
    if len(pts) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))


def resample_polyline(pts, ds):
    """Resample a polyline so consecutive points are about ds apart (keeps vertices)."""
    pts = np.asarray(pts, float)
    out = [pts[0]]
    for a, b in zip(pts[:-1], pts[1:]):
        seg = np.linalg.norm(b - a)
        n = max(1, int(math.ceil(seg / ds)))
        for k in range(1, n + 1):
            out.append(a + (b - a) * k / n)
    return np.array(out)
