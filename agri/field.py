"""Field model: crop area, crop rows, headlands and obstacles (stones, poles).

Owner: M1 (field and world design).

World frame (same as Webots ENU): x east, y north, z up, metres.
The crop area is the rectangle that must be sprayed. Crop rows run along x.
The robot drives in the inter-row lanes, whose centres are `lane_spacing` apart.
A headland strip of soil surrounds the crop area so the robot can turn.
"""

import json
import math
import random
from dataclasses import asdict, dataclass, field as dc_field
from typing import List


@dataclass
class Obstacle:
    x: float
    y: float
    r: float            # footprint radius at ground level [m]
    kind: str = "stone"  # "stone" or "pole"
    height: float = 0.5

    def distance_to(self, px, py):
        """Distance from a point to the obstacle surface (negative inside)."""
        return math.hypot(px - self.x, py - self.y) - self.r


@dataclass
class Field:
    x_min: float = -8.0
    x_max: float = 8.0
    y_min: float = -5.0
    y_max: float = 5.0
    headland: float = 3.0        # soil strip around the crop area [m]
    lane_spacing: float = 1.0    # distance between lane centres = crop-row spacing [m]
    tool_width: float = 1.2      # sprayer boom width [m]
    start: tuple = (-10.0, -4.5, 0.0)   # robot start pose (x, y, yaw)
    obstacles: List[Obstacle] = dc_field(default_factory=list)
    seed: int = 0

    # ------------------------------------------------------------------ geometry
    @property
    def width(self):
        return self.x_max - self.x_min

    @property
    def height(self):
        return self.y_max - self.y_min

    @property
    def area(self):
        return self.width * self.height

    @property
    def soil_bounds(self):
        h = self.headland
        return (self.x_min - h, self.x_max + h, self.y_min - h, self.y_max + h)

    def lanes(self, spacing=None):
        """y-coordinates of lane centres, first lane half a spacing inside the edge."""
        s = spacing or self.lane_spacing
        ys = []
        y = self.y_min + s / 2.0
        while y < self.y_max - 1e-9:
            ys.append(round(y, 6))
            y += s
        return ys

    def crop_rows(self):
        """y-coordinates of crop rows (between and outside the lanes)."""
        s = self.lane_spacing
        n = int(round(self.height / s))
        return [round(self.y_min + k * s, 6) for k in range(n + 1)]

    def in_crop(self, x, y):
        return self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max

    def in_soil(self, x, y, margin=0.0):
        x0, x1, y0, y1 = self.soil_bounds
        return x0 + margin <= x <= x1 - margin and y0 + margin <= y <= y1 - margin

    # ------------------------------------------------------------------ I/O
    def to_dict(self):
        d = asdict(self)
        d["start"] = list(self.start)
        return d

    @classmethod
    def from_dict(cls, d):
        d = dict(d)
        d["obstacles"] = [Obstacle(**o) for o in d.get("obstacles", [])]
        d["start"] = tuple(d.get("start", (-10.0, -4.5, 0.0)))
        return cls(**d)

    def save(self, path):
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=2)

    @classmethod
    def load(cls, path):
        with open(path) as f:
            return cls.from_dict(json.load(f))


# ---------------------------------------------------------------------- generator
STONE_R = (0.30, 0.42)
POLE_R = (0.06, 0.10)


def random_field(seed=0, n_stones=6, n_poles=4, lane_bias=0.7, **kwargs):
    """Random field with stones and poles.

    lane_bias is the probability that an obstacle is placed on a lane centre line,
    i.e. directly in the robot's way (the hard case for coverage).
    """
    rng = random.Random(seed)
    f = Field(seed=seed, **kwargs)
    lanes = f.lanes()
    placed = []

    def ok(x, y, r):
        for o in placed:
            if math.hypot(o.x - x, o.y - y) < o.r + r + 1.6:
                return False
        return True

    def place(kind, rr, height):
        for _ in range(500):
            r = rng.uniform(*rr)
            x = rng.uniform(f.x_min + 1.5, f.x_max - 1.5)
            if rng.random() < lane_bias:
                y = rng.choice(lanes) + rng.uniform(-0.15, 0.15)
            else:
                y = rng.uniform(f.y_min + 0.5, f.y_max - 0.5)
            if ok(x, y, r):
                o = Obstacle(round(x, 3), round(y, 3), round(r, 3), kind, height)
                placed.append(o)
                return

    for _ in range(n_stones):
        place("stone", STONE_R, 0.55)
    for _ in range(n_poles):
        place("pole", POLE_R, 1.6)
    f.obstacles = placed
    return f


def default_field():
    """The field used for the Webots world and the demo video."""
    return random_field(seed=11, n_stones=6, n_poles=4)
