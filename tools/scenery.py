"""Realistic scenery for the crop-field world (owner: M1, used by generate_world.py).

Everything inside the soil plot is either below the lidar plane (crops, weeds, pebbles: visual
only, no bounding object) or one of the field's obstacles, whose footprint radius at sensor
height is kept exactly equal to the radius in field_config.json. The farmyard (fence, barn,
silo, tractor, hay bales, trees) stands outside the soil plot, where lidar returns are ignored.
"""

import math
import random

import numpy as np

W = "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects"
SCENERY_PROTOS = [
    W + "/appearances/protos/RoughOak.proto",
    W + "/appearances/protos/DryMud.proto",
    W + "/objects/buildings/protos/Barn.proto",
    W + "/objects/buildings/protos/Silo.proto",
    W + "/objects/street_furniture/protos/Fence.proto",
    W + "/objects/garden/protos/Wheelbarrow.proto",
    W + "/objects/garden/protos/WateringCan.proto",
    W + "/vehicles/protos/generic/Tractor.proto",
]


# ====================================================================== soil texture
def make_soil_texture(field, path, size=(2048, 2048)):
    """Tilled soil: raised crop beds, wheel ruts in every lane, clods, moisture patches."""
    from PIL import Image, ImageFilter
    x0, x1, y0, y1 = field.soil_bounds
    w, h = size
    rng = np.random.default_rng(7)

    def smooth_noise(cells_x, cells_y):
        n = rng.normal(0, 1, (cells_y, cells_x))
        im = Image.fromarray(((n + 3) * 42).clip(0, 255).astype(np.uint8)).resize((w, h), Image.BICUBIC)
        return np.asarray(im, float) / 255.0 - 0.5

    ys = y1 - (np.arange(h) + 0.5) / h * (y1 - y0)          # image row 0 = north edge
    xs = x0 + (np.arange(w) + 0.5) / w * (x1 - x0)
    X, Y = np.meshgrid(xs, ys)
    in_crop = (X >= field.x_min - 0.3) & (X <= field.x_max + 0.3) & (Y >= field.y_min - 0.4) & (Y <= field.y_max + 0.4)

    shade = np.zeros((h, w))
    # raised beds under the crop rows: lit top, shadowed north flank
    for yr in field.crop_rows():
        d = Y - yr
        shade += np.where(in_crop, 0.12 * np.exp(-(d / 0.22) ** 2) - 0.05 * np.exp(-((d - 0.25) / 0.08) ** 2), 0)
    # wheel ruts (track 0.4 m) along each lane, compacted and darker
    for yl in field.lanes():
        for off in (-0.2, 0.2):
            d = Y - (yl + off)
            shade -= np.where(in_crop, 0.09 * np.exp(-(d / 0.07) ** 2), 0)
    # headland: compacted, slightly greyer, with turning marks
    headland = ~in_crop
    moisture = smooth_noise(12, 9) * 0.35 + smooth_noise(60, 44) * 0.18
    fine = rng.normal(0, 0.07, (h, w))
    k = 1.0 + moisture + fine + shade
    base = np.empty((h, w, 3))
    base[...] = (112, 82, 56)
    base[headland] = (124, 98, 74)
    img = base * k[:, :, None]
    # clods: small light and dark blobs
    n_clods = 9000
    cx = rng.integers(0, w, n_clods)
    cy = rng.integers(0, h, n_clods)
    tone = rng.choice([0.72, 0.8, 1.18, 1.25], n_clods)
    for px, py, t in zip(cx, cy, tone):
        r = rng.integers(1, 4)
        img[max(0, py - r):py + r, max(0, px - r):px + r] *= t
    img = img.clip(0, 255).astype(np.uint8)
    im = Image.fromarray(img).filter(ImageFilter.GaussianBlur(0.7))
    im.save(path, quality=92)


# ====================================================================== crops
LEAF_APPS = {
    "LEAF_DARK": "PBRAppearance { baseColor 0.20 0.45 0.12 roughness 0.55 metalness 0 }",
    "LEAF_MID": "PBRAppearance { baseColor 0.32 0.60 0.18 roughness 0.5 metalness 0 }",
    "LEAF_LIGHT": "PBRAppearance { baseColor 0.55 0.78 0.30 roughness 0.45 metalness 0 }",
    "LEAF_BLUE": "PBRAppearance { baseColor 0.30 0.52 0.36 roughness 0.5 metalness 0 }",
}


class _Defs:
    """Emits DEF on the first use of a node name and USE afterwards (VRML order rule)."""

    def __init__(self):
        self.done = set()

    def app(self, name):
        if name in self.done:
            return "USE " + name
        self.done.add(name)
        return "DEF %s %s" % (name, LEAF_APPS[name])

    def geom(self):
        if "LEAF_GEOM" in self.done:
            return "USE LEAF_GEOM"
        self.done.add("LEAF_GEOM")
        return "DEF LEAF_GEOM Sphere { radius 1 subdivision 2 }"


def _leaf(defs, yaw, dist, z, tilt, length, width, appearance):
    """One leaf: a flattened ellipsoid pushed out from the centre and tilted upwards."""
    return ("Transform { rotation 0 0 1 %.3f children [ Transform { translation %.3f 0 %.3f rotation 0 1 0 %.3f "
            "scale %.3f %.3f 0.010 children [ Shape { appearance %s geometry %s } ] } ] }"
            % (yaw, dist, z, -tilt, length, width, defs.app(appearance), defs.geom()))


LETTUCE_SPECS = {
    # name: (outer leaves, outer appearance, inner leaves, inner appearance, heart appearance)
    "LETTUCE_A": (7, "LEAF_DARK", 5, "LEAF_MID", "LEAF_LIGHT"),
    "LETTUCE_B": (8, "LEAF_MID", 6, "LEAF_LIGHT", "LEAF_LIGHT"),
    "LETTUCE_C": (6, "LEAF_BLUE", 5, "LEAF_DARK", "LEAF_MID"),
}


def lettuce_body(defs, name):
    """Leaves of one lettuce variety (rosette, about 0.16 m high)."""
    n_out, a_out, n_in, a_in, a_heart = LETTUCE_SPECS[name]
    rng = random.Random(name)
    leaves = []
    for k in range(n_out):
        yaw = 2 * math.pi * k / n_out + rng.uniform(-0.15, 0.15)
        leaves.append(_leaf(defs, yaw, 0.062, 0.028, rng.uniform(0.35, 0.55), rng.uniform(0.07, 0.085),
                            rng.uniform(0.042, 0.05), a_out))
    for k in range(n_in):
        yaw = 2 * math.pi * (k + 0.5) / n_in + rng.uniform(-0.2, 0.2)
        leaves.append(_leaf(defs, yaw, 0.035, 0.05, rng.uniform(0.9, 1.15), rng.uniform(0.05, 0.06),
                            rng.uniform(0.032, 0.038), a_in))
    leaves.append("Pose { translation 0 0 0.06 children [ Transform { scale 0.035 0.035 0.04 children [ "
                  "Shape { appearance %s geometry %s } ] } ] }" % (defs.app(a_heart), defs.geom()))
    return "DEF %s Group { children [\n  %s\n] }" % (name, "\n  ".join(leaves))


def plant_nodes(field, spacing=0.4, seed=5):
    """Lettuce along every crop row with natural variation and a few gaps (visual only)."""
    rng = random.Random(seed)
    defs = _Defs()
    variants = list(LETTUCE_SPECS)
    defined = set()
    out = []
    for yr in field.crop_rows():
        n = int(field.width / spacing) + 1
        for k in range(n):
            x = field.x_min + k * spacing + rng.uniform(-0.05, 0.05)
            if any(math.hypot(x - o.x, yr - o.y) < o.r + 0.18 for o in field.obstacles):
                continue
            if rng.random() < 0.05:                  # missing plants (failed germination)
                continue
            # the first plants define the three varieties, the rest re-use them
            name = variants[len(defined)] if len(defined) < len(variants) else rng.choice(variants)
            body = "USE " + name if name in defined else lettuce_body(defs, name)
            defined.add(name)
            s = rng.uniform(0.75, 1.15)
            out.append("Transform { translation %.3f %.3f 0 rotation 0 0 1 %.2f scale %.2f %.2f %.2f children [ %s ] }"
                       % (x, yr + rng.uniform(-0.04, 0.04), rng.uniform(0, 6.28), s, s, s * rng.uniform(0.85, 1.1), body))
    return out


WEED = """DEF WEED Group { children [
  Transform { rotation 0.3 0.95 0 0.45 scale 0.006 0.006 0.07 children [ Shape { appearance DEF WEED_APP PBRAppearance { baseColor 0.42 0.58 0.20 roughness 0.7 metalness 0 } geometry DEF WEED_BLADE Cone { bottomRadius 1 height 1 subdivision 5 } } ] }
  Transform { rotation -0.8 0.6 0 0.5 scale 0.006 0.006 0.06 children [ Shape { appearance USE WEED_APP geometry USE WEED_BLADE } ] }
  Transform { rotation 0.9 -0.4 0 0.4 scale 0.005 0.005 0.08 children [ Shape { appearance USE WEED_APP geometry USE WEED_BLADE } ] }
  Transform { rotation -0.2 -0.98 0 0.55 scale 0.006 0.006 0.05 children [ Shape { appearance USE WEED_APP geometry USE WEED_BLADE } ] }
] }"""


def weed_nodes(field, n=140, seed=8):
    rng = random.Random(seed)
    out = []
    for i in range(n):
        x = rng.uniform(field.x_min - 2, field.x_max + 2)
        y = rng.uniform(field.y_min - 2, field.y_max + 2)
        body = WEED if i == 0 else "USE WEED"
        s = rng.uniform(0.7, 1.4)
        out.append("Transform { translation %.2f %.2f 0 rotation 0 0 1 %.2f scale %.2f %.2f %.2f children [ %s ] }"
                   % (x, y, rng.uniform(0, 6.28), s, s, s, body))
    return out


PEBBLE_APPS = ["PBRAppearance { baseColor 0.55 0.52 0.47 roughness 0.9 metalness 0 }",
               "PBRAppearance { baseColor 0.43 0.40 0.36 roughness 0.9 metalness 0 }",
               "PBRAppearance { baseColor 0.64 0.58 0.50 roughness 0.85 metalness 0 }"]


def pebble_nodes(field, n=320, seed=9):
    """Scattered pebbles plus small rubble around each stone (visual only, < 6 cm high)."""
    rng = random.Random(seed)
    x0, x1, y0, y1 = field.soil_bounds
    pts = [(rng.uniform(x0 + 0.3, x1 - 0.3), rng.uniform(y0 + 0.3, y1 - 0.3), rng.uniform(0.012, 0.035)) for _ in range(n)]
    for o in field.obstacles:
        if o.kind != "stone":
            continue
        for _ in range(7):
            a = rng.uniform(0, 2 * math.pi)
            d = o.r + rng.uniform(0.02, 0.35)
            pts.append((o.x + d * math.cos(a), o.y + d * math.sin(a), rng.uniform(0.025, 0.06)))
    out = []
    for i, (x, y, r) in enumerate(pts):
        if i < 3:
            app = "DEF PEBBLE_APP%d %s" % (i, PEBBLE_APPS[i])
            geom = "DEF PEBBLE_GEOM Sphere { radius 1 subdivision 1 }" if i == 0 else "USE PEBBLE_GEOM"
        else:
            app = "USE PEBBLE_APP%d" % rng.randrange(3)
            geom = "USE PEBBLE_GEOM"
        out.append("Transform { translation %.2f %.2f %.3f rotation %.2f %.2f %.2f %.2f scale %.3f %.3f %.3f children [ Shape { appearance %s geometry %s } ] }"
                   % (x, y, r * 0.25, rng.random(), rng.random(), rng.random(), rng.uniform(0, 3), r * rng.uniform(1.0, 1.5),
                      r, r * rng.uniform(0.5, 0.8), app, geom))
    return out


# ====================================================================== obstacles
ROCK_MESH_R = 0.066
ROCK_COLORS = ["0.78 0.74 0.68", "0.66 0.63 0.58", "0.74 0.68 0.58", "0.60 0.60 0.60", "0.72 0.70 0.64", "0.70 0.66 0.62"]


def stone_node(i, o):
    scale = o.r / ROCK_MESH_R
    return ('DEF STONE_%d Rock { translation %.3f %.3f %.3f rotation 0 0 1 %.2f name "stone %d" scale %.2f '
            'color %s locked TRUE }' % (i, o.x, o.y, 0.035 * scale, (i * 1.7) % 6.28, i, scale,
                                        ROCK_COLORS[i % len(ROCK_COLORS)]))


def pole_node(i, o, style):
    """Three realistic pole types; the collision cylinder keeps the field radius o.r."""
    r = o.r
    parts = []
    if style == "utility":
        h = 5.5
        parts += [
            "Pose { translation 0 0 %.3f children [ Shape { appearance RoughOak { colorOverride 0.55 0.45 0.35 textureTransform TextureTransform { scale 1 8 } } geometry Cylinder { radius %.3f height %.3f subdivision 20 } } ] }" % (h / 2, r, h),
            "Pose { translation 0 0 0.3 children [ Shape { appearance PBRAppearance { baseColor 0.16 0.12 0.09 roughness 0.9 metalness 0 } geometry Cylinder { radius %.3f height 0.6 subdivision 20 } } ] }" % (r + 0.004),
            "Pose { translation 0 0 5.15 children [ Shape { appearance RoughOak { colorOverride 0.6 0.5 0.4 } geometry Box { size 0.09 1.7 0.1 } } ] }",
            "Pose { translation 0.02 0 4.9 rotation 1 0 0 0.6 children [ Shape { appearance DEF STEEL PBRAppearance { baseColor 0.5 0.5 0.52 roughness 0.4 metalness 0.8 } geometry Box { size 0.02 0.05 0.55 } } ] }",
            "Pose { translation 0.02 0 4.9 rotation 1 0 0 -0.6 children [ Shape { appearance USE STEEL geometry Box { size 0.02 0.05 0.55 } } ] }",
            "Pose { translation %.3f 0 1.6 children [ Shape { appearance PBRAppearance { baseColor 0.9 0.8 0.1 roughness 0.4 metalness 0.3 } geometry Box { size 0.005 0.09 0.12 } } ] }" % (r + 0.002),
        ]
        for y in (-0.7, 0.0, 0.7):
            parts.append("Pose { translation 0 %.2f 5.29 children [ Shape { appearance PBRAppearance { baseColor 0.55 0.75 0.65 roughness 0.1 metalness 0 transparency 0.15 } geometry Cylinder { radius 0.035 height 0.18 subdivision 12 } } ] }" % y)
        bound_h = h
    elif style == "riser":
        h = 1.15
        wheel = "0.75 0.08 0.06" if i % 2 == 0 else "0.1 0.3 0.7"
        parts += [
            "Pose { translation 0 0 %.3f children [ Shape { appearance DEF GALV PBRAppearance { baseColor 0.72 0.74 0.76 roughness 0.32 metalness 0.85 } geometry Cylinder { radius %.3f height %.3f subdivision 20 } } ] }" % (h / 2, r, h),
            "Pose { translation 0 0 0.04 children [ Shape { appearance PBRAppearance { baseColor 0.62 0.64 0.66 roughness 0.4 metalness 0.85 } geometry Cylinder { radius %.3f height 0.08 subdivision 20 } } ] }" % (r + 0.025),
            "Pose { translation 0 0 0.85 children [ Shape { appearance PBRAppearance { baseColor 0.66 0.68 0.70 roughness 0.35 metalness 0.85 } geometry Cylinder { radius %.3f height 0.12 subdivision 20 } } ] }" % (r + 0.015),
            "Pose { translation %.3f 0 0.85 rotation 0 1 0 1.5708 children [ Shape { appearance DEF VALVE_STEM PBRAppearance { baseColor 0.3 0.3 0.3 roughness 0.5 metalness 0.7 } geometry Cylinder { radius 0.008 height 0.07 } } ] }" % (r + 0.04),
            "Pose { translation %.3f 0 0.85 rotation 0 1 0 1.5708 children [ Shape { appearance PBRAppearance { baseColor %s roughness 0.4 metalness 0.2 } geometry Cylinder { radius 0.055 height 0.012 subdivision 16 } } ] }" % (r + 0.075, wheel),
            "Pose { translation 0 0 %.3f children [ Shape { appearance DEF BRASS PBRAppearance { baseColor 0.72 0.55 0.2 roughness 0.3 metalness 0.9 } geometry Cylinder { radius 0.022 height 0.09 subdivision 12 } } ] }" % (h + 0.045),
            "Pose { translation 0.05 0 %.3f rotation 0 1 0 1.5708 children [ Shape { appearance USE BRASS geometry Cylinder { radius 0.007 height 0.12 } } ] }" % (h + 0.08),
        ]
        bound_h = h
    else:   # weathered wooden marker post with a painted top
        h = 1.35
        parts += [
            "Pose { translation 0 0 %.3f children [ Shape { appearance RoughOak { colorOverride 0.75 0.68 0.58 textureTransform TextureTransform { scale 1 4 } } geometry Cylinder { radius %.3f height %.3f subdivision 16 } } ] }" % (h / 2, r, h),
            "Pose { translation 0 0 %.3f children [ Shape { appearance PBRAppearance { baseColor 0.95 0.95 0.92 roughness 0.6 metalness 0 } geometry Cylinder { radius %.3f height 0.12 subdivision 16 } } ] }" % (h - 0.2, r + 0.002),
            "Pose { translation 0 0 %.3f children [ Shape { appearance PBRAppearance { baseColor 0.85 0.15 0.08 roughness 0.6 metalness 0 } geometry Cylinder { radius %.3f height 0.1 subdivision 16 } } ] }" % (h - 0.06, r + 0.003),
            "Pose { translation 0 0 %.3f children [ Shape { appearance PBRAppearance { baseColor 0.85 0.15 0.08 roughness 0.6 metalness 0 } geometry Cone { bottomRadius %.3f height 0.05 subdivision 16 } } ] }" % (h + 0.015, r + 0.003),
        ]
        bound_h = h
    return """DEF POLE_%d Solid {
  translation %.3f %.3f 0
  children [
    %s
  ]
  name "pole %d"
  boundingObject Pose { translation 0 0 %.3f children [ Cylinder { radius %.3f height %.3f } ] }
  locked TRUE
}""" % (i, o.x, o.y, "\n    ".join(parts), i, bound_h / 2, r, bound_h)


def obstacle_nodes(field):
    styles = ["utility", "riser", "post", "riser"]
    out, k = [], 0
    for i, o in enumerate(field.obstacles):
        if o.kind == "stone":
            out.append(stone_node(i, o))
        else:
            out.append(pole_node(i, o, styles[k % len(styles)]))
            k += 1
    return out


# ====================================================================== farmyard
def farmyard_nodes(field, seed=3):
    x0, x1, y0, y1 = field.soil_bounds
    m = 1.2                                     # fence offset outside the soil plot
    fx0, fx1, fy0, fy1 = x0 - m, x1 + m, y0 - m, y1 + m
    gate_y0, gate_y1 = field.start[1] - 2.0, field.start[1] + 2.0
    fence = ('Fence {{ translation 0 0 0 name "{name}" height 1.1 poleGap 2.4 poleRadius 0.06 poleShape "cylinder" '
             'poleAppearance RoughOak {{ colorOverride 0.6 0.5 0.4 }} horizontalBarsNumber 2 '
             'horizontalBarShape [0.025 0.05, 0.025 -0.05, -0.025 -0.05, -0.025 0.05, 0.025 0.05] '
             'horizontalBarAppearance RoughOak {{ colorOverride 0.65 0.55 0.45 }} path [ {path} ] }}')

    def pth(pts):
        return ", ".join("%.2f %.2f 0" % p for p in pts)

    out = [
        fence.format(name="fence north-east", path=pth([(fx0, fy1), (fx1, fy1), (fx1, fy0)])),
        fence.format(name="fence south", path=pth([(fx1, fy0), (fx0, fy0), (fx0, gate_y0)])),
        fence.format(name="fence west", path=pth([(fx0, gate_y1), (fx0, fy1)])),
        # dirt track from the gate to the farmyard
        'Solid { translation %.2f %.2f 0.004 children [ Shape { appearance DryMud { textureTransform TextureTransform { scale 12 1.5 } } '
        'geometry Plane { size 10 3.2 } } ] name "farm track" }' % (fx0 - 5, field.start[1]),
        'Solid { translation %.2f %.2f 0.006 rotation 0 0 1 1.5708 children [ Shape { appearance DryMud { textureTransform TextureTransform { scale 8 1.5 } } '
        'geometry Plane { size 13.2 3.2 } } ] name "farm track 2" }' % (fx0 - 8.5, field.start[1] + 4.9),
        'Barn { translation %.2f %.2f 0 rotation 0 0 1 1.5708 name "barn" }' % (fx0 - 8.5, field.start[1] + 16.5),
        'Silo { translation %.2f %.2f 0 radius 2.4 height 11 roofHeight 1.6 name "silo" enableBoundingObject FALSE }' % (fx0 - 16.5, field.start[1] + 18.5),
        'Tractor { translation %.2f %.2f 0.75 rotation 0 0 1 0.35 color 0.15 0.45 0.2 name "tractor" controller "<none>" }' % (fx0 - 4.6, field.start[1] + 4.2),
        'Wheelbarrow { translation %.2f %.2f 0 rotation 0 0 1 2.2 name "wheelbarrow" }' % (fx0 - 2.2, gate_y0 - 3.0),
        'WateringCan { translation %.2f %.2f 0 rotation 0 0 1 0.6 name "watering can" }' % (fx0 - 1.6, gate_y0 - 3.8),
    ]
    # round hay bales lying near the barn
    bale_app = "DEF HAY PBRAppearance { baseColor 0.80 0.66 0.36 roughness 0.95 metalness 0 }"
    sy = field.start[1]
    for k, (bx, by, yaw) in enumerate([(-3.0, sy + 11.0, 0.2), (-1.6, sy + 11.6, 0.5), (-2.4, sy + 13.1, 1.3),
                                         (-4.3, sy + 12.6, 0.9)]):
        app = bale_app if k == 0 else "USE HAY"
        out.append('Pose { translation %.2f %.2f 0.7 rotation 0 0 1 %.2f children [ Pose { rotation 1 0 0 1.5708 children [ '
                   'Shape { appearance %s geometry Cylinder { radius 0.7 height 1.2 subdivision 24 } } ] } ] }'
                   % (fx0 + bx, by, yaw, app))
    # trees around the yard (none on the south side, where the camera is)
    rng = random.Random(seed)
    types = ["oak tree", "cherry tree", "birch tree", "hazel tree", "spruce", "white pine"]
    k = 0
    spots = [(x1 + rng.uniform(4, 9), y0 + t * (y1 - y0)) for t in np.linspace(0.05, 0.95, 5)]
    spots += [(x0 + t * (x1 - x0) + 4, y1 + rng.uniform(5, 10)) for t in np.linspace(0.2, 1.0, 5)]
    spots += [(fx0 - 24 + rng.uniform(-3, 3), y0 + t * 26) for t in np.linspace(0, 1, 5)]
    for (tx, ty) in spots:
        hgt = rng.uniform(5, 8)
        out.append('SimpleTree { translation %.2f %.2f 0 name "tree %d" type "%s" height %.1f radius %.1f enableBoundingObject FALSE }'
                   % (tx, ty, k, rng.choice(types), hgt, hgt * 0.45))
        k += 1
    return out
