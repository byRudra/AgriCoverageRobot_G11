"""Generate the Webots world (crop field, obstacles, robot) from a field description.

Owner: M1 (field and world design).

    python tools/generate_world.py                 # demo field (seed 11) -> worlds/crop_field.wbt
    python tools/generate_world.py --seed 3 --stones 8 --poles 5 --out worlds/field_seed3.wbt

Also writes worlds/field_config.json (read by the controller) and the soil texture
worlds/textures/field_soil.jpg, which the robot's Pen device paints while spraying.
"""

import argparse
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from agri.field import default_field, random_field   # noqa: E402
from tools.scenery import (SCENERY_PROTOS, farmyard_nodes, make_soil_texture,   # noqa: E402
                           obstacle_nodes, pebble_nodes, plant_nodes, weed_nodes)

W = "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects"
EXTERNPROTOS = [
    W + "/objects/backgrounds/protos/TexturedBackground.proto",
    W + "/objects/backgrounds/protos/TexturedBackgroundLight.proto",
    W + "/objects/floors/protos/Floor.proto",
    W + "/appearances/protos/Grass.proto",
    W + "/appearances/protos/RoughPine.proto",
    W + "/objects/rocks/protos/Rock.proto",
    W + "/objects/trees/protos/SimpleTree.proto",
    W + "/robots/adept/pioneer3/protos/Pioneer3at.proto",
] + SCENERY_PROTOS

VIEW_POS = (-30.0, -22.0, 15.5)      # default camera: south of the field, elevated     # radius of the Rock proto mesh at scale 1


# ---------------------------------------------------------------------- viewpoint
def look_at(pos, target):
    """Webots Viewpoint orientation (axis-angle) for a camera at pos looking at target.
    The Webots camera looks along its local +x axis with +z up."""
    d = np.asarray(target, float) - np.asarray(pos, float)
    yaw = math.atan2(d[1], d[0])
    pitch = math.atan2(-d[2], math.hypot(d[0], d[1]))      # positive = looking down
    cz, sz, cy, sy = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    R = Rz @ Ry
    ang = math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2)))
    if ang < 1e-9:
        return (0, 0, 1, 0)
    ax = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * math.sin(ang))
    return (ax[0], ax[1], ax[2], ang)


ROBOT = """DEF AGRI_ROBOT Pioneer3at {
  translation %(x).3f %(y).3f 0.0
  rotation 0 0 1 %(yaw).4f
  name "AgriBot"
  controller "agri_coverage_controller"
  controllerArgs [ %(args)s ]
  supervisor TRUE
  extensionSlot [
    Lidar {
      translation 0.145 0 0.35
      name "lidar"
      horizontalResolution 180
      fieldOfView 3.14159
      numberOfLayers 1
      verticalFieldOfView 0.004
      near 0.05
      minRange 0.08
      maxRange 10
      noise 0.001
      children [ Shape {
        appearance PBRAppearance { baseColor 0.95 0.75 0.1 roughness 0.4 metalness 0.2 }
        geometry Cylinder { radius 0.045 height 0.07 subdivision 16 } } ]
    }
    GPS { translation 0 0 0.3 name "gps" }
    InertialUnit { translation 0 0 0.3 name "imu" }
    Pen {
      translation -0.30 0 0.25
      name "sprayer"
      inkColor 0.10 0.24 0.36
      inkDensity 0.5
      leadSize %(tool).2f
      maxDistance 0.6
      write FALSE
    }
    Display { name "dashboard" width 330 height 230 }
    DEF SPRAY_TANK Pose { translation -0.08 0 0.36 children [ Shape {
      appearance PBRAppearance { baseColor 0.1 0.45 0.85 roughness 0.35 metalness 0 transparency 0.15 }
      geometry Cylinder { radius 0.11 height 0.16 subdivision 20 } } ] }
    DEF TANK_CAP Pose { translation -0.08 0 0.45 children [ Shape {
      appearance PBRAppearance { baseColor 0.95 0.95 0.95 roughness 0.5 metalness 0 }
      geometry Cylinder { radius 0.04 height 0.03 subdivision 12 } } ] }
    DEF BOOM_MAST Pose { translation -0.27 0 0.27 children [ Shape {
      appearance DEF BOOM_METAL PBRAppearance { baseColor 0.75 0.76 0.78 roughness 0.3 metalness 0.8 }
      geometry Box { size 0.04 0.04 0.08 } } ] }
    DEF BOOM Pose { translation -0.30 0 0.30 children [ Shape {
      appearance USE BOOM_METAL geometry Box { size 0.035 %(tool).2f 0.025 } } ] }
%(nozzles)s
  ]
}"""

NOZZLE = """    Pose { translation -0.30 %.3f 0.28 children [ Shape {
      appearance PBRAppearance { baseColor 0.1 0.1 0.1 roughness 0.6 metalness 0 }
      geometry Cone { bottomRadius 0.015 height 0.03 subdivision 8 } } ] }"""


def build_world(field, out_path, method="proposed", turn="arc"):
    x0, x1, y0, y1 = field.soil_bounds
    sw, sh = x1 - x0, y1 - y0
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    lines = ["#VRML_SIM R2025a utf8", ""]
    lines += ['EXTERNPROTO "%s"' % p for p in EXTERNPROTOS]
    lines += ["", """WorldInfo {
  info [
    "AISR Project P04 (Group G11): Agricultural Field Robot for Crop-Row Coverage"
    "Pioneer 3-AT sprays a %gx%g m crop field lane by lane (boustrophedon) and avoids stones and poles."
  ]
  title "Agricultural Field Robot - Crop-Row Coverage"
  basicTimeStep 32
  inkEvaporation 0
  lineScale 0.6
}""" % (field.width, field.height),
              """Viewpoint {
  orientation %.5f %.5f %.5f %.5f
  position %.2f %.2f %.2f
  near 0.1
  followSmoothness 0.5
}""" % (look_at(VIEW_POS, (-9.0, 3.0, 0)) + VIEW_POS),
              'TexturedBackground { texture "noon_cloudy_countryside" }',
              "TexturedBackgroundLight { texture \"noon_cloudy_countryside\" }",
              'Floor { translation 0 0 -0.01 name "meadow" size 120 120 tileSize 4 4 appearance Grass { type "maintained" } }',
              """DEF FIELD Solid {
  translation %(cx).3f %(cy).3f 0
  children [
    Pose { translation 0 0 0.002 children [ Shape {
      appearance PBRAppearance {
        baseColorMap ImageTexture { url [ "textures/field_soil.jpg" ] repeatS FALSE repeatT FALSE }
        roughness 1
        metalness 0
      }
      geometry Plane { size %(sw).3f %(sh).3f }
    } ] }
  ]
  name "crop field"
  boundingObject Pose { translation 0 0 -0.003 children [ Box { size %(sw).3f %(sh).3f 0.01 } ] }
  locked TRUE
}""" % dict(cx=cx, cy=cy, sw=sw, sh=sh)]
    # corner flags of the crop area
    for i, (fx, fy) in enumerate([(field.x_min, field.y_min), (field.x_max, field.y_min),
                                   (field.x_max, field.y_max), (field.x_min, field.y_max)]):
        dx = -0.6 if fx < 0 else 0.6
        dy = -0.6 if fy < 0 else 0.6
        lines.append("""Pose { translation %.2f %.2f 0 children [
  Pose { translation 0 0 0.5 children [ Shape { appearance PBRAppearance { baseColor 0.95 0.95 0.95 roughness 0.5 metalness 0 } geometry Cylinder { radius 0.015 height 1.0 } } ] }
  Pose { translation 0.12 0 0.9 children [ Shape { appearance PBRAppearance { baseColor 0.9 0.15 0.1 roughness 0.6 metalness 0 } geometry Box { size 0.22 0.01 0.14 } } ] }
] }""" % (fx + dx, fy + dy))
    lines += farmyard_nodes(field)
    lines += obstacle_nodes(field)
    lines.append("DEF CROPS Group { children [")
    lines += ["  " + p for p in plant_nodes(field)]
    lines.append("] }")
    lines.append("DEF WEEDS Group { children [")
    lines += ["  " + p for p in weed_nodes(field)]
    lines.append("] }")
    lines.append("DEF PEBBLES Group { children [")
    lines += ["  " + p for p in pebble_nodes(field)]
    lines.append("] }")
    hw = field.tool_width / 2
    nozzles = "\n".join(NOZZLE % y for y in np.linspace(-hw + 0.05, hw - 0.05, 6))
    sx, sy, syaw = field.start
    args = '"--method=%s" "--turn=%s"' % (method, turn)
    lines.append(ROBOT % dict(x=sx, y=sy, yaw=syaw, tool=field.tool_width, nozzles=nozzles, args=args))
    with open(out_path, "w", newline="\n") as f:
        f.write("\n".join(lines) + "\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--stones", type=int, default=6)
    ap.add_argument("--poles", type=int, default=4)
    ap.add_argument("--method", default="proposed", choices=["proposed", "bug2", "apf"])
    ap.add_argument("--turn", default="arc", choices=["arc", "pivot"])
    ap.add_argument("--out", default=os.path.join(ROOT, "worlds", "crop_field.wbt"))
    args = ap.parse_args()

    field = default_field() if args.seed is None else random_field(args.seed, args.stones, args.poles)
    wdir = os.path.dirname(os.path.abspath(args.out))
    cfg = os.path.join(wdir, "field_config.json")
    field.save(cfg)
    tex = os.path.join(wdir, "textures", "field_soil.jpg")
    os.makedirs(os.path.dirname(tex), exist_ok=True)
    make_soil_texture(field, tex)
    build_world(field, args.out, args.method, args.turn)
    print("world  :", args.out)
    print("config :", cfg)
    print("texture:", tex)
    print("obstacles:", len(field.obstacles), "| lanes:", len(field.lanes()))


if __name__ == "__main__":
    main()
