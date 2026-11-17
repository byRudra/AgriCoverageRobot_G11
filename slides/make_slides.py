"""Build the 17-slide presentation (editable .pptx) from the report figures.

    python slides/make_slides.py        -> slides/AISR_P04_G11_Slides.pptx

Rules followed (Project Instructions, Section 3): max 17 slides, heading 28-32 pt,
body >= 20 pt, one idea per slide, own figures, presenter name in the corner.
"""

import os

from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_CONNECTOR, MSO_SHAPE
from pptx.enum.text import MSO_ANCHOR, PP_ALIGN
from pptx.util import Inches, Pt

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIG = os.path.join(ROOT, "report", "figures")
OUT = os.path.join(ROOT, "slides", "AISR_P04_G11_Slides.pptx")

GREEN = RGBColor(0x1F, 0x6F, 0x4A)
DARK = RGBColor(0x14, 0x2B, 0x20)
SOIL = RGBColor(0x8A, 0x6D, 0x3B)
INK = RGBColor(0x1F, 0x29, 0x37)
GREY = RGBColor(0x6B, 0x72, 0x80)
LIGHT = RGBColor(0xF3, 0xF6, 0xF1)
WHITE = RGBColor(0xFF, 0xFF, 0xFF)
ORANGE = RGBColor(0xC0, 0x53, 0x2A)
BLUE = RGBColor(0x2B, 0x6C, 0xB0)
FONT = "Calibri"

M = {1: "Jahanvi Sengar", 2: "Aagman Sharma", 3: "Rudra Gupta", 4: "Sahil Agarwal", 5: "Harsh Prakash Sinha"}

prs = Presentation()
prs.slide_width = Inches(13.333)
prs.slide_height = Inches(7.5)
W, H = prs.slide_width, prs.slide_height
BLANK = prs.slide_layouts[6]
slide_no = [0]


def text(slide, x, y, w, h, s, size=20, bold=False, color=INK, align=PP_ALIGN.LEFT, anchor=MSO_ANCHOR.TOP,
         font=FONT, italic=False):
    tb = slide.shapes.add_textbox(x, y, w, h)
    tf = tb.text_frame
    tf.word_wrap = True
    tf.vertical_anchor = anchor
    tf.margin_left = tf.margin_right = Inches(0.05)
    lines = s if isinstance(s, list) else [s]
    for i, line in enumerate(lines):
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        p.alignment = align
        r = p.add_run()
        r.text = line
        r.font.size = Pt(size)
        r.font.bold = bold
        r.font.italic = italic
        r.font.color.rgb = color
        r.font.name = font
    return tb


def bullets(slide, x, y, w, h, items, size=22, gap=10, color=INK):
    tb = slide.shapes.add_textbox(x, y, w, h)
    tf = tb.text_frame
    tf.word_wrap = True
    for i, it in enumerate(items):
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        p.space_after = Pt(gap)
        if isinstance(it, tuple):
            head, rest = it
            r = p.add_run()
            r.text = "■  " + head
            r.font.bold = True
            r.font.size = Pt(size)
            r.font.color.rgb = GREEN
            r.font.name = FONT
            r2 = p.add_run()
            r2.text = " " + rest
            r2.font.size = Pt(size)
            r2.font.color.rgb = color
            r2.font.name = FONT
        else:
            r = p.add_run()
            r.text = "■  " + it
            r.font.size = Pt(size)
            r.font.color.rgb = color
            r.font.name = FONT
    return tb


def rect(slide, x, y, w, h, fill, line=None, shape=MSO_SHAPE.RECTANGLE):
    s = slide.shapes.add_shape(shape, x, y, w, h)
    s.fill.solid()
    s.fill.fore_color.rgb = fill
    if line is None:
        s.line.fill.background()
    else:
        s.line.color.rgb = line
        s.line.width = Pt(1.5)
    s.shadow.inherit = False
    return s


def picture(slide, name, x, y, w=None, h=None):
    path = os.path.join(FIG, name)
    return slide.shapes.add_picture(path, x, y, width=w, height=h)


def picture_fit(slide, name, x, y, w, h):
    """Insert a picture scaled to fit (x, y, w, h), centred."""
    from PIL import Image
    iw, ih = Image.open(os.path.join(FIG, name)).size
    s = min(w / iw, h / ih)
    pw, ph = int(iw * s), int(ih * s)
    return slide.shapes.add_picture(os.path.join(FIG, name), x + (w - pw) // 2, y + (h - ph) // 2, pw, ph)


def new_slide(title, presenter, subtitle=None):
    slide_no[0] += 1
    s = prs.slides.add_slide(BLANK)
    rect(s, 0, 0, W, H, WHITE)
    rect(s, 0, 0, Inches(0.22), H, GREEN)
    text(s, Inches(0.55), Inches(0.28), Inches(11.2), Inches(0.8), title, size=32, bold=True, color=DARK)
    if subtitle:
        text(s, Inches(0.57), Inches(0.98), Inches(11.5), Inches(0.5), subtitle, size=20, color=GREY)
    ln = s.shapes.add_connector(MSO_CONNECTOR.STRAIGHT, Inches(0.55), Inches(1.45 if subtitle else 1.1),
                                Inches(12.8), Inches(1.45 if subtitle else 1.1))
    ln.line.color.rgb = RGBColor(0xD1, 0xD5, 0xDB)
    ln.line.width = Pt(1)
    # footer: project left, presenter right (corner), slide number
    text(s, Inches(0.55), Inches(7.0), Inches(7), Inches(0.4), "AISR P04 | G11 | Agricultural Field Robot for Crop-Row Coverage",
         size=12, color=GREY)
    tag = rect(s, Inches(10.0), Inches(6.95), Inches(3.05), Inches(0.42), LIGHT, shape=MSO_SHAPE.ROUNDED_RECTANGLE)
    tf = tag.text_frame
    tf.margin_top = tf.margin_bottom = 0
    p = tf.paragraphs[0]
    p.alignment = PP_ALIGN.CENTER
    r = p.add_run()
    r.text = "Presenter: " + presenter
    r.font.size = Pt(13)
    r.font.bold = True
    r.font.color.rgb = GREEN
    r.font.name = FONT
    text(s, Inches(12.95), Inches(0.3), Inches(0.35), Inches(0.4), str(slide_no[0]), size=12, color=GREY,
         align=PP_ALIGN.RIGHT)
    return s


def stat(slide, x, y, value, label, color=GREEN, w=Inches(2.9)):
    rect(slide, x, y, w, Inches(1.45), LIGHT, shape=MSO_SHAPE.ROUNDED_RECTANGLE)
    text(slide, x, y + Inches(0.08), w, Inches(0.75), value, size=34, bold=True, color=color, align=PP_ALIGN.CENTER)
    text(slide, x + Inches(0.1), y + Inches(0.82), w - Inches(0.2), Inches(0.6), label, size=15, color=INK,
         align=PP_ALIGN.CENTER)


def table(slide, x, y, w, rows, col_w, size=18, header_fill=GREEN, row_h=Inches(0.48), bold_first_col=False):
    n, m = len(rows), len(rows[0])
    gt = slide.shapes.add_table(n, m, x, y, w, row_h * n).table
    for j, cw in enumerate(col_w):
        gt.columns[j].width = cw
    for i, row in enumerate(rows):
        for j, val in enumerate(row):
            c = gt.cell(i, j)
            c.text = ""
            c.margin_left = c.margin_right = Inches(0.08)
            c.margin_top = c.margin_bottom = Inches(0.03)
            p = c.text_frame.paragraphs[0]
            r = p.add_run()
            r.text = str(val)
            r.font.size = Pt(size)
            r.font.name = FONT
            r.font.bold = (i == 0) or (bold_first_col and j == 0)
            r.font.color.rgb = WHITE if i == 0 else INK
            c.fill.solid()
            c.fill.fore_color.rgb = header_fill if i == 0 else (LIGHT if i % 2 == 0 else WHITE)
    return gt


def box(slide, x, y, w, h, title, body, fill, title_color=DARK, size=16):
    rect(slide, x, y, w, h, fill, line=RGBColor(0x9C, 0xA3, 0xAF), shape=MSO_SHAPE.ROUNDED_RECTANGLE)
    text(slide, x + Inches(0.1), y + Inches(0.05), w - Inches(0.2), Inches(0.45), title, size=size + 2, bold=True,
         color=title_color, align=PP_ALIGN.CENTER)
    text(slide, x + Inches(0.1), y + Inches(0.48), w - Inches(0.2), h - Inches(0.5), body, size=size,
         align=PP_ALIGN.CENTER)


def arrow(slide, x1, y1, x2, y2):
    c = slide.shapes.add_connector(MSO_CONNECTOR.STRAIGHT, x1, y1, x2, y2)
    c.line.color.rgb = INK
    c.line.width = Pt(2)
    ln = c.line._get_or_add_ln()
    from pptx.oxml.ns import qn
    tail = ln.makeelement(qn("a:tailEnd"), {"type": "triangle", "w": "med", "len": "med"})
    ln.append(tail)
    return c


# ====================================================================== 1 Title
slide_no[0] += 1
s = prs.slides.add_slide(BLANK)
s.shapes.add_picture(os.path.join(FIG, "shot_overview_end.jpg"), 0, 0, W, H)
rect(s, 0, 0, W, H, DARK).fill.transparency = 0  # placeholder for z-order
s.shapes._spTree.remove(s.shapes[-1]._element)
ov = rect(s, Inches(0.0), Inches(0.0), Inches(7.6), H, DARK)
from pptx.oxml.ns import qn  # noqa: E402
sp = ov.fill._xPr.find(qn("a:solidFill"))
clr = sp.find(qn("a:srgbClr"))
alpha = clr.makeelement(qn("a:alpha"), {"val": "88000"})
clr.append(alpha)
text(s, Inches(0.6), Inches(0.5), Inches(6.8), Inches(0.5), "CSAI4014 Algorithms for Intelligent Systems and Robotics",
     size=16, color=RGBColor(0xBB, 0xE5, 0xC4))
text(s, Inches(0.6), Inches(1.0), Inches(6.8), Inches(2.2), ["Agricultural Field Robot", "for Crop-Row Coverage"],
     size=40, bold=True, color=WHITE)
text(s, Inches(0.6), Inches(2.85), Inches(6.8), Inches(0.9),
     "Boustrophedon coverage with lidar-based, lane-preserving detours in Webots", size=20,
     color=RGBColor(0xE5, 0xE7, 0xEB), italic=True)
text(s, Inches(0.6), Inches(3.85), Inches(6.8), Inches(0.45), "Project P04   |   Group G11   |   Batch B4", size=18,
     bold=True, color=RGBColor(0xBB, 0xE5, 0xC4))
names = ["Jahanvi Sengar  (500120543)", "Aagman Sharma  (500123462)", "Rudra Gupta  (500120437)",
         "Sahil Agarwal  (500120211)", "Harsh Prakash Sinha  (500123860)"]
text(s, Inches(0.6), Inches(4.35), Inches(6.8), Inches(2.0), names, size=18, color=WHITE)
text(s, Inches(0.6), Inches(6.45), Inches(6.8), Inches(0.8),
     ["Submitted to: Course Faculty, CSAI4014", "School of Computer Science, UPES Dehradun  |  October 2026"],
     size=14, color=RGBColor(0xD1, 0xD5, 0xDB))
tag = rect(s, Inches(10.0), Inches(6.95), Inches(3.05), Inches(0.42), LIGHT, shape=MSO_SHAPE.ROUNDED_RECTANGLE)
p = tag.text_frame.paragraphs[0]
p.alignment = PP_ALIGN.CENTER
r = p.add_run()
r.text = "Presenter: " + M[1]
r.font.size = Pt(13)
r.font.bold = True
r.font.color.rgb = GREEN

# ====================================================================== 2 Problem
s = new_slide("Problem and Motivation", M[1], "Spraying robots must treat every crop row, but stones and poles block the lanes")
picture_fit(s, "shot_farm.jpg", Inches(0.55), Inches(1.65), Inches(7.4), Inches(5.1))
bullets(s, Inches(8.2), Inches(1.75), Inches(4.8), Inches(5), [
    ("Missed strip", "= weeds and disease"),
    ("Overlap", "= wasted chemicals"),
    ("Unknown obstacles", "in the rows: stones, irrigation and power poles"),
    ("Reactive swerving", "keeps the robot safe but loses the lane"),
    ("Industry:", "Thorvald, field sprayers, weeding robots"),
], size=22, gap=14)

# ====================================================================== 3 Objectives
s = new_slide("Objectives", M[1], "Four measurable goals")
objs = [("1", "Build the field world", "16 x 10 m crop field, 11 rows, stones and poles; Pioneer 3-AT with lidar, GPS, IMU, 1.2 m boom"),
        ("2", "Cover row by row", "boustrophedon plan with arc and pivot headland turns; lane error < 5 cm RMS"),
        ("3", "Avoid unknown obstacles", "zero collisions and at least 98 % coverage of the coverable area"),
        ("4", "Compare and analyse", "vs Bug2 and APF on 20 random fields; study turns, density, boom width, PID gains")]
for i, (n, head, body) in enumerate(objs):
    y = Inches(1.75 + i * 1.28)
    c = rect(s, Inches(0.7), y, Inches(0.95), Inches(0.95), GREEN, shape=MSO_SHAPE.OVAL)
    c.text_frame.paragraphs[0].alignment = PP_ALIGN.CENTER
    r = c.text_frame.paragraphs[0].add_run()
    r.text = n
    r.font.size = Pt(30)
    r.font.bold = True
    r.font.color.rgb = WHITE
    text(s, Inches(1.9), y - Inches(0.02), Inches(10.8), Inches(0.5), head, size=24, bold=True, color=DARK)
    text(s, Inches(1.9), y + Inches(0.45), Inches(10.8), Inches(0.6), body, size=20, color=INK)

# ====================================================================== 4 Existing solutions
s = new_slide("Existing Solutions and the Gap", M[2])
table(s, Inches(0.6), Inches(1.45), Inches(12.1), [
    ["Approach", "Strength", "Limitation"],
    ["Boustrophedon decomposition (Choset 2000)", "Complete, low overlap", "Obstacles must be known in advance"],
    ["Artificial potential field (Khatib 1986)", "Simple, fast, reactive", "Local minima, oscillation, loses lane"],
    ["Bug2 boundary following (Lumelsky 1987)", "Complete for point robot", "Long loops, rejoins late"],
    ["Industry: lane following + safety stop", "Certified (ISO 18497)", "Stops; needs an operator"],
], [Inches(4.6), Inches(3.4), Inches(4.1)], size=20, row_h=Inches(0.72))
rect(s, Inches(0.6), Inches(5.35), Inches(12.1), Inches(1.3), LIGHT, shape=MSO_SHAPE.ROUNDED_RECTANGLE)
text(s, Inches(0.85), Inches(5.45), Inches(11.6), Inches(1.15),
     ["Gap: a sensor-based method that keeps the lane pattern intact when an unknown obstacle",
      "appears, measured against the classical reactive methods."], size=22, bold=True, color=GREEN)

# ====================================================================== 5 Novelty
s = new_slide("Novelty", M[2], "What is new compared with the textbook planner and reactive methods")
nov = [("Lane-preserving detour:", "blocked lane segment replaced by a safe arc that rejoins the same lane -> 20/20 fields, 0 skipped strips"),
       ("Robust lidar map in crops:", "ghost-return + tilt filters -> 17 phantom obstacles removed, robot no longer stuck"),
       ("Layered agent, one codebase:", "same Python agent in Webots and headless sim (98.43 % vs 98.97 %)"),
       ("Design study:", "arc turns 7.4 % faster than pivot; 1.2 m boom = 98.8 % coverage, 24 % overlap"),
       ("Live coverage accounting:", "5 cm grid, sprayed soil painted in Webots, on-screen dashboard")]
bullets(s, Inches(0.7), Inches(1.75), Inches(12), Inches(5), nov, size=22, gap=16)

# ====================================================================== 6 Syllabus
s = new_slide("Syllabus Mapping", M[2])
table(s, Inches(0.6), Inches(1.4), Inches(12.1), [
    ["AISR syllabus topic", "Where it is used"],
    ["Uninhabited ground robots", "Pioneer 3-AT skid steer, wheel limits, lidar / GPS / IMU"],
    ["Path planning", "Boustrophedon plan, geometric detour, Bug2 and APF"],
    ["Planning to move", "Arc / pivot headland turns, pure pursuit, speed scheduling"],
    ["Application domains", "Precision agriculture: spraying, coverage and overlap"],
    ["Closed-loop control", "PID heading loop tuned from a step response"],
    ["Perception + software architecture", "Lidar clustering; declarative-procedural-reflexive agent"],
], [Inches(4.5), Inches(7.6)], size=20, row_h=Inches(0.7), bold_first_col=True)

# ====================================================================== 7 Architecture
s = new_slide("System Architecture", M[3], "Sensors -> layered agent -> actuators, every 32 ms")
sens = [("2-D lidar", "180 rays, 10 m"), ("GPS", "position x, y"), ("Inertial unit", "roll, pitch, yaw")]
acts = [("4 wheel motors", "skid steer"), ("Sprayer boom", "Pen, 1.2 m"), ("Dashboard + logs", "CSV / JSON")]
for i, (t, b) in enumerate(sens):
    box(s, Inches(0.6), Inches(1.8 + i * 1.6), Inches(2.6), Inches(1.25), t, b, RGBColor(0xE0, 0xEC, 0xFA), size=16)
for i, (t, b) in enumerate(acts):
    box(s, Inches(10.1), Inches(1.8 + i * 1.6), Inches(2.65), Inches(1.25), t, b, RGBColor(0xFD, 0xEB, 0xDC), size=16)
layers = [("Declarative layer", "boustrophedon plan: lanes, turns, spray tags"),
          ("Procedural layer", "lidar filters -> obstacle map -> detour splicing -> pure pursuit + PID"),
          ("Reflexive layer", "stop and back off if anything is closer than 0.4 m")]
for i, (t, b) in enumerate(layers):
    box(s, Inches(3.9), Inches(1.8 + i * 1.6), Inches(5.5), Inches(1.25), t, b, RGBColor(0xE3, 0xF2, 0xE6), size=16)
for i in range(3):
    arrow(s, Inches(3.2), Inches(2.42 + i * 1.6), Inches(3.9), Inches(2.42 + i * 1.6))
    arrow(s, Inches(9.4), Inches(2.42 + i * 1.6), Inches(10.1), Inches(2.42 + i * 1.6))
for i in range(2):
    arrow(s, Inches(6.65), Inches(3.05 + i * 1.6), Inches(6.65), Inches(3.4 + i * 1.6))
text(s, Inches(3.9), Inches(6.45), Inches(5.5), Inches(0.4), "agri/agent.py  (same code in Webots and headless sim)",
     size=15, color=GREY, align=PP_ALIGN.CENTER, italic=True)

# ====================================================================== 8 Robot and environment
s = new_slide("Robot and Environment Model", M[3])
picture_fit(s, "shot_robot.jpg", Inches(0.55), Inches(1.35), Inches(6.6), Inches(5.4))
bullets(s, Inches(7.4), Inches(1.45), Inches(5.6), Inches(5.4), [
    ("Pioneer 3-AT:", "4WD skid steer, r = 0.11 m, max 0.70 m/s"),
    ("Kinematics:", "unicycle; wheels = (v -/+ w b/2) / r"),
    ("Lidar:", "180 deg, 180 rays, 0.35 m high"),
    ("Field:", "16 x 10 m, 11 lettuce rows 1 m apart"),
    ("Obstacles:", "6 stones, power pole, irrigation risers, post"),
    ("World generated", "from a seed: tools/generate_world.py"),
], size=21, gap=12)

# ====================================================================== 9 Algorithm 1
s = new_slide("Algorithm 1: Coverage Plan and Path Tracking", M[3])
picture_fit(s, "fig_plan_turns.png", Inches(0.5), Inches(1.3), Inches(12.3), Inches(3.4))
rect(s, Inches(0.6), Inches(4.85), Inches(5.9), Inches(1.9), LIGHT, shape=MSO_SHAPE.ROUNDED_RECTANGLE)
text(s, Inches(0.8), Inches(4.92), Inches(5.6), Inches(1.8),
     ["Pure pursuit + PID", "e = angle to look-ahead point - heading", "w = Kp e + Ki ∫e + Kd de/dt",
      "Kp 2.2, Ki 0.05, Kd 0.12"], size=20, color=INK)
rect(s, Inches(6.8), Inches(4.85), Inches(5.95), Inches(1.9), LIGHT, shape=MSO_SHAPE.ROUNDED_RECTANGLE)
text(s, Inches(7.0), Inches(4.92), Inches(5.6), Inches(1.8),
     ["Speed scheduling", "v = 0.5 m/s on lanes, 0.3 m/s in turns", "slow down below 2 m clearance",
      "pivot in place if |e| > 55 deg"], size=20, color=INK)

# ====================================================================== 10 Algorithm 2
s = new_slide("Algorithm 2: Lidar Map and Geometric Detour", M[4])
picture_fit(s, "fig_perception.png", Inches(0.45), Inches(1.3), Inches(6.5), Inches(4.0))
picture_fit(s, "fig_detour_geometry.png", Inches(7.0), Inches(1.3), Inches(5.9), Inches(3.4))
bullets(s, Inches(0.6), Inches(5.3), Inches(12.3), Inches(1.6), [
    ("Map:", "drop low + isolated returns -> cluster -> circle fit -> confirm after 8 scans"),
    ("Detour:", "R = r + 0.32 + 0.30 m; cut blocked segment; splice shorter safe arc on the same lane"),
], size=20, gap=6)

# ====================================================================== 11 Setup
s = new_slide("Simulation Setup", M[4])
table(s, Inches(0.6), Inches(1.4), Inches(6.4), [
    ["Parameter", "Value"],
    ["Simulator", "Webots R2025a, 32 ms step"],
    ["Controller", "Python 3 (agri package)"],
    ["Max / turn speed", "0.5 / 0.3 m/s"],
    ["Boom / lane spacing", "1.2 m / 1.0 m"],
    ["Detour margin", "0.30 m, horizon 5 m"],
    ["Experiments", "20 random fields x 3 methods"],
], [Inches(2.7), Inches(3.7)], size=19, row_h=Inches(0.62))
rect(s, Inches(7.3), Inches(1.4), Inches(5.5), Inches(5.3), DARK, shape=MSO_SHAPE.ROUNDED_RECTANGLE)
text(s, Inches(7.5), Inches(1.55), Inches(5.2), Inches(5.1),
     ["# live demo", "open worlds/crop_field.wbt -> Run", "", "# new random field",
      "python tools/generate_world.py --seed 3", "", "# Webots batch runs", "python experiments/run_webots.py",
      "", "# statistics + figures", "python experiments/run_experiments.py", "python experiments/make_figures.py"],
     size=16, color=RGBColor(0xD1, 0xFA, 0xE5), font="Consolas")

# ====================================================================== 12 Live demo
s = new_slide("Live Demonstration", "Rudra + Sahil", "Switching to Webots: crop_field.wbt")
picture_fit(s, "shot_follow.jpg", Inches(0.55), Inches(1.65), Inches(7.6), Inches(5.1))
bullets(s, Inches(8.4), Inches(1.8), Inches(4.6), Inches(5), [
    "Lane following, sprayer ON",
    "Stone ahead -> slow down, arc detour",
    "Back on the same lane",
    "Headland arc turn, sprayer OFF",
    "Blue soil = sprayed area",
    "Dashboard: state, coverage, minimap",
], size=22, gap=14)

# ====================================================================== 13 Results 1
s = new_slide("Results 1: Webots, Same Field, Three Methods", M[5])
picture_fit(s, "fig_webots_traj.png", Inches(0.45), Inches(1.25), Inches(12.4), Inches(4.75))
text(s, Inches(0.6), Inches(6.1), Inches(12.2), Inches(0.8),
     "Proposed: 98.43 % coverage, 414.6 s, 0 collisions  |  Bug2: 95.51 %, 449.0 s  |  APF: 97.74 %, 2.1 cm from a stone",
     size=19, bold=True, color=GREEN, align=PP_ALIGN.CENTER)

# ====================================================================== 14 Results 2
s = new_slide("Results 2: 20 Random Fields", M[5])
picture_fit(s, "fig_mc_box.png", Inches(0.4), Inches(1.2), Inches(12.5), Inches(3.6))
stat(s, Inches(0.6), Inches(5.0), "98.85 %", "coverage, proposed (Bug2 95.4, APF 85.3)")
stat(s, Inches(3.7), Inches(5.0), "20 / 20", "fields finished (APF 15 / 20)")
stat(s, Inches(6.8), Inches(5.0), "412 s", "time (Bug2 472 s, APF 781 s)")
stat(s, Inches(9.9), Inches(5.0), "2.1 cm", "RMS lane error (Bug2 11.3, APF 28.3)")

# ====================================================================== 15 Contribution
s = new_slide("Team Contribution", M[5])
table(s, Inches(0.6), Inches(1.35), Inches(12.1), [
    ["Member", "Technical module (code)", "Report"],
    ["Jahanvi Sengar", "Field model + Webots world generator (field.py, generate_world.py)", "Abstract, 1, 2, 13"],
    ["Aagman Sharma", "Boustrophedon planner, arc / pivot turns, result figures (planner.py, make_figures.py)", "3, 4"],
    ["Rudra Gupta", "PID + pure-pursuit tracker, layered agent, Webots controller (navigation.py, agent.py)", "5, 6"],
    ["Sahil Agarwal", "Lidar filters, obstacle map, detour, Bug2, APF (avoidance.py)", "7, 8"],
    ["Harsh P. Sinha", "Coverage metrics, headless sim, Monte-Carlo and Webots experiments (coverage.py)", "9, 12, App."],
], [Inches(2.6), Inches(7.4), Inches(2.1)], size=18, row_h=Inches(0.78), bold_first_col=True)
text(s, Inches(0.6), Inches(6.25), Inches(12), Inches(0.5), "Every module is traceable to the member's GitHub commits.",
     size=18, color=GREY, italic=True)

# ====================================================================== 16 Conclusion
s = new_slide("Conclusion and Industry Use", "All members")
bullets(s, Inches(0.6), Inches(1.4), Inches(6.4), Inches(5.3), [
    ("Result:", "98.85 % coverage, 0 collisions, lane kept in all 20 fields"),
    ("vs baselines:", "4-13x less missed area, 13-47 % faster"),
    ("Design:", "arc turns, 1.2 m boom on 1 m lanes"),
    ("Industry:", "spraying, weeding, scouting robots; coverage map = treatment record"),
], size=21, gap=14)
rect(s, Inches(7.3), Inches(1.4), Inches(5.5), Inches(5.3), LIGHT, shape=MSO_SHAPE.ROUNDED_RECTANGLE)
text(s, Inches(7.5), Inches(1.5), Inches(5.1), Inches(0.5), "Future scope", size=24, bold=True, color=GREEN)
bullets(s, Inches(7.5), Inches(2.1), Inches(5.1), Inches(4.5), [
    "Real robot: RTK-GNSS + lidar via ROS 2",
    "Polygon obstacles (fences, ditches)",
    "Sensor-fusion localisation",
    "Crop-row-aware detour side",
    "What we learned: Git workflow, Webots, fair experiments",
], size=19, gap=10)

# ====================================================================== 17 End
slide_no[0] += 1
s = prs.slides.add_slide(BLANK)
rect(s, 0, 0, W, H, DARK)
picture_fit(s, "shot_top_proposed.jpg", Inches(7.2), Inches(0.9), Inches(5.6), Inches(4.2))
text(s, Inches(0.8), Inches(1.2), Inches(6), Inches(1.2), "Thank you", size=54, bold=True, color=WHITE)
text(s, Inches(0.8), Inches(2.4), Inches(6), Inches(0.8), "Questions and answers", size=28, color=RGBColor(0xBB, 0xE5, 0xC4))
text(s, Inches(0.8), Inches(3.6), Inches(6.2), Inches(2.5),
     ["GitHub:  github.com/byRudra/AgriCoverageRobot_G11", "",
      "Jahanvi Sengar  |  Aagman Sharma  |  Rudra Gupta", "Sahil Agarwal  |  Harsh Prakash Sinha"],
     size=20, color=WHITE)
text(s, Inches(7.2), Inches(5.2), Inches(5.6), Inches(0.5), "Field after the run: blue = sprayed", size=16,
     color=RGBColor(0xD1, 0xD5, 0xDB), align=PP_ALIGN.CENTER)
tag = rect(s, Inches(10.0), Inches(6.95), Inches(3.05), Inches(0.42), LIGHT, shape=MSO_SHAPE.ROUNDED_RECTANGLE)
p = tag.text_frame.paragraphs[0]
p.alignment = PP_ALIGN.CENTER
r = p.add_run()
r.text = "Presenter: All members"
r.font.size = Pt(13)
r.font.bold = True
r.font.color.rgb = GREEN

assert len(prs.slides) <= 17, len(prs.slides)
prs.save(OUT)
print("saved", OUT, "slides:", len(prs.slides))
