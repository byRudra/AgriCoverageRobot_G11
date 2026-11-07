"""Webots controller for the Pioneer 3-AT crop-row coverage robot (AISR P04, G11).

Devices: 4 wheel motors, Lidar "lidar" (180 rays, 180 deg), GPS "gps", InertialUnit "imu",
Pen "sprayer" (paints the soil where the boom sprays), Display "dashboard" (overlay).

All decision making happens in agri.agent.CoverageAgent (shared with the headless
simulator); this file only reads sensors, drives the wheels and logs results.

Controller arguments (set in the world file, or override with environment variables):
    --method=proposed|bug2|apf     AGRI_METHOD
    --turn=arc|pivot               AGRI_TURN
    --quit                         AGRI_QUIT=1   quit Webots when the field is done (batch runs)
    --run=<name>                   AGRI_RUN      name of the results folder
    --shots=t:view:file,...        AGRI_SHOTS    screenshots (view = overview|follow|top|side, t = s or "end")
    --tmax=<s>                     AGRI_TMAX     time limit (default 1500 s)
    AGRI_NODASH=1 disables the dashboard overlay (faster batch runs)
"""

import csv
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, ROOT)

from controller import Supervisor   # noqa: E402

from agri.agent import CoverageAgent, lidar_angles   # noqa: E402
from agri.avoidance import filter_low_returns   # noqa: E402
from agri.field import Field   # noqa: E402

WHEEL_R = 0.11
TRACK_EFF = 0.394 * 1.4      # effective track width of the skid-steer base [m]
WHEEL_MAX = 6.35             # slightly below the motor limit (6.4 rad/s)


def get_arg(name, default):
    env = os.environ.get("AGRI_" + name.upper())
    if env:
        return env
    for a in sys.argv[1:]:
        if a.startswith("--%s=" % name):
            return a.split("=", 1)[1]
        if a == "--%s" % name:
            return "1"
    return default


def look_at(pos, target):
    """Viewpoint orientation (axis-angle) for a camera at pos looking at target."""
    d = np.asarray(target, float) - np.asarray(pos, float)
    yaw = math.atan2(d[1], d[0])
    pitch = math.atan2(-d[2], math.hypot(d[0], d[1]))
    cz, sz, cy, sy = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    R = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]]) @ np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    ang = math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2)))
    if ang < 1e-9:
        return [0, 0, 1, 0]
    ax = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * math.sin(ang))
    return [float(ax[0]), float(ax[1]), float(ax[2]), ang]


class Camera:
    """Moves the 3-D view for screenshots: AGRI_SHOTS="t:view:file,..." (view = overview|farm|follow|top|side)."""

    def __init__(self, robot, spec):
        self.robot = robot
        self.vp = None
        root = robot.getRoot().getField("children")
        for i in range(root.getCount()):
            n = root.getMFNode(i)
            if n.getTypeName() == "Viewpoint":
                self.vp = n
        self.shots = []
        self.final_shots = []
        for item in filter(None, spec.split(",")):
            t, view, name = item.split(":")
            if t == "end":
                self.final_shots.append((view, name))
            else:
                self.shots.append([float(t), view, name, False])

    def place(self, view, pose):
        x, y, th = pose
        if view == "overview":
            pos, tgt = (-3.0, -20.5, 12.5), (0, -0.8, 0)
        elif view == "farm":
            pos, tgt = (-30.0, -22.0, 15.5), (-9.0, 3.0, 0)
        elif view == "top":
            pos, tgt = (0.0, -0.01, 27.0), (0, 0, 0)
        elif view == "side":
            pos, tgt = (x + 3.2 * math.cos(th + 2.2), y + 3.2 * math.sin(th + 2.2), 1.6), (x, y, 0.2)
        else:   # follow: behind and above the robot
            pos, tgt = (x - 3.0 * math.cos(th), y - 3.0 * math.sin(th), 2.0), (x + 1.5 * math.cos(th), y + 1.5 * math.sin(th), 0.0)
        self.vp.getField("position").setSFVec3f([float(v) for v in pos])
        self.vp.getField("orientation").setSFRotation(look_at(pos, tgt))

    def final(self, pose, out_dir):
        """Shots taken when the field is finished (one simulation step per view)."""
        for view, name in self.final_shots:
            if self.vp is not None:
                self.place(view, pose)
            self.robot.step(int(self.robot.getBasicTimeStep()))
            self.robot.exportImage(os.path.join(out_dir, name), 92)

    def update(self, t, pose, out_dir):
        for shot in self.shots:
            if shot[3]:
                continue
            if t >= shot[0] - 0.6 and not shot[3] and self.vp is not None:
                self.place(shot[1], pose)
            if t >= shot[0]:
                self.robot.exportImage(os.path.join(out_dir, shot[2]), 92)
                shot[3] = True


class Dashboard:
    """Live overlay: state, progress, coverage bar and a coverage minimap.

    Static parts are drawn once; every update only redraws the text panel and the
    minimap blocks whose colour changed. (Redrawing everything each time accumulates
    millions of draw calls in long runs, which the Webots Display does not tolerate.)
    """

    BG = 0x1B2A20
    FG = 0xF2F5EF
    ACCENT = 0x7FD48A
    WARN = 0xF2B544
    SOIL = 0x76563A
    ONE = 0x6EBE5A
    MULTI = 0x3C8C32
    NX, NY = 48, 30

    def __init__(self, display, field):
        self.d = display
        self.f = field
        self.w = display.getWidth()
        self.h = display.getHeight()
        self.ox, self.oy = 10, 116
        self.map_w = self.w - 20
        self.map_h = min(int(self.map_w * field.height / field.width), self.h - self.oy - 4)
        self.cw = self.map_w / self.NX
        self.ch = self.map_h / self.NY
        self.state = np.zeros((self.NY, self.NX), np.int8)    # 0 soil, 1 one pass, 2 multi
        self.marker = None
        d = self.d
        d.setAlpha(0.85)
        d.setColor(self.BG)
        d.fillRectangle(0, 0, self.w, self.h)
        d.setAlpha(1.0)
        d.setColor(self.ACCENT)
        d.setFont("Arial", 14, True)
        d.drawText("AgriBot  |  crop-row coverage", 10, 8)
        d.setColor(self.SOIL)
        d.fillRectangle(self.ox, self.oy, self.map_w, self.map_h)
        self._draw_obstacles()

    # ------------------------------------------------------------------ minimap
    def _block_rect(self, i, j):
        x = self.ox + int(i * self.cw)
        y = self.oy + int((self.NY - 1 - j) * self.ch)
        return x, y, max(1, int(self.cw) + 1), max(1, int(self.ch) + 1)

    def _paint_block(self, i, j):
        st = self.state[j, i]
        self.d.setColor(self.SOIL if st == 0 else (self.ONE if st == 1 else self.MULTI))
        x, y, w, h = self._block_rect(i, j)
        w = min(w, self.ox + self.map_w - x)
        h = min(h, self.oy + self.map_h - y)
        if w > 0 and h > 0:
            self.d.fillRectangle(x, y, w, h)

    def _draw_obstacles(self):
        self.d.setColor(0x505050)
        for o in self.f.obstacles:
            px, py = self._to_px(o.x, o.y, 3)
            rr = max(2, int(o.r / self.f.width * self.map_w))
            self.d.fillOval(px, py, rr, rr)

    def _to_px(self, x, y, pad):
        """Field coordinates -> minimap pixel, clamped inside the minimap."""
        px = self.ox + int((x - self.f.x_min) / self.f.width * self.map_w)
        py = self.oy + int((self.f.y_max - y) / self.f.height * self.map_h)
        px = min(max(px, self.ox + pad), self.ox + self.map_w - pad)
        py = min(max(py, self.oy + pad), self.oy + self.map_h - pad)
        return px, py

    def _px_to_block(self, px, py):
        i = int((px - self.ox) / self.cw)
        j = self.NY - 1 - int((py - self.oy) / self.ch)
        return min(max(i, 0), self.NX - 1), min(max(j, 0), self.NY - 1)

    def _update_minimap(self, agent):
        cov = agent.coverage
        cnt = cov.count + cov._pass
        by, bx = cov.ny // self.NY, cov.nx // self.NX
        blocks = cnt[:by * self.NY, :bx * self.NX].reshape(self.NY, by, self.NX, bx)
        frac = (blocks > 0).mean(axis=(1, 3))
        multi = (blocks > 1).mean(axis=(1, 3))
        new = np.where(frac >= 0.5, np.where(multi > 0.5, 2, 1), 0).astype(np.int8)
        changed = np.argwhere(new != self.state)
        self.state = new
        for j, i in changed:
            self._paint_block(i, j)
        if len(changed):
            self._draw_obstacles()
        # robot marker: repaint the blocks under the old marker, draw the new one
        if self.marker is not None:          # fillOval(cx, cy, rx, ry): erase +/-4 px
            mx, my = self.marker
            i0, j1 = self._px_to_block(mx - 4, my - 4)
            i1, j0 = self._px_to_block(mx + 4, my + 4)
            for i in range(i0, i1 + 1):
                for j in range(j0, j1 + 1):
                    self._paint_block(i, j)
            self._draw_obstacles()
        x, y = agent.metrics.prev if agent.metrics.prev else self.f.start[:2]
        px, py = self._to_px(x, y, 5)
        self.d.setColor(0xFF3B30)
        self.d.fillOval(px, py, 3, 3)
        self.marker = (px, py)

    # ------------------------------------------------------------------ panel
    def draw(self, agent, cmd, sim_time):
        d = self.d
        d.setColor(self.BG)
        d.fillRectangle(0, 26, self.w, 86)
        st = agent.coverage.stats()
        lanes_total = len(self.f.lanes())
        lane = int(agent.path.lane[agent.tracker.idx])
        d.setFont("Arial", 11, True)
        d.setColor(self.WARN if cmd.state in ("AVOID", "REFLEX") else self.FG)
        d.drawText("State: %-7s  Sprayer: %s" % (cmd.state.split()[0], "ON" if cmd.spray else "off"), 10, 30)
        d.setColor(self.FG)
        d.drawText("Method: %s / %s turns" % (agent.method, agent.turn), 10, 46)
        d.drawText("Lane %d of %d    t = %5.1f s" % (max(lane, 0) + 1, lanes_total, sim_time), 10, 62)
        d.drawText("Path %.1f m    Obstacles mapped: %d" % (agent.metrics.path_len, len(agent.omap.confirmed())), 10, 78)
        d.drawText("Coverage %.1f %%" % st["coverage_pct"], 10, 96)
        bx, by, bw, bh = 130, 98, self.w - 140, 10
        d.setColor(0x3A4A3E)
        d.fillRectangle(bx, by, bw, bh)
        fill = int(bw * min(1.0, st["coverage_pct"] / 100.0))
        if fill > 0:
            d.setColor(self.ACCENT)
            d.fillRectangle(bx, by, fill, bh)
        self._update_minimap(agent)


def main():
    robot = Supervisor()
    dt_ms = int(robot.getBasicTimeStep())
    dt = dt_ms / 1000.0

    method = get_arg("method", "proposed")
    turn = get_arg("turn", "arc")
    quit_when_done = get_arg("quit", "0") == "1"
    run_name = get_arg("run", "webots_%s_%s" % (method, turn))

    field = Field.load(os.path.join(ROOT, "worlds", "field_config.json"))

    motors = [robot.getDevice(n) for n in ("front left wheel", "back left wheel",
                                           "front right wheel", "back right wheel")]
    for m in motors:
        m.setPosition(float("inf"))
        m.setVelocity(0.0)
    lidar = robot.getDevice("lidar")
    lidar.enable(dt_ms)
    gps = robot.getDevice("gps")
    gps.enable(dt_ms)
    imu = robot.getDevice("imu")
    imu.enable(dt_ms)
    pen = None if os.environ.get("AGRI_NOPEN") else robot.getDevice("sprayer")
    display = None if os.environ.get("AGRI_NODASH") else robot.getDevice("dashboard")
    dash = Dashboard(display, field) if display else None

    n_rays = lidar.getHorizontalResolution()
    fov = lidar.getFov()
    agent = CoverageAgent(field, method=method, turn=turn, angles=lidar_angles(n_rays, fov),
                          max_range=min(8.0, lidar.getMaxRange()))

    out_dir = os.path.join(ROOT, "results", run_name)
    os.makedirs(out_dir, exist_ok=True)
    log_f = open(os.path.join(out_dir, "trajectory.csv"), "w", newline="")
    log = csv.writer(log_f)
    log.writerow(["t", "x", "y", "yaw", "v", "w", "spray", "state", "front_clearance", "pitch_deg"])

    camera = Camera(robot, get_arg("shots", ""))
    print("[agri] method=%s turn=%s lanes=%d obstacles=%d" %
          (method, turn, len(field.lanes()), len(field.obstacles)))
    wall0 = time.time()
    step = 0
    pen_on = False
    finished = False
    timed_out = False
    t_max = float(get_arg("tmax", "1500"))
    dump_t = float(get_arg("scandump", "-1"))   # save one raw scan at this time (report figure)
    while robot.step(dt_ms) != -1:
        t = robot.getTime()
        if step < 3:                      # let GPS / IMU produce their first values
            step += 1
            continue
        p = gps.getValues()
        roll, pitch, yaw = imu.getRollPitchYaw()
        pose = (p[0], p[1], yaw)
        ranges = np.array(lidar.getRangeImage(), float)
        # drop returns from crop tops / soil when the body pitches or rolls
        ranges = filter_low_returns(ranges, agent.angles, roll, pitch)
        cmd = agent.step(pose, ranges, dt)
        if dump_t > 0 and abs(t - dump_t) < dt / 2:
            np.savez(os.path.join(out_dir, "scan_t%.0f.npz" % dump_t),
                     raw=np.array(lidar.getRangeImage(), float), filtered=ranges,
                     angles=agent.angles, pose=np.array(pose),
                     obstacles=np.array([(o.x, o.y, o.radius, o.hits) for o in agent.omap.obstacles]))

        # differential-drive mapping v, w -> wheel speeds
        vl = (cmd.v - cmd.w * TRACK_EFF / 2) / WHEEL_R
        vr = (cmd.v + cmd.w * TRACK_EFF / 2) / WHEEL_R
        s = max(1.0, abs(vl) / WHEEL_MAX, abs(vr) / WHEEL_MAX)
        vl, vr = vl / s, vr / s
        motors[0].setVelocity(vl)
        motors[1].setVelocity(vl)
        motors[2].setVelocity(vr)
        motors[3].setVelocity(vr)

        if pen and cmd.spray != pen_on:
            pen.write(cmd.spray)
            pen_on = cmd.spray

        if step % 5 == 0:
            fc = float(np.min(ranges[np.isfinite(ranges)])) if np.isfinite(ranges).any() else -1
            log.writerow(["%.3f" % t, "%.4f" % p[0], "%.4f" % p[1], "%.4f" % yaw, "%.3f" % cmd.v,
                          "%.3f" % cmd.w, int(cmd.spray), cmd.state, "%.3f" % fc,
                          "%.2f" % math.degrees(pitch)])
        camera.update(t, pose, out_dir)
        if step % 1500 == 0:
            log_f.flush()
            st = agent.coverage.stats()
            print("[agri] t=%6.1f s  %-8s coverage %5.1f %%  path %6.1f m" %
                  (t, cmd.state, st["coverage_pct"], agent.metrics.path_len))
        if dash and step % 24 == 0:
            dash.draw(agent, cmd, t)
        step += 1

        if agent.done and not finished:
            finished = True
            for m in motors:
                m.setVelocity(0.0)
            if pen:
                pen.write(False)
            summary = agent.summary()
            summary["platform"] = "webots"
            summary["wall_clock_s"] = time.time() - wall0
            with open(os.path.join(out_dir, "summary.json"), "w") as f:
                json.dump(summary, f, indent=2)
            np.save(os.path.join(out_dir, "coverage_count.npy"), agent.coverage.count)
            np.save(os.path.join(out_dir, "path_driven_plan.npy"), agent.path.pts)
            log_f.flush()
            if dash:
                dash.draw(agent, cmd, t)
                ir = display.imageCopy(0, 0, display.getWidth(), display.getHeight())
                display.imageSave(ir, os.path.join(out_dir, "dashboard_end.png"))
                display.imageDelete(ir)
            print("[agri] DONE  coverage %.2f %%  path %.1f m  time %.1f s  detours %d  collisions %d"
                  % (summary["coverage_pct"], summary["path_m"], summary["time_s"],
                     summary["detours"], summary["collisions"]))
            camera.final(pose, out_dir)
            if quit_when_done:
                log_f.close()
                robot.simulationQuit(0)
                break
        if t > t_max and not finished and not timed_out:
            timed_out = True
            with open(os.path.join(out_dir, "summary.json"), "w") as f:
                json.dump(agent.summary(), f, indent=2)
            print("[agri] time limit reached")
            if quit_when_done:
                robot.simulationQuit(1)
                break
    if not log_f.closed:
        log_f.close()


if __name__ == "__main__":
    main()
