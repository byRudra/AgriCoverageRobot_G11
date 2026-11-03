"""Generate every figure used in the report and slides from saved results (owner: M5).

    python experiments/make_figures.py

Needs: results/e1_monte_carlo.csv, e2_density.csv, e3_tool_width.csv, e4_curves.npy,
results/webots_<method>_arc/ (trajectory.csv, coverage_count.npy), results/shots_*/,
results/scan_dump/scan_t18.npz
"""

import csv
import json
import math
import os
import shutil
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from agri.avoidance import remove_isolated, scan_to_points   # noqa: E402
from agri.coverage import CoverageGrid      # noqa: E402
from agri.field import Field  # noqa: E402
from agri.planner import plan_coverage_path  # noqa: E402
from experiments import plotting as P       # noqa: E402
from experiments.plotting import plt        # noqa: E402
from matplotlib.patches import Circle   # noqa: E402

RES = os.path.join(ROOT, "results")
FIG = os.path.join(ROOT, "report", "figures")
COL = {"proposed": P.C_PROPOSED, "bug2": "#2b6cb0", "apf": P.C_BASELINE}
LABEL = {"proposed": "Proposed (geometric detour)", "bug2": "Bug2 boundary following",
         "apf": "Reactive APF"}
SHORT = {"proposed": "Proposed", "bug2": "Bug2", "apf": "APF"}
METHODS = ["proposed", "bug2", "apf"]


def read_csv(name):
    with open(os.path.join(RES, name)) as f:
        return list(csv.DictReader(f))


def fnum(r, k):
    v = r[k]
    return float(v) if v not in ("", "inf") else float("inf")


def save(fig, name):
    P.savefig(fig, os.path.join(FIG, name))
    print("  ", name)


# ---------------------------------------------------------------------- figures
def fig_plan_turns(field):
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.6))
    for ax, turn in zip(axes, ("arc", "pivot")):
        P.draw_field(ax, field)
        path = plan_coverage_path(field, turn=turn)
        on = path.lane >= 0
        p = path.pts
        ax.plot(p[:, 0], p[:, 1], color="#9aa3ad", lw=0.9, zorder=2)
        ax.scatter(p[on, 0], p[on, 1], s=1.2, color=P.C_PROPOSED, zorder=3, label="lane (spray on)")
        ax.scatter(p[~on, 0], p[~on, 1], s=1.2, color="#d97706", zorder=3, label="turn / transit (spray off)")
        if turn == "pivot":
            c = path.corner
            ax.scatter(p[c, 0], p[c, 1], s=12, marker="s", color="#7c3aed", zorder=4, label="pivot vertex")
        ax.plot(*field.start[:2], "k^", ms=6, zorder=6)
        ax.set_title("Boustrophedon plan with %s headland turns (%.0f m)" % (turn, path.length))
        ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.17), ncol=3, fontsize=8, markerscale=5)
    save(fig, "fig_plan_turns.png")


def fig_detour_geometry():
    fig, ax = plt.subplots(figsize=(6.4, 3.6))
    c, r, rr, m = np.array([0.0, 0.12]), 0.36, 0.32, 0.30
    R = r + rr + m
    ax.axhline(0, color=P.C_PLAN, ls=":", lw=1.2)
    ax.text(-2.45, 0.05, "planned lane (M-line)", fontsize=9, color="#4b5563")
    ax.add_patch(Circle(c, r, color=P.C_STONE))
    ax.add_patch(Circle(c, R, fill=False, ec="#b91c1c", ls="--", lw=1.2))
    ax.text(c[0] + 0.05, c[1] - 0.06, "stone", color="white", fontsize=8, ha="center")
    # entry / exit points on the lane
    dx = math.sqrt(R ** 2 - c[1] ** 2)
    pe, px = np.array([-dx, 0]), np.array([dx, 0])
    a0 = math.atan2(pe[1] - c[1], pe[0] - c[0])
    a1 = math.atan2(px[1] - c[1], px[0] - c[0])
    a = np.linspace(a0, a1 + 2 * math.pi if a1 < a0 else a1, 60)   # ccw (below)
    a_cw = np.linspace(a0, a1 - 2 * math.pi if a1 > a0 else a1, 60)
    arc_ccw = np.column_stack([c[0] + R * np.cos(a), c[1] + R * np.sin(a)])
    arc_cw = np.column_stack([c[0] + R * np.cos(a_cw), c[1] + R * np.sin(a_cw)])
    short, long_ = (arc_ccw, arc_cw) if len(a) and abs(a[-1] - a[0]) < abs(a_cw[-1] - a_cw[0]) else (arc_cw, arc_ccw)
    ax.plot(long_[:, 0], long_[:, 1], color="#9ca3af", lw=1.2, ls="--", label="longer side (rejected)")
    ax.plot([-2.5, pe[0]], [0, 0], color=P.C_PROPOSED, lw=2.2)
    ax.plot(short[:, 0], short[:, 1], color=P.C_PROPOSED, lw=2.2, label="spliced detour arc")
    ax.plot([px[0], 2.5], [0, 0], color=P.C_PROPOSED, lw=2.2)
    ax.plot(*pe, "o", color="#111827", ms=5)
    ax.plot(*px, "o", color="#111827", ms=5)
    ax.annotate("entry $p_e$", pe, (pe[0] - 0.75, -0.55), fontsize=9, arrowprops=dict(arrowstyle="-", lw=0.6))
    ax.annotate("exit $p_x$", px, (px[0] + 0.25, -0.55), fontsize=9, arrowprops=dict(arrowstyle="-", lw=0.6))
    ax.annotate("", c + np.array([R * math.cos(2.3), R * math.sin(2.3)]), c,
                arrowprops=dict(arrowstyle="->", lw=0.9, color="#b91c1c"))
    ax.text(-0.85, 0.88, r"$R = \hat r + r_{robot} + m$", color="#b91c1c", fontsize=10)
    ax.set_aspect("equal")
    ax.set_xlim(-2.5, 2.5)
    ax.set_ylim(-1.05, 1.35)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.legend(loc="upper right", fontsize=8)
    ax.set_title("Geometric detour: replace the blocked lane segment by an arc")
    save(fig, "fig_detour_geometry.png")


def fig_perception(field):
    d = np.load(os.path.join(RES, "scan_dump", "scan_t18.npz"))
    pose = d["pose"]
    ang = d["angles"]
    lp = (pose[0] + 0.145 * math.cos(pose[2]), pose[1] + 0.145 * math.sin(pose[2]), pose[2])
    raw = d["raw"]
    clean = remove_isolated(raw)
    pts_raw, v_raw = scan_to_points(raw, ang, lp, 8.0)
    pts, v = scan_to_points(clean, ang, lp, 8.0)
    ghosts = v_raw & ~v
    fig, ax = plt.subplots(figsize=(9, 4.2))
    P.draw_field(ax, field)
    for o in field.obstacles:   # ground truth outline
        ax.add_patch(Circle((o.x, o.y), o.r, fill=False, ec="#111827", lw=0.6, zorder=6))
    # rays
    for i in np.where(v_raw)[0]:
        ax.plot([lp[0], pts_raw[i, 0]], [lp[1], pts_raw[i, 1]], color="#f59e0b", lw=0.3, alpha=0.6, zorder=2)
    ax.scatter(pts[v, 0], pts[v, 1], s=10, color="#dc2626", zorder=7, label="lidar returns (kept)")
    if ghosts.any():
        ax.scatter(pts_raw[ghosts, 0], pts_raw[ghosts, 1], s=30, marker="x", color="#7c3aed", zorder=7,
                   label="isolated ghost returns (removed)")
    for o in d["obstacles"]:
        if o[3] >= 8:
            ax.add_patch(Circle((o[0], o[1]), o[2], fill=False, ec=P.C_PROPOSED, lw=1.6, zorder=8))
            ax.add_patch(Circle((o[0], o[1]), o[2] + 0.62, fill=False, ec=P.C_PROPOSED, lw=0.8, ls="--", zorder=8))
    ax.plot([], [], color=P.C_PROPOSED, lw=1.6, label="mapped obstacle (circle fit)")
    ax.plot([], [], color=P.C_PROPOSED, lw=0.8, ls="--", label="inflated safety circle R")
    ax.plot(pose[0], pose[1], marker=(3, 0, math.degrees(pose[2]) - 90), ms=11, color="k", zorder=9, ls="")
    ax.set_xlim(-6.5, 9)
    ax.set_ylim(-6, 1.8)
    ax.set_title("Webots lidar scan at t = 17.6 s (robot on lane 1) and the obstacle map built so far")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.16), ncol=3, fontsize=8)
    save(fig, "fig_perception.png")


def load_webots(method):
    run = os.path.join(RES, "webots_%s_arc" % method)
    rows = list(csv.DictReader(open(os.path.join(run, "trajectory.csv"))))
    summ = json.load(open(os.path.join(run, "summary.json")))
    cnt = np.load(os.path.join(run, "coverage_count.npy"))
    return rows, summ, cnt


def fig_webots_traj(field):
    fig, axes = plt.subplots(2, 3, figsize=(13, 6.4))
    from matplotlib.colors import ListedColormap
    cmap = ListedColormap(["#d64545", "#7fbf7b", "#2f7d4f", "#555555"])
    for k, m in enumerate(METHODS):
        rows, summ, cnt = load_webots(m)
        x = np.array([float(r["x"]) for r in rows])
        y = np.array([float(r["y"]) for r in rows])
        ax = axes[0, k]
        P.draw_field(ax, field)
        ax.plot(x, y, color=COL[m], lw=1.1)
        ax.set_title("%s\n%.1f s, %.1f m, min clearance %.2f m" % (LABEL[m], summ["time_s"], summ["path_m"],
                                                                 summ["min_clearance_m"]), fontsize=10)
        ax.set_xlim(-11, 11)
        ax.set_ylim(-6.5, 6.5)
        g = CoverageGrid(field)
        img = np.zeros(cnt.shape)
        img[cnt == 1] = 1
        img[cnt >= 2] = 2
        img[~g.free] = 3
        ax = axes[1, k]
        ax.imshow(img, origin="lower", cmap=cmap, vmin=-0.5, vmax=3.5, interpolation="nearest",
                  extent=(field.x_min, field.x_max, field.y_min, field.y_max))
        ax.set_title("Coverage %.2f %%, missed %.2f m$^2$" % (summ["coverage_pct"], summ["missed_m2"]), fontsize=10)
        ax.grid(False)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
    from matplotlib.patches import Patch
    fig.legend(handles=[Patch(color="#d64545", label="missed"), Patch(color="#7fbf7b", label="1 pass"),
                        Patch(color="#2f7d4f", label="2+ passes"), Patch(color="#555555", label="obstacle")],
               loc="lower center", ncol=4, fontsize=9, bbox_to_anchor=(0.5, -0.02))
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    save(fig, "fig_webots_traj.png")


def fig_mc_box():
    e1 = [r for r in read_csv("e1_monte_carlo.csv") if r["turn"] == "arc"]
    metrics = [("coverage_pct", "Coverage of coverable area [%]"), ("time_s", "Completion time [s]"),
               ("min_clearance_m", "Minimum clearance to obstacles [m]"), ("xte_rms_cm", "RMS cross-track error on lanes [cm]")]
    fig, axes = plt.subplots(1, 4, figsize=(13, 3.4))
    for ax, (k, title) in zip(axes, metrics):
        data = [[fnum(r, k) for r in e1 if r["method"] == m] for m in METHODS]
        bp = ax.boxplot(data, patch_artist=True, widths=0.55, showfliers=True,
                        medianprops=dict(color="black", lw=1.2))
        for patch, m in zip(bp["boxes"], METHODS):
            patch.set_facecolor(COL[m])
            patch.set_alpha(0.75)
        for i, (vals, m) in enumerate(zip(data, METHODS)):
            jitter = np.random.default_rng(1).uniform(-0.12, 0.12, len(vals))
            ax.scatter(np.full(len(vals), i + 1) + jitter, vals, s=7, color="black", alpha=0.5, zorder=3)
        ax.set_xticks([1, 2, 3])
        ax.set_xticklabels([SHORT[m] for m in METHODS])
        ax.set_title(title, fontsize=10)
        if k == "coverage_pct":
            ax.set_ylim(0, 102)
        if k == "time_s":
            ax.axhline(1500, color="#b91c1c", ls=":", lw=1)
            ax.text(0.6, 1440, "time limit (not finished)", fontsize=7, color="#b91c1c")
    fig.suptitle("Monte-Carlo evaluation on 20 random fields (6 stones + 4 poles, arc turns)", fontsize=11, y=1.02)
    save(fig, "fig_mc_box.png")


def fig_density():
    e2 = read_csv("e2_density.csv")
    dens = sorted({int(r["n_obs"]) for r in e2})
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.4))
    for m in METHODS:
        cov = [np.mean([fnum(r, "coverage_pct") for r in e2 if r["method"] == m and int(r["n_obs"]) == n]) for n in dens]
        cs = [np.std([fnum(r, "coverage_pct") for r in e2 if r["method"] == m and int(r["n_obs"]) == n]) for n in dens]
        tm = [np.mean([fnum(r, "time_s") for r in e2 if r["method"] == m and int(r["n_obs"]) == n]) for n in dens]
        axes[0].errorbar(dens, cov, yerr=cs, color=COL[m], marker="o", capsize=3, label=SHORT[m])
        axes[1].plot(dens, tm, color=COL[m], marker="o", label=SHORT[m])
    axes[0].set_xlabel("number of obstacles in the field")
    axes[0].set_ylabel("coverage [%]")
    axes[0].set_title("Coverage vs obstacle density (mean $\\pm$ 1 s.d., 10 fields)")
    axes[1].set_xlabel("number of obstacles in the field")
    axes[1].set_ylabel("completion time [s]")
    axes[1].set_title("Completion time vs obstacle density")
    for ax in axes:
        ax.legend()
        ax.set_xticks(dens)
    save(fig, "fig_density.png")


def fig_toolwidth():
    e3 = read_csv("e3_tool_width.csv")
    tw = [float(r["tool_width"]) for r in e3]
    cov = [float(r["coverage_mean"]) for r in e3]
    ov = [float(r["overlap_mean"]) for r in e3]
    fig, ax = plt.subplots(figsize=(6.4, 3.4))
    ax.plot(tw, cov, "o-", color=P.C_PROPOSED, label="coverage")
    ax.set_xlabel("sprayer boom width [m] (lane spacing fixed at 1.0 m)")
    ax.set_ylabel("coverage [%]", color=P.C_PROPOSED)
    ax2 = ax.twinx()
    ax2.plot(tw, ov, "s--", color="#b45309", label="overlap (double-sprayed)")
    ax2.set_ylabel("overlap [% of sprayed area]", color="#b45309")
    ax2.spines["right"].set_visible(True)
    ax2.grid(False)
    ax.axvline(1.2, color="#6b7280", ls=":", lw=1)
    ax.text(1.205, cov[0] + 0.2, "chosen 1.2 m", fontsize=8, color="#374151")
    ax.set_title("Boom width trade-off: coverage vs chemical overlap")
    lines = ax.get_legend_handles_labels()
    lines2 = ax2.get_legend_handles_labels()
    ax.legend(lines[0] + lines2[0], lines[1] + lines2[1], loc="center right", fontsize=8)
    save(fig, "fig_toolwidth.png")


def fig_pid():
    curves = np.load(os.path.join(RES, "e4_curves.npy"), allow_pickle=True)
    fig, ax = plt.subplots(figsize=(6.6, 3.4))
    colors = ["#9ca3af", P.C_PROPOSED, "#b45309"]
    for (name, t, e, w), c in zip(curves, colors):
        ax.plot(t, np.asarray(e) * 100, color=c, lw=1.6 if "used" in name else 1.1, label=name)
    ax.axhline(0, color="black", lw=0.6)
    ax.axhspan(-2, 2, color=P.C_PROPOSED, alpha=0.08, label="$\\pm$5 % band")
    ax.set_xlim(0, 10)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("cross-track error [cm]")
    ax.set_title("Lateral step response: robot starts 40 cm beside the lane")
    ax.legend(fontsize=8)
    save(fig, "fig_pid_step.png")


def fig_timeline():
    rows, summ, _ = load_webots("proposed")
    t = np.array([float(r["t"]) for r in rows])
    v = np.array([float(r["v"]) for r in rows])
    w = np.array([float(r["w"]) for r in rows])
    st = [r["state"].split()[0] for r in rows]
    fig, ax = plt.subplots(figsize=(11, 3.0))
    shade = {"AVOID": "#fde68a", "TURN": "#dbeafe", "PIVOT": "#e9d5ff"}
    i = 0
    while i < len(t):
        j = i
        while j + 1 < len(t) and st[j + 1] == st[i]:
            j += 1
        if st[i] in shade:
            ax.axvspan(t[i], t[j], color=shade[st[i]], lw=0)
        i = j + 1
    ax.plot(t, v, color=P.C_PROPOSED, lw=1, label="forward speed v [m/s]")
    ax.plot(t, w / 4, color="#6b7280", lw=0.6, alpha=0.8, label="yaw rate w / 4 [rad/s]")
    from matplotlib.patches import Patch
    h, l = ax.get_legend_handles_labels()
    h += [Patch(color=shade["AVOID"], label="detour (AVOID)"), Patch(color=shade["TURN"], label="headland turn")]
    ax.legend(handles=h, loc="upper center", bbox_to_anchor=(0.5, -0.22), ncol=4, fontsize=8)
    ax.set_xlabel("simulation time [s]")
    ax.set_xlim(0, t[-1])
    ax.set_title("Webots run (proposed, arc turns): speed scheduling around obstacles and turns")
    save(fig, "fig_timeline.png")


def copy_shots():
    pairs = [("shots_proposed/overview_mid.jpg", "shot_overview.jpg"),
             ("shots_proposed/side_view.jpg", "shot_robot.jpg"),
             ("shots_proposed/follow_detour.jpg", "shot_follow.jpg"),
             ("shots_proposed/top_end.jpg", "shot_top_proposed.jpg"),
             ("shots_bug2/top_end.jpg", "shot_top_bug2.jpg"),
             ("shots_apf/top_end.jpg", "shot_top_apf.jpg"),
             ("shots_proposed/overview_end.jpg", "shot_overview_end.jpg"),
             ("shots_proposed/dashboard_end.png", "shot_dashboard.png"),
             ("shots_proposed/farm_view.jpg", "shot_farm.jpg"),
             ("shots_proposed/follow_poles.jpg", "shot_poles.jpg")]
    for src, dst in pairs:
        s = os.path.join(RES, src)
        if os.path.exists(s):
            shutil.copy(s, os.path.join(FIG, dst))
            print("  ", dst)


def main():
    os.makedirs(FIG, exist_ok=True)
    field = Field.load(os.path.join(ROOT, "worlds", "field_config.json"))
    fig_plan_turns(field)
    fig_detour_geometry()
    fig_perception(field)
    fig_webots_traj(field)
    fig_mc_box()
    fig_density()
    fig_toolwidth()
    fig_pid()
    fig_timeline()
    copy_shots()


if __name__ == "__main__":
    main()
