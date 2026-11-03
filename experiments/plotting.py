"""Shared matplotlib style and plots for the report (owner: M5)."""

import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt   # noqa: E402
import numpy as np                # noqa: E402
from matplotlib.colors import ListedColormap   # noqa: E402
from matplotlib.patches import Circle, Rectangle   # noqa: E402

# one consistent palette for every figure
C_PROPOSED = "#1f6f4a"   # deep green
C_BASELINE = "#c0532a"   # burnt orange
C_PLAN = "#6b7280"       # grey
C_SOIL = "#efe6d8"
C_STONE = "#7c7166"
C_POLE = "#3b3b3b"

plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "DejaVu Serif"],
    "font.size": 10,
    "axes.titlesize": 11,
    "axes.labelsize": 10,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "axes.grid": True,
    "grid.alpha": 0.25,
    "legend.frameon": False,
    "savefig.dpi": 200,
    "savefig.bbox": "tight",
})


def draw_field(ax, field, soil=True, rows=True):
    x0, x1, y0, y1 = field.soil_bounds
    if soil:
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, color=C_SOIL, zorder=0))
    ax.add_patch(Rectangle((field.x_min, field.y_min), field.width, field.height,
                           fill=False, ec="#8a6d3b", lw=1.2, ls="--", zorder=1))
    if rows:
        for yr in field.crop_rows():
            ax.plot([field.x_min, field.x_max], [yr, yr], color="#9bc59d", lw=0.8, zorder=1)
    for o in field.obstacles:
        ax.add_patch(Circle((o.x, o.y), o.r, color=C_STONE if o.kind == "stone" else C_POLE,
                            zorder=5))
    ax.set_aspect("equal")
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")


def plot_run(field, agent, traj, out, title=""):
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.9))
    ax = axes[0]
    draw_field(ax, field)
    p = agent.planned_path.pts
    ax.plot(p[:, 0], p[:, 1], color=C_PLAN, lw=0.8, ls=":", label="planned path", zorder=2)
    t = np.array([(r[1], r[2]) for r in traj])
    ax.plot(t[:, 0], t[:, 1], color=C_PROPOSED if agent.method == "proposed" else C_BASELINE,
            lw=1.3, label="driven path", zorder=3)
    ax.plot(*field.start[:2], "k^", ms=6, label="start", zorder=6)
    ax.set_title("Trajectory (%s)" % title)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.15), ncol=3, fontsize=8)
    draw_coverage(axes[1], field, agent)
    fig.savefig(out)
    plt.close(fig)


def draw_coverage(ax, field, agent, title=None):
    cov = agent.coverage
    cnt = cov.count + cov._pass
    img = np.zeros(cnt.shape)
    img[cnt == 1] = 1
    img[cnt >= 2] = 2
    img[~cov.free] = 3
    cmap = ListedColormap(["#d64545", "#7fbf7b", "#2f7d4f", "#555555"])
    ax.imshow(img, origin="lower", cmap=cmap, vmin=-0.5, vmax=3.5, interpolation="nearest",
              extent=(field.x_min, field.x_max, field.y_min, field.y_max))
    st = cov.stats()
    ax.set_title(title or "Coverage %.2f %%  (overlap %.1f %%)" % (st["coverage_pct"], st["overlap_pct"]))
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(False)
    from matplotlib.patches import Patch
    ax.legend(handles=[Patch(color="#d64545", label="missed"), Patch(color="#7fbf7b", label="1 pass"),
                       Patch(color="#2f7d4f", label="2+ passes"), Patch(color="#555555", label="obstacle")],
              loc="upper center", bbox_to_anchor=(0.5, -0.15), ncol=4, fontsize=8)


def savefig(fig, path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fig.savefig(path)
    plt.close(fig)
