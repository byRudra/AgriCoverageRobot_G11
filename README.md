# Agricultural Field Robot for Crop-Row Coverage

**AISR (CSAI4014) Project P04 | Group G11 | Batch B4 | UPES Dehradun, October 2026**

A Pioneer 3-AT robot in Webots sprays a 16 m x 10 m crop field lane by lane
(boustrophedon coverage), maps unknown stones and poles with a 2-D lidar, and avoids
them with **lane-preserving geometric detours**. It logs the sprayed area, overlap and path length,
paints the sprayed soil in the 3-D view, and shows a live dashboard.

| Member | SAP ID | Module |
|---|---|---|
| Jahanvi Sengar | 500120543 | Field model and Webots world (`agri/field.py`, `tools/generate_world.py`) |
| Aagman Sharma | 500123462 | Coverage planner, headland turns, result figures (`agri/planner.py`, `experiments/make_figures.py`, `experiments/plotting.py`) |
| Rudra Gupta (team lead) | 500120437 | PID and pure-pursuit tracking, layered agent, Webots controller and integration (`agri/navigation.py`, `agri/agent.py`, `controllers/`) |
| Sahil Agarwal | 500120211 | Lidar perception, detour, Bug2 and APF baselines (`agri/avoidance.py`) |
| Harsh Prakash Sinha | 500123860 | Coverage metrics, headless simulator, experiments (`agri/coverage.py`, `sim/`, `experiments/run_experiments.py`, `experiments/run_webots.py`) |

## Results (20 random fields, arc turns)

| Metric | Proposed | Bug2 | APF |
|---|---|---|---|
| Fields finished | **20 / 20** | 20 / 20 | 15 / 20 |
| Coverage [%] | **98.85 ± 0.14** | 95.41 ± 2.15 | 85.30 ± 26.69 |
| Completion time [s] | **412 ± 2** | 472 ± 50 | 781 ± 437 |
| RMS lane error [cm] | **2.1** | 11.3 | 28.3 |
| Collisions | 0 | 0 | 0 |

Webots (demo field): proposed 98.43 % coverage in 414.6 s, no collisions.

## Requirements

* Webots **R2025a** (the world downloads standard assets such as the Pioneer 3-AT, rocks and trees on first open, so you need internet the first time)
* Python 3.9+ with `numpy`, `matplotlib`, `pillow` (`pip install -r requirements.txt`)
* Optional: `python-pptx` (slides), a LaTeX distribution (report)

Webots must use a Python that has numpy installed: set **Tools > Preferences > Python command**
(for example `python` from Anaconda).

## Run the live demo

1. Open `worlds/crop_field.wbt` in Webots.
2. Press **Run** (real time). The run takes about 7 minutes of simulated time; use **Fast** to speed it up.
3. To change the method, select `AgriBot` in the scene tree and edit `controllerArgs`:
   `--method=proposed|bug2|apf` and `--turn=arc|pivot`, then reset and run again.

The blue area on the soil is what the sprayer has covered. The dashboard (top left) shows the state,
lane, coverage and a coverage minimap. Results are written to `results/webots_<method>_<turn>/`.

## Commands

```bash
# new random field (writes worlds/crop_field.wbt, field_config.json, soil texture)
python tools/generate_world.py --seed 3 --stones 8 --poles 5
python tools/generate_world.py                      # back to the demo field (seed 11)

# headless run of one field, with a trajectory + coverage plot in results/
python sim/headless_sim.py --method proposed --turn arc --plot
python sim/headless_sim.py --method bug2 --seed 7 --plot

# Webots batch runs (fast, no rendering) for all methods and turn types
python experiments/run_webots.py

# all statistics (about 5 min on 8+ cores) and all report figures
python experiments/run_experiments.py
python experiments/make_figures.py

# slides (editable .pptx)
python slides/make_slides.py
```

## Repository layout

```
agri/                    core algorithm (no Webots dependency, shared by Webots and the headless sim)
  field.py               field model, random generator           (M1)
  planner.py             boustrophedon plan, arc / pivot turns     (M2)
  navigation.py          PID + pure pursuit, speed scheduling       (M3)
  avoidance.py           lidar filters, obstacle map, detour, Bug2, APF (M4)
  coverage.py            coverage grid and run metrics            (M5)
  agent.py               layered declarative-procedural-reflexive agent
controllers/agri_coverage_controller/   Webots controller: devices, Pen, dashboard, logging, screenshots
worlds/                  crop_field.wbt, field_config.json, textures/
tools/generate_world.py  world generator
tools/scenery.py         lettuce, obstacles, farmyard and soil texture
sim/headless_sim.py      2-D skid-steer + lidar simulator for batch experiments
experiments/             Monte-Carlo experiments, Webots batch runs, figures
results/                 CSV / JSON results, Webots logs and screenshots
report/                  LaTeX report (main.tex, figures/) and PDF
slides/                  slide generator, .pptx and .pdf
```

## How the proposed method works

1. **Plan:** the whole field is planned as 10 lanes joined by headland arcs. Every path point is tagged lane or turn, so the sprayer is on only on lanes.
2. **Perceive:** each lidar scan has its low returns (crop tops when the body tilts) and isolated ghost returns removed. Consecutive points are clustered and fitted with circles, and these are fused into an obstacle map; an entry counts as confirmed after 8 scans.
3. **Detour:** if a confirmed obstacle blocks the lane ahead, the blocked segment is replaced by an arc of radius `R = r + 0.32 + 0.30 m` on the shorter safe side. The arc ends on the same lane, so the coverage pattern is kept.
4. **Track:** pure pursuit plus a PID heading loop (Kp 2.2, Ki 0.05, Kd 0.12) drives the path. The speed drops near obstacles and in turns, and a reflex layer stops the robot if anything comes within 0.4 m.
