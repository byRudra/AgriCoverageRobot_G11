"""agri - core library of the Agricultural Field Robot for Crop-Row Coverage (AISR P04, G11).

The same code runs inside the Webots controller and in the headless simulator,
so every result in the report is produced by exactly the algorithm shown in the demo.

Modules (one owner per module, see README):
    field       - field model, random field generator, JSON I/O          (M1)
    planner     - boustrophedon coverage path and headland turns          (M2)
    navigation  - PID heading control and pure-pursuit path tracking      (M3)
    avoidance   - lidar obstacle mapping, geometric detour, APF baseline  (M4)
    coverage    - coverage grid, spray gating and run metrics             (M5)
    agent       - layered agent that ties all modules together
"""

__version__ = "1.0.0"
