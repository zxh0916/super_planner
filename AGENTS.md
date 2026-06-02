# Repository Guidelines

## Project Structure & Module Organization

This repository is a ROS-based UAV planning workspace. Core planning code lives in `super_planner/`, with public headers under `super_planner/include/`, implementations under `super_planner/src/`, launch files in `super_planner/launch/`, and runtime configuration in `super_planner/config/`. `mission_planner/` contains waypoint mission entry points, configs, benchmark data, and ROS launch files. `rog_map/` provides mapping support. `mars_uav_sim/` contains simulation packages, meshes, maps, and simulator configuration. `misc/` stores paper and README media assets; avoid changing it unless documentation assets are required.

## Build, Test, and Development Commands

Select the target ROS API before building:

```bash
bash scripts/select_ros_version.sh ROS1
bash scripts/select_ros_version.sh ROS2
```

For ROS1 Noetic, build from the workspace root that contains this repository in `src/`:

```bash
catkin_make -DBUILD_TYPE=Release
source devel/setup.bash
roslaunch mission_planner benchmark_high_speed.launch
```

For ROS2 Foxy:

```bash
colcon build --symlink-install
source install/local_setup.bash
ros2 launch mission_planner benchmark_dense.launch.py
```

Use `roslaunch mission_planner click_demo.launch` or `ros2 launch mission_planner click_demo.launch.py` for interactive RViz testing.

## Coding Style & Naming Conventions

Use C++17-compatible, ROS-friendly C++. Follow the surrounding file style: two-space indentation in CMake/XML/YAML where present, compact C++ namespaces, `snake_case` for files and functions, and `PascalCase` only for types that already follow that pattern. Keep ROS-specific adapters in `include/ros_interface/` or package-level `Apps/`; keep planner logic in `src/super_core/`, trajectory optimization in `src/traj_opt/`, and reusable math/utilities in `src/utils/`. Do not introduce new abstraction layers unless current code duplication or boundary pressure justifies them.

## Testing Guidelines

There is no unified test harness in the repository. Validate changes by building the affected ROS version and running the nearest launch scenario. For planner behavior, prefer `benchmark_high_speed` and `benchmark_dense`; for UI or goal-setting changes, use `click_demo`. If adding tests, place them in the relevant package test directory and make them runnable through the existing ROS build tool.

## Commit & Pull Request Guidelines

Recent history uses short messages such as `[fea] minor typo in readme`, `[fea+fix] ...`, and `Update README.md`. Keep commits focused and use a concise prefix when useful, for example `[fix]`, `[fea]`, or `[docs]`. Pull requests should describe the behavior change, list tested ROS version and launch command, link related issues, and include screenshots or RViz recordings for visualization changes.

## Agent-Specific Instructions

Prefer the smallest correct change. Preserve package boundaries, avoid speculative helpers, and remove temporary scaffolding before handoff. Flag changes that affect public ROS topics, message contracts, dependencies, schemas, or cross-package responsibilities.

## ROS-less SUPER Core Feature

`super_rosless/` implements the feature described in `refactor.md`: it exposes SUPER's core replanning loop as a ROS-free C++ library with a pybind11 Python module, while leaving the original `super_planner/` and `rog_map/` packages unchanged. The implementation keeps the existing planner, optimizer, trajectory, and ROG-Map core code, and adds only local compatibility shims for ROS/PCL/YAML-facing headers needed by the standalone build.

The main entry point is `super_planner_py.PlannerSession`. It intentionally keeps planner state instead of forcing the whole pipeline into one stateless function, because SUPER's committed trajectory, backup trajectory, map memory, goal state, and replanning timers are part of the core behavior. The callable split is:

- `PlannerSession(config_path)` initializes config, map, planner, and local FSM state.
- `load_static_pcd(pcd_path, clear=True)` loads a static ASCII PCD map into ROG-Map.
- `update_sensing(points, state, time_s)` updates local point cloud sensing and robot state.
- `set_goal(position, yaw=None)` sets the planning target.
- `step(time_s)` advances the replanning state machine once.
- `get_trajectory()` exports polynomial trajectory durations and coefficients.
- `sample_command(time_s)` samples flatness-level command output.
- `reset(clear_map=False)` and `get_debug_state()` support experiments and diagnostics.

Build the module without catkin or colcon from the repository root:

```bash
conda run -n super_ros cmake -S super_rosless -B super_rosless/build
conda run -n super_ros cmake --build super_rosless/build -j$(nproc)
```

Run the smoke test:

```bash
conda run -n super_ros python super_rosless/tests/smoke_plan.py
```

Minimal Python usage:

```python
import sys
import numpy as np

sys.path.insert(0, "super_rosless/build")
import super_planner_py as super

planner = super.PlannerSession("super_planner/config/static_high_speed.yaml")
state = {
    "position": [0.0, 0.0, 1.5],
    "velocity": [0.0, 0.0, 0.0],
    "acceleration": [0.0, 0.0, 0.0],
    "jerk": [0.0, 0.0, 0.0],
    "quat": [1.0, 0.0, 0.0, 0.0],
    "yaw": 0.0,
}

planner.update_sensing(np.empty((0, 3)), state, 0.0)
planner.set_goal([5.0, 0.0, 1.5])
planner.step(0.0)
planner.step(0.1)
trajectory = planner.get_trajectory()
command = planner.sample_command(0.1)
```

Validation performed for this feature included the Python smoke test, static PCD loading, and a Docker ROS1 Noetic reference comparison against the original planner. The ROS-less trajectory matched the ROS reference within `1e-4` absolute tolerance.

Known limitation: `ROGMap::initProbMap()` has function-local static initialization in the original core, so the current standalone wrapper is intended for one active `PlannerSession` per Python process. `reset(clear_map=True)` does not fully reconstruct map memory without changing upstream `rog_map` internals.
