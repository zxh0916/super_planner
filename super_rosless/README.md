# SUPER ROS-less Python Binding

This directory builds a ROS-free shared object for the existing SUPER planner
without modifying `super_planner/` or `rog_map/`.

Build in the requested conda environment:

```bash
conda run -n super_ros cmake -S super_rosless -B super_rosless/build
conda run -n super_ros cmake --build super_rosless/build -j
```

Smoke test:

```bash
conda run -n super_ros python super_rosless/tests/smoke_plan.py
```

The module exposes `super_planner_py.PlannerSession`, matching the callable
split described in `refactor.md`: load a static PCD, update state/sensing, set a
goal, step the replanning state machine, export trajectories, and sample a
flatness-level command.
