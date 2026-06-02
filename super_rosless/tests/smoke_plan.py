import pathlib
import sys

import numpy as np


ROOT = pathlib.Path(__file__).resolve().parents[2]
BUILD = ROOT / "super_rosless" / "build"
sys.path.insert(0, str(BUILD))

import super_planner_py as super  # noqa: E402


def main():
    planner = super.PlannerSession(str(ROOT / "super_planner/config/static_high_speed.yaml"))
    state = {
        "position": [0.0, 0.0, 1.5],
        "velocity": [0.0, 0.0, 0.0],
        "acceleration": [0.0, 0.0, 0.0],
        "jerk": [0.0, 0.0, 0.0],
        "quat": [1.0, 0.0, 0.0, 0.0],
        "yaw": 0.0,
    }
    print(planner.update_sensing(np.empty((0, 3), dtype=np.float64), state, 0.0))
    print(planner.set_goal([5.0, 0.0, 1.5]))
    for t in np.linspace(0.0, 0.8, 9):
        result = planner.step(float(t))
        print(t, result)
        if result["new_trajectory"]:
            break
    traj = planner.get_trajectory()
    print("pieces", traj["position"]["piece_count"], "duration", traj["position"]["total_duration"])
    cmd = planner.sample_command(0.1)
    print("cmd_position", cmd["position"], "yaw", cmd["yaw"])
    assert traj["position"]["piece_count"] > 0


if __name__ == "__main__":
    main()
