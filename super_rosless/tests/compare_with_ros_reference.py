import argparse
import pathlib
import re
import sys

import numpy as np


def parse_reference(path):
    text = pathlib.Path(path).read_text(errors="ignore")
    data = {}
    for raw_line in text.splitlines():
        line = re.sub(r"\x1b\[[0-9;]*m", "", raw_line).strip()
        if not line:
            continue
        parts = line.split()
        key = parts[0]
        if key == "RET":
            data["ret"] = int(parts[1])
        elif key.endswith("_START") or key.endswith("_TOTAL"):
            data[key.lower()] = float(parts[1])
        elif key.endswith("_PIECES"):
            data[key.lower()] = int(parts[1])
        elif key.endswith("_DUR"):
            name = key[: -len("_DUR")].lower()
            data.setdefault(name + "_durations", []).append(float(parts[2]))
        elif key.endswith("_COEFF"):
            name = key[: -len("_COEFF")].lower()
            rows = int(parts[2])
            cols = int(parts[3])
            values = np.array([float(x) for x in parts[4:]], dtype=np.float64).reshape(rows, cols)
            data.setdefault(name + "_coeffs", []).append(values)
    for name in ("pos", "yaw"):
        data[name + "_durations"] = np.array(data.get(name + "_durations", []), dtype=np.float64)
        coeffs = data.get(name + "_coeffs", [])
        data[name + "_coeffs"] = np.stack(coeffs, axis=0) if coeffs else np.empty((0, 0, 0))
    return data


def run_rosless(config, module_dir):
    sys.path.insert(0, str(module_dir))
    import super_planner_py as super

    planner = super.PlannerSession(str(config))
    state = {
        "position": [0.0, 0.0, 1.5],
        "velocity": [0.0, 0.0, 0.0],
        "acceleration": [0.0, 0.0, 0.0],
        "jerk": [0.0, 0.0, 0.0],
        "quat": [1.0, 0.0, 0.0, 0.0],
        "yaw": 0.0,
    }
    planner.update_sensing(np.array([[7.0, 50.0, 1.5, 1.0]], dtype=np.float64), state, 0.0)
    planner.set_goal([5.0, 0.0, 1.5])
    planner.step(0.0)
    step = planner.step(0.1)
    traj = planner.get_trajectory()
    return {
        "ret": int(step["ret_code"]),
        "pos_start": float(traj["position"]["start_time"]),
        "pos_total": float(traj["position"]["total_duration"]),
        "pos_pieces": int(traj["position"]["piece_count"]),
        "pos_durations": np.asarray(traj["position"]["durations"], dtype=np.float64),
        "pos_coeffs": np.asarray(traj["position"]["coeffs"], dtype=np.float64),
        "yaw_start": float(traj["yaw"]["start_time"]),
        "yaw_total": float(traj["yaw"]["total_duration"]),
        "yaw_pieces": int(traj["yaw"]["piece_count"]),
        "yaw_durations": np.asarray(traj["yaw"]["durations"], dtype=np.float64),
        "yaw_coeffs": np.asarray(traj["yaw"]["coeffs"], dtype=np.float64),
    }


def assert_close(name, actual, expected, atol):
    if np.shape(actual) != np.shape(expected):
        raise AssertionError(f"{name} shape mismatch: {np.shape(actual)} != {np.shape(expected)}")
    diff = np.max(np.abs(np.asarray(actual) - np.asarray(expected))) if np.size(actual) else 0.0
    if diff > atol:
        raise AssertionError(f"{name} max abs diff {diff:.6g} > {atol:.6g}")
    print(f"{name}: max_abs_diff={diff:.6g}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="/tmp/super_compare.yaml")
    parser.add_argument("--ros-reference", default="/tmp/ros_reference.out")
    parser.add_argument("--module-dir", default="super_rosless/build")
    parser.add_argument("--atol", type=float, default=1e-4)
    args = parser.parse_args()

    ref = parse_reference(args.ros_reference)
    cur = run_rosless(pathlib.Path(args.config), pathlib.Path(args.module_dir).resolve())
    if cur["ret"] != ref["ret"]:
        raise AssertionError(f"ret mismatch: {cur['ret']} != {ref['ret']}")
    for key in ("pos_start", "pos_total", "pos_pieces", "yaw_start", "yaw_total", "yaw_pieces"):
        assert_close(key, cur[key], ref[key], args.atol)
    for key in ("pos_durations", "pos_coeffs", "yaw_durations", "yaw_coeffs"):
        assert_close(key, cur[key], ref[key], args.atol)


if __name__ == "__main__":
    main()
