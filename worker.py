from __future__ import annotations

import argparse
import faulthandler
import math
import os
import pickle
import struct
import sys
import traceback
from pathlib import Path

import numpy as np


DIFFAERO_ROOT = Path(__file__).resolve().parent.parent
SUPER_ROOT = DIFFAERO_ROOT / "third_party" / "super_planner"
SUPER_BUILD = SUPER_ROOT / "super_rosless" / "build"


def _load_super_module():
    try:
        import super_planner_py as super  # type: ignore

        return super
    except ModuleNotFoundError:
        pass

    sys.path.insert(0, str(SUPER_ROOT))
    sys.path.insert(0, str(SUPER_BUILD))
    import super_planner_py as super  # type: ignore

    return super


def _finite_vec(value, dim: int) -> list[float]:
    arr = np.asarray(value, dtype=np.float64).reshape(-1)
    if arr.size < dim or not np.isfinite(arr[:dim]).all():
        return [0.0] * dim
    return arr[:dim].astype(float).tolist()


def _fallback_command(state: dict, reason: str) -> dict:
    return {
        "ok": False,
        "message": reason,
        "trajectory_points": [],
        "command": {
            "position": _finite_vec(state.get("position", [0.0, 0.0, 1.0]), 3),
            "velocity": [0.0, 0.0, 0.0],
            "acceleration": [0.0, 0.0, 0.0],
            "jerk": [0.0, 0.0, 0.0],
            "attitude_rpy": [0.0, 0.0, float(state.get("yaw", 0.0)) if math.isfinite(float(state.get("yaw", 0.0))) else 0.0],
            "angular_velocity": [0.0, 0.0, 0.0],
            "trajectory_finished": True,
        },
    }


def _eval_position_piece(coeff: np.ndarray, local_t: float) -> np.ndarray:
    """Match SUPER Piece::getPos(): coeff columns are high order to constant."""
    pos = np.zeros(3, dtype=np.float64)
    tn = 1.0
    for col_idx in range(coeff.shape[1] - 1, -1, -1):
        pos += tn * coeff[:, col_idx]
        tn *= local_t
    return pos


def _eval_position_trajectory(position_trajectory: dict, trajectory_time: float) -> np.ndarray | None:
    durations = np.asarray(position_trajectory.get("durations", []), dtype=np.float64).reshape(-1)
    coeffs = np.asarray(position_trajectory.get("coeffs", []), dtype=np.float64)
    if durations.size == 0 or coeffs.ndim != 3 or coeffs.shape[0] != durations.size or coeffs.shape[1] != 3:
        return None

    total_duration = float(position_trajectory.get("total_duration", np.sum(durations)))
    if not math.isfinite(trajectory_time) or trajectory_time < -1e-9 or trajectory_time > total_duration + 1e-9:
        return None

    local_t = min(max(float(trajectory_time), 0.0), total_duration)
    piece_idx = 0
    while piece_idx < durations.size and local_t > durations[piece_idx]:
        local_t -= float(durations[piece_idx])
        piece_idx += 1
    if piece_idx == durations.size:
        piece_idx -= 1
        local_t += float(durations[piece_idx])
    return _eval_position_piece(coeffs[piece_idx], local_t)


def _sample_position_trajectory(
    position_trajectory: dict,
    sample_world_time: float,
    dt: float,
    max_points: int = 64,
) -> list[list[float]]:
    max_points = max(0, int(max_points))
    if max_points == 0:
        return []

    durations = np.asarray(position_trajectory.get("durations", []), dtype=np.float64).reshape(-1)
    if durations.size == 0:
        return []

    start_time = float(position_trajectory.get("start_time", 0.0))
    total_duration = float(position_trajectory.get("total_duration", np.sum(durations)))
    if not math.isfinite(start_time) or not math.isfinite(total_duration) or total_duration <= 0.0:
        return []

    start_t = max(0.0, float(sample_world_time) - start_time)
    if start_t > total_duration + 1e-9:
        return []

    step = max(float(dt), 0.05)
    if not math.isfinite(step) or step <= 0.0:
        step = 0.05

    times = [min(start_t, total_duration)]
    t = times[0]
    while len(times) < max_points and t + step < total_duration - 1e-9:
        t += step
        times.append(t)
    if abs(times[-1] - total_duration) > 1e-9:
        if len(times) < max_points:
            times.append(total_duration)
        else:
            times[-1] = total_duration

    points: list[list[float]] = []
    for t in times[:max_points]:
        pos = _eval_position_trajectory(position_trajectory, t)
        if pos is None or not np.isfinite(pos).all():
            continue
        points.append(pos.astype(float).tolist())
    return points


def _read_exact(fd: int, n_bytes: int) -> bytes:
    chunks = []
    remaining = n_bytes
    while remaining > 0:
        chunk = os.read(fd, remaining)
        if not chunk:
            raise EOFError("planner worker pipe closed")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def _recv_msg(fd: int):
    size = struct.unpack("!Q", _read_exact(fd, 8))[0]
    return _restore_pickle_payload(pickle.loads(_read_exact(fd, size)))


def _pickle_safe_payload(value):
    if isinstance(value, np.ndarray):
        arr = np.ascontiguousarray(value)
        return {
            "__diffaero_ndarray__": True,
            "dtype": arr.dtype.str,
            "shape": arr.shape,
            "data": arr.tobytes(),
        }
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {key: _pickle_safe_payload(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_pickle_safe_payload(item) for item in value]
    return value


def _restore_pickle_payload(value):
    if isinstance(value, dict):
        if value.get("__diffaero_ndarray__"):
            arr = np.frombuffer(value["data"], dtype=np.dtype(value["dtype"]))
            return arr.reshape(tuple(value["shape"])).copy()
        return {key: _restore_pickle_payload(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_restore_pickle_payload(item) for item in value]
    return value


def _send_msg(fd: int, payload) -> None:
    data = pickle.dumps(_pickle_safe_payload(payload), protocol=pickle.HIGHEST_PROTOCOL)
    for chunk in (struct.pack("!Q", len(data)), data):
        view = memoryview(chunk)
        while view:
            written = os.write(fd, view)
            view = view[written:]


def main() -> None:
    faulthandler.enable(file=sys.stderr, all_threads=True)

    parser = argparse.ArgumentParser()
    parser.add_argument("--read-fd", type=int, required=True)
    parser.add_argument("--write-fd", type=int, required=True)
    parser.add_argument("--config", required=True)
    args = parser.parse_args()

    try:
        super = _load_super_module()
        planner = super.PlannerSession(args.config)
    except Exception as exc:  # pragma: no cover - worker initialization boundary
        _send_msg(args.write_fd, {
            "ok": False,
            "message": f"worker initialization failed: {type(exc).__name__}: {exc}",
            "traceback": traceback.format_exc(),
        })
        return

    has_goal = False
    has_trajectory = False
    last_step_time = -float("inf")

    while True:
        request = _recv_msg(args.read_fd)
        op = request.get("op")
        try:
            if op == "close":
                _send_msg(args.write_fd, {"ok": True})
                break

            if op == "set_map":
                point_cloud = np.asarray(request["point_cloud"], dtype=np.float64).reshape(-1, 3)
                map_result = planner.load_static_points(point_cloud, True)
                if not bool(map_result.get("success", False)):
                    _send_msg(args.write_fd, {"ok": False, "message": map_result.get("message", "map setup failed")})
                    continue
                planner.reset(False)
                has_goal = False
                has_trajectory = False
                last_step_time = -float("inf")
                _send_msg(
                    args.write_fd,
                    {"ok": True, "point_count": int(map_result.get("point_count", point_cloud.shape[0]))},
                )
                continue

            if op != "step":
                _send_msg(args.write_fd, {"ok": False, "message": f"unknown op: {op}"})
                continue

            state = request["state"]
            target = _finite_vec(request["target"], 3)
            time_s = float(request["time_s"])
            dt = float(request["dt"])
            replan_dt = float(request["replan_dt"])
            force_reset = bool(request.get("force_reset", False))
            periodic_replan = replan_dt > 0.0

            if force_reset:
                planner.reset(False)
                has_goal = False
                has_trajectory = False
                last_step_time = -float("inf")

            # The static scene map is loaded by set_map. During replanning,
            # update_sensing only refreshes SUPER's robot state.
            should_step = (
                force_reset
                or (periodic_replan and not has_trajectory)
                or (periodic_replan and time_s - last_step_time >= replan_dt - 1e-9)
            )
            if should_step:
                update_result = planner.update_sensing(state, time_s)
                if not bool(update_result.get("success", False)):
                    _send_msg(args.write_fd, _fallback_command(state, update_result.get("message", "sensing update failed")))
                    continue

                if force_reset or not has_goal:
                    goal_result = planner.set_goal(target)
                    has_goal = bool(goal_result.get("accepted", False))
                    if not has_goal:
                        _send_msg(args.write_fd, _fallback_command(state, goal_result.get("message", "goal rejected")))
                        continue

                new_trajectory = False
                message = ""
                # The ROS-less FSM first consumes the new goal, then generates
                # a trajectory on a later tick. Drive a few ticks synchronously
                # so a freshly reset episode can move immediately.
                for i in range(6 if not has_trajectory else 1):
                    step_time = time_s + i * max(replan_dt, dt)
                    step_result = planner.step(step_time)
                    message = str(step_result.get("message", ""))
                    new_trajectory = new_trajectory or bool(step_result.get("new_trajectory", False))
                    has_trajectory = has_trajectory or new_trajectory
                    last_step_time = step_time
                    if has_trajectory:
                        break

                if not has_trajectory:
                    _send_msg(args.write_fd, _fallback_command(state, message or "planner has no trajectory"))
                    continue

            if not has_trajectory:
                _send_msg(args.write_fd, _fallback_command(state, "planner has no trajectory"))
                continue

            command = planner.sample_command(time_s + dt)
            if bool(command.get("trajectory_finished", False)) or bool(command.get("on_backup_trajectory", False)):
                trajectory_points = []
            else:
                trajectory = planner.get_trajectory()
                trajectory_points = _sample_position_trajectory(
                    trajectory.get("position", {}),
                    time_s + dt,
                    dt,
                )
            _send_msg(
                args.write_fd,
                {
                    "ok": True,
                    "new_trajectory": bool(has_trajectory),
                    "trajectory_points": trajectory_points,
                    "command": command,
                },
            )
        except Exception as exc:  # pragma: no cover - defensive worker boundary
            _send_msg(args.write_fd, {
                "ok": False,
                "message": f"{type(exc).__name__}: {exc}",
                "traceback": traceback.format_exc(),
            })


if __name__ == "__main__":
    main()
