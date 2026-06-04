#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <super_core_py/planner_session.hpp>

namespace py = pybind11;

namespace {

using super_core_py::PlannerSession;
using super_core_py::RobotStateInput;
using super_utils::Quatf;
using super_utils::Vec3f;

Vec3f vec3FromObject(const py::object& obj, const char* name) {
  const auto seq = py::cast<std::vector<double>>(obj);
  if (seq.size() != 3) {
    throw std::invalid_argument(std::string(name) + " must have length 3");
  }
  return Vec3f(seq[0], seq[1], seq[2]);
}

Quatf quatFromObject(const py::object& obj, const char* name) {
  const auto seq = py::cast<std::vector<double>>(obj);
  if (seq.size() != 4) {
    throw std::invalid_argument(std::string(name) + " must have length 4 as [w, x, y, z]");
  }
  return Quatf(seq[0], seq[1], seq[2], seq[3]);
}

RobotStateInput robotStateFromObject(const py::object& obj) {
  if (py::isinstance<RobotStateInput>(obj)) {
    return py::cast<RobotStateInput>(obj);
  }
  if (!py::isinstance<py::dict>(obj)) {
    throw std::invalid_argument("state must be RobotState or dict");
  }

  py::dict d = py::cast<py::dict>(obj);
  RobotStateInput state;
  if (d.contains("position")) state.position = vec3FromObject(d["position"], "position");
  if (d.contains("velocity")) state.velocity = vec3FromObject(d["velocity"], "velocity");
  if (d.contains("acceleration")) state.acceleration = vec3FromObject(d["acceleration"], "acceleration");
  if (d.contains("jerk")) state.jerk = vec3FromObject(d["jerk"], "jerk");
  if (d.contains("quat")) state.quat = quatFromObject(d["quat"], "quat");
  if (d.contains("yaw")) state.yaw = py::cast<double>(d["yaw"]);
  return state;
}

std::vector<std::array<double, 4>> pointsFromArray(const py::array& array) {
  py::buffer_info info = array.request();
  if (info.ndim != 2 || (info.shape[1] != 3 && info.shape[1] != 4)) {
    throw std::invalid_argument("points must be a numpy array with shape (N, 3) or (N, 4)");
  }

  py::array_t<double, py::array::c_style | py::array::forcecast> casted(array);
  py::buffer_info cinfo = casted.request();
  const auto* data = static_cast<const double*>(cinfo.ptr);
  const ssize_t n = cinfo.shape[0];
  const ssize_t cols = cinfo.shape[1];

  std::vector<std::array<double, 4>> points;
  points.reserve(static_cast<std::size_t>(n));
  for (ssize_t i = 0; i < n; ++i) {
    std::array<double, 4> p{data[i * cols], data[i * cols + 1], data[i * cols + 2], 1.0};
    if (cols == 4) {
      p[3] = data[i * cols + 3];
    }
    points.push_back(p);
  }
  return points;
}

py::array_t<double> vecToArray(const Vec3f& v) {
  py::array_t<double> arr({3});
  auto m = arr.mutable_unchecked<1>();
  m(0) = v.x();
  m(1) = v.y();
  m(2) = v.z();
  return arr;
}

py::dict mapStatsToDict(const super_core_py::MapStats& s) {
  py::dict d;
  d["success"] = s.success;
  d["point_count"] = s.point_count;
  d["map_empty"] = s.map_empty;
  d["message"] = s.message;
  return d;
}

py::dict mapUpdateToDict(const super_core_py::MapUpdateResult& r) {
  py::dict d;
  d["success"] = r.success;
  d["input_point_count"] = r.input_point_count;
  d["used_point_count"] = r.used_point_count;
  d["robot_state_received"] = r.robot_state_received;
  d["message"] = r.message;
  return d;
}

py::dict goalToDict(const super_core_py::GoalResult& r) {
  py::dict d;
  d["accepted"] = r.accepted;
  d["goal_position"] = vecToArray(r.goal_position);
  d["goal_yaw"] = r.goal_yaw;
  d["message"] = r.message;
  return d;
}

py::dict stepToDict(const super_core_py::StepResult& r) {
  py::dict d;
  d["success"] = r.success;
  d["ret_code"] = r.ret_code;
  d["state"] = r.state;
  d["new_trajectory"] = r.new_trajectory;
  d["trajectory_finished"] = r.trajectory_finished;
  d["used_backup"] = r.used_backup;
  d["message"] = r.message;
  return d;
}

py::dict commandToDict(const super_core_py::PositionCommand& c) {
  py::dict d;
  d["position"] = vecToArray(c.position);
  d["velocity"] = vecToArray(c.velocity);
  d["acceleration"] = vecToArray(c.acceleration);
  d["jerk"] = vecToArray(c.jerk);
  d["yaw"] = c.yaw;
  d["yaw_rate"] = c.yaw_rate;
  d["attitude_rpy"] = vecToArray(c.attitude_rpy);
  d["angular_velocity"] = vecToArray(c.angular_velocity);
  d["thrust"] = c.thrust;
  d["on_backup_trajectory"] = c.on_backup_trajectory;
  d["trajectory_finished"] = c.trajectory_finished;
  return d;
}

py::dict diagnosticsToDict(const super_core_py::Diagnostics& d0) {
  py::dict d;
  d["fsm_state"] = d0.fsm_state;
  d["has_goal"] = d0.has_goal;
  d["has_map"] = d0.has_map;
  d["has_trajectory"] = d0.has_trajectory;
  d["robot_state_received"] = d0.robot_state_received;
  d["last_ret_code"] = d0.last_ret_code;
  d["robot_position"] = vecToArray(d0.robot_position);
  d["latest_message"] = d0.latest_message;
  return d;
}

py::dict trajectoryToDict(const geometry_utils::Trajectory& traj) {
  py::dict d;
  const int pieces = traj.getPieceNum();
  d["start_time"] = traj.start_WT;
  d["total_duration"] = traj.getTotalDuration();
  d["piece_count"] = pieces;

  py::array_t<double> durations({pieces});
  auto dur = durations.mutable_unchecked<1>();
  int rows = 0;
  int cols = 0;
  if (pieces > 0) {
    rows = traj[0].getCoeffMat().rows();
    cols = traj[0].getCoeffMat().cols();
  }
  py::array_t<double> coeffs({pieces, rows, cols});
  auto coeff = coeffs.mutable_unchecked<3>();

  for (int i = 0; i < pieces; ++i) {
    const auto& piece = traj[i];
    const auto& mat = piece.getCoeffMat();
    dur(i) = piece.getDuration();
    if (mat.rows() != rows || mat.cols() != cols) {
      throw std::runtime_error("trajectory pieces have inconsistent coefficient matrix sizes");
    }
    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        coeff(i, r, c) = mat(r, c);
      }
    }
  }

  d["durations"] = durations;
  d["coeffs"] = coeffs;
  return d;
}

}  // namespace

PYBIND11_MODULE(super_planner_py, m) {
  m.doc() = "ROS-free Python bindings for the SUPER planner core";

  py::class_<RobotStateInput>(m, "RobotState")
      .def(py::init<>())
      .def_readwrite("position", &RobotStateInput::position)
      .def_readwrite("velocity", &RobotStateInput::velocity)
      .def_readwrite("acceleration", &RobotStateInput::acceleration)
      .def_readwrite("jerk", &RobotStateInput::jerk)
      .def_readwrite("quat", &RobotStateInput::quat)
      .def_readwrite("yaw", &RobotStateInput::yaw);

  py::class_<PlannerSession>(m, "PlannerSession")
      .def(py::init<const std::string&>(), py::arg("config_path"))
      .def("load_static_pcd",
           [](PlannerSession& self, const std::string& pcd_path, bool clear) {
             return mapStatsToDict(self.load_static_pcd(pcd_path, clear));
           },
           py::arg("pcd_path"), py::arg("clear") = true)
      .def("load_static_points",
           [](PlannerSession& self, const py::array& points, bool clear) {
             return mapStatsToDict(self.load_static_points(pointsFromArray(points), clear));
           },
           py::arg("points"), py::arg("clear") = true)
      .def("update_sensing",
           [](PlannerSession& self, const py::array& points, const py::object& state, double time_s) {
             return mapUpdateToDict(self.update_sensing(pointsFromArray(points),
                                                        robotStateFromObject(state), time_s));
           },
           py::arg("points"), py::arg("state"), py::arg("time_s"))
      .def("update_sensing",
           [](PlannerSession& self, const py::object& state, double time_s) {
             return mapUpdateToDict(self.update_sensing({}, robotStateFromObject(state), time_s));
           },
           py::arg("state"), py::arg("time_s"))
      .def("set_goal",
           [](PlannerSession& self, const py::object& position, py::object yaw) {
             const double goal_yaw = yaw.is_none() ? NAN : py::cast<double>(yaw);
             return goalToDict(self.set_goal(vec3FromObject(position, "position"), goal_yaw));
           },
           py::arg("position"), py::arg("yaw") = py::none())
      .def("step",
           [](PlannerSession& self, double time_s) { return stepToDict(self.step(time_s)); },
           py::arg("time_s"))
      .def("get_trajectory",
           [](PlannerSession& self) {
             py::dict d;
             d["position"] = trajectoryToDict(self.get_position_trajectory());
             d["yaw"] = trajectoryToDict(self.get_yaw_trajectory());
             return d;
           })
      .def("sample_command",
           [](PlannerSession& self, double time_s) { return commandToDict(self.sample_command(time_s)); },
           py::arg("time_s"))
      .def("reset", &PlannerSession::reset, py::arg("clear_map") = false)
      .def("get_debug_state",
           [](PlannerSession& self) { return diagnosticsToDict(self.get_debug_state()); });
}
