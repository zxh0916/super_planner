#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Eigen>

#include <fsm/config.hpp>
#include <super_core/super_planner.h>
#include <super_core_py/no_ros_interface.hpp>
#include <utils/geometry/geometry_utils.h>

namespace super_core_py {

using super_utils::Quatf;
using super_utils::StatePVAJ;
using super_utils::Vec3f;

struct RobotStateInput {
  Vec3f position{0.0, 0.0, 0.0};
  Vec3f velocity{0.0, 0.0, 0.0};
  Vec3f acceleration{0.0, 0.0, 0.0};
  Vec3f jerk{0.0, 0.0, 0.0};
  Quatf quat{1.0, 0.0, 0.0, 0.0};
  double yaw{0.0};
};

struct MapStats {
  bool success{false};
  std::size_t point_count{0};
  bool map_empty{true};
  std::string message;
};

struct MapUpdateResult {
  bool success{false};
  std::size_t input_point_count{0};
  std::size_t used_point_count{0};
  bool robot_state_received{false};
  std::string message;
};

struct GoalResult {
  bool accepted{false};
  Vec3f goal_position{0.0, 0.0, 0.0};
  double goal_yaw{NAN};
  std::string message;
};

struct StepResult {
  bool success{false};
  int ret_code{super_utils::FAILED};
  std::string state;
  bool new_trajectory{false};
  bool trajectory_finished{false};
  bool used_backup{false};
  std::string message;
};

struct PositionCommand {
  Vec3f position{0.0, 0.0, 0.0};
  Vec3f velocity{0.0, 0.0, 0.0};
  Vec3f acceleration{0.0, 0.0, 0.0};
  Vec3f jerk{0.0, 0.0, 0.0};
  double yaw{NAN};
  double yaw_rate{0.0};
  Vec3f attitude_rpy{0.0, 0.0, 0.0};
  Vec3f angular_velocity{0.0, 0.0, 0.0};
  double thrust{0.0};
  bool on_backup_trajectory{false};
  bool trajectory_finished{true};
};

struct Diagnostics {
  std::string fsm_state;
  bool has_goal{false};
  bool has_map{false};
  bool has_trajectory{false};
  bool robot_state_received{false};
  int last_ret_code{super_utils::FAILED};
  Vec3f robot_position{0.0, 0.0, 0.0};
  std::string latest_message;
};

class PlannerSession {
 public:
  explicit PlannerSession(const std::string& config_path);

  MapStats load_static_pcd(const std::string& pcd_path, bool clear = true);
  MapStats load_static_points(const std::vector<std::array<double, 4>>& points, bool clear = true);
  MapUpdateResult update_sensing(const std::vector<std::array<double, 4>>& points,
                                 const RobotStateInput& state,
                                 double time_s);
  GoalResult set_goal(const Vec3f& position, double yaw = NAN);
  StepResult step(double time_s);
  PositionCommand sample_command(double time_s);
  geometry_utils::Trajectory get_position_trajectory() const;
  geometry_utils::Trajectory get_yaw_trajectory() const;
  Diagnostics get_debug_state() const;
  void reset(bool clear_map = false);

 private:
  enum class MachineState { INIT = 0, WAIT_GOAL, GENERATE_TRAJ, FOLLOW_TRAJ, EMER_STOP };

  struct GoalInfo {
    bool new_goal{false};
    bool has_goal{false};
    Vec3f goal_p{0.0, 0.0, 0.0};
    double goal_yaw{NAN};
  };

  static std::string stateName(MachineState state);
  void setTime(double time_s);
  bool closeToGoal(double threshold) const;
  void refreshRobotState();

  std::string config_path_;
  fsm::Config fsm_cfg_;
  NoRosInterface::Ptr ros_ptr_;
  rog_map::ROGMapROS::Ptr map_ptr_;
  super_planner::SuperPlanner::Ptr planner_ptr_;
  MachineState machine_state_{MachineState::INIT};
  GoalInfo goal_;
  rog_map::RobotState robot_state_;
  bool plan_from_rest_{false};
  bool finish_plan_{false};
  bool has_map_{false};
  double last_replan_time_{-1.0};
  int last_ret_code_{super_utils::FAILED};
  std::string latest_message_;
};

}  // namespace super_core_py
