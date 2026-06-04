#include <super_core_py/planner_session.hpp>

#include <cmath>
#include <utility>

namespace super_core_py {

PlannerSession::PlannerSession(const std::string& config_path)
    : config_path_(config_path),
      fsm_cfg_(config_path),
      ros_ptr_(std::make_shared<NoRosInterface>()),
      map_ptr_(std::make_shared<rog_map::ROGMapROS>(config_path)) {
  planner_ptr_ = std::make_shared<super_planner::SuperPlanner>(config_path_, ros_ptr_, map_ptr_);
  latest_message_ = "initialized";
}

std::string PlannerSession::stateName(MachineState state) {
  switch (state) {
    case MachineState::INIT:
      return "INIT";
    case MachineState::WAIT_GOAL:
      return "WAIT_GOAL";
    case MachineState::GENERATE_TRAJ:
      return "GENERATE_TRAJ";
    case MachineState::FOLLOW_TRAJ:
      return "FOLLOW_TRAJ";
    case MachineState::EMER_STOP:
      return "EMER_STOP";
  }
  return "UNKNOWN";
}

void PlannerSession::setTime(double time_s) {
  ros_ptr_->setSimTime(time_s);
  map_ptr_->setSimTime(time_s);
}

rog_map::PointCloud makePointCloud(const std::vector<std::array<double, 4>>& points) {
  rog_map::PointCloud cloud;
  cloud.reserve(points.size());
  for (const auto& item : points) {
    rog_map::PclPoint p;
    p.x = static_cast<float>(item[0]);
    p.y = static_cast<float>(item[1]);
    p.z = static_cast<float>(item[2]);
    p.intensity = static_cast<float>(item[3]);
    cloud.push_back(p);
  }
  return cloud;
}

MapStats PlannerSession::load_static_pcd(const std::string& pcd_path, bool clear) {
  if (clear) {
    map_ptr_->clearStaticMap();
  }

  std::size_t point_count = 0;
  const bool ok = map_ptr_->loadStaticPcd(pcd_path, point_count);
  has_map_ = ok && point_count > 0;
  latest_message_ = ok ? "static PCD loaded" : "failed to load static PCD";
  return MapStats{ok, point_count, map_ptr_->mapEmpty(), latest_message_};
}

MapStats PlannerSession::load_static_points(const std::vector<std::array<double, 4>>& points, bool clear) {
  if (clear) {
    map_ptr_->clearStaticMap();
  }

  const auto cloud = makePointCloud(points);
  map_ptr_->loadStaticPoints(cloud);
  has_map_ = !map_ptr_->mapEmpty();
  latest_message_ = "static points loaded";
  return MapStats{true, points.size(), map_ptr_->mapEmpty(), latest_message_};
}

MapUpdateResult PlannerSession::update_sensing(const std::vector<std::array<double, 4>>& points,
                                               const RobotStateInput& state,
                                               double time_s) {
  setTime(time_s);

  super_utils::RobotState rs;
  rs.p = state.position;
  rs.v = state.velocity;
  rs.a = state.acceleration;
  rs.j = state.jerk;
  rs.q = state.quat;
  rs.yaw = state.yaw;
  rs.rcv = true;
  rs.rcv_time = time_s;
  map_ptr_->updateRobotStateFull(rs);

  refreshRobotState();
  latest_message_ = points.empty() ? "state updated" : "state updated; point cloud ignored";
  return MapUpdateResult{true, points.size(), 0, robot_state_.rcv, latest_message_};
}

GoalResult PlannerSession::set_goal(const Vec3f& position, double yaw) {
  refreshRobotState();
  Vec3f click_point = position;
  if (fsm_cfg_.click_height > -5.0) {
    click_point.z() = fsm_cfg_.click_height;
  }

  Vec3f adjusted_goal;
  if (!map_ptr_->getNearestInfCellNot(super_utils::OCCUPIED, click_point, adjusted_goal, 3.0)) {
    latest_message_ = "goal is deeply occupied";
    return GoalResult{false, position, yaw, latest_message_};
  }

  if (robot_state_.rcv && (robot_state_.p - adjusted_goal).norm() < 0.1) {
    latest_message_ = "goal is too close to current state";
    return GoalResult{false, adjusted_goal, yaw, latest_message_};
  }

  goal_.goal_p = adjusted_goal;
  goal_.goal_yaw = yaw;
  goal_.new_goal = true;
  goal_.has_goal = true;
  finish_plan_ = false;
  if (machine_state_ == MachineState::INIT) {
    machine_state_ = MachineState::WAIT_GOAL;
  }
  latest_message_ = "goal accepted";
  return GoalResult{true, goal_.goal_p, goal_.goal_yaw, latest_message_};
}

bool PlannerSession::closeToGoal(double threshold) const {
  return robot_state_.rcv && goal_.has_goal && (robot_state_.p - goal_.goal_p).norm() < threshold;
}

void PlannerSession::refreshRobotState() {
  if (planner_ptr_) {
    planner_ptr_->getRobotState(robot_state_);
  }
}

StepResult PlannerSession::step(double time_s) {
  setTime(time_s);
  refreshRobotState();
  StepResult result;
  result.state = stateName(machine_state_);
  result.ret_code = last_ret_code_;

  if (!robot_state_.rcv) {
    latest_message_ = "waiting for robot state";
    result.message = latest_message_;
    return result;
  }

  switch (machine_state_) {
    case MachineState::INIT:
      machine_state_ = MachineState::WAIT_GOAL;
      latest_message_ = "waiting for goal";
      break;

    case MachineState::WAIT_GOAL:
      if (goal_.has_goal && goal_.new_goal) {
        machine_state_ = MachineState::GENERATE_TRAJ;
        latest_message_ = "new goal received";
      } else {
        latest_message_ = "waiting for goal";
      }
      break;

    case MachineState::GENERATE_TRAJ: {
      if (closeToGoal(0.1)) {
        goal_.new_goal = false;
        finish_plan_ = true;
        machine_state_ = MachineState::WAIT_GOAL;
        last_ret_code_ = super_utils::FINISH;
        latest_message_ = "already close to goal";
        break;
      }
      const auto ret = planner_ptr_->PlanFromRest(goal_.goal_p, goal_.goal_yaw, goal_.new_goal);
      last_ret_code_ = ret;
      result.ret_code = ret;
      if (ret == super_utils::SUCCESS || ret == super_utils::FINISH) {
        goal_.new_goal = false;
        plan_from_rest_ = true;
        finish_plan_ = (ret == super_utils::FINISH);
        machine_state_ = MachineState::FOLLOW_TRAJ;
        result.success = true;
        result.new_trajectory = true;
        latest_message_ = "plan from rest succeeded";
      } else {
        latest_message_ = "plan from rest failed";
      }
      break;
    }

    case MachineState::FOLLOW_TRAJ: {
      bool traj_finish = false;
      double start_wt = 0.0;
      planner_ptr_->getOneHeartbeatTime(start_wt, traj_finish);
      result.trajectory_finished = traj_finish;
      if (finish_plan_ || traj_finish) {
        finish_plan_ = true;
        machine_state_ = MachineState::WAIT_GOAL;
        latest_message_ = "trajectory finished";
        break;
      }
      if (plan_from_rest_) {
        plan_from_rest_ = false;
        result.success = true;
        latest_message_ = "skip immediate replan after plan from rest";
        break;
      }
      if (last_replan_time_ < 0.0 || time_s - last_replan_time_ >= 1.0 / std::max(1e-6, fsm_cfg_.replan_rate)) {
        Vec3f safe_goal = goal_.goal_p;
        map_ptr_->getNearestInfCellNot(super_utils::OCCUPIED, safe_goal, safe_goal, 3.0);
        const auto ret = planner_ptr_->ReplanOnce(safe_goal, goal_.goal_yaw, goal_.new_goal);
        last_replan_time_ = time_s;
        last_ret_code_ = ret;
        result.ret_code = ret;
        if (ret == super_utils::EMER) {
          machine_state_ = MachineState::EMER_STOP;
          result.used_backup = true;
          latest_message_ = "replan requested emergency stop";
        } else if (ret == super_utils::NEW_TRAJ) {
          machine_state_ = MachineState::GENERATE_TRAJ;
          latest_message_ = "replan requested new trajectory";
        } else if (ret == super_utils::SUCCESS || ret == super_utils::FINISH ||
                   ret == super_utils::NO_NEED) {
          goal_.new_goal = false;
          result.success = true;
          result.new_trajectory = (ret == super_utils::SUCCESS);
          latest_message_ = "replan succeeded";
        } else {
          latest_message_ = "replan failed";
        }
      } else {
        result.success = true;
        latest_message_ = "following trajectory";
      }
      break;
    }

    case MachineState::EMER_STOP:
      machine_state_ = MachineState::WAIT_GOAL;
      latest_message_ = "emergency stop handled";
      break;
  }

  result.state = stateName(machine_state_);
  result.message = latest_message_;
  result.ret_code = last_ret_code_;
  return result;
}

PositionCommand PlannerSession::sample_command(double time_s) {
  setTime(time_s);
  PositionCommand cmd;
  StatePVAJ pvaj;
  pvaj.setZero();
  bool on_backup = false;
  bool traj_finish = true;
  double yaw = NAN;
  double yaw_dot = 0.0;
  planner_ptr_->getOneCommandFromTraj(pvaj, yaw, yaw_dot, on_backup, traj_finish);
  cmd.position = pvaj.col(0);
  cmd.velocity = pvaj.col(1);
  cmd.acceleration = pvaj.col(2);
  cmd.jerk = pvaj.col(3);
  cmd.yaw = yaw;
  cmd.yaw_rate = yaw_dot;
  cmd.on_backup_trajectory = on_backup;
  cmd.trajectory_finished = traj_finish;
  geometry_utils::convertFlatOutputToAttAndOmg(cmd.position, cmd.velocity, cmd.acceleration, cmd.jerk,
                                               cmd.yaw, cmd.yaw_rate, cmd.attitude_rpy,
                                               cmd.angular_velocity, cmd.thrust);
  return cmd;
}

geometry_utils::Trajectory PlannerSession::get_position_trajectory() const {
  return planner_ptr_->getCommittedPositionTrajectory();
}

geometry_utils::Trajectory PlannerSession::get_yaw_trajectory() const {
  return planner_ptr_->getCommittedYawTrajectory();
}

Diagnostics PlannerSession::get_debug_state() const {
  Diagnostics d;
  d.fsm_state = stateName(machine_state_);
  d.has_goal = goal_.has_goal;
  d.has_map = has_map_;
  d.has_trajectory = !planner_ptr_->getCommittedPositionTrajectory().empty();
  d.robot_state_received = robot_state_.rcv;
  d.last_ret_code = last_ret_code_;
  d.robot_position = robot_state_.p;
  d.latest_message = latest_message_;
  return d;
}

void PlannerSession::reset(bool clear_map) {
  goal_ = GoalInfo{};
  robot_state_ = rog_map::RobotState{};
  plan_from_rest_ = false;
  finish_plan_ = false;
  machine_state_ = MachineState::INIT;
  last_replan_time_ = -1.0;
  last_ret_code_ = super_utils::FAILED;
  latest_message_ = "reset";
  if (clear_map) {
    has_map_ = false;
    latest_message_ = "reset requested map clear; existing ROGMap memory is retained";
  }
}

}  // namespace super_core_py
