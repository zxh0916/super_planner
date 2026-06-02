#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>

#include <ros/ros.h>

#include <rog_map_ros/rog_map_ros1.hpp>
#include <ros_interface/ros1/ros1_interface.hpp>
#include <super_core/super_planner.h>

namespace {

void printTrajectory(const char* name, const geometry_utils::Trajectory& traj) {
  std::cout << name << "_START " << std::setprecision(17) << traj.start_WT << "\n";
  std::cout << name << "_TOTAL " << std::setprecision(17) << traj.getTotalDuration() << "\n";
  std::cout << name << "_PIECES " << traj.getPieceNum() << "\n";
  for (int i = 0; i < traj.getPieceNum(); ++i) {
    const auto& piece = traj[i];
    const auto& coeff = piece.getCoeffMat();
    std::cout << name << "_DUR " << i << " " << std::setprecision(17) << piece.getDuration() << "\n";
    std::cout << name << "_COEFF " << i << " " << coeff.rows() << " " << coeff.cols();
    for (int r = 0; r < coeff.rows(); ++r) {
      for (int c = 0; c < coeff.cols(); ++c) {
        std::cout << " " << std::setprecision(17) << coeff(r, c);
      }
    }
    std::cout << "\n";
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: ros_reference_plan <config.yaml>\n";
    return 2;
  }

  ros::init(argc, argv, "super_ros_reference_plan", ros::init_options::AnonymousName);
  ros::Time::init();
  ros::Time::setNow(ros::Time(0.0));
  ros::NodeHandle nh;

  const std::string cfg_path = argv[1];
  auto map = std::make_shared<rog_map::ROGMapROS>(nh, cfg_path);
  auto ros_ptr = std::make_shared<ros_interface::Ros1Interface>(nh);
  auto planner = std::make_shared<super_planner::SuperPlanner>(cfg_path, ros_ptr, map);

  rog_map::PointCloud cloud;
  rog_map::PclPoint point;
  point.x = 7.0f;
  point.y = 50.0f;
  point.z = 1.5f;
  point.intensity = 1.0f;
  cloud.push_back(point);

  const super_utils::Vec3f state_p(0.0, 0.0, 1.5);
  const super_utils::Quatf state_q(1.0, 0.0, 0.0, 0.0);
  ros_ptr->setSimTime(0.0);
  planner->updateROGMap(cloud, std::make_pair(state_p, state_q));
  rog_map::RobotState robot_state;
  planner->getRobotState(robot_state);

  super_utils::Vec3f goal = super_utils::Vec3f(5.0, 0.0, 1.5);
  map->getNearestInfCellNot(super_utils::OCCUPIED, goal, goal, 3.0);

  ros_ptr->setSimTime(0.1);
  const auto ret = planner->PlanFromRest(goal, std::numeric_limits<double>::quiet_NaN(), true);
  std::cout << "RET " << ret << "\n";
  printTrajectory("POS", planner->getCommittedPositionTrajectory());
  printTrajectory("YAW", planner->getCommittedYawTrajectory());
  return ret == super_utils::SUCCESS || ret == super_utils::FINISH ? 0 : 1;
}
