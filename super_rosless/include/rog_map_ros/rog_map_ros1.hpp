#pragma once

#include <chrono>
#include <string>

#include <rog_map/rog_map.h>

namespace rog_map {

class ROGMapROS : public ROGMap {
 public:
  using Ptr = std::shared_ptr<ROGMapROS>;

  ROGMapROS() = default;

  explicit ROGMapROS(const std::string& cfg_path) {
    cfg_ = rog_map::Config(cfg_path);
    cfg_.ros_callback_en = false;
    init();
  }

  void setConfigPathAndInit(const std::string& cfg_path) {
    cfg_ = rog_map::Config(cfg_path);
    cfg_.ros_callback_en = false;
    init();
  }

  const double getSystemWalltimeNow() override {
    return sim_time_;
  }

  void setSimTime(double time_s) {
    sim_time_ = time_s;
  }

  void updateRobotStateOnly(const super_utils::Pose& pose) {
    updateRobotState(pose);
  }

  void updateRobotStateFull(const super_utils::RobotState& state) {
    updateRobotState(std::make_pair(state.p, state.q));
    robot_state_.v = state.v;
    robot_state_.a = state.a;
    robot_state_.j = state.j;
    robot_state_.yaw = state.yaw;
    robot_state_.rcv_time = sim_time_;
    robot_state_.rcv = true;
  }

  bool loadStaticPcd(const std::string& pcd_path, std::size_t& point_count) {
    PointCloud::Ptr pcd_map(new PointCloud);
    if (pcl::io::loadPCDFile(pcd_path, *pcd_map) == -1) {
      point_count = 0;
      return false;
    }
    updateOccPointCloud(*pcd_map);
    if (cfg_.esdf_en) {
      esdf_map_->updateESDF3D(robot_state_.p);
    }
    map_empty_ = false;
    point_count = pcd_map->size();
    return true;
  }

  bool mapEmpty() const {
    return map_empty_;
  }

 private:
  double sim_time_{0.0};
};

}  // namespace rog_map
