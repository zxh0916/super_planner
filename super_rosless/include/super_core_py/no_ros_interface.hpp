#pragma once

#include <iostream>
#include <string>

#include <ros_interface/ros_interface.hpp>

namespace super_core_py {

using namespace geometry_utils;
using namespace super_utils;

class NoRosInterface final : public ros_interface::RosInterface {
 public:
  using Ptr = std::shared_ptr<NoRosInterface>;

  void debug(const std::string& msg) override { log("DEBUG", msg); }
  void info(const std::string& msg) override { log("INFO", msg); }
  void warn(const std::string& msg) override { log("WARN", msg); }
  void error(const std::string& msg) override { log("ERROR", msg); }
  void fatal(const std::string& msg) override { log("FATAL", msg); }

  void setSimTime(const double& sim_time) override { sim_time_ = sim_time; }
  double getSimTime() override { return sim_time_; }
  void getSimTime(int32_t& sec, uint32_t& nsec) override {
    sec = static_cast<int32_t>(sim_time_);
    nsec = static_cast<uint32_t>((sim_time_ - static_cast<double>(sec)) * 1e9);
  }

  void vizExpTraj(const Trajectory&, const std::string& = "exp_traj") override {}
  void vizBackupTraj(const Trajectory&) override {}
  void vizFrontendPath(const vec_Vec3f&) override {}
  void vizExpSfc(const PolytopeVec&) override {}
  void vizBackupSfc(const Polytope&) override {}
  void vizGoalPath(const vec_Vec3f&) override {}
  void vizCommittedTraj(const Trajectory&, const double&) override {}
  void vizYawTraj(const Trajectory&, const Trajectory&) override {}
  void vizAstarBoundingBox(const Vec3f&, const Vec3f&) override {}
  void vizAstarPoints(const Vec3f&, const Color&, const std::string&, const double& = 0.1,
                      const int& = 0) override {}
  void vizReplanLog(const Trajectory&, const Trajectory&, const Trajectory&, const Trajectory&,
                    const PolytopeVec&, const Polytope&, const vec_Vec3f&, const int&) override {}
  void vizCiriSeedLine(const Vec3f&, const Vec3f&, const double&) override {}
  void vizCiriEllipsoid(const Ellipsoid&) override {}
  void vizCiriInfeasiblePoint(const Vec3f) override {}
  void vizCiriPolytope(const Polytope&, const std::string&) override {}
  void vizCiriPointCloud(const vec_Vec3f&) override {}

  void setVerbose(bool verbose) { verbose_ = verbose; }

 private:
  void log(const char* level, const std::string& msg) const {
    if (verbose_) {
      std::cerr << "[super_rosless][" << level << "] " << msg << std::endl;
    }
  }

  double sim_time_{0.0};
  bool verbose_{false};
};

}  // namespace super_core_py
