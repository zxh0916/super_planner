#pragma once

namespace pcl {

struct PointXYZ {
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
};

struct PointXYZI {
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
  float intensity{1.0f};
};

}  // namespace pcl
