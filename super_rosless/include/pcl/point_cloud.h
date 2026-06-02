#pragma once

#include <cstdint>
#include <memory>
#include <vector>

namespace pcl {

template <typename PointT>
struct PointCloud {
  using Ptr = std::shared_ptr<PointCloud<PointT>>;
  using ConstPtr = std::shared_ptr<const PointCloud<PointT>>;
  using iterator = typename std::vector<PointT>::iterator;
  using const_iterator = typename std::vector<PointT>::const_iterator;

  std::vector<PointT> points;
  std::uint32_t width{0};
  std::uint32_t height{1};
  bool is_dense{true};

  bool empty() const { return points.empty(); }
  std::size_t size() const { return points.size(); }
  void clear() {
    points.clear();
    width = 0;
    height = 1;
  }
  void reserve(std::size_t n) { points.reserve(n); }
  void push_back(const PointT& point) {
    points.push_back(point);
    width = static_cast<std::uint32_t>(points.size());
    height = 1;
  }

  PointT& operator[](std::size_t i) { return points[i]; }
  const PointT& operator[](std::size_t i) const { return points[i]; }
  iterator begin() { return points.begin(); }
  iterator end() { return points.end(); }
  const_iterator begin() const { return points.begin(); }
  const_iterator end() const { return points.end(); }
};

}  // namespace pcl

#include <pcl/io/pcd_io.h>
