#pragma once

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcl::io {

namespace detail {
template <typename T, typename = void>
struct has_intensity : std::false_type {};

template <typename T>
struct has_intensity<T, std::void_t<decltype(std::declval<T&>().intensity)>> : std::true_type {};

inline std::string trim(std::string s) {
  auto not_space = [](unsigned char c) { return !std::isspace(c); };
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
  s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
  return s;
}
}  // namespace detail

template <typename PointT>
int loadPCDFile(const std::string& path, pcl::PointCloud<PointT>& cloud) {
  std::ifstream in(path);
  if (!in.is_open()) {
    return -1;
  }

  cloud.clear();
  std::vector<std::string> fields;
  bool ascii_data = false;
  std::size_t point_count = 0;
  std::string line;

  while (std::getline(in, line)) {
    line = detail::trim(line);
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream iss(line);
    std::string key;
    iss >> key;
    if (key == "FIELDS") {
      std::string field;
      while (iss >> field) {
        fields.push_back(field);
      }
    } else if (key == "POINTS") {
      iss >> point_count;
    } else if (key == "DATA") {
      std::string mode;
      iss >> mode;
      ascii_data = (mode == "ascii");
      break;
    }
  }

  if (!ascii_data) {
    return -1;
  }
  if (point_count > 0) {
    cloud.reserve(point_count);
  }

  int x_idx = -1;
  int y_idx = -1;
  int z_idx = -1;
  int intensity_idx = -1;
  for (int i = 0; i < static_cast<int>(fields.size()); ++i) {
    if (fields[i] == "x") x_idx = i;
    if (fields[i] == "y") y_idx = i;
    if (fields[i] == "z") z_idx = i;
    if (fields[i] == "intensity") intensity_idx = i;
  }
  if (x_idx < 0 || y_idx < 0 || z_idx < 0) {
    return -1;
  }

  while (std::getline(in, line)) {
    line = detail::trim(line);
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream iss(line);
    std::vector<float> values;
    float value = 0.0f;
    while (iss >> value) {
      values.push_back(value);
    }
    if (static_cast<int>(values.size()) <= std::max({x_idx, y_idx, z_idx, intensity_idx})) {
      continue;
    }

    PointT point;
    point.x = values[x_idx];
    point.y = values[y_idx];
    point.z = values[z_idx];
    if constexpr (detail::has_intensity<PointT>::value) {
      point.intensity = intensity_idx >= 0 ? values[intensity_idx] : 1.0f;
    }
    cloud.push_back(point);
  }

  return 0;
}

}  // namespace pcl::io
