#pragma once

#include <vector>

namespace backward {

class SignalHandling {
 public:
  explicit SignalHandling(const std::vector<int>& = std::vector<int>()) {}
  bool loaded() const { return false; }
};

}  // namespace backward
