#pragma once

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace yaml_loader {

template <typename T>
struct is_vector : std::false_type {};

template <typename T, typename Alloc>
struct is_vector<std::vector<T, Alloc>> : std::true_type {};

class YamlLoader {
 public:
  explicit YamlLoader(const std::string& file_path) : path_(file_path) {
    parse();
    std::cout << "Load config file: " << path_ << std::endl;
  }

  template <typename T>
  bool LoadParam(const std::string& param_name,
                 T& param_value,
                 const T& default_value = T{},
                 const bool& required = false) {
    return loadParamInternal(param_name, param_value, default_value, required);
  }

  template <class T>
  bool LoadParam(const std::string& param_name,
                 std::vector<T>& param_value,
                 const std::vector<T> default_value = {},
                 const bool& required = false) {
    return loadParamInternal(param_name, param_value, default_value, required);
  }

 private:
  std::string path_;
  std::unordered_map<std::string, std::string> values_;

  static std::string trim(std::string s) {
    auto not_space = [](unsigned char c) { return !std::isspace(c); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
    s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
    return s;
  }

  static std::string stripComment(const std::string& line) {
    bool in_single = false;
    bool in_double = false;
    for (std::size_t i = 0; i < line.size(); ++i) {
      const char c = line[i];
      if (c == '\'' && !in_double) in_single = !in_single;
      if (c == '"' && !in_single) in_double = !in_double;
      if (c == '#' && !in_single && !in_double) {
        return line.substr(0, i);
      }
    }
    return line;
  }

  static std::string unquote(std::string s) {
    s = trim(std::move(s));
    if (s.size() >= 2 &&
        ((s.front() == '"' && s.back() == '"') || (s.front() == '\'' && s.back() == '\''))) {
      return s.substr(1, s.size() - 2);
    }
    return s;
  }

  void parse() {
    std::ifstream in(path_);
    if (!in.is_open()) {
      throw std::runtime_error("Cannot open YAML config: " + path_);
    }

    std::vector<std::string> key_stack;
    std::string line;
    while (std::getline(in, line)) {
      line = stripComment(line);
      if (trim(line).empty()) {
        continue;
      }

      const std::size_t first = line.find_first_not_of(' ');
      const int level = static_cast<int>(first == std::string::npos ? 0 : first / 2);
      std::string content = trim(line);
      if (content.rfind("- ", 0) == 0) {
        if (static_cast<int>(key_stack.size()) > level) {
          std::string full;
          for (int i = 0; i <= level; ++i) {
            if (i) full += "/";
            full += key_stack[static_cast<std::size_t>(i)];
          }
          std::string item = trim(content.substr(2));
          auto& raw = values_[full];
          if (raw.empty()) {
            raw = "[" + item + "]";
          } else if (raw.back() == ']') {
            raw.pop_back();
            raw += ", " + item + "]";
          }
        }
        continue;
      }

      const auto colon = content.find(':');
      if (colon == std::string::npos) {
        continue;
      }

      std::string key = trim(content.substr(0, colon));
      std::string value = trim(content.substr(colon + 1));
      if (key.empty()) {
        continue;
      }

      if (static_cast<int>(key_stack.size()) > level) {
        key_stack.resize(level);
      }
      if (static_cast<int>(key_stack.size()) == level) {
        key_stack.push_back(key);
      } else {
        key_stack[level] = key;
        key_stack.resize(level + 1);
      }

      if (!value.empty()) {
        std::string full;
        for (std::size_t i = 0; i < key_stack.size(); ++i) {
          if (i) full += "/";
          full += key_stack[i];
        }
        values_[full] = unquote(value);
      }
    }
  }

  template <typename T>
  static T parseScalar(const std::string& raw) {
    if constexpr (std::is_same_v<T, std::string>) {
      return unquote(raw);
    } else if constexpr (std::is_same_v<T, bool>) {
      std::string v = raw;
      std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
      });
      return v == "true" || v == "1" || v == "yes";
    } else if constexpr (std::is_integral_v<T>) {
      return static_cast<T>(std::stoll(raw));
    } else if constexpr (std::is_floating_point_v<T>) {
      return static_cast<T>(std::stod(raw));
    } else {
      static_assert(!sizeof(T), "Unsupported YAML scalar type");
    }
  }

  template <typename T>
  static std::vector<T> parseVector(std::string raw) {
    raw = trim(std::move(raw));
    if (raw.size() >= 2 && raw.front() == '[' && raw.back() == ']') {
      raw = raw.substr(1, raw.size() - 2);
    }
    std::vector<T> out;
    std::string token;
    std::stringstream ss(raw);
    while (std::getline(ss, token, ',')) {
      token = trim(token);
      if (!token.empty()) {
        out.push_back(parseScalar<T>(token));
      }
    }
    return out;
  }

  template <typename T>
  bool loadParamInternal(const std::string& param_name,
                         T& param_value,
                         const T& default_value,
                         bool required) {
    const auto it = values_.find(param_name);
    if (it == values_.end()) {
      param_value = default_value;
      if (required) {
        throw std::invalid_argument("Required param " + param_name + " not found");
      }
      return false;
    }
    try {
      if constexpr (is_vector<T>::value) {
        using Elem = typename T::value_type;
        param_value = parseVector<Elem>(it->second);
      } else {
        param_value = parseScalar<T>(it->second);
      }
      return true;
    } catch (const std::exception&) {
      param_value = default_value;
      if (required) {
        throw;
      }
      return false;
    }
  }
};

}  // namespace yaml_loader
