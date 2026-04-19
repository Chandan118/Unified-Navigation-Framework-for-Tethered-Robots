#pragma once

#include "unitree_aliengo_bridge/bridge_types.hpp"
#include <string>
#include <fstream>
#include <iomanip>

namespace unitree_aliengo_bridge
{

class DataLogger
{
public:
  explicit DataLogger(const std::string& log_directory);
  ~DataLogger();

  void log(
    double timestamp,
    double x, double y, double heading,
    double linear_vel, double angular_vel,
    int yielding_state);  // Changed to int for enum serialization

  void set_active_tether_count(size_t count) {
    active_tether_count_ = static_cast<int>(count);
  }

  std::string get_filename() const { return filename_; }

private:
  std::string log_dir_;
  std::string filename_;
  std::ofstream csv_file_;
  bool csv_open_;
  int active_tether_count_ = 0;
};

}  // namespace unitree_aliengo_bridge
