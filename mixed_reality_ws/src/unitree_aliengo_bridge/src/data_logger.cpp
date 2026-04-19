#include "unitree_aliengo_bridge/data_logger.hpp"
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <string>
#include <rcpputils/fs.hpp>

namespace unitree_aliengo_bridge
{

DataLogger::DataLogger(const std::string& log_directory)
: log_dir_(log_directory), csv_open_(false)
{
  rcpputils::fs::create_directory(log_dir_);

  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);

  std::stringstream ss;
  ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
  filename_ = log_dir_ + "/velocity_log_" + ss.str() + ".csv";

  csv_file_.open(filename_, std::ios::out);
  csv_file_ << "timestamp_s,robot_x,robot_y,robot_heading,"
               "linear_vel_mps,angular_vel_radps,yielding_state,"
               "active_tether_count\n";
  csv_open_ = true;

  RCLCPP_INFO(rclcpp::get_logger("data_logger"),
              "DataLogger initialized: %s", filename_.c_str());
}

DataLogger::~DataLogger()
{
  if (csv_open_) {
    csv_file_.close();
  }
}

void DataLogger::log(
  double timestamp,
  double x, double y, double heading,
  double linear_vel, double angular_vel,
  int yielding_state)
{
  if (!csv_open_) return;

  csv_file_ << std::fixed << std::setprecision(6)
            << timestamp << ","
            << x << "," << y << "," << heading << ","
            << linear_vel << "," << angular_vel << ","
            << yielding_state << ","
            << active_tether_count_ << "\n";
  csv_file_.flush();
}

}  // namespace unitree_aliengo_bridge
