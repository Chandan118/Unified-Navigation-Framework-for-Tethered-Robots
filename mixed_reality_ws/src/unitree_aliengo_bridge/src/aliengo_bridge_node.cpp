#include "unitree_aliengo_bridge/aliengo_bridge_node.hpp"
#include <cmath>
#include <sstream>
#include <iomanip>
#include <json/json.h>  // 需要安装 jsoncpp 或在 C++17 用 nlohmann/json

namespace unitree_aliengo_bridge
{

AliengoBridgeNode::AliengoBridgeNode(const rclcpp::NodeOptions& options)
: Node("aliengo_bridge", options)
{
  this->declare_parameter("max_speed_mps", 0.4);
  this->declare_parameter("safety_margin_m", 0.5);
  this->declare_parameter("yield_stop_duration_s", 2.0);
  this->declare_parameter("robot_frame", "base");

  max_speed_mps_ = this->get_parameter("max_speed_mps").as_double();
  safety_margin_m_ = this->get_parameter("safety_margin_m").as_double();
  yield_stop_duration_s_ = this->get_parameter("yield_stop_duration_s").as_double();
  robot_frame_ = this->get_parameter("robot_frame").as_string();

  collision_checker_ = std::make_unique<TetherCollisionChecker>(
    safety_margin_m_, max_speed_mps_);

  std::string home = std::getenv("HOME") ? std::getenv("HOME") : "/tmp";
  data_logger_ = std::make_unique<DataLogger>(home + "/mixed_reality_data");

  yield_state_ = YieldingState::NORMAL;
  yield_start_time_ = 0.0;
  last_brake_time_ = 0.0;

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10,
    std::bind(&AliengoBridgeNode::odom_callback, this, std::placeholders::_1));

  tether_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/swarm_tether_states", 10,
    std::bind(&AliengoBridgeNode::tether_callback, this, std::placeholders::_1));

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "goal_point", 10,
    std::bind(&AliengoBridgeNode::goal_callback, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  current_goal_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
    "current_goal", 10);

  control_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(0.02),
    std::bind(&AliengoBridgeNode::control_loop, this));

  RCLCPP_INFO(this->get_logger(),
    "AliengoBridgeNode started. Max speed: %.2f m/s, Safety margin: %.2f m",
    max_speed_mps_, safety_margin_m_);
}

void AliengoBridgeNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  auto p = msg->pose.pose.position;
  robot_pos_.x = p.x;
  robot_pos_.y = p.y;
  auto q = msg->pose.pose.orientation;
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  robot_heading_ = atan2(siny_cosp, cosy_cosp);
}

void AliengoBridgeNode::tether_callback(const std_msgs::msg::String::SharedPtr msg)
{
  active_tethers_.clear();

  // 解析 JSON
  Json::Value root;
  Json::CharReaderBuilder builder;
  std::string errs;
  std::istringstream ss(msg->data);
  
  if (Json::parseFromStream(builder, ss, &root, &errs)) {
    for (const auto& tether_val : root) {
      SwarmTether tether;
      tether.robot_id = tether_val["robot_id"].asString();
      tether.start.x = tether_val["start"][0].asDouble();
      tether.start.y = tether_val["start"][1].asDouble();
      tether.end.x = tether_val["end"][0].asDouble();
      tether.end.y = tether_val["end"][1].asDouble();
      tether.thickness_m = tether_val["thickness"].asDouble();
      tether.is_active = tether_val["active"].asBool();

      if (tether.is_active) {
        active_tethers_.push_back(tether);
      }
    }
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Failed to parse tether JSON: %s", errs.c_str());
  }

  data_logger_->set_active_tether_count(active_tethers_.size());
}

void AliengoBridgeNode::goal_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  current_goal_ = msg->point;
}

void AliengoBridgeNode::control_loop()
{
  double now = this->now().seconds();

  if (yield_state_ == YieldingState::NORMAL) {
    auto risk = assess_collision_risk();
    if (risk.collision_imminent) {
      RCLCPP_WARN(this->get_logger(),
        "[YIELD] Tether from '%s' crossing in %.2fs! Stopping.",
        risk.conflicting_robot_id.c_str(), risk.time_to_collision_s);
      yield_state_ = YieldingState::YIELDING;
      yield_start_time_ = now;
    }
  }

  geometry_msgs::msg::Twist cmd;

  switch (yield_state_) {
    case YieldingState::NORMAL:
      cmd = compute_twist_to_goal(current_goal_);
      break;

    case YieldingState::YIELDING:
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      if (now - yield_start_time_ >= yield_stop_duration_s_) {
        yield_state_ = YieldingState::RECOVERING;
        yield_start_time_ = now;
      }
      break;

    case YieldingState::RECOVERING:
      {
        double ramp_time = now - yield_start_time_;
        double ramp_duration = 1.0;
        if (ramp_time < ramp_duration) {
          double factor = ramp_time / ramp_duration;
          cmd = compute_twist_to_goal(current_goal_);
          cmd.linear.x *= factor;
        } else {
          yield_state_ = YieldingState::NORMAL;
          cmd = compute_twist_to_goal(current_goal_);
        }
      }
      break;

    case YieldingState::AVOIDING:
      cmd.linear.x = 0.1;
      cmd.angular.z = 0.0;
      break;
  }

  cmd_vel_pub_->publish(cmd);
  log_velocity(cmd.linear.x, cmd.angular.z);
}

geometry_msgs::msg::Twist AliengoBridgeNode::compute_twist_to_goal(
  const geometry_msgs::msg::Point& goal)
{
  geometry_msgs::msg::Twist cmd;
  double dx = goal.x - robot_pos_.x;
  double dy = goal.y - robot_pos_.y;
  double dist = sqrt(dx*dx + dy*dy);

  if (dist < 0.3) {
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    return cmd;
  }

  double desired_heading = atan2(dy, dx);
  double heading_error = desired_heading - robot_heading_;
  heading_error = atan2(sin(heading_error), cos(heading_error));

  cmd.angular.z = 2.0 * heading_error;
  double speed = max_speed_mps_ * (1.0 - fabs(heading_error) / M_PI * 0.5);
  cmd.linear.x = std::max(0.0, speed);

  return cmd;
}

CollisionRisk AliengoBridgeNode::assess_collision_risk()
{
  CollisionRisk risk;
  risk.collision_imminent = false;
  risk.time_to_collision_s = 999.0;
  risk.conflicting_robot_id = "";
  risk.min_distance_m = 999.0;

  if (active_tethers_.empty()) {
    return risk;
  }

  for (const auto& tether : active_tethers_) {
    auto check = collision_checker_->predict_crossing(
      robot_pos_, robot_heading_,
      tether.start, tether.end,
      max_speed_mps_);

    if (check.collision_imminent) {
      risk.collision_imminent = true;
      if (check.time_to_collision_s < risk.time_to_collision_s) {
        risk.time_to_collision_s = check.time_to_collision_s;
        risk.conflicting_robot_id = tether.robot_id;
        risk.min_distance_m = check.min_distance_m;
      }
    }
  }

  return risk;
}

void AliengoBridgeNode::log_velocity(double linear_x, double angular_z)
{
  data_logger_->log(
    this->now().seconds(),
    robot_pos_.x, robot_pos_.y, robot_heading_,
    linear_x, angular_z,
    static_cast<int>(yield_state_));
}

}  // namespace unitree_aliengo_bridge
