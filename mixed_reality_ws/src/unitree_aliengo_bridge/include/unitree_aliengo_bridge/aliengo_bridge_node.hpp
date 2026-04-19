#include "unitree_aliengo_bridge/bridge_types.hpp"
#include "unitree_aliengo_bridge/tether_collision_checker.hpp"
#include "unitree_aliengo_bridge/data_logger.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>

namespace unitree_aliengo_bridge
{

class AliengoBridgeNode : public rclcpp::Node
{
public:
  explicit AliengoBridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // ROS 2 parameters
  float max_speed_mps_;
  float safety_margin_m_;
  float yield_stop_duration_s_;
  std::string robot_frame_;

  // Subscriptions
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tether_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;

  // Publications
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr current_goal_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr state_viz_pub_;

  // State
  geometry_msgs::msg::Point robot_pos_;
  double robot_heading_;
  geometry_msgs::msg::Point current_goal_;
  std::vector<SwarmTether> active_tethers_;

  // Yielding state machine
  enum class YieldingState {
    NORMAL,       // Cruising to goal
    YIELDING,     // Stopped for tether crossing
    AVOIDING,     // Actively avoiding tether
    RECOVERING    // Resuming normal speed after yield
  };
  YieldingState yield_state_;
  double yield_start_time_;
  double last_brake_time_;

  // Components
  std::unique_ptr<TetherCollisionChecker> collision_checker_;
  std::unique_ptr<DataLogger> data_logger_;

  // Timer (50 Hz control loop)
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Callbacks
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void tether_callback(const std_msgs::msg::String::SharedPtr msg);
  void goal_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void control_loop();

  // Helpers
  geometry_msgs::msg::Twist compute_twist_to_goal(
    const geometry_msgs::msg::Point& goal);
  CollisionRisk assess_collision_risk();
  void publish_state_marker();
  void log_velocity(double linear_x, double angular_z);
};

}  // namespace unitree_aliengo_bridge
