#include "swarm_tether_estimator/tether_types.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace swarm_tether_estimator
{

class TetherPublisher : public rclcpp::Node
{
public:
  explicit TetherPublisher(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // Parameters
  std::string world_frame_;
  float tether_thickness_m_;
  float publish_rate_hz_;

  // Publishers
  rclcpp::Publisher<TetherStateArray>::SharedPtr tether_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Simulated robot positions (would come from /robot_N/odom in real setup)
  std::vector<geometry_msgs::msg::Point> simulated_robot_positions_;

  // For demo: animate virtual robots crossing
  size_t demo_step_ = 0;

  void timer_callback();
  void update_simulated_robots();
  TetherLine generate_tether_for_robot(
    const std::string& robot_id,
    const geometry_msgs::msg::Point& robot_pos);
  visualization_msgs::msg::Marker create_tether_marker(
    const TetherLine& tether,
    size_t index);
};

}  // namespace swarm_tether_estimator
