#include "swarm_tether_estimator/tether_publisher.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace swarm_tether_estimator
{

TetherPublisher::TetherPublisher(const rclcpp::NodeOptions& options)
: Node("tether_publisher", options)
{
  this->declare_parameter("world_frame", "map");
  this->declare_parameter("tether_thickness_m", 0.02f);
  this->declare_parameter("publish_rate_hz", 20.0f);
  this->declare_parameter("n_virtual_robots", 3);

  world_frame_ = this->get_parameter("world_frame").as_string();
  tether_thickness_m_ = this->get_parameter("tether_thickness_m").as_double();
  publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();
  auto n_virtual = this->get_parameter("n_virtual_robots").as_int();

  simulated_robot_positions_.resize(n_virtual);
  for (int i = 0; i < n_virtual; ++i) {
    simulated_robot_positions_[i] = geometry_msgs::msg::Point();
  }

  tether_pub_ = this->create_publisher<TetherStateArray>(
    "/swarm_tether_states", 10);
  viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/swarm_tether_viz", 10);

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate_hz_),
    std::bind(&TetherPublisher::timer_callback, this));

  RCLCPP_INFO(this->get_logger(),
    "TetherPublisher started. Publishing /swarm_tether_states at %.1f Hz",
    publish_rate_hz_);
}

void TetherPublisher::timer_callback()
{
  update_simulated_robots();

  TetherStateArray tether_array;
  tether_array.header.stamp = this->now();
  tether_array.header.frame_id = world_frame_;

  for (size_t i = 0; i < simulated_robot_positions_.size(); ++i) {
    std::string robot_id = "virtual_robot_" + std::to_string(i);
    auto tether = generate_tether_for_robot(robot_id, simulated_robot_positions_[i]);
    tether_array.tethers.push_back(tether);
  }

  tether_pub_->publish(tether_array);

  visualization_msgs::msg::MarkerArray viz_array;
  for (size_t i = 0; i < tether_array.tethers.size(); ++i) {
    viz_array.markers.push_back(create_tether_marker(tether_array.tethers[i], i));
  }
  viz_pub_->publish(viz_array);

  demo_step_++;
}

void TetherPublisher::update_simulated_robots()
{
  double t = demo_step_ * 0.1;

  simulated_robot_positions_[0].x = 0.5 + fmod(t * 0.5, 8.0);
  simulated_robot_positions_[0].y = 3.0;

  simulated_robot_positions_[1].x = 8.5 - fmod(t * 0.3 + 2.0, 8.0);
  simulated_robot_positions_[1].y = 5.0;

  if (simulated_robot_positions_.size() > 2) {
    simulated_robot_positions_[2].x = 4.5;
    simulated_robot_positions_[2].y = 1.0 + fmod(t * 0.4, 7.0);
  }
}

TetherLine TetherPublisher::generate_tether_for_robot(
  const std::string& robot_id,
  const geometry_msgs::msg::Point& robot_pos)
{
  TetherLine tether;
  tether.robot_id = robot_id;
  tether.start = robot_pos;
  tether.end = robot_pos;
  tether.end.x += 1.5;
  tether.thickness_m = tether_thickness_m_;
  tether.is_active = true;
  return tether;
}

visualization_msgs::msg::Marker TetherPublisher::create_tether_marker(
  const TetherLine& tether,
  size_t index)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = world_frame_;
  marker.header.stamp = this->now();
  marker.ns = "swarm_tethers";
  marker.id = static_cast<int>(index);
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.orientation.w = 1.0;
  marker.scale.x = tether.thickness_m;
  marker.color.a = 0.7;
  marker.color.r = 1.0;
  marker.color.g = 0.5;
  marker.color.b = 0.0;

  marker.points.push_back(tether.start);
  marker.points.push_back(tether.end);

  return marker;
}

}  // namespace swarm_tether_estimator

RCLCPP_COMPONENTS_REGISTER_NODE(swarm_tether_estimator::TetherPublisher)
