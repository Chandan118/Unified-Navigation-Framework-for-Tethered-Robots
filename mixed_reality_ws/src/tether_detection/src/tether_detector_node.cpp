#include "tether_detection/tether_detector_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace tether_detection
{

TetherDetectorNode::TetherDetectorNode(const rclcpp::NodeOptions& options)
: Node("tether_detector", options)
{
  this->declare_parameter("min_line_length_m", 1.0);
  this->declare_parameter("max_line_length_m", 4.0);
  this->declare_parameter("height_min_m", 0.05);
  this->declare_parameter("height_max_m", 0.5);
  this->declare_parameter("point_density_threshold", 20.0);

  config_.min_line_length_m = this->get_parameter("min_line_length_m").as_double();
  config_.max_line_length_m = this->get_parameter("max_line_length_m").as_double();
  config_.point_density_threshold = this->get_parameter("point_density_threshold").as_double();

  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/camera/depth/points", 10,
    std::bind(&TetherDetectorNode::point_cloud_callback, this, std::placeholders::_1));

  lines_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/detected_tethers", 10);
  avoidance_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
    "/avoidance_point", 10);

  extractor_ = LineExtractor(config_);

  RCLCPP_INFO(this->get_logger(), "TetherDetectorNode initialized");
}

void TetherDetectorNode::point_cloud_callback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  current_tethers_ = extractor_.extract_lines(msg);
  publish_detected_lines();

  if (!current_tethers_.empty()) {
    for (const auto& tether : current_tethers_) {
      auto avoid_pt = compute_avoidance_point(tether);
      geometry_msgs::msg::PointStamped pts;
      pts.header.stamp = this->now();
      pts.header.frame_id = "base_link";
      pts.point = avoid_pt;
      avoidance_point_pub_->publish(pts);
      break;
    }
  }
}

void TetherDetectorNode::publish_detected_lines()
{
  visualization_msgs::msg::MarkerArray arr;
  int id = 0;
  for (const auto& tether : current_tethers_) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "base_link";
    m.header.stamp = this->now();
    m.ns = "detected_tethers";
    m.id = id++;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.02;
    m.color.a = 0.8;
    m.color.r = 1.0;
    m.color.g = 0.2;
    m.color.b = 0.2;

    m.points.push_back(tether.start);
    m.points.push_back(tether.end);
    arr.markers.push_back(m);
  }
  lines_pub_->publish(arr);
}

geometry_msgs::msg::Point TetherDetectorNode::compute_avoidance_point(
  const LineSegment& tether)
{
  geometry_msgs::msg::Point avoid_pt;
  double mid_x = (tether.start.x + tether.end.x) / 2.0;
  double mid_y = (tether.start.y + tether.end.y) / 2.0;

  double dx = tether.end.x - tether.start.x;
  double dy = tether.end.y - tether.start.y;
  double len = sqrt(dx*dx + dy*dy);
  if (len > 0) {
    double perp_x = -dy / len;
    double perp_y =  dx / len;
    avoid_pt.x = mid_x + perp_x * 0.8;
    avoid_pt.y = mid_y + perp_y * 0.8;
    avoid_pt.z = 0.0;
  }

  return avoid_pt;
}

}  // namespace tether_detection

RCLCPP_COMPONENTS_REGISTER_NODE(tether_detection::TetherDetectorNode)
