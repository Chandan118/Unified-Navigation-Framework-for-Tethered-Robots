#pragma once

#include <vector>
#include <cmath>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace tether_detection
{

struct LineSegment
{
  geometry_msgs::msg::Point start;
  geometry_msgs::msg::Point end;
  float length;
  float confidence;  // 0-1 based on point density
  bool is_tether_candidate;
};

struct DetectionConfig
{
  float min_line_length_m;      // Minimum tether segment length
  float max_line_length_m;      // Maximum (typical tether ~2-3m)
  float point_density_threshold; // Min points per meter for valid line
  float height_threshold_m;     // Tether typically at specific height
  float angle_inference_deg;    // Tether usually vertical/horizontal
};

}  // namespace tether_detection
