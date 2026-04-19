#pragma once

#include <vector>
#include <memory>
#include <string>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/header.hpp"

namespace unitree_aliengo_bridge
{

// Swarm tether state - matching the estimator's TetherLine
struct SwarmTether
{
  std::string robot_id;
  geometry_msgs::msg::Point start;  // Robot position (tether attachment)
  geometry_msgs::msg::Point end;    // Tether end point
  float thickness_m;
  bool is_active;

  // Line segment representation for collision checking
  geometry_msgs::msg::Point get_start_3d() const { return start; }
  geometry_msgs::msg::Point get_end_3d() const {
    geometry_msgs::msg::Point p = end;
    p.z = 0.1;  // tether height above ground (10cm)
    return p;
  }
};

// Collision prediction result
struct CollisionRisk
{
  bool collision_imminent;      // Will tether cross robot's path?
  float time_to_collision_s;   // Estimated time (if collision_imminent)
  std::string conflicting_robot_id;
  float min_distance_m;        // Closest approach distance
};

// Configuration for yielding behavior
struct YieldingConfig
{
  float safety_margin_m;         // Extra clearance around tethers (m)
  float yield_stop_duration_s;   // How long to stop for crossing
  float deceleration_rate;       // Smoothness of braking
  float max_speed_mps;           // Normal cruise speed
  float min_speed_mps;           // Minimum speed during cautious movement
};

}  // namespace unitree_aliengo_bridge
