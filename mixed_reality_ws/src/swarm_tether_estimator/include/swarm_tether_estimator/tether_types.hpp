#pragma once

#include <vector>
#include <memory>
#include <string>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace swarm_tether_estimator
{

struct TetherLine
{
  std::string robot_id;
  geometry_msgs::msg::Point start;
  geometry_msgs::msg::Point end;
  float thickness_m;
  bool is_active;
};

struct TetherStateArray
{
  std::vector<TetherLine> tethers;

  void add_tether(
    const std::string& robot_id,
    const geometry_msgs::msg::Point& start,
    const geometry_msgs::msg::Point& end,
    float thickness = 0.02f,
    bool active = true)
  {
    TetherLine t;
    t.robot_id = robot_id;
    t.start = start;
    t.end = end;
    t.thickness_m = thickness;
    t.is_active = active;
    tethers.push_back(t);
  }

  void clear() { tethers.clear(); }
};

}  // namespace swarm_tether_estimator

