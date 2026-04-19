#pragma once

#include <vector>
#include <memory>
#include <string>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace unitree_aliengo_bridge
{

// 碰撞预测器 - 独立类，只负责几何计算
class TetherCollisionChecker
{
public:
  TetherCollisionChecker(float safety_margin_m, float robot_speed_mps);

  // 预测机器人路径是否会与系留线相交
  struct Prediction
  {
    bool collision_imminent;
    float time_to_collision_s;
    float min_distance_m;
    geometry_msgs::msg::Point ip_point;
  };

  Prediction predict_crossing(
    const geometry_msgs::msg::Point& robot_pos,
    double robot_heading,
    const geometry_msgs::msg::Point& tether_start,
    const geometry_msgs::msg::Point& tether_end,
    float robot_speed);

private:
  float safety_margin_;
  float robot_speed_;

  std::optional<std::pair<double,double>> line_segment_intersection(
    double x1, double y1, double x2, double y2,
    double x3, double y3, double x4, double y4);

  double point_to_segment_distance(
    double px, double py,
    double x1, double y1, double x2, double y2);
};

}  // namespace unitree_aliengo_bridge
