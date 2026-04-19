#ifndef TETHER_COLLISION_CHECKER_CPP
#define TETHER_COLLISION_CHECKER_CPP

#include "unitree_aliengo_bridge/tether_collision_checker.hpp"
#include <cmath>
#include <algorithm>

namespace unitree_aliengo_bridge
{

TetherCollisionChecker::TetherCollisionChecker(float safety_margin_m, float robot_speed_mps)
: safety_margin_(safety_margin_m), robot_speed_(robot_speed_mps) {}

TetherCollisionChecker::Prediction TetherCollisionChecker::predict_crossing(
  const geometry_msgs::msg::Point& robot_pos,
  double robot_heading,
  const geometry_msgs::msg::Point& tether_start,
  const geometry_msgs::msg::Point& tether_end,
  float robot_speed)
{
  Prediction pred;
  pred.collision_imminent = false;
  pred.time_to_collision_s = 999.0f;
  pred.min_distance_m = 999.0f;

  double vx = cos(robot_heading) * robot_speed;
  double vy = sin(robot_heading) * robot_speed;

  double tx1 = tether_start.x, ty1 = tether_start.y;
  double tx2 = tether_end.x,   ty2 = tether_end.y;

  auto intersection = line_segment_intersection(
    robot_pos.x, robot_pos.y,
    robot_pos.x + vx*10.0, robot_pos.y + vy*10.0,
    tx1, ty1, tx2, ty2);

  if (intersection.has_value()) {
    auto [ix, iy] = intersection.value();
    double dist_to_ip = sqrt((ix - robot_pos.x)*(ix - robot_pos.x) +
                             (iy - robot_pos.y)*(iy - robot_pos.y));
    if (robot_speed > 0.01) {
      double ttc = dist_to_ip / robot_speed;
      if (ttc < 3.0) {
        pred.collision_imminent = true;
        pred.time_to_collision_s = ttc;
        pred.ip_point.x = ix;
        pred.ip_point.y = iy;
      }
    }
  }

  double min_dist = point_to_segment_distance(robot_pos.x, robot_pos.y, tx1, ty1, tx2, ty2);
  pred.min_distance_m = static_cast<float>(min_dist);

  if (min_dist < safety_margin_) {
    pred.collision_imminent = true;
    if (pred.time_to_collision_s > 2.0) {
      pred.time_to_collision_s = 2.0;
    }
  }

  return pred;
}

std::optional<std::pair<double,double>> TetherCollisionChecker::line_segment_intersection(
  double x1, double y1, double x2, double y2,
  double x3, double y3, double x4, double y4)
{
  double denom = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4);
  if (fabs(denom) < 1e-9) return std::nullopt;

  double t = ((x1-x3)*(y3-y4) - (y1-y3)*(x3-x4)) / denom;
  double u = -((x1-x2)*(y1-y3) - (y1-y2)*(x1-x3)) / denom;

  if (t >= 0 && u >= 0 && u <= 1) {
    return std::make_pair(x1 + t*(x2-x1), y1 + t*(y2-y1));
  }
  return std::nullopt;
}

double TetherCollisionChecker::point_to_segment_distance(
  double px, double py, double x1, double y1, double x2, double y2)
{
  double A = px - x1;
  double B = py - y1;
  double C = x2 - x1;
  double D = y2 - y1;

  double dot = A * C + B * D;
  double len_sq = C * C + D * D;
  double param = -1.0;
  if (len_sq != 0.0) param = dot / len_sq;

  double xx, yy;
  if (param < 0) {
    xx = x1; yy = y1;
  } else if (param > 1) {
    xx = x2; yy = y2;
  } else {
    xx = x1 + param * C;
    yy = y1 + param * D;
  }

  double dx = px - xx;
  double dy = py - yy;
  return sqrt(dx*dx + dy*dy);
}

}  // namespace unitree_aliengo_bridge

#endif
