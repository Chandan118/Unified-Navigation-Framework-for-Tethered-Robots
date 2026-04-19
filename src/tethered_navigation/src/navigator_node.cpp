// All includes are now in the header, so we only need to include our header.
#include "tethered_navigation/navigator.hpp"
#include <cmath>
#include <memory>

// Use the standard chrono literals
using namespace std::chrono_literals;

// Constructor implementation
Navigator::Navigator()
    : Node("navigator_node"), current_state_(State::MOVE_TO_GOAL), current_waypoint_idx_(0) 
{
    // --- 1. Parameters ---
    this->declare_parameter<double>("linear_speed", 0.3);
    this->declare_parameter<double>("angular_speed", 0.5);
    this->declare_parameter<double>("obstacle_hit_dist", 0.5);
    this->declare_parameter<double>("waypoint_tolerance", 0.3);
    this->declare_parameter<double>("max_tether_length", 10.0);
    this->declare_parameter<std::string>("robot_id", "robot_1");
    
    linear_speed_ = this->get_parameter("linear_speed").as_double();
    angular_speed_ = this->get_parameter("angular_speed").as_double();
    obstacle_hit_dist_ = this->get_parameter("obstacle_hit_dist").as_double();
    waypoint_tolerance_ = this->get_parameter("waypoint_tolerance").as_double();
    max_tether_length_ = this->get_parameter("max_tether_length").as_double();
    robot_id_ = this->get_parameter("robot_id").as_string();
    
    // --- 2. TF Listener ---
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // --- 3. Subscribers ---
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&Navigator::odom_callback, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&Navigator::scan_callback, this, std::placeholders::_1));
    fuzzy_adj_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/fuzzy_adjustment", 10, std::bind(&Navigator::fuzzy_adj_callback, this, std::placeholders::_1));
    plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/plan", 10, std::bind(&Navigator::plan_callback, this, std::placeholders::_1));
    yielding_sub_ = this->create_subscription<tethered_navigation::msg::YieldingDecision>(
        "/yielding_decisions", 10, std::bind(&Navigator::yielding_callback, this, std::placeholders::_1));
    
    // --- 4. Publishers ---
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    fuzzy_input_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/fuzzy_input", 10);
    tether_state_pub_ = this->create_publisher<tethered_navigation::msg::TetherState>("/tether_state", 10);
    
    // --- 5. Timers ---
    timer_ = this->create_wall_timer(100ms, std::bind(&Navigator::control_loop, this));
    tether_pub_timer_ = this->create_wall_timer(100ms, std::bind(&Navigator::publish_tether_state, this));
    
    // --- 6. Initialize anchor point (will be set on first odom) ---
    start_pose_ = geometry_msgs::msg::Point();
    
    RCLCPP_INFO(this->get_logger(), "Navigator Node started (robot_id: %s)", robot_id_.c_str());
}

// ==================== Callbacks ====================

void Navigator::plan_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received new plan with %zu waypoints", msg->poses.size());
    current_plan_ = *msg;
    current_waypoint_idx_ = 0;
    
    // Set goal from the last waypoint
    if (!msg->poses.empty()) {
        goal_pose_ = msg->poses.back().pose.position;
    }
}

void Navigator::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_odom_ = *msg;
    
    // Set start pose as anchor on first odometry message
    if (start_pose_.x == 0.0 && start_pose_.y == 0.0) {
        start_pose_ = msg->pose.pose.position;
        RCLCPP_INFO(this->get_logger(), "Anchor point set to (%.2f, %.2f)", 
                    start_pose_.x, start_pose_.y);
    }
}

void Navigator::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Find minimum distance
    float min_dist = msg->range_max;
    for (const auto& range : msg->ranges) {
        if (range < min_dist && range > msg->range_min) {
            min_dist = range;
        }
    }
    
    // Compute tension based on distance from anchor
    double dist_from_start = std::hypot(
        current_odom_.pose.pose.position.x - start_pose_.x,
        current_odom_.pose.pose.position.y - start_pose_.y
    );
    current_tether_length_ = dist_from_start;
    current_tension_ = std::min(50.0, dist_from_start * 5.0);  // Simplified model
    
    // Publish fuzzy input
    std_msgs::msg::Float32MultiArray fuzzy_input;
    fuzzy_input.data.push_back(min_dist);
    fuzzy_input.data.push_back(static_cast<float>(current_tension_));
    fuzzy_input_pub_->publish(fuzzy_input);
    
    // Obstacle-triggered state transition
    if (current_state_ == State::MOVE_TO_GOAL && min_dist < obstacle_hit_dist_) {
        RCLCPP_INFO(this->get_logger(), "Obstacle detected! Switching to BOUNDARY_FOLLOWING.");
        current_state_ = State::BOUNDARY_FOLLOWING;
        hit_point_ = current_odom_.pose.pose.position;
        closest_dist_to_goal_on_boundary_ = 1e9;
    }
}

void Navigator::fuzzy_adj_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    fuzzy_adjustment_ = *msg;
}

void Navigator::yielding_callback(const tethered_navigation::msg::YieldingDecision::SharedPtr msg) {
    // Only process decisions targeting this robot
    if (msg->target_robot_id == robot_id_) {
        current_yielding_action_ = msg->suggested_action;
        RCLCPP_DEBUG(this->get_logger(), 
                     "Received yielding decision: %s (severity=%.2f)",
                     msg->suggested_action.c_str(), msg->crossing_severity);
    }
}

// ==================== Tether State Publishing ====================

void Navigator::publish_tether_state() {
    if (current_odom_.pose.pose.position.x == 0.0 && 
        current_odom_.pose.pose.position.y == 0.0) {
        return; // Not initialized yet
    }
    
    auto msg = tethered_navigation::msg::TetherState();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "map";
    msg.robot_id = robot_id_;
    
    // Anchor pose
    msg.anchor_pose.header = msg.header;
    msg.anchor_pose.pose.position = start_pose_;
    
    msg.tension = current_tension_;
    msg.slack_length = std::max(0.0, max_tether_length_ - current_tether_length_);
    msg.current_length = current_tether_length_;
    
    // Priority score (based on distance to goal and tension)
    if (!goal_pose_.has_value()) {
        msg.priority_score = 0.5;
    } else {
        double dist_to_goal = std::hypot(
            current_odom_.pose.pose.position.x - goal_pose_.x,
            current_odom_.pose.pose.position.y - goal_pose_.y
        );
        double dist_factor = std::max(0.0, 1.0 - dist_to_goal / 20.0);
        double tension_factor = std::min(1.0, current_tension_ / 50.0);
        msg.priority_score = 0.4 + 0.4 * dist_factor + 0.2 * tension_factor;
    }
    
    tether_state_pub_->publish(msg);
}

geometry_msgs::msg::Point Navigator::get_anchor_point() const {
    return start_pose_;
}

double Navigator::compute_tension_from_odom() {
    // Already computed in scan_callback
    return current_tension_;
}

double Navigator::compute_distance_to_goal() const {
    if (!goal_pose_.has_value()) return 0.0;
    return std::hypot(
        current_odom_.pose.pose.position.x - goal_pose_.x,
        current_odom_.pose.pose.position.y - goal_pose_.y
    );
}

// ==================== Control Loop ====================

void Navigator::control_loop() {
    auto current_waypoint = get_current_waypoint();
    
    if (current_waypoint.header.frame_id.empty()) {
        cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
        return;
    }
    
    double dist_to_waypoint = std::hypot(
        current_waypoint.pose.position.x - current_odom_.pose.pose.position.x,
        current_waypoint.pose.position.y - current_odom_.pose.pose.position.y
    );
    
    // Check if waypoint reached
    if (dist_to_waypoint < waypoint_tolerance_) {
        current_waypoint_idx_++;
        if (current_waypoint_idx_ >= current_plan_.poses.size()) {
            RCLCPP_INFO(this->get_logger(), "Plan completed! All waypoints reached.");
            current_plan_.poses.clear();
            cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
            return;
        }
        RCLCPP_DEBUG(this->get_logger(), "Waypoint %zu reached. Moving to %zu", 
                     current_waypoint_idx_-1, current_waypoint_idx_);
    }
    
    // Generate velocity command
    geometry_msgs::msg::Twist cmd_vel;
    double current_yaw = get_yaw_from_quaternion(current_odom_.pose.pose.orientation);
    
    if (current_state_ == State::MOVE_TO_GOAL) {
        double bearing = get_bearing_to_goal();
        double angle_diff = normalize_angle(bearing - current_yaw);
        cmd_vel.linear.x = linear_speed_;
        cmd_vel.angular.z = 2.0 * angle_diff;
    } else if (current_state_ == State::BOUNDARY_FOLLOWING) {
        cmd_vel.linear.x = linear_speed_ * 0.5;
        cmd_vel.angular.z = angular_speed_;
        
        double bearing = get_bearing_to_goal();
        double angle_diff_to_goal = std::abs(normalize_angle(bearing - current_yaw));
        
        if (dist_to_waypoint < closest_dist_to_goal_on_boundary_ && 
            angle_diff_to_goal < 0.3) {
            RCLCPP_INFO(this->get_logger(), "Path clear. Switching to MOVE_TO_GOAL.");
            current_state_ = State::MOVE_TO_GOAL;
        }
        if (dist_to_waypoint < closest_dist_to_goal_on_boundary_) {
            closest_dist_to_goal_on_boundary_ = dist_to_waypoint;
        }
    }
    
    // Apply fuzzy adjustment
    cmd_vel.linear.x += fuzzy_adjustment_.linear.x * 0.1;
    cmd_vel.angular.z += fuzzy_adjustment_.angular.z * 0.2;
    
    // Apply yielding-based adjustment
    if (current_yielding_action_ == "yield") {
        cmd_vel.linear.x *= 0.5;  // Slow down significantly
    } else if (current_yielding_action_ == "reroute") {
        cmd_vel.angular.z += 0.3;  // Turn slightly
    } else if (current_yielding_action_ == "speed_up") {
        cmd_vel.linear.x *= 1.2;   // Accelerate slightly
    }
    
    cmd_vel.linear.x = std::max(0.0, cmd_vel.linear.x);
    
    cmd_vel_pub_->publish(cmd_vel);
}

// ==================== Helper Implementations ====================

geometry_msgs::msg::PoseStamped Navigator::get_current_waypoint() {
    if (current_plan_.poses.empty() || 
        current_waypoint_idx_ >= current_plan_.poses.size()) {
        return geometry_msgs::msg::PoseStamped();
    }
    return current_plan_.poses[current_waypoint_idx_];
}

double Navigator::get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q) {
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(tf_q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

double Navigator::normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double Navigator::get_bearing_to_goal() {
    auto waypoint = get_current_waypoint();
    if (waypoint.header.frame_id.empty()) return 0.0;
    
    double cx = current_odom_.pose.pose.position.x;
    double cy = current_odom_.pose.pose.position.y;
    double gx = waypoint.pose.position.x;
    double gy = waypoint.pose.position.y;
    return std::atan2(gy - cy, gx - cx);
}

// ==================== Main ====================

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Navigator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
