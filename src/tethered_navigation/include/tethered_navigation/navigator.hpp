#ifndef NAVIGATOR_HPP
#define NAVIGATOR_HPP

// --- CORE LIBRARIES ---
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Custom messages
#include <tethered_navigation/msg/tether_state.hpp>
#include <tethered_navigation/msg/yielding_decision.hpp>

#include <memory>
#include <string>
#include <vector>

// An enumeration to manage the robot's current state for the Bug algorithm
enum class State {
    MOVE_TO_GOAL,
    BOUNDARY_FOLLOWING
};

// Forward declarations
class geometry_msgs::msg::Point;

// Declare the Navigator class
class Navigator : public rclcpp::Node {
public:
    // Constructor
    Navigator();

private:
    // ==================== Callback Functions ====================
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void fuzzy_adj_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void plan_callback(const nav_msgs::msg::Path::SharedPtr msg);
    void yielding_callback(const tethered_navigation::msg::YieldingDecision::SharedPtr msg);

    // ==================== Core Logic ====================
    void control_loop();
    void publish_tether_state();
    double compute_tension_from_odom();
    geometry_msgs::msg::Point get_anchor_point() const;

    // ==================== Helper Functions ====================
    double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q);
    double get_bearing_to_goal();
    double normalize_angle(double angle);
    double compute_distance_to_goal() const;

    // ==================== ROS 2 Publishers ====================
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr fuzzy_input_pub_;
    rclcpp::Publisher<tethered_navigation::msg::TetherState>::SharedPtr tether_state_pub_;

    // ==================== ROS 2 Subscribers ====================
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr fuzzy_adj_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
    rclcpp::Subscription<tethered_navigation::msg::YieldingDecision>::SharedPtr yielding_sub_;

    // ==================== ROS 2 Timer ====================
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr tether_pub_timer_;

    // ==================== TF2 ====================
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ==================== State and Data Members ====================
    State current_state_;
    nav_msgs::msg::Odometry current_odom_;
    geometry_msgs::msg::Twist fuzzy_adjustment_;

    // Tether state
    geometry_msgs::msg::Point start_pose_;          // 起始位置（锚点）
    double current_tether_length_ = 0.0;
    double max_tether_length_ = 10.0;
    double current_tension_ = 0.0;
    std::string robot_id_;

    // Swarm coordination
    std::string current_yielding_action_ = "maintain";
    double priority_score_ = 0.5;
    geometry_msgs::msg::Point goal_pose_;

    // Path following logic
    nav_msgs::msg::Path current_plan_;
    size_t current_waypoint_idx_;
    geometry_msgs::msg::PoseStamped get_current_waypoint();

    // ==================== Configuration Parameters ====================
    double linear_speed_;
    double angular_speed_;
    double obstacle_hit_dist_;
    double waypoint_tolerance_;

    // Bug Algorithm State Variables
    geometry_msgs::msg::Point hit_point_;
    double closest_dist_to_goal_on_boundary_;
};

#endif // NAVIGATOR_HPP
