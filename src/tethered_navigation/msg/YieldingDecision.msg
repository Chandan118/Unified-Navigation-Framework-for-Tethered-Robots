std_msgs/Header header
string requesting_robot_id      # robot requesting path modification
string target_robot_id          # target robot (whose tether is crossed)
geometry_msgs/Point crossing_point  # crossing point position (world frame)
float64 crossing_severity       # crossing severity (0-1)
bool yield_requested           # whether yield is requested to target robot
string suggested_action        # suggested action: "yield", "reroute", "maintain"
float64 yield_duration_est     # estimated yield duration (s)
float64 priority_comparison   # priority comparison: >0 means I have higher priority
