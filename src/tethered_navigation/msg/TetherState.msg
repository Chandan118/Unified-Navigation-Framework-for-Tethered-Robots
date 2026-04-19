std_msgs/Header header
string robot_id                    # robot unique identifier (e.g., "robot_1")
geometry_msgs/PoseStamped anchor_pose  # anchor position (world frame)
float64 tension                    # current tether tension (N)
float64 slack_length               # slack length (m)
float64 current_length             # current tether length (m)
geometry_msgs/Point[] tether_segment_points  # tether key points (for crossing detection)
bool is_crossing                   # whether tether is crossing with other tethers
float64 priority_score            # robot priority (0-1, for yield decision)
