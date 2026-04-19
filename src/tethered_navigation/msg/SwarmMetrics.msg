std_msgs/Header header
int32 active_robots               # number of active robots
int32 total_entanglements        # total entanglement count
int32 resolved_entanglements     # resolved entanglement count
float64 avg_tension             # average tension (N)
float64 max_tension             # maximum tension (N)
float64 swarm_traversal_time    # swarm traversal time (s)
bool all_robots_at_goal         # whether all robots reached goal
string[] entanglement_pairs     # entangled robot pairs ["robot_1-robot_2", ...]
