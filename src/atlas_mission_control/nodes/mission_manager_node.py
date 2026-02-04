#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from hybrid_navigation.msg import TetherStatus

class MissionManager:
    def __init__(self):
        rospy.init_node('mission_manager', anonymous=True)
        self.tether_sub = rospy.Subscriber('/tether_status', TetherStatus, self.tether_callback)
        self.mission_status_pub = rospy.Publisher('/mission_status', String, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        
        rospy.loginfo("ATLAS-T Mission Manager Initialized")
        self.current_tether_tension = 0.0

    def tether_callback(self, data):
        self.current_tether_tension = data.tension
        if self.current_tether_tension > 20.0:
            rospy.logwarn("High tether tension detected! Modifying mission strategy.")
            self.mission_status_pub.publish("EMERGENCY_RECOVERY")

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.mission_status_pub.publish("MISSION_ACTIVE")
            rate.sleep()

if __name__ == '__main__':
    try:
        manager = MissionManager()
        manager.run()
    except rospy.ROSInterruptException:
        pass
