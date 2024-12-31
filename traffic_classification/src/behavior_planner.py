#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import String, UInt8

# Behavior
KEEPING_WAYPOINT = 0
TRAFFIC_STOP = 1 # When traffic light is red

class BehaviorPlanner():
    def __init__(self):
        # Initialize node
        rospy.init_node('behavior_planner', anonymous=True)

        # Subscribe topics
        self.traffic_sub = rospy.Subscriber("/traffic", String, self.traffic_callback)

        # Publish topics
        self.behavior_pub = rospy.Publisher("/behavior", UInt8, queue_size=100)
        # State
        self.behavior = UInt8()
        self.traffic = None
        self.is_traffic_detected = False
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            if self.is_traffic_detected and self.is_traffic_stop(self.traffic):
                self.behavior.data = TRAFFIC_STOP
                rospy.loginfo("STOP")
            else:
                self.behavior.data = KEEPING_WAYPOINT

            # Publish behavior
            self.behavior_pub.publish(self.behavior)
            rospy.loginfo(f"Behavior: {self.behavior.data}")
            rate.sleep()
        
        rospy.spin()

    def traffic_callback(self, msg):
        if msg.data:
            self.traffic = msg.data
            self.is_traffic_detected = True

    def is_traffic_stop(self, traffic):
        # Check if traffic light is red
        if traffic == "red" or traffic == "yellow":
            return True
        else:
            return False
    

if __name__ == "__main__":
    try:
        BehaviorPlanner()
    except rospy.ROSInterruptException:
        pass