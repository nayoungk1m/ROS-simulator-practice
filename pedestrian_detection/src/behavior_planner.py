#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import UInt8
from pedestrian_detection.msg import Pedestrian

# Behavior
KEEPING_WAYPOINT = 0
TRAFFIC_STOP = 1  # When traffic light is red
EMERGENCY_STOP = 2  # When pedestrian detected

class BehaviorPlanner():
    def __init__(self):
        # Initialize node
        rospy.init_node('behavior_planner', anonymous=True)

        # Subscribe topics
        self.ped_sub = rospy.Subscriber("/ped_result", Pedestrian, self.ped_callback)

        # Publish topics
        self.behavior_pub = rospy.Publisher("/behavior", UInt8, queue_size=100)
        # State
        self.behavior = UInt8()
        self.ped = None
        self.is_pedestrian_detected = False
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            if self.is_pedestrian_detected and self.is_ped_in_front(self.ped) and self.ped_count > 0:
                self.behavior.data = EMERGENCY_STOP
                rospy.loginfo("Pedestrian detected")
            else:
                self.behavior.data = KEEPING_WAYPOINT

            # Publish behavior
            self.behavior_pub.publish(self.behavior)
            rospy.loginfo(f"Behavior: {self.behavior.data}")
            rate.sleep()

        rospy.spin()

    def ped_callback(self, msg):
        self.ped_count = msg.count
        self.ped = msg.bbox
        self.is_pedestrian_detected = True
    
    def is_ped_in_front(self, ped):
        # Check if pedestrian bigger than certain size
        if len(ped) == 0:
            return False
        for bbox in ped:
            if bbox.w > 25 and bbox.h > 60:
                return True
        return False

if __name__ == "__main__":
    try:
        BehaviorPlanner()
    except rospy.ROSInterruptException:
        pass