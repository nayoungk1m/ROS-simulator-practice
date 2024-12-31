#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
from math import sqrt, atan2, pow
from geometry_msgs.msg import Point
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float32, UInt8, Bool
from morai_msgs.msg import CtrlCmd, EventInfo, CollisionData
from morai_msgs.srv import MoraiEventCmdSrv

# Behavior States
KEEPING_WAYPOINT = 0
EMERGENCY_STOP = 1
TRAFFIC_STOP = 2

class Controller:
    def __init__(self):
        # Subscribers
        rospy.Subscriber('behavior', UInt8, self.behavior_callback)

        # Publishers
        self.ctrl_cmd_pub = rospy.Publisher('ctrl_cmd', CtrlCmd, queue_size=1)

        # ROS Service for gear control
        rospy.wait_for_service('/Service_MoraiEventCmd')
        self.event_cmd_service = rospy.ServiceProxy('/Service_MoraiEventCmd', MoraiEventCmdSrv)

        # Initialize Variables
        self.behavior = KEEPING_WAYPOINT
        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 2

        # Main Control Loop
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.keep_straight()
            rospy.loginfo(f"velocity: {self.ctrl_cmd_msg.velocity}, steering: {self.ctrl_cmd_msg.steering}")
            if self.behavior == TRAFFIC_STOP:
                self.traffic_stop()
            rate.sleep()

    def behavior_callback(self, msg):
        self.behavior = msg.data

    def keep_straight(self):
        """
        차량을 직진시키는 함수
        """
        self.ctrl_cmd_msg.velocity = 10
        self.ctrl_cmd_msg.steering = 0
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

    def traffic_stop(self):
        """
        차량을 정지시키고 기어를 N으로 변경
        """
        rospy.loginfo("Emergency Stop Activated")

        # 차량 속도와 조향 초기화
        self.ctrl_cmd_msg.velocity = 0
        self.ctrl_cmd_msg.steering = 0
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

        # 2초 대기 (안전 정지를 위한 시간)
        time.sleep(2)

        # 기어를 N으로 설정
        self.publish_event_gear(gear=3)  # 3은 N 기어를 나타냄
        rospy.loginfo("Gear changed to Neutral")

    def publish_event_gear(self, gear):
        """
        기어 변경 이벤트를 퍼블리시
        """
        event_info_msg = EventInfo()
        event_info_msg.option = 2
        event_info_msg.ctrl_mode = 3
        event_info_msg.gear = gear
        try:
            self.event_cmd_service(event_info_msg)
            rospy.loginfo(f"Gear change command sent: {gear}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to send gear change command: {e}")

if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    controller = Controller()