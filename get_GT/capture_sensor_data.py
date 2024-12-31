#!/usr/bin/env python3
'''
실행하기 전, 센서 캡처 기능을 활성화해야 합니다.
'''
import rospy
from morai_msgs.msg import SaveSensorData

class SaveSensorDataPublisher:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("save_sensor_data_publisher", anonymous=True)
        #create publisher
        self.pub = rospy.Publisher("/SaveSensorData", SaveSensorData, queue_size=10)
        # Publish rate setting (2 seconds)
        self.rate = rospy.Rate(0.5)  # 0.5Hz = 2s interval

    def publish_data(self):
        while not rospy.is_shutdown():
            # publsih message
            msg = SaveSensorData()
            msg.is_custom_file_name = False  # 기본 저장 경로 사용

            self.pub.publish(msg)
            rospy.loginfo("Published /SaveSensorData message")

            self.rate.sleep()

if __name__ == "__main__":
    try:
        publisher = SaveSensorDataPublisher()
        publisher.publish_data()
    except rospy.ROSInterruptException:
        pass
