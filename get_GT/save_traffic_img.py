#!/usr/bin/env python3
'''
실행하기 전, 센서 캡처 기능을 꺼야 됩니다.
'''
import rospy
import cv2
import os
import numpy as np
from sensor_msgs.msg import CompressedImage
from morai_msgs.msg import GetTrafficLightStatus

class SaveTrafficPic:
    def __init__(self):
        # Subscribers
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.image_callback)
        self.traffic_sub = rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, self.traffic_callback)

        # Variables
        self.current_image = None

        # Image save directory
        self.base_dir = "./traffic_images"
        os.makedirs(self.base_dir, exist_ok=True)

    def traffic_callback(self, data):
        """교통 신호 상태 업데이트 및 이미지 저장"""
        # Set traffic status
        if data.trafficLightStatus == 1:
            traffic_status = "red"
        elif data.trafficLightStatus == 4:
            traffic_status = "yellow"
        elif data.trafficLightStatus == 16:
            traffic_status = "green"
        elif data.trafficLightStatus == 48:
            traffic_status = "left"
        else:
            traffic_status = "none"

        # Save image every 10th frame
        if data.header.seq % 10 == 0:
            if self.current_image is not None:
                # Image save directory according to traffic status
                folder_path = os.path.join(self.base_dir, traffic_status)
                os.makedirs(folder_path, exist_ok=True)
                # Save image
                image_path = os.path.join(folder_path, f"{traffic_status}_{data.header.seq:05d}.jpg")
                cv2.imwrite(image_path, self.current_image)
                rospy.loginfo(f"Saved image to {image_path} under {traffic_status} status")

    def image_callback(self, data):
        """Update the current image"""
        np_arr = np.frombuffer(data.data, np.uint8)
        self.current_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

if __name__ == "__main__":
    rospy.init_node("save_traffic_pic_node", anonymous=True)
    save_traffic_pic = SaveTrafficPic()
    rospy.spin()