#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO detection result
Custom Message(string or integer) publish
std_msgs 참고
"""
import cv2
import rospy
import numpy as np
from ultralytics import YOLO
from sensor_msgs.msg import CompressedImage
from pedestrian_detection.msg import Pedestrian, Bbox
import time
import os

class PedestrianDetection:
    def __init__(self):
        # Initialize node
        rospy.init_node('pedestrian_detection', anonymous=True)

        # Subscribe image
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback, queue_size=10)

        # Publish result
        self.result_publish = rospy.Publisher("/ped_result", Pedestrian, queue_size=5)

        # Load YOLO model
        self.model = YOLO(os.path.join(os.path.dirname(__file__), 'ped_weight.pt'))
        
        self.frame_counter = 0
        self.frame_interval = 4  # YOLO를 실행할 프레임 간격(딜레이 방지)
        
        rospy.spin()
        
    def callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if img_bgr is not None:
                self.frame_counter += 1
                if self.frame_counter % self.frame_interval == 0:
                    res = self.model.predict(img_bgr)
                    plots = res[0].plot()

                    # Create Pedestrian message
                    msg = Pedestrian()

                    # 보행자가 감지되면 메시지에 보행자 수와 bbox 정보 담음
                    if len(res[0].boxes.xywh) > 0: 
                        xywh_box = res[0].boxes.xywh.cpu().numpy()  # Convert tensor to numpy array
                        msg.count = len(xywh_box)
                        # pedestrian array
                        for bbox in xywh_box:
                            bbox_msg = Bbox()
                            bbox_msg.x, bbox_msg.y, bbox_msg.w, bbox_msg.h = bbox
                            msg.bbox.append(bbox_msg)

                        rospy.loginfo(f"{len(xywh_box)} Pedestrian detected")
                    else:
                        msg.count = 0
                        rospy.loginfo("No pedestrian detected")

                    # Publish Pedestrian message
                    self.result_publish.publish(msg)
                    cv2.imshow("Pedestrian detection", plots)
                    cv2.waitKey(1)

        except Exception as e:
            rospy.logerr("Error processing image: {0}".format(e))

if __name__ == '__main__':
    try:
        PedestrianDetection()
    except rospy.ROSInterruptException:
        pass