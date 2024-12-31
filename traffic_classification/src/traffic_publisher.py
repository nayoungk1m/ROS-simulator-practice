#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import torch
import torchvision.models as models
import torch.nn as nn
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from torchvision import transforms
import os

class TrafficClassification:
    def __init__(self):
        # Initialize node
        rospy.init_node('traffic_publisher', anonymous=True)
        
        # Subscribe topics
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        
        # Publish topics
        self.result_publish = rospy.Publisher("/traffic", String, queue_size=10)

        # Variables
        self.signal_dict = {0: "green", 1: "red", 2: "yellow"}
        self.frame_counter = 0
        self.frame_interval = 4

        # Load trained classification model
        self.model = models.alexnet(weights=None)
        
        # Modify the classifier (for 3 classes)
        self.model.classifier[6] = nn.Linear(self.model.classifier[6].in_features, 3)
        
        # Load the trained model
        self.model.load_state_dict(torch.load(os.path.join(os.path.dirname(__file__), 'traffic_weight.pt'), weights_only=True))
        
        # Set the model to evaluation mode
        self.model.eval()
        self.model.to('cuda')

        # Image preprocessing
        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((224, 224)),  # 이미지 사이즈 조정
            transforms.ToTensor(),
            transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))
        ])

        rospy.spin()

    def callback(self, msg):
        self.frame_counter += 1
        if self.frame_counter % self.frame_interval == 0:
            # Image decoding
            np_arr = np.frombuffer(msg.data, np.uint8)
            img_rgb = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
            img_bgr = self.region_of_interest(img_bgr)

            if img_bgr is not None:
                # Image preprocessing to tensor
                input_tensor = self.transform(img_bgr)
                input_batch = input_tensor.unsqueeze(0).to('cuda')

                with torch.no_grad():
                    outputs = self.model(input_batch)
                    _, predicted = torch.max(outputs, 1)

                # Publish the traffic signal
                result = self.signal_dict[predicted.item()]
                rospy.loginfo("Traffic signal: %s" % result)
                self.signal_publish(result)

                # Display the image
                cv2.imshow("Image window", cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB))
                cv2.waitKey(1)

    def region_of_interest(self, image):
        # Extract ROI coordinates
        x_min, y_min, x_max, y_max = 160, 0, 480, 180

        # Crop the image directly to the ROI box
        cropped_image = image[y_min:y_max, x_min:x_max]

        return cropped_image

    def signal_publish(self, result):
        self.result_publish.publish(result)

if __name__ == '__main__':
    try:
        TrafficClassification()
    except rospy.ROSInterruptException:
        pass
