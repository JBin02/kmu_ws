#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float32
from math import *


class Sub_class:
    def __init__(self):
        rospy.init_node("steer_node")  # 1. node의 이름 설정
        sub = rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.camera_cb)  # 2. node의 역할 설정
        self.pub = rospy.Publisher("/steer", Float32, queue_size=1)  # 2. node의 역할 설정
        self.bridge = CvBridge()
        self.pub_msg = Float32()  # 메세지 설정
        self.window_split_x = 0

    def camera_cb(self, msg):  # 3. callback 실행
        # print(msg)
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg)
        y, x, channel = cv_img.shape
        hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        yellow_lower = np.array([10, 80, 40])
        yellow_upper = np.array([50, 255, 255])
        yellow_mask = cv2.inRange(hsv_img, yellow_lower, yellow_upper)
        yellow_img = cv2.bitwise_and(cv_img, cv_img, mask=yellow_mask)
        croped_img = yellow_mask[420:y, :]
        bin_img = np.zeros_like(croped_img)
        bin_img[croped_img != 0] = 1
        histogram = np.sum(bin_img, axis=0)
        histogram[histogram < 10] = 0
        indices = np.nonzero(histogram)[0]
        goal_index = 80
        try:
            avg_index = (indices[0] + indices[-1]) // 2
        except:
            avg_index = 0
        steer = (goal_index - avg_index) * pi / x

        cv2.circle(cv_img, (avg_index, 450), 5, [0, 0, 255], -1)
        self.pub_msg.data = steer
        self.pub.publish(self.pub_msg)
        cv2.imshow("cv_img", cv_img)
        cv2.waitKey(1)


def main():
    sub_class = Sub_class()
    rospy.spin()


if __name__ == "__main__":
    main()
