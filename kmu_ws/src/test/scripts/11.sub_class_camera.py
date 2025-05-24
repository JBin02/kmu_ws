#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2


class Sub_class:
    def __init__(self):
        rospy.init_node("wego_sub_node")  # 1. node의 이름 설정
        sub = rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.camera_cb)  # 2. node의 역할 설정
        self.bridge = CvBridge()

    def camera_cb(self, msg):  # 3. callback 실행
        # print(msg)
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg)
        cv2.imshow("cv_img", cv_img)
        cv2.waitKey(1)


def main():
    sub_class = Sub_class()
    rospy.spin()


if __name__ == "__main__":
    main()
