#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from math import *
from time import *


class Sub_class:
    def __init__(self):
        rospy.init_node("wego_sub_node")  # 1. node의 이름 설정
        sub = rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.camera_cb)  # 2. node의 역할 설정
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)  # 2. node의 역할 설정
        self.bridge = CvBridge()
        self.pub_msg = Twist()  # 메세지 설정
        self.window_split_x = 0
        self.color_lower = np.array([0, 0, 0])
        self.color_upper = np.array([179, 50, 100])
        self.any_lane_flag = False
        self.any_lane_flag_previous = False
        self.start_time_flag = False
        self.start_time = 0

    def camera_cb(self, msg):  # 3. callback 실행
        # print(msg)
        cv_img = self.bridge.compressed_imgmsg_to_cv2(msg)
        y, x, channel = cv_img.shape
        if self.window_split_x == 0:
            self.window_split_x = x // 2
        hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv_img)

        color_mask = cv2.inRange(hsv_img, self.color_lower, self.color_upper)
        color_bit_img = cv2.bitwise_and(cv_img, cv_img, mask=color_mask)
        combined_img = cv2.bitwise_or(color_bit_img, color_bit_img)
        margin_x1 = 0
        margin_x2 = 190
        margin_y = 330
        src1 = (margin_x1, y)
        src2 = (margin_x2, margin_y)
        src3 = (x - margin_x2, margin_y)
        src4 = (x - margin_x1, y)
        srcs = np.float32([src1, src2, src3, src4])
        dst1 = (80, y)
        dst2 = (80, 0)
        dst3 = (x - 80, 0)
        dst4 = (x - 80, y)
        dsts = np.float32([dst1, dst2, dst3, dst4])
        matrix = cv2.getPerspectiveTransform(srcs, dsts)
        warp_img = cv2.warpPerspective(combined_img, matrix, [x, y])
        gray_img = cv2.cvtColor(warp_img, cv2.COLOR_BGR2GRAY)
        bin_img = np.zeros_like(gray_img)
        bin_img[gray_img != 0] = 1
        bin_img = bin_img[400:y, :]

        histogram = np.sum(bin_img, axis=0)
        histogram[histogram < 10] = 0
        left_lane = histogram[0 : self.window_split_x]
        right_lane = histogram[self.window_split_x : x]
        try:
            left_indices = np.nonzero(left_lane)[0]
            left_lane_avg = (left_indices[0] + left_indices[-1]) // 2
            left_flag = True
        except:
            left_flag = False
        try:
            right_indices = np.nonzero(right_lane)[0]
            right_lane_avg = (right_indices[0] + right_indices[-1]) // 2
            right_lane_avg = right_lane_avg + self.window_split_x
            right_flag = True

        except:
            right_flag = False

        self.any_lane_flag = False
        if left_flag == True and right_flag == True:
            index_avg = (left_lane_avg + right_lane_avg) // 2
            goal_index = 320
            goal_diff = goal_index - index_avg
            self.window_split_x = x // 2
            print(f"both lane:{goal_diff}")
        else:
            if left_flag == True:
                index_avg = left_lane_avg
                goal_index = 100
                goal_diff = goal_index - left_lane_avg
                self.window_split_x = x * 2 // 3
                print(f"left only:{goal_diff}")
            elif right_flag == True:
                index_avg = right_lane_avg
                goal_index = 540
                goal_diff = goal_index - right_lane_avg
                self.window_split_x = x // 3
                print(f"right only:{goal_diff}")
            else:
                index_avg = 320
                goal_diff = 0
                print(f"any lane:{goal_diff}")
                self.window_split_x = x // 2
                self.any_lane_flag = True

        self.pub_msg.angular.z = goal_diff * pi / x
        self.pub_msg.linear.x = 0.2
        self.pub.publish(self.pub_msg)
        cv2.line(cv_img, [320, 0], [320, y], [0, 255, 0], 5)
        cv2.line(cv_img, [index_avg, 0], [index_avg, y], [0, 255, 255], 5)
        print(index_avg)
        cv2.circle(warp_img, [index_avg, 440], 10, [0, 0, 255], 20)
        cv2.imshow("cv_img", cv_img)
        cv2.imshow("combined_img", combined_img)
        cv2.imshow("warp_img", warp_img)
        cv2.waitKey(1)

        if self.any_lane_flag == True and self.any_lane_flag_previous == True:
            if self.start_time_flag == False:
                self.start_time = time()
                self.start_time_flag = True
        else:
            self.start_time = time()
            self.start_time_flag = False

        self.any_lane_flag_previous = self.any_lane_flag
        if self.any_lane_flag == True:
            end_time = time()
            diff_time = end_time - self.start_time
            print(diff_time)

            if diff_time > 3:
                self.color_lower = np.array([10, 40, 40])
                self.color_upper = np.array([50, 255, 255])


def main():
    sub_class = Sub_class()
    rospy.spin()


if __name__ == "__main__":
    main()
