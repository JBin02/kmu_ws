#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from math import *
from time import *
from tf.transformations import euler_from_quaternion
import os


class Pub_class:
    def __init__(self):
        rospy.init_node("wego_pub_node")  # 1. node의 이름 설정
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)  # 2. node의 역할 설정
        sub = rospy.Subscriber("/imu", Imu, self.imu_cb)  # 2. node의 역할 설정
        self.rate = rospy.Rate(10)  # 주기 설정
        self.pub_msg = Twist()  # 메세지 설정
        self.imu_msg = Imu()  # 메세지 설정
        self.imu_msg
        self.imu_flag = False
        self.total_time = 0
        self.goal_z = [45, 135, -135, -45]
        self.n = 0

    def run(self):
        if self.imu_flag == True:
            x = self.imu_msg.orientation.x
            y = self.imu_msg.orientation.y
            z = self.imu_msg.orientation.z
            w = self.imu_msg.orientation.w
            ang_x, ang_y, ang_z = euler_from_quaternion([x, y, z, w])
            # print(ang_x * 180 / pi)
            # print(ang_y * 180 / pi)
            degree_z = ang_z * 180 / pi
            diff = (self.goal_z[self.n] - degree_z + 180) % 360 - 180
            print(self.goal_z[self.n])
            # print(diff)
            # print(self.pub_msg.angular.z)
            if abs(diff) < 3:
                self.n = (self.n + 1) % len(self.goal_z)
            self.pub_msg.angular.z = diff * pi / 180
            self.pub_msg.angular.z = self.pub_msg.angular.z
            self.pub.publish(self.pub_msg)
            self.rate.sleep()

    def imu_cb(self, msg):
        self.imu_msg = msg
        self.imu_flag = True


def main():
    pub_class = Pub_class()
    while not rospy.is_shutdown():
        pub_class.run()


if __name__ == "__main__":
    main()
