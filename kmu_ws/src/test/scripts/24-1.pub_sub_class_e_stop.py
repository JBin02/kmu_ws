#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist  # 추가
from math import *
from time import *


class Pub_class:
    def __init__(self):
        rospy.init_node("e_stop_node")  # 1. node의 이름 설정
        self.pub = rospy.Publisher("/e_stop", Bool, queue_size=1)  # 2. node의 역할 설정
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)  # 스피드 명령 추가
        sub = rospy.Subscriber("/scan", LaserScan, self.lidar_cb)  # 2. node의 역할 설정
        self.rate = rospy.Rate(10)  # 주기 설정
        self.pub_msg = Bool()  # 메세지 설정
        self.twist_msg = Twist()  # 스피드 메시지 추가
        self.lidar_msg = LaserScan()  # 메세지 설정
        self.lidar_flag = False
        self.total_time = 0
        self.degrees = []

    def lidar_cb(self, msg):
        degree_min = msg.angle_min * 180 / pi
        degree_increment = msg.angle_increment * 180 / pi

        if self.degrees == []:
            self.degrees = [degree_min + degree_increment * index for index, value in enumerate(msg.ranges)]
        obstacle = 0
        for index, value in enumerate(msg.ranges):
            if abs(self.degrees[index]) < 30 and 0 < value < 0.5:
                print(f"장애물:{self.degrees[index]}")
                obstacle += 1
            else:
                pass

        if obstacle > 0:
            self.pub_msg.data = True
            # 로봇 정지
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = 0.0
            print("로봇 정지!")
        else:
            self.pub_msg.data = False
            # 로봇 전진
            self.twist_msg.linear.x = 0.5  # 전진 속도 (m/s)
            self.twist_msg.angular.z = 0.0  # 회전 속도
            print("전진 중...")

        self.pub.publish(self.pub_msg)
        self.cmd_pub.publish(self.twist_msg)  # 스피드 명령 송신


def main():
    pub_class = Pub_class()
    rospy.spin()


if __name__ == "__main__":
    main()