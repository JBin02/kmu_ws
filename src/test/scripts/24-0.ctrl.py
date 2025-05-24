#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist
from math import *
from time import *


class Pub_class:
    def __init__(self):
        rospy.init_node("ctrl_node")  # 1. node의 이름 설정
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)  # 2. node의 역할 설정
        rospy.Subscriber("/e_stop", Bool, self.e_stop_cb)  # 2. node의 역할 설정
        rospy.Subscriber("/steer", Float32, self.steer_cb)  # 2. node의 역할 설정
        self.rate = rospy.Rate(10)  # 주기 설정
        self.pub_msg = Twist()  # 메세지 설정
        self.e_stop_msg = Bool()  # 메세지 설정
        self.steer_msg = Float32()
        self.total_time = 0
        self.degrees = []
        self.e_stop_msg_flag = False
        self.steer_msg_flag = False

    def run(self):
        if self.e_stop_msg_flag and self.steer_msg_flag:
            if self.e_stop_msg.data == True:
                self.pub_msg.linear.x = 0.0
                self.pub_msg.angular.z = 0.0
                print("e_stop")
            else:
                self.pub_msg.linear.x = 0.2
                self.pub_msg.angular.z = self.steer_msg.data
                print("lane detect")
        else:
            self.pub_msg.linear.x = 0.0
            self.pub_msg.angular.z = 0.2
            print("any msg")

        self.pub.publish(self.pub_msg)  # 3. publish 실행
        self.rate.sleep()  # 5. 주기 실행

    def e_stop_cb(self, msg):
        self.e_stop_msg = msg
        self.e_stop_msg_flag = True

    def steer_cb(self, msg):
        self.steer_msg = msg
        self.steer_msg_flag = True


def main():
    pub_class = Pub_class()
    while not rospy.is_shutdown():
        pub_class.run()


if __name__ == "__main__":
    main()
