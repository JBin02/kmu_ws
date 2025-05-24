#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy
from geometry_msgs.msg import Twist


class Pub_class:
    def __init__(self):
        rospy.init_node("wego_pub_node")  # 1. node의 이름 설정
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)  # 2. node의 역할 설정
        self.rate = rospy.Rate(10)  # 주기 설정
        self.pub_msg = Twist()  # 메세지 설정

    def run(self):
        while not rospy.is_shutdown():
            self.pub_msg.linear.x = -2.0
            self.pub.publish(self.pub_msg)  # 3. publish 실행
            self.rate.sleep()  # 5. 주기 실행


def main():
    pub_class = Pub_class()
    pub_class.run()


if __name__ == "__main__":
    main()
