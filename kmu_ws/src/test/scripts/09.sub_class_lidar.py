#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy
from sensor_msgs.msg import LaserScan


class Sub_class:
    def __init__(self):
        rospy.init_node("wego_sub_node")  # 1. node의 이름 설정
        sub = rospy.Subscriber("/scan", LaserScan, self.int_cb)  # 2. node의 역할 설정

    def int_cb(self, msg):  # 3. callback 실행
        print(msg)


def main():
    sub_class = Sub_class()
    rospy.spin()


if __name__ == "__main__":
    main()
