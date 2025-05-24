#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy
from std_msgs.msg import Int32

rospy.init_node("wego_sub_node")  # 1. node의 이름 설정


def int_cb(msg):  # 3. callback 실행
    print(msg)


sub = rospy.Subscriber("/counter", Int32, callback=int_cb)  # 2. node의 역할 설정

rospy.spin()
