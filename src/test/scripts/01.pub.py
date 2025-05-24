#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy
from std_msgs.msg import Int32

rospy.init_node("wego_pub_node")  # 1. node의 이름 설정
pub = rospy.Publisher("/counter", Int32, queue_size=1)  # 2. node의 역할 설정
pub_msg = Int32()
pub_msg.data = 0
rate = rospy.Rate(10)  # 4. 주기 설정
while not rospy.is_shutdown():
    pub_msg.data += 1
    pub.publish(pub_msg)  # 3. publish 실행
    rate.sleep()  # 5. 주기 실행
