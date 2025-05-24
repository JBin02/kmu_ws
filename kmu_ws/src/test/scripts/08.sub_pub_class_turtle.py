#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class Sub_class:
    def __init__(self):
        rospy.init_node("wego_sub_node")  # 1. node의 이름 설정
        sub = rospy.Subscriber("/turtle1/pose", Pose, self.int_cb)  # 2. node의 역할 설정
        self.pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)  # 2. node의 역할 설정
        self.pose_msg = Pose()
        self.cmd_msg = Twist()
        # self.pose_msg.y

    def int_cb(self, msg):  # 3. callback 실행
        if msg.x < 8:
            self.cmd_msg.linear.x = 0.5
        else:
            self.cmd_msg.linear.x = 0.0
        self.pub.publish(self.cmd_msg)


def main():
    sub_class = Sub_class()
    rospy.spin()


if __name__ == "__main__":
    main()
