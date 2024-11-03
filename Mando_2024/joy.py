#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

#!!! 주의
#2팀은 /cmd_vel_steer 로 변경필요!!

class joy_drive:
    def __init__(self):
        rospy.init_node('joy_drive')

        rospy.Subscriber("/joy", Joy, self.joy_callback)
        rospy.Subscriber("/main_cmd_joy", Bool, self.cmd_joy_callback)

        self.drive_pub = rospy.Publisher("cmd_vel_steer", Twist, queue_size=1)

        self.joy_toggle = True
        self.manual = 0
        self.speed = 0
        self.steering = 0
        

        rospy.Timer(rospy.Duration(0.05), self.drive_control)

    def cmd_joy_callback(self, data):
        self.joy_toggle = data.data 

    def joy_callback(self, data):
        self.joy_btn = data.buttons
        self.joy_axes = data.axes
        self.auto = self.joy_btn[4]
        self.manual = self.joy_btn[5]
    
    def drive_control(self, event):
        drive = Twist()

        if self.joy_toggle:
            if self.manual == 1:
                self.speed = self.joy_axes[1]
                self.steering = self.joy_axes[2]
                drive.linear.x = self.speed * 1.2
                drive.angular.z = self.steering * 20.0
            else:
                drive.linear.x = 0
                drive.angular.z = 0
            self.drive_pub.publish(drive)


if __name__ == '__main__':
    try:
        joy_drive()
        rospy.spin()
    except KeyboardInterrupt:
        print("program down")