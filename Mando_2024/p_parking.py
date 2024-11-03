#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import cv2
import numpy as np
import rospy
import math
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class P_park_mando:
    def __init__(self):
        rospy.init_node('P_park_control', anonymous=True)
        rospy.loginfo("P_park Node initialized")

        ######[파라미터 부분]######
        self.back_vel = -0.3  # 후진 속도 (m/s)
        self.back_steer = 30  # 후진 각도 (도)
        self.forward_vel = 0.2  # 전진 속도 (m/s)
        self.flag = 0
        ##########################

        # 구독파트 #
        rospy.Subscriber("/main_cmd_P_parking", Bool, self.parking_callback)

        # 발행파트 #
        self.drive_pub = rospy.Publisher("cmd_vel_steer", Twist, queue_size=1)
        self.PP_done_pub = rospy.Publisher("PP_done", Bool, queue_size=1)

        self.P_back_flag = 0
        
        # 현재 헤딩 및 주차 모드 상태 초기화
        self.parking_mode = False  # 기본값으로 False로 설정
        self.adjust_heading_and_park()

    def parking_callback(self, data):
        self.parking_mode = data.data

    def adjust_heading_and_park(self):
        print("p_park start")

        rate = rospy.Rate(10)  # 초당 10회 발행

        while not rospy.is_shutdown():  # ROS 노드가 종료되지 않았을 때 반복
            if self.parking_mode:  # 주차 명령이 true일 때만 후진
                current_time = rospy.Time.now().to_sec()  # ROS 시간을 사용

                if self.P_back_flag == 0:  # 후진 시작 시 타이머 초기화
                    start_time = rospy.Time.now().to_sec()
                    self.P_back_flag = 1

                if current_time - start_time < 3.0:
                    drive_1 = Twist()
                    drive_1.linear.x = 0.0
                    drive_1.angular.z = 0.0
                    self.drive_pub.publish(drive_1)
                
                # 10초간 스티어링을 유지하며 후진
                elif 3.0 <= current_time - start_time < 14.0:
                    drive_2 = Twist()
                    drive_2.linear.x = -0.3
                    drive_2.angular.z = -20.0
                    self.drive_pub.publish(drive_2)

                
                elif 14.0 <= current_time - start_time < 26.0:
                    drive_3 = Twist()
                    drive_3.linear.x = -0.3
                    drive_3.angular.z = 20.0
                    self.drive_pub.publish(drive_3)

                elif 26.0 <= current_time - start_time < 32.0:
                    drive_4 = Twist()
                    drive_4.linear.x = -0.3
                    drive_4.angular.z = 0
                    self.drive_pub.publish(drive_4)
                
                #주차 주행 완료, 주차완료 노드 발행f
                elif current_time - start_time >= 32.0 and self.flag == 0:
                    rospy.loginfo("Ppark_done, PP_done_pub")
                    self.PP_done_pub.publish(True)
                    self.flag = 1
                else:
                    pass
                rate.sleep()
            else:
                print("p_park wait")
                rate.sleep()

if __name__ == '__main__':
    try:
        park_controller = P_park_mando()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("!! Ppark Program terminated !!")
