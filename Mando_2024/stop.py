#!/usr/bin/env python3
# -*- coding:utf-8 -*-


import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class StopController:
    def __init__(self):
        rospy.init_node('Stop_controller')

        # 정지 토픽을 발행할 Publisher 설정
        self.drive_pub = rospy.Publisher('/cmd_vel_steer', Twist, queue_size=10)
        # 정지 명령 상태 변수 초기화
        self.stop_command = False
        # self.obs_command = False
        # self.line_command = False
        # 구독자 설정
        rospy.Subscriber('/main_cmd_stop', Bool, self.stop_callback)
        # rospy.Subscriber('/main_cmd_obstacle', Bool, self.obs_callback)
        # rospy.Subscriber('/main_cmd_stop_line', Bool, self.line_callback)
        # 발행자 설정
        # self.stop_done_pub = rospy.Publisher('/stop_done', Bool, queue_size=10)
        # 타이머로 제어 반복문 실행
        rospy.Timer(rospy.Duration(0.05), self.control_loop)

    def stop_callback(self, data):
        self.stop_command = data.data

    # def obs_callback(self, data):
    #     """장애물 감지 콜백"""
    #     self.obs_command = data.data

    # def line_callback(self, data):
    #     """흰색 선 감지 콜백"""
    #     self.line_command = data.data

    def control_loop(self, event):
        """정지 제어 반복문"""
        drive = Twist()
        
        # 빨간불, 장애물, 흰색 선 중 하나라도 감지되면 정지
        if self.stop_command:
            drive.linear.x = 0.0
            drive.angular.z = 0.0
            self.drive_pub.publish(drive)
            rospy.loginfo("정지 명령 실행")

        else:
            rospy.loginfo("주행 중")
        #     self.stop_done_pub.publish(False)  # 주행 상태일 때 초기화

        # 드라이브 명령을 발행
        

    # def publish_stop_done(self, event):
    #     """5초 후 Stop_done 발행"""
    #     self.stop_done_pub.publish(True)

if __name__ == '__main__':
    try:
        StopController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

