#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
from std_msgs.msg import Float64, Bool, Int32, String
from sensor_msgs.msg import Joy


class main_mando:
    def __init__(self):
        rospy.init_node('main_24mando', anonymous=True)
        rospy.loginfo("Main Node initialized")
        
        ######[파라미터 부분]######
        self.judgment_radius = 1.1 # 허용반경(m)
        ##########################

        # 구독 파트 #
        # stopline 추가해야함
        rospy.Subscriber("/joy", Joy, self.joy_callback)
        rospy.Subscriber('/utm_x_topic', Float64, self.utm_x_callback)
        rospy.Subscriber('/utm_y_topic', Float64, self.utm_y_callback)

        rospy.Subscriber('/TP_done', Bool, self.TP_finish_callback) #직각 주차
        rospy.Subscriber('/PP_done', Bool, self.PP_finish_callback) #평행 주차
        rospy.Subscriber('/red_detected', Bool, self.red_callback)
        rospy.Subscriber('/obstacle_detected', Bool, self.obstacle_callback)
        rospy.Subscriber('/stop_line_detected', Bool, self.stop_line_callback)
        rospy.Subscriber('/csv_n_done', Int32, self.csv_n_done_callback)

        # 발행 파트 #
        self.pub_joy_toggle = rospy.Publisher('main_cmd_joy', Bool, queue_size=10)
        self.pub_way_toggle = rospy.Publisher('main_cmd_way', Bool, queue_size=10)
        self.way_num_pub = rospy.Publisher('main_cmd_way_num', Int32, queue_size=10)
        
        self.pub_T_toggle = rospy.Publisher('main_cmd_t_parking', Bool, queue_size=10) #직각 주차    
        self.pub_P_toggle = rospy.Publisher('main_cmd_P_parking', Bool, queue_size=10) #평행 주차    
        self.pub_S_toggle = rospy.Publisher('main_cmd_stop', Bool, queue_size=10)

        # 플래그 변경 리스트 => 나중에 csv 파일로 변경 (시작점, 장애물, T주차시작, 정지선, P주차시작, 신호등(로타리 중심점))
        self.flag_point_list = [(332228.97411792, 4128665.55033392),
                                (332217.754558, 4128553.44357)]
                                	



	

        # 변수 초기화 #
        self.auto = 0
        self.start = 0
        self.my_pose_utm_x = 0
        self.my_pose_utm_y = 0
        self.drive_mode_flag = "Joy"
        self.misson_mode_flag = "None"
        self.csv_num = None
        self.red_detected = False
        self.obstacle_detected = False
        self.TP_done = False #직각 주차 
        self.PP_done = False #평행 주차 
        self.stop_line_detected = False 
        self.csv_n_done = 0

        rospy.Timer(rospy.Duration(0.05), self.drive_main)

    def joy_callback(self, data):
        self.joy_btn = data.buttons
        self.auto = self.joy_btn[4]

    def utm_x_callback(self, data):
        self.my_pose_utm_x = data.data

    def utm_y_callback(self, data):
        self.my_pose_utm_y = data.data

    def TP_finish_callback(self, data):
        self.TP_done = data.data

    def PP_finish_callback(self, data):
        self.PP_done = data.data

    def red_callback(self, data):
        self.red_detected = data.data

    def obstacle_callback(self, data):
        self.obstacle_detected = data.data        

    def stop_line_callback(self, data):
        self.stop_line_detected = data.data 

    def csv_n_done_callback(self, data):
        self.csv_n_done = data.data


    def distance_from_flag_point(self, num):
        # 현재 위치와 각 구간의 point 사이의 거리 제곱 계산
        return (self.flag_point_list[num][0] - self.my_pose_utm_x)**2 + (self.flag_point_list[num][1] - self.my_pose_utm_y)**2
    

    def drive_flag_change(self):
        if  self.auto and self.misson_mode_flag == "None": #주행시작, 최초 실행후, 변경없음
            self.drive_mode_flag = "Way"
        elif self.auto == 0:
            self.drive_mode_flag = "Joy"
        else:
            self.drive_mode_flag = "Misson"

    def drive_toggle_change(self): # AUTO 모드에서의 주행 토글 변경       
        if self.drive_mode_flag == "Joy": #조이 / 정지 주행
            self.pub_joy_toggle.publish(True)   
            self.pub_way_toggle.publish(False)  
        elif self.drive_mode_flag == "Way": 
            self.pub_joy_toggle.publish(False) 
            self.pub_way_toggle.publish(True) 
        elif self.drive_mode_flag == "Misson": 
            self.pub_joy_toggle.publish(False)   
            self.pub_way_toggle.publish(False)                


    # +추가적으로 플래그 포인트의 위치점을 프린트 해주는 부분이 필요함.
    # 시작점과 교차로만 범위 판단이 필요할듯

    def misson_flag_change(self):
        if self.drive_mode_flag == "Joy":
            self.misson_mode_flag = "None"

        elif self.distance_from_flag_point(0) < self.judgment_radius**2: #1번주행 시작점
            self.way_num(1)
            self.misson_mode_flag = "None"    

        elif self.distance_from_flag_point(1) < self.judgment_radius**2: #P주차 시작
            self.misson_mode_flag = "T_parking"

        elif self.TP_done: #3번주행 P주차 끝
            self.way_num(2)
            self.misson_mode_flag = "None"
            self.TP_done = False



    def misson_toggle_change(self): # AUTO 모드에서의 주행 토글 변경  
        self.pub_T_toggle.publish(False)
        self.pub_P_toggle.publish(False)
        self.pub_S_toggle.publish(False)
        if self.misson_mode_flag == "T_parking":   # T 주차
            self.pub_T_toggle.publish(True)         
        elif self.misson_mode_flag == "P_parking":   # 평행 주차
            self.pub_P_toggle.publish(True)
        elif self.misson_mode_flag == "Stop":   # 정지
            self.pub_S_toggle.publish(True)   
        else:
            pass

    def way_num (self, data): #Way 코드의 주행경로 선택 명령
        self.csv_num = data
        self.way_num_pub.publish(self.csv_num)

    def display (self):
        print("==========display==========")
        print("drive_mode_flag : ",self.drive_mode_flag)
        print("misson_mode_flag : ",self.misson_mode_flag)
        print("way_num : ",self.csv_num)

    def drive_main(self, event): 
        self.display()
    
        if  self.auto == 1:
            self.drive_flag_change()
            self.drive_toggle_change()
            self.misson_flag_change()
            self.misson_toggle_change()
        else:
            self.drive_flag_change()
            self.drive_toggle_change()
            self.misson_flag_change()
            self.misson_toggle_change()

if __name__ == '__main__':
    try:
        main_controller = main_mando()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("!! Main Program terminated !!")