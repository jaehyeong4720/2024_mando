#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from std_msgs.msg import Float64, Bool, Int32, String
from shapely.geometry import Point, Polygon
from sensor_msgs.msg import Joy

class main_mando:
    def __init__(self):
        rospy.init_node('main_24mando', anonymous=True)
        rospy.loginfo("Main Node initialized")
        
        ######[파라미터 부분]######
        self.judgment_radius = 1 # 허용반경(m)
        self.traffic_judgment_radius = 5
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
        # rospy.Subscriber('/stop_done', Bool, self.stop_done_callback)
        rospy.Subscriber('/csv_n_done', Int32, self.csv_n_done_callback)

        # 발행 파트 #
        self.pub_joy_toggle = rospy.Publisher('main_cmd_joy', Bool, queue_size=10)
        self.pub_way_toggle = rospy.Publisher('main_cmd_way', Bool, queue_size=10)
        self.way_num_pub = rospy.Publisher('main_cmd_way_num', Int32, queue_size=10)
        
        self.pub_T_toggle = rospy.Publisher('main_cmd_T_parking', Bool, queue_size=10) #직각 주차    
        self.pub_P_toggle = rospy.Publisher('main_cmd_P_parking', Bool, queue_size=10) #평행 주차    
        self.pub_S_toggle = rospy.Publisher('main_cmd_stop', Bool, queue_size=10)
        # self.pub_obstacle_toggle = rospy.Publisher('main_cmd_obstacle', Bool, queue_size=10)    
        # self.pub_stop_line_toggle = rospy.Publisher('main_cmd_stop_line', Bool, queue_size=10)

        self.flag_point_list = [(332218.8575,4128619.114), #white_line 
                                (332222.0065,4128595.297), #traffic_1
                                (332208.6095,4128583.341), #traffic_2
                                (332218.2458,4128554.912), #T_start
                                (332214.064,4128568.38), #traffic_3
                                (332237.0416,4128576.316)] #P_start
                                
        
        self.obs_bp  = [(332232.4255, 4128612.468),
                        (332233.8335, 4128615.459),
                        (332256.1504, 4128607.209),
                        (332254.9271, 4128604.137)] #obs_boundery
        
        self.white_bp = [(332221.9564,4128619.7926),
                        (332220.4685,4128616.792),
                        (332225.4781,4128614.96),
                        (332226.8069,4128617.9861)] #white_boundery
        
        self.traffic_1_bp = [(332219.6472,4128595.2636),
                            (332217.528,4128589.9669),
                            (332220.966,4128588.488),
                            (332223.5817,4128593.331)] #traffic_1_bp
        
        self.traffic_2_bp = [(332206.1543,4128586.221),
                            (332204.5433,4128583.2784),
                            (332209.1821,4128581.0876),
                            (332210.6419,4128584.455)] #traffic_2_bp
        
        self.traffic_3_bp = [(332215.1073,4128574.487),
                            (332213.3562,4128570.304),
                            (332216.4887,4128569.0644),
                            (332218.3794,4128573.1335)] #traffic_3_bp
        
        self.t_start_bp = [(332216.5785,4128556.7194),
                            (332214.8687,4128552.3803),
                            (332220.2723,4128550.296),
                            (332222.0547,4128554.268)] #t_start_bp


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
        

        self.prev_button_states = [0] * 12

        rospy.Timer(rospy.Duration(0.05), self.drive_main)

    def joy_callback(self, data):
        joy_btn = data.buttons
        
        if joy_btn[4] == 1 and self.prev_button_states[4] == 0:
            self.auto = not self.auto  # self.auto 값을 토글
        self.prev_button_states = joy_btn[:]

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
    
    def geofence (self,num): #geofence _ 생성 및 범위내부 판별코드
        if num == 1:
            self.boundery_points = self.obs_bp
        elif num == 2:
            self.boundery_points = self.white_bp
        elif num == 3:
            self.boundery_points = self.traffic_1_bp
        elif num == 4:
            self.boundery_points = self.traffic_2_bp
        elif num == 5:
            self.boundery_points = self.t_start_bp
        elif num == 6:
            self.boundery_points = self.traffic_3_bp
        boundery = Polygon(self.boundery_points)
        self_point = Point(self.my_pose_utm_x, self.my_pose_utm_y)
        inside_flag = boundery.contains(self_point)
        return inside_flag

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
        # #real_way
        if self.drive_mode_flag == "Joy":
            self.misson_mode_flag = "None"

        elif self.geofence(1): #장애물 감지 범위:
            print("obs ready")
            if self.obstacle_detected:
                self.misson_mode_flag = "Stop"
            else:
                self.misson_mode_flag = "None"

        elif self.geofence(2): #정지선 감지 범위
            print("line ready")
            if self.stop_line_detected:
                self.misson_mode_flag = "Stop"
            else:
                self.misson_mode_flag = "None"
        
        elif self.geofence(3): #1차 신호등 감지 범위
            print("red_1 ready")
            if self.red_detected:
                self.misson_mode_flag = "Stop"
            else:
                self.misson_mode_flag = "None"

        elif self.geofence(4): #2차 신호등 감지 범위
            print("red_2 ready")
            if self.red_detected:
                self.misson_mode_flag = "Stop"
            else:
                self.misson_mode_flag = "None"

        elif self.geofence(5): #T주차 시작 범위
            print("t_park_ready")
            self.misson_mode_flag = "T_parking"
        elif self.TP_done:
            self.way_num(2)
            self.misson_mode_flag = "None"
            self.TP_done = False

        elif self.geofence(6): #3차 신호등 감지 범위
            print("red_3 ready")
            if self.red_detected:
                self.misson_mode_flag = "Stop"
            else:
                self.misson_mode_flag = "None"
        
        elif self.distance_from_flag_point(5) <= self.judgment_radius**2: #P주차 시작 범위
            print("p_park_ready")
            self.misson_mode_flag = "P_parking"
        elif self.PP_done:
            self.way_num(3)
            self.misson_mode_flag = "None"
            self.PP_done = False


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