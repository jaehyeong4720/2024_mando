#!/usr/bin/env python
# -*- coding:utf-8 -*-  
import rospy
import math
import pandas as pd
from std_msgs.msg import Float64, Bool, Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy  # 조이스틱 입력을 받기 위해 추가

class way_mando:
    def __init__(self):
        rospy.init_node('control', anonymous=True)
        rospy.loginfo("Way Node initialized")
        
        ######[파라미터 부분]######
        file_path = '/home/itm1103/kace_2_team/예선테스트용/예선경로.csv'  # 주행 경로 csv파일 위치
        self.offset_dis = 1.09  # base_pos 기준 앞뒤로 offset
        self.Kp = 0.35  # kp 값
        self.two_way = True
        self.i_range = 1000
        ##########################

        # 변수 초기화 #
        self.auto_vel = 0
        self.yaw_raw = 0  # gps_mbc 기준 자신의 헤딩값 (N 0도, 시계반대방향 0~360)
        self.yaw = 0  # utm_pos 기준 자신의 헤딩값 (E 0도, 시계반대방향 0~360)
        self.utm_x_raw = 0  # base_pos utm 좌표_x
        self.utm_y_raw = 0  # base_pos utm 좌표_y
        self.utm_x = 0.0  # offset한 자신의 utm 좌표_x
        self.utm_y = 0.0  # offset한 자신의 utm 좌표_y
        self.way_num = 1  # 기본값으로 1을 사용
        self.main_cmd_way = False  # 주행 활성화 여부
        self.num = 1

        # 조이스틱 버튼 상태 초기화 (디바운싱을 위해) #
        self.prev_button_states = [0] * 12  # 버튼 개수에 맞게 조절

        # 경로 csv 파일 파싱 #
        self.reading_csv = pd.read_csv(file_path)

        # 토픽 구독 #
        # rospy.Subscriber("/main_cmd_way", Bool, self.main_cmd_way_callback)  # 기존 main_cmd_way 구독 제거
        rospy.Subscriber("/gps_heading_topic", Float64, self.heading_callback)  # 헤딩값 토픽
        rospy.Subscriber("/utm_x_topic", Float64, self.utm_x_callback)  # utm_x 좌표값
        rospy.Subscriber("/utm_y_topic", Float64, self.utm_y_callback)  # utm_y 좌표값

        # 조이스틱 토픽 추가 구독 #
        rospy.Subscriber("/joy", Joy, self.joy_callback)  # 조이스틱 입력을 받는 토픽 추가

        # 토픽 발행 #
        self.drive_pub = rospy.Publisher("cmd_vel_steer", Twist, queue_size=1)
        self.csv_done_pub = rospy.Publisher("csv_n_done", Int32, queue_size=1)

        rospy.Timer(rospy.Duration(0.05), self.drive_way)

    def joy_callback(self, data):  # 조이스틱 버튼 입력 처리
        self.joy_btn = list(data.buttons)
        #print("joy")  # 리스트로 변환

        # 조이스틱 버튼에 따라 원하는 웨이포인트로 설정
        if self.joy_btn[0]:  # 직각
            self.way_num = 1
            print(self.way_num)
        elif self.joy_btn[1]:  # S자
            self.way_num = 2
            print(self.way_num)
        elif self.joy_btn[2]:  # 직각_역주행
            self.way_num = 3
            print(self.way_num)
        elif self.joy_btn[3]:  # S자_역주행
            self.way_num = 4
            print(self.way_num)

        # 주행 시작/정지 토글 버튼 처리 (예: 버튼 4를 토글 버튼으로 사용)
        if self.joy_btn[4] and not self.prev_button_states[4]:
            self.main_cmd_way = not self.main_cmd_way  # 토글
            rospy.loginfo("Main Command Way toggled to {} format.{self.main_cmd_way}")

        # 이전 버튼 상태 업데이트
        self.prev_button_states = self.joy_btn[:]  # 리스트 복사


    def csv_to_list(self, num):
        lx, ly, lvel = [], [], []
        if num == 1:
            lx = self.reading_csv.iloc[:, 0].tolist()
            ly = self.reading_csv.iloc[:, 1].tolist()
            lvel = self.reading_csv.iloc[:, 2].tolist()
        elif num == 2:
            lx = self.reading_csv.iloc[:, 3].tolist()
            ly = self.reading_csv.iloc[:, 4].tolist()
            lvel = self.reading_csv.iloc[:, 5].tolist()
        elif num == 3:
            lx = self.reading_csv.iloc[:, 6].tolist()
            ly = self.reading_csv.iloc[:, 7].tolist()
            lvel = self.reading_csv.iloc[:, 8].tolist()
        elif num == 4:
            lx = self.reading_csv.iloc[:, 9].tolist()
            ly = self.reading_csv.iloc[:, 10].tolist()
            lvel = self.reading_csv.iloc[:, 11].tolist()
        elif num == 5:
            lx = self.reading_csv.iloc[:, 12].tolist()
            ly = self.reading_csv.iloc[:, 13].tolist()
            lvel = self.reading_csv.iloc[:, 14].tolist()
        elif num == 6:
            lx = self.reading_csv.iloc[:, 15].tolist()
            ly = self.reading_csv.iloc[:, 16].tolist()
            lvel = self.reading_csv.iloc[:, 17].tolist()
        else:
            lx = self.reading_csv.iloc[:, 0].tolist()
            ly = self.reading_csv.iloc[:, 1].tolist()
            lvel = self.reading_csv.iloc[:, 2].tolist()

        return lx, ly, lvel

    def utm_x_callback(self, data):  # utm_x 콜백함수
        self.utm_x_raw = data.data
        self.utm_x = self.utm_x_raw + self.offset_dis * math.cos(self.yaw * math.pi/180)  # base_offset_x

    def utm_y_callback(self, data):  # utm_y 콜백함수
        self.utm_y_raw = data.data
        self.utm_y = self.utm_y_raw + self.offset_dis * math.sin(self.yaw * math.pi/180)  # base_offset_y
        
    def heading_callback(self, data):  # 각도좌표계 변환
        self.yaw_raw = data.data
        if self.two_way:
            if self.yaw_raw > 0 and self.yaw_raw < 90:  # ublox 2way 사용 시
                self.yaw = 90 - self.yaw_raw
            else:
                self.yaw = 450 - self.yaw_raw
        else:
            self.yaw = self.yaw_raw

    def stanley_control_angle(self):  # 스탠리 제어 계산
        slx, sly, slvel = self.csv_to_list(self.way_num)

        if not hasattr(self, 'prev_way_num') or self.prev_way_num != self.way_num:
            self.target_i = 0
            self.prev_way_num = self.way_num

            min_dist = float('inf')
            for i in range(len(slx)):
                dx = slx[i] - self.utm_x
                dy = sly[i] - self.utm_y
                dist = math.sqrt(dx ** 2 + dy ** 2)

                if dist < min_dist:
                    min_dist = dist
                    self.target_i = i
        else:
            min_dist = float('inf')
            start_i = max(0, self.target_i - self.i_range)
            end_i = min(len(slx) - 1, self.target_i + self.i_range)

            for i in range(start_i, end_i + 1):
                dx = slx[i] - self.utm_x
                dy = sly[i] - self.utm_y
                dist = math.sqrt(dx ** 2 + dy ** 2)

                if dist < min_dist:
                    min_dist = dist
                    self.target_i = i

        if self.target_i + 1 >= len(slx):
            print("finish, idx= ", self.target_i)
            stop_drive = Twist()
            stop_drive.linear.x = 0
            stop_drive.angular.z = 0
            finish_num = self.way_num 
            self.csv_done_pub.publish(finish_num)
            return stop_drive
        else:
            map_yaw = math.atan2(sly[self.target_i + 1] - sly[self.target_i],
                                slx[self.target_i + 1] - slx[self.target_i]) * 180 / math.pi

            if map_yaw < 0:
                map_yaw += 360

            cte = map_yaw - self.yaw
            if cte < -180:
                cte += 360
            elif cte > 180:
                cte -= 360

            if min_dist < 0.05:
                min_dist = 0

            flag = ((self.utm_x - slx[self.target_i]) * -math.sin(map_yaw * (math.pi / 180))) + \
                ((self.utm_y - sly[self.target_i]) * math.cos(map_yaw * (math.pi / 180)))

            if flag >= 0:
                min_dist = -min_dist

            add_angle = math.atan2(self.Kp * min_dist, slvel[self.target_i]) * 180 / math.pi

            stanley_steer_angle = cte + add_angle
            way_vel = slvel[self.target_i]

            print("===========================================")
            print('way_angle: ', map_yaw)
            print('car_angle: ', self.yaw)
            print('cte angle: ', cte)
            print('add_angle: ', add_angle)
            print('target_dis: ', min_dist)
            print('target_i: ', self.target_i)
            print("stanley_angle: ", stanley_steer_angle)
            print("csv_num", self.way_num)
            print("way_vel: ", way_vel)

            auto_drive = Twist()
            auto_drive.linear.x = way_vel
            auto_drive.angular.z = float(stanley_steer_angle)
            drive_num = self.way_num
            self.csv_done_pub.publish(drive_num)
            return auto_drive

    def drive_way(self, event):
        if self.main_cmd_way:  # 주행 활성화 여부 확인
            drive = self.stanley_control_angle()
            self.drive_pub.publish(drive)
        else:
            # 주행이 비활성화된 경우 차량을 정지시킴
            stop_drive = Twist()
            stop_drive.linear.x = 0
            stop_drive.angular.z = 0
            self.drive_pub.publish(stop_drive)

if __name__ == '__main__':
    try:
        way_controller = way_mando()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("!!Way Program terminated!!")
