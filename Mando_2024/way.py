#!/usr/bin/env python
# -*- coding:utf-8 -*-  
import rospy
import math
import pandas as pd
from std_msgs.msg import Float64, Bool, Int32
from geometry_msgs.msg import Twist

class way_mando:
    def __init__(self):
        rospy.init_node('control', anonymous=True)
        rospy.loginfo("Way Node initialized")
        
        ######[파라미터 부분]######
        file_path = '/home/itm1103/Downloads/3분할.csv' # 주행 경로 csv파일 위치
        self.offset_dis = 1.09 # base_pos기준 앞뒤로 offset
        self.Kp = 0.35 # kp값
        self.two_way = True
        self.i_range = 1000
        ##########################

        # 변수 초기화 #
        self.auto_vel = 0
        self.yaw_raw = 0 # gps_mbc기준 자신의 헤딩값 (N 0도,시계반대방향 0~360)
        self.yaw = 0 # utm_pos기준 자신의 헤딩값 (E 0도,시계반대방향 0~360)
        self.utm_x_raw = 0 # base_pos utm좌표_x
        self.utm_y_raw = 0 # base_pos utm좌표_y
        self.utm_x = 0.0 # offset한 자신의 utm좌표_x
        self.utm_y = 0.0 # offset한 자신의 utm좌표_y
        self.way_num = 1  # 기본값으로 1을 사용
        self.main_cmd_way = False
        self.num = 1

        # 경로 csv파일 파싱 #
        self.reading_csv = pd.read_csv(file_path)

        # 토픽 구독 #
        rospy.Subscriber("/main_cmd_way", Bool, self.main_cmd_way_callback) # 메인함수 명령토픽
        rospy.Subscriber("/main_cmd_way_num", Int32, self.way_num_callback) # 웨이포인트 번호 명령
        rospy.Subscriber("/gps_heading_topic", Float64, self.heading_callback) # 헤딩값 토픽
        rospy.Subscriber("/utm_x_topic", Float64, self.utm_x_callback) # utm_x 좌표값
        rospy.Subscriber("/utm_y_topic", Float64, self.utm_y_callback) # utm_y 좌표값

        # 토픽 발행 #
        self.drive_pub = rospy.Publisher("cmd_vel_steer", Twist, queue_size=1)
        self.csv_done_pub = rospy.Publisher("csv_n_done", Int32, queue_size=1)

        rospy.Timer(rospy.Duration(0.05), self.drive_way)
    
    def main_cmd_way_callback(self, data):  # 메인함수에서 현재함수 스위칭
        self.main_cmd_way = data.data 

    def way_num_callback(self, data):  # 웨이포인트 번호를 받아서 사용하는 함수
        self.way_num = data.data  # 받은 값을 way_num으로 저장

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
        else:
            lx = self.reading_csv.iloc[:, 0].tolist()
            ly = self.reading_csv.iloc[:, 1].tolist()
            lvel = self.reading_csv.iloc[:, 2].tolist()

        return lx, ly, lvel

    def utm_x_callback(self, data): # utm_x 콜백함수
        self.utm_x_raw = data.data
        self.utm_x = self.utm_x_raw + self.offset_dis * math.cos(self.yaw * math.pi/180) # base_offset_x

    def utm_y_callback(self, data): # utm_y 콜백함수
        self.utm_y_raw = data.data
        self.utm_y = self.utm_y_raw + self.offset_dis * math.sin(self.yaw * math.pi/180) # base_offset_y
        
    def heading_callback(self, data): # 각도좌표계 변환, 오도메트리 주행시 토글로 self.two_way false로.
        self.yaw_raw = data.data
        if self.two_way:
            if self.yaw_raw > 0 and self.yaw_raw < 90: #use when ublox 2way
                self.yaw = 90 - self.yaw_raw
            else:
                self.yaw = 450 - self.yaw_raw
        else:
            self.yaw = self.yaw_raw

    def stanley_control_angle(self):  # 스탠리 컨트롤 계산
        slx, sly, slvel = self.csv_to_list(self.way_num)
        print(self.utm_x,self.utm_y)
        # 경로가 변경되었는지 확인하는 플래그 변수
        if not hasattr(self, 'prev_way_num') or self.prev_way_num != self.way_num:
            self.target_i = 0  # 타겟 i 초기화
            self.prev_way_num = self.way_num  # 현재 경로 번호를 저장
            # 전체 경로 중 가장 가까운 점을 계산하여 타겟 i 설정
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

            # target_i의 +- 1000개의 점만 보도록 범위 설정
            start_i = max(0, self.target_i - self.i_range)
            end_i = min(len(slx) - 1, self.target_i + self.i_range)

            for i in range(start_i, end_i + 1):
                dx = slx[i] - self.utm_x
                dy = sly[i] - self.utm_y
                dist = math.sqrt(dx ** 2 + dy ** 2)

                if dist < min_dist:
                    min_dist = dist
                    self.target_i = i

        # 경로 끝에 도달했는지 확인
        if self.target_i + 1 >= len(slx):
            print("finish, idx= ", self.target_i)
            stop_drive = Twist()
            stop_drive.linear.x = 0
            stop_drive.angular.z = 0
            finish_num = self.way_num * 10
            self.csv_done_pub.publish(finish_num)
            return stop_drive
        else:
            # Stanley 제어 계산
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
            drive_num = self.way_num * 10 + 1
            self.csv_done_pub.publish(drive_num)
            return auto_drive


    def drive_way(self, event):
        drive = Twist()
        if self.utm_x==0 and self.utm_y ==0:
            pass
        else:
            drive = self.stanley_control_angle()
        
        if self.main_cmd_way:  # 메인함수에서 실행 명령이 True일 때
            self.drive_pub.publish(drive)


if __name__ == '__main__':
    try:
        way_controller = way_mando()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("!!Way Program terminated!!")