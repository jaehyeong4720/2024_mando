#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
import csv
import numpy as np
import os

class JoyDrive:
    def __init__(self):
        rospy.init_node('joy_drive', anonymous=True)
        self.sub_joy = rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.sub_utm_x = rospy.Subscriber("/utm_x_topic", Float64, self.utm_x_callback)
        self.sub_utm_y = rospy.Subscriber("/utm_y_topic", Float64, self.utm_y_callback)
        
        self.utm_x = None  # 현재 UTM x값
        self.utm_y = None  # 현재 UTM y값
        self.utm_points = []  # UTM 좌표 저장 리스트
        self.rate = rospy.Rate(20)  # 20Hz 주기 설정
        self.spacing = 0.01  # 간격을 m로 설정

        self.csv_file_path = 'utm_points.csv'  # CSV 파일 경로

        # 이전 버튼 상태를 저장하는 리스트 (디바운싱을 위한 변수)
        self.prev_button_states = [0] * 12  # 버튼 개수에 맞게 조정

        self.manual = False
        print("3번(왼쪽)을 누르면 현재 UTM 좌표를 기록하고, 2번(오른쪽)을 누르면 기록된 좌표 사이에 간격을 계산하여 CSV를 생성합니다.")
        print("현재 간격: {}m".format(self.spacing))

    def joy_callback(self, data):
        # button[3]을 눌렀을 때 현재 UTM 좌표 기록 (디바운싱 처리 추가)
        if data.buttons[3] and not self.prev_button_states[3] and self.utm_x is not None and self.utm_y is not None:
            self.utm_points.append([self.utm_x, self.utm_y])
            rospy.loginfo("UTM 좌표 기록됨: {}, {}".format(self.utm_x, self.utm_y))

        # button[2]을 눌렀을 때 기록된 좌표들 사이에 간격을 두고 CSV 생성 (디바운싱 처리 추가)
        if data.buttons[2] and not self.prev_button_states[2] and len(self.utm_points) >= 2:
            self.save_points_to_csv()

        # 현재 버튼 상태를 이전 버튼 상태로 업데이트
        self.prev_button_states = data.buttons[:]

    def utm_x_callback(self, data):
        self.utm_x = data.data  # UTM x 값 저장

    def utm_y_callback(self, data):
        self.utm_y = data.data  # UTM y 값 저장

    def save_points_to_csv(self):
        points = []

        # 저장된 좌표들 사이에 간격을 두고 점을 계산하여 저장
        for i in range(len(self.utm_points) - 1):
            utm1 = np.array(self.utm_points[i])
            utm2 = np.array(self.utm_points[i + 1])
            
            # 두 좌표 사이의 x, y 변화량 계산
            delta_x = utm2[0] - utm1[0]
            delta_y = utm2[1] - utm1[1]
            
            # 두 좌표 사이의 전체 거리 계산
            total_distance = np.linalg.norm([delta_x, delta_y])
            
            # 거리 값이 0이 아닌 경우에만 계산 진행
            if total_distance > 0:
                # 간격에 따른 점의 수 계산
                num_points = int(total_distance / self.spacing)

                # 방향 벡터 계산 (x, y 변화량을 총 거리로 나눈 값)
                direction_vector = np.array([delta_x, delta_y]) / total_distance

                # 각 점을 간격에 맞춰 생성
                for j in range(num_points + 1):
                    new_point = utm1 + direction_vector * self.spacing * j
                    
                    # 만약 nan 값이 발생하면 이전 좌표를 사용
                    if np.isnan(new_point).any():
                        rospy.logwarn("계산 중 nan이 발생, 이전 좌표 사용.")
                        if points:  # points 리스트에 이전 값이 있으면 사용
                            new_point = points[-1]
                        else:
                            rospy.logwarn("이전 좌표가 없어 새로운 좌표 생성 불가.")
                    
                    points.append([round(new_point[0], 10), round(new_point[1], 10)])  # 소수점 10자리까지 저장
            else:
                rospy.logwarn("두 좌표가 동일하여 거리 계산 불가.")

        # CSV 파일로 저장 (헤더 없이)
        with open(self.csv_file_path, 'w') as csvfile:
            csvwriter = csv.writer(csvfile)
            for point in points:
                csvwriter.writerow(point)

        rospy.loginfo("CSV 파일로 UTM 좌표 저장 완료. 간격: {}m".format(self.spacing))

    def run(self):
        rospy.spin()  # 노드를 계속 실행 상태로 유지

if __name__ == '__main__':
    joy_drive = JoyDrive()
    joy_drive.run()
