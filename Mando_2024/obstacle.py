#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool, Int32, String

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance')
        ######### 왜있는지 모르겠음 ##########
        self.safe_distance = 1.8  # 장애물 감지 거리 (미터)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.obs_pub = rospy.Publisher("/obstacle_detected", Bool, queue_size=1)
        self.rate = rospy.Rate(10)  # 10Hz
        self.count = 0  # 장애물 감지 카운트
        ######### 점 갯수 임계값으로 필터링 계수 정하는거임##########
        self.min_detection_points = 10  # 최소 감지 포인트 수
        self.obstacle_detected = False  # 장애물 감지 상태
        self.progress_count = 0
        self.progress_not_count = 0


######### 스캔 데이터 콜백 ##########
    def scan_callback(self, data):
        obstacle_detected = False
        obstacle_distances = []

######### 장애물 전처리 및 탐지 거리 설정 ##########
        for i, n in enumerate(data.ranges):
            if n > 0 and n < float('inf'):
                angle = data.angle_min + i * data.angle_increment
                x = -n * math.cos(angle)
                y = n * math.sin(angle)
                ######### 감지 거리 설정 여기서 바꾸는거임 ##########
                if -0.55 < y < 0.55 and 0 < x < 2:
                    obstacle_distances.append(n)
        print(obstacle_distances)
######### 점 갯수 임계값으로 필터링 ##########
        if len(obstacle_distances) >= self.min_detection_points:
            obstacle_detected = True

        # 장애물이 계속 감지되면 카운트를 증가
        if obstacle_detected:
            self.count += 1
            ######### !!!!!여기서 토글 보낼 주기 조절해야함!!!!! ##########
            if self.count >= 5:
                # 장애물이 계속 있으면 계속 정지 명령을 보냄
                self.obs_toggle_update(min(obstacle_distances) if obstacle_distances else self.safe_distance)
                # print(min(obstacle_distances))
        else:
            # 장애물이 없으면 카운트와 상태를 리셋
            self.count = 0
            self.obstacle_detected = False
            self.obs_toggle_update(obstacle_distances)

        # 프로그레스 바 업데이트, obstacle_distances를 넘겨줌
        self.update_progress(len(obstacle_distances))

######### 시각화, 100%부터 시작 ##########
    def update_progress(self, num_obstacles):
        # 10% 단위로 프로그레스 바 출력
        progress = int((self.count / 10.0) * 100)
        if progress >= 100:
            progress = 100
            self.progress_count += 1
        bar = '#' * (progress // 10) + '-' * (10 - (progress // 10))
        print('{} {}% - 점 {}개 발견'.format(bar, progress, num_obstacles))


######### 토글 업데이트 ##########
    def obs_toggle_update(self, closest_obstacle):
        ######### 몇개 쌓이면 발행할지 ##########
        if self.progress_count <= 60: # 이 카운트는 100%이후부터 오름
            # 그냥 주행시에는 프로그레스 카운트가 0임
            # print(self.progress_count)
            if closest_obstacle < 1.8:  # 안전거리 보다 작은경우 True를 날림
                self.obs_pub.publish(True)
            else:
                # 장애물이 있지만 가까운 점이 1.5보다 멀리있다고 판단할 때임
                # obs 토글이 False여야함
                self.obs_pub.publish(False)
        else:
            print("차량 주행 중")
            # obs 토글이 False여야함
            self.obs_pub.publish(False)

if __name__ == '__main__':
    node = ObstacleAvoidance()
    rospy.spin()  # 노드 실행 유지