#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Twist

class PositionControl:
    def __init__(self):
        rospy.init_node('position_control', anonymous=True)
        self.target_distance = 34.5  # 목표 거리 (미터)
        self.tolerance = 0.1  # 목표 거리의 허용 오차 (미터)
        self.pulse_2_m = 1.0 / 353  # Pulse to meter 변환 값 (아두이노 코드에서 가져온 값)
        self.current_distance = 0.0  # 현재 이동한 거리
        self.prev_encoder_value = 0  # 이전 엔코더 값
        self.max_change_threshold = 100  # 갑작스러운 엔코더 변화 필터링
        self.stop_flag = False  # 정지 후 재시작을 위한 플래그
        # self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  # 차량 속도 명령 퍼블리셔
        self.stop_line_detect_pub = rospy.Publisher('/stop_line_detected', Bool, queue_size=10)
        self.encoder_sub = rospy.Subscriber('/encoder1', Int32, self.encoder_callback)  # 엔코더 데이터 구독

    def encoder_callback(self, data):
        if self.stop_flag:
            return  # 이미 정지 후 재시작 중이라면 더 이상 계산하지 않음

        current_encoder_value = data.data

        # 엔코더 값 변화량 계산 (절대값 제거)
        change_in_encoder = current_encoder_value - self.prev_encoder_value

        # 갑작스러운 엔코더 값 변화는 무시
        if abs(change_in_encoder) > self.max_change_threshold:
            rospy.logwarn("급격한 엔코더 변화 감지됨, 거리 계산에서 제외")
        else:
            # 앞/뒤로 이동 여부에 따라 거리 계산
            distance_increment = change_in_encoder * self.pulse_2_m
            self.current_distance += distance_increment
            rospy.loginfo("현재 이동 거리: {:.2f} m".format(self.current_distance))

            # 목표 거리 구역에 도달하면 정지 명령
            if self.target_distance - self.tolerance <= abs(self.current_distance) <= self.target_distance + self.tolerance:
                self.send_stop_command()

        # 이전 엔코더 값을 현재 값으로 업데이트
        self.prev_encoder_value = current_encoder_value

    def send_stop_command(self):
        if self.stop_flag:
            return  # 이미 정지한 후 다시 재시작 중이면 동작하지 않음

        rospy.loginfo("목표 지점에 도착하여 정지선을 감지했습니다.")

        # 5초 동안 True 발행
        for _ in range(50):  # 5초 동안 정지 명령 전송 (10Hz 기준)
            self.stop_line_detect_pub.publish(True)
            self.stop_flag = True
            rospy.sleep(0.1)
        rospy.loginfo("5초 지남 False발행")
        # 5초 후 False 발행
        self.stop_line_detect_pub.publish(False)


if __name__ == '__main__':
    try:
        controller = PositionControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
