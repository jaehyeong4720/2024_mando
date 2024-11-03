#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import subprocess  # rosbag play를 실행하기 위한 subprocess
from sensor_msgs.msg import Joy

class JoyBagPlayer:
    def __init__(self):
        rospy.init_node('joy_bag_player')

        rospy.Subscriber("/joy", Joy, self.joy_callback)

        # 각 bag 파일과 그에 해당하는 토픽, 시작 시간 설정 (버튼 번호: [bag 파일 경로, 재생할 토픽 리스트, 시작 시간])
        self.bag_files = {
            0: {"file": u"stop_line_t.bag", "topics": ["/camera1/usb_cam1/image_raw"], "start_time": 0},  # 버튼 0에 대한 설정
            1: {"file": u"traffic_t.bag", "topics": ["/camera2/usb_cam2/image_raw/compressed"], "start_time": 22},  # 버튼 1에 대한 설정, 10초부터 시작
            2: {"file": u"traffic04_com.bag", "topics": ["/gps/fix", "/imu/data"], "start_time": 5}  # 버튼 2에 대한 설정, 5초부터 시작
        }

    def joy_callback(self, data):
        # 각 버튼에 맞는 bag 파일 및 토픽 재생
        for button_index, config in self.bag_files.items():
            if data.buttons[button_index] == 1:  # 해당 버튼이 눌렸을 때
                self.play_bag_file(config["file"], config["start_time"], config["topics"])

    def play_bag_file(self, bag_file, start_time, topics):
        """
        rosbag 파일을 특정 시간과 특정 토픽만 재생하는 함수
        :param bag_file: 재생할 bag 파일 경로
        :param start_time: 시작할 시간 (초 단위)
        :param topics: 재생할 토픽 리스트
        """
        rospy.loginfo("Playing bag file: {} from {} seconds on topics: {}".format(bag_file, start_time, topics))
        try:
            # 파일 경로를 UTF-8로 인코딩하여 subprocess에 전달
            encoded_bag_file = bag_file.encode('utf-8')
            # --start 옵션을 사용하여 특정 시간부터 재생
            cmd = ['rosbag', 'play', encoded_bag_file, '--start', str(start_time)]

            # 토픽을 재생할 경우 명령어에 토픽 리스트 추가
            if topics:
                cmd.extend(['--topics'] + topics)

            # subprocess를 이용해 명령 실행
            subprocess.Popen(cmd)
        except Exception as e:
            rospy.logerr("Error while playing bag file: {}".format(e))

if __name__ == '__main__':
    try:
        JoyBagPlayer()
        rospy.spin()
    except KeyboardInterrupt:
        print("program down")
