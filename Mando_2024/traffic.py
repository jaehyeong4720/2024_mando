#!/usr/bin/env python
# -*- coding: utf-8 -*-  # 파일 인코딩을 UTF-8로 지정

import cv2
import numpy as np
import rospy
from std_msgs.msg import Float64, Bool, Int32, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# 신호등의 색상을 HSV(Hue, Saturation, Value) 색상 공간에서 범위로 정의합니다.
red_lower = np.array([33, 33, 88])  # 빨간색의 HSV 범위 하한값
red_upper = np.array([81, 53, 133])  # 빨간색의 HSV 범위 상한값

# 색상 감지 카운트 임계값
# 이거 한 10개 넘게 아마 설정해야할지도 몰라

def nothing(x):
    pass

# 트랙바로 색상 임계값 설정하는 곳
cv2.namedWindow("Threshold Trackbars")
cv2.createTrackbar("Red Threshold", "Threshold Trackbars", 1, 1000, nothing)

def count_color_pixels(mask, x, y, window_radius):
    mask_circle = np.zeros_like(mask)
    cv2.circle(mask_circle, (x, y), window_radius, 1, thickness=-1)
    return cv2.countNonZero(mask & mask_circle)

class TrafficLightDetector:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node('traffic_light_detector', anonymous=True)
        
        self.red_pub = rospy.Publisher('/red_detected', Bool, queue_size=10)
        self.image_sub = rospy.Subscriber("/camera2/usb_cam2/image_raw", Image, self.image_callback)

        self.red_count = 0
        self.none_count = 0

        self.red_count_threshold = 10
        self.none_count_threshold = 10

        
        self.window_radius = 15
        self.step_size = 20

    def image_callback(self, data):
        try:
            # ROS 이미지 메시지를 OpenCV 이미지로 변환합니다.
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        frame = cv2.resize(frame, (640, 480))
        # roi = frame[30:200, 260:400]
        roi = frame[120:360, 160:440]

        blurred_roi = cv2.GaussianBlur(roi, (7, 7), 2)
        hsv_roi = cv2.cvtColor(blurred_roi, cv2.COLOR_BGR2HSV)

        red_mask = cv2.inRange(roi, red_lower, red_upper)
        red_threshold = cv2.getTrackbarPos("Red Threshold", "Threshold Trackbars")
        
        max_red = 0
        best_red_position = None

        for y in range(self.window_radius, roi.shape[0] - self.window_radius, self.step_size):
            for x in range(self.window_radius, roi.shape[1] - self.window_radius, self.step_size):
                red_count_temp = count_color_pixels(red_mask, x, y, self.window_radius)
                if red_count_temp > max_red and red_count_temp > red_threshold:
                    max_red = red_count_temp
                    best_red_position = (x, y)         

        detected = False

        if best_red_position:
            cv2.circle(roi, best_red_position, self.window_radius, (0, 0, 255), 2)
            cv2.putText(roi, "Red", (best_red_position[0] - 50, best_red_position[1] - 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            self.red_count += 1
            ######레드카운트 임계값 설정하기 위해 프린트 하는 곳######
            print (self.red_count)
            detected = True

        if self.red_count >= self.red_count_threshold:
            self.red_pub.publish(True)
            self.red_count = 0

        if not detected:
            self.none_count += 1
            if self.none_count >= self.none_count_threshold:
                self.red_pub.publish(False)
                self.none_count = 0

        cv2.imshow("Original", frame)
        cv2.imshow("ROI", roi) 
        cv2.imshow("red_mask", red_mask) 
      
    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            if cv2.waitKey(10) == 27:
                rospy.signal_shutdown("User exit")
                cv2.destroyAllWindows()
                break
            rate.sleep()

if __name__ == '__main__':
    try:
        detector = TrafficLightDetector()
        detector.run()  # run 함수를 통해 이벤트 루프를 돌림
    except rospy.ROSInterruptException:
        pass
