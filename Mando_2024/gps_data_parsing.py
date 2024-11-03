#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import utm
import math
from sensor_msgs.msg import NavSatFix
from ublox_msgs.msg import NavRELPOSNED
from std_msgs.msg import Float64

class LLAtoUTM:
    def __init__(self):
        rospy.init_node("gps_data_parsing")
        print("gps_data_parsing node start")

        #토픽구독
        rospy.Subscriber("/ublox_gps_base/fix", NavSatFix, self.gps_fix_callback,queue_size=1) # Ublox fix 토픽 구독
        rospy.Subscriber("/ublox_gps_rover/navrelposned", NavRELPOSNED, self.gps_heading_callback,queue_size=1) # Ublox 토픽 구독


        #토픽발행
        self.gps_lat_pub = rospy.Publisher("/gps_lat", Float64, queue_size=1) # latitude 토픽발행
        self.gps_lon_pub = rospy.Publisher("/gps_lon", Float64, queue_size=1) # logitude 토픽발행
        self.UTM_gps_x_pub = rospy.Publisher("/utm_x_topic", Float64, queue_size=1) # UTM_x(동쪽) 토픽발행
        self.UTM_gps_y_pub = rospy.Publisher("/utm_y_topic", Float64, queue_size=1) # UTM_y(북쪽) 토픽발행
        self.gps_heading_pub = rospy.Publisher("/gps_heading_topic", Float64, queue_size=1) # gps_heading 토픽발행

        #변수초기화
        self.utm_x = self.utm_y = None

    def gps_fix_callback(self,gps):
        gps_lat, gps_lon = gps.latitude, gps.longitude # 위경도 값 파싱
        self.utm_x, self.utm_y, Z, ZL = utm.from_latlon(gps_lat, gps_lon) # 위경도를 utm 값으로 변환
        self.gps_lat_pub.publish(gps_lat)
        self.gps_lon_pub.publish(gps_lon)
        self.UTM_gps_x_pub.publish(self.utm_x)
        self.UTM_gps_y_pub.publish(self.utm_y) # 토픽발행 실행

    def gps_heading_callback(self,data):
        heading_raw = data.relPosHeading
        heading = heading_raw * 1e-5
        self.gps_heading_pub.publish(heading)

if __name__ == '__main__':
    try:
        LLAtoUTM()
        rospy.spin()
    except KeyboardInterrupt:
        print("Program terminated")