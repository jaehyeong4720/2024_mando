#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge

class CompressedImageConverter:
    def __init__(self):
        rospy.init_node('compressed_image_converter')

        self.bridge = CvBridge()

        # 카메라 1과 카메라 2의 압축된 이미지 구독
        self.image_sub1 = rospy.Subscriber('/camera1/usb_cam1/image_raw/compressed', CompressedImage, self.callback_cam1)
        self.image_sub2 = rospy.Subscriber('/camera2/usb_cam2/image_raw/compressed', CompressedImage, self.callback_cam2)
        
        # 압축 해제 후 퍼블리시할 토픽
        self.image_pub1 = rospy.Publisher('/camera1/usb_cam1/image_raw', Image, queue_size=10)
        self.image_pub2 = rospy.Publisher('/camera2/usb_cam2/image_raw', Image, queue_size=10)
    
    def callback_cam1(self, msg):
        try:
            # 카메라 1: 압축된 이미지를 OpenCV 이미지로 변환
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            
            # OpenCV 이미지를 sensor_msgs/Image로 변환
            img_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            
            # 변환된 이미지를 퍼블리시 (카메라 1)
            self.image_pub1.publish(img_msg)

        except Exception as e:
            rospy.logerr("Error converting camera1 image: %s" % str(e))
    
    def callback_cam2(self, msg):
        try:
            # 카메라 2: 압축된 이미지를 OpenCV 이미지로 변환
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            
            # OpenCV 이미지를 sensor_msgs/Image로 변환
            img_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            
            # 변환된 이미지를 퍼블리시 (카메라 2)
            self.image_pub2.publish(img_msg)

        except Exception as e:
            rospy.logerr("Error converting camera2 image: %s" % str(e))

if __name__ == '__main__':
    try:
        CompressedImageConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
