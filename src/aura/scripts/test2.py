#!/usr/bin/env python2

import rospy
import sensor_msgs.msg
import cv_bridge
import numpy as np
import cv2 as cv

def get_image(img_msg):
    image = bridge.imgmsg_to_cv2(img_msg, 'bgr8')
    cv.imshow('a', image)
    cv.waitKey(1)


if __name__ == '__main__':
    print(cv.__version__)
    rospy.init_node('image_python')
    bridge = cv_bridge.CvBridge()
    rospy.Subscriber('/bebop/image_raw', sensor_msgs.msg.Image, get_image)
    rospy.spin()
