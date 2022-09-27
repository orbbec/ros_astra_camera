#!/usr/bin/env python3

import message_filters
from sensor_msgs.msg import Image
import rospy
import cv2
from cv_bridge import CvBridge
import numpy as np

d2c_pub = None

def callback(rgb_msg : Image, depth_msg : Image):
  cv_bridge = CvBridge()
  if rgb_msg.width != depth_msg.width or rgb_msg.height != depth_msg.height:
    print ("only support same size images")
    return

  rgb_img = cv_bridge.imgmsg_to_cv2(rgb_msg, "rgb8")
  depth_img = cv_bridge.imgmsg_to_cv2(depth_msg, "16UC1")
  gray_depth = np.uint8(depth_img)
  gray_depth = cv2.cvtColor(gray_depth, cv2.COLOR_GRAY2BGR)
  d2c_image = np.zeros((rgb_img.shape[0], rgb_img.shape[1], 3), dtype=np.uint8)
  cv2.bitwise_or(rgb_img, gray_depth, d2c_image)
  d2c_msg = cv_bridge.cv2_to_imgmsg(d2c_image, "rgb8")
  d2c_pub.publish(d2c_msg)

def main():
    rospy.init_node('test_sync', anonymous=True)
    rgb_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
    depth_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)
    global d2c_pub
    d2c_pub = rospy.Publisher('/camera/depth_to_color/image_raw', Image, queue_size=1)
    ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], 10, 1)
    ts.registerCallback(callback)
    rospy.spin()


if __name__ == '__main__':
    main()
