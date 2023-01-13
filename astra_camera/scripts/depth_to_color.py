#!/usr/bin/env python3

import message_filters
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class D2CTestNode(Node):

    def __init__(self):
        super().__init__("D2CTestNode")
        self.rgb_sub = message_filters.Subscriber(
            self,
            Image,
            "/camera/color/image_raw",
            qos_profile=qos_profile_sensor_data)
        self.depth_sub = message_filters.Subscriber(
            self,
            Image,
            "/camera/depth/image_raw",
            qos_profile=qos_profile_sensor_data)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], 10, 0.1, allow_headerless=True)
        self.ts.registerCallback(self.callback)
        self.d2c_pub = self.create_publisher(
            Image, "/camera/depth_to_color/image_raw", qos_profile_sensor_data)

    def callback(self, rgb_msg: Image, depth_msg: Image):
        cv_bridge = CvBridge()
        if rgb_msg.width != depth_msg.width or rgb_msg.height != depth_msg.height:
            print("only support same size images")
            return

        rgb_img = cv_bridge.imgmsg_to_cv2(rgb_msg, "rgb8")
        depth_img = cv_bridge.imgmsg_to_cv2(depth_msg, "16UC1")
        gray_depth = np.uint8(depth_img)
        gray_depth = cv2.cvtColor(gray_depth, cv2.COLOR_GRAY2BGR)
        d2c_image = np.zeros((rgb_img.shape[0], rgb_img.shape[1], 3),
                             dtype=np.uint8)
        cv2.bitwise_or(rgb_img, gray_depth, d2c_image)
        d2c_msg = cv_bridge.cv2_to_imgmsg(d2c_image, "rgb8")
        self.d2c_pub.publish(d2c_msg)


def main(args=None):
    rclpy.init(args=args)
    d2c_node = D2CTestNode()
    rclpy.spin(d2c_node)
    d2c_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
