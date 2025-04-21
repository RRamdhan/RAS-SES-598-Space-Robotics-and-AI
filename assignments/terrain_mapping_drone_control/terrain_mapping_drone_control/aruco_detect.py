#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv-bridge import CvBridge
from sensor_msgs.msg import Image

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.bridge = CvBridge()

        # Create ArUco dictionary and parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()

        # Publisher for marked image
        self.publisher = self.create_publisher(Image, '/aruco/image_marked', 10)

    def image_callback(self, msg: Image):
        try:
            # Use from_image() with format conversion
            bridge_img = self.bridge.from_image(msg, desired_encoding='bgr8')
            frame = bridge_img.image

            # Detect markers
            corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.parameters)

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # Convert back to ROS2 image and publish
            out_msg = self.bridge.to_image_msg(frame, encoding='bgr8')
            out_msg.header = msg.header  # Preserve original timestamp/frame_id
            self.publisher.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
