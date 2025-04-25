#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import TransformStamped, Point
from tf2_ros import TransformBroadcaster
from std_msgs.msg import String
from transforms3d.euler import mat2euler, euler2quat
from transforms3d.quaternions import mat2quat
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class ArucoTracker(Node):
    def __init__(self):
        super().__init__('aruco_tracker')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Frame counter for logging
        self.frame_counter = 0

        # Print OpenCV version for debugging
        self.get_logger().info(f'OpenCV version: {cv2.__version__}')

        try:
            # ArUco dictionary and parameters (OpenCV 4.7+)
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        except Exception as e:
            self.get_logger().error(f"Failed to initialize ArUco: {e}")
            return

        # TF broadcaster
        self.br = TransformBroadcaster(self)

        # Subscription to camera image
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, qos_profile)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, qos_profile)

        self.camera_matrix = None
        self.dist_coeffs = None

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)
        self.get_logger().info("Camera info received.")

    def image_callback(self, msg):
        if self.camera_matrix is None or self.dist_coeffs is None:
            return

        try:
            frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            for i, corner in enumerate(corners):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, 0.15, self.camera_matrix, self.dist_coeffs)
                rvec = rvec[0][0]
                tvec = tvec[0][0]

                # Convert rotation vector to rotation matrix
                rmat, _ = cv2.Rodrigues(rvec)

                # Convert rotation matrix to quaternion using transforms3d
                quat = mat2quat(rmat)  # [w, x, y, z]
                qx, qy, qz, qw = quat[1], quat[2], quat[3], quat[0]

                # Broadcast transform
                transform_stamped = TransformStamped()
                transform_stamped.header.stamp = self.get_clock().now().to_msg()
                transform_stamped.header.frame_id = "camera_link"
                transform_stamped.child_frame_id = f"aruco_marker_{ids[i][0]}"

                transform_stamped.transform.translation.x = tvec[0]
                transform_stamped.transform.translation.y = tvec[1]
                transform_stamped.transform.translation.z = tvec[2]
                transform_stamped.transform.rotation.x = qx
                transform_stamped.transform.rotation.y = qy
                transform_stamped.transform.rotation.z = qz
                transform_stamped.transform.rotation.w = qw

                self.br.sendTransform(transform_stamped)
                self.get_logger().info(f"Published transform for marker ID {ids[i][0]}")

        self.frame_counter += 1
        if self.frame_counter % 30 == 0:
            self.get_logger().info(f"Processed {self.frame_counter} frames.")

def main(args=None):
    rclpy.init(args=args)
    node = ArucoTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
