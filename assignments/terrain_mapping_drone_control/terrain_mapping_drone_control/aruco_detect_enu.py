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
from transforms3d.euler import mat2euler
from transforms3d.quaternions import quat2mat
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from px4_msgs.msg import VehicleLocalPosition

class ArucoTracker(Node):
    def __init__(self):
        super().__init__('aruco_tracker')

        self.cv_bridge = CvBridge()
        self.frame_counter = 0

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.camera_matrix = np.array([
            [554.254691191187, 0.0, 320.5],
            [0.0, 554.254691191187, 240.5],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.zeros(5)
        self.calibration_received = False

        self.marker_size = 0.8
        self.current_local_position = None

        self.debug_image_pub = self.create_publisher(Image, '/aruco/debug_image', 10)
        self.marker_pose_pub = self.create_publisher(String, '/aruco/marker_pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Image, '/drone/down_mono', self.image_callback, 10)
        camera_info_qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(CameraInfo, '/drone/down_mono/camera_info', self.camera_info_callback, qos_profile=camera_info_qos)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_position_callback, 10)

    def local_position_callback(self, msg):
        self.current_local_position = (msg.x, msg.y, msg.z)

    def image_callback(self, msg):
        if not self.calibration_received:
            self.get_logger().debug('Using default camera calibration')

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'mono8')
            self.frame_counter += 1
            corners, ids, rejected = self.detector.detectMarkers(cv_image)
            debug_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)

            if ids is not None:
                for i in range(len(ids)):
                    objPoints = np.array([
                        [-self.marker_size/2, self.marker_size/2, 0],
                        [self.marker_size/2, self.marker_size/2, 0],
                        [self.marker_size/2, -self.marker_size/2, 0],
                        [-self.marker_size/2, -self.marker_size/2, 0]
                    ], dtype=np.float32)
                    imgPoints = corners[i].reshape((4,2))

                    success, rvec, tvec = cv2.solvePnP(objPoints, imgPoints, self.camera_matrix, self.dist_coeffs)
                    if success:
                        cv2.drawFrameAxes(debug_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_size/2)

                        rot_matrix, _ = cv2.Rodrigues(rvec)
                        euler_angles = mat2euler(rot_matrix)

                        # If we have the drone's local position, convert marker pose to ENU
                        if self.current_local_position:
                            marker_position = tvec.flatten()
                            x_enu = self.current_local_position[0] + marker_position[0]
                            y_enu = self.current_local_position[1] + marker_position[1]
                            z_enu = self.current_local_position[2] + marker_position[2]

                            pose_msg = String()
                            pose_msg.data = f"Marker {ids[i][0]} ENU position: x={x_enu:.2f}, y={y_enu:.2f}, z={z_enu:.2f}"
                            self.marker_pose_pub.publish(pose_msg)
                            self.get_logger().info(pose_msg.data)

            debug_msg = self.cv_bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def camera_info_callback(self, msg):
        try:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.calibration_received = True
            self.get_logger().info('Camera calibration received')
        except Exception as e:
            self.get_logger().error(f'Error processing camera calibration: {str(e)}')

def main():
    rclpy.init()
    node = ArucoTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
