#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from std_msgs.msg import String
from transforms3d.euler import mat2euler
import math
import tf_transformations
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class ArucoTracker(Node):
    def __init__(self):
        super().__init__('aruco_tracker')

        self.cv_bridge = CvBridge()
        self.frame_counter = 0
        self.get_logger().info(f'OpenCV version: {cv2.__version__}')

        try:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.get_logger().info('Using OpenCV 4.7+ ArUco API with 4x4 dictionary')
        except AttributeError:
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.detector = None
            self.get_logger().info('Using older OpenCV ArUco API')

        self.camera_matrix = np.array([
            [554.254691191187, 0.0, 320.5],
            [0.0, 554.254691191187, 240.5],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.zeros(5)
        self.calibration_received = False
        self.marker_size = 0.8

        self.debug_image_pub = self.create_publisher(Image, '/aruco/debug_image', 10)
        self.marker_pose_pub = self.create_publisher(String, '/aruco/marker_pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.local_frame = 'odom'
        self.camera_frame = 'camera_frame'

        self.create_subscription(Image, '/drone/down_mono', self.image_callback, 10)
        camera_info_qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(CameraInfo, '/drone/down_mono/camera_info', self.camera_info_callback, qos_profile=camera_info_qos)

    def image_callback(self, msg):
        if not self.calibration_received:
            self.get_logger().debug('Using default camera calibration')
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'mono8')
            self.frame_counter += 1
            corners, ids, rejected = self.detect_markers(cv_image)
            debug_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(debug_image, corners, ids)
                for i in range(len(ids)):
                    objPoints = np.array([
                        [-self.marker_size/2, self.marker_size/2, 0],
                        [self.marker_size/2, self.marker_size/2, 0],
                        [self.marker_size/2, -self.marker_size/2, 0],
                        [-self.marker_size/2, -self.marker_size/2, 0]], dtype=np.float32)
                    imgPoints = corners[i].reshape((4,2))
                    success, rvec, tvec = cv2.solvePnP(objPoints, imgPoints, self.camera_matrix, self.dist_coeffs)
                    if success:
                        marker_pose_cam = PoseStamped()
                        marker_pose_cam.header.stamp = self.get_clock().now().to_msg()
                        marker_pose_cam.header.frame_id = self.camera_frame
                        marker_pose_cam.pose.position.x = tvec[0][0]
                        marker_pose_cam.pose.position.y = tvec[1][0]
                        marker_pose_cam.pose.position.z = tvec[2][0]

                        rmat, _ = cv2.Rodrigues(rvec)
                        quat = tf_transformations.quaternion_from_matrix(np.vstack((np.hstack((rmat, [[0],[0],[0]])), [0,0,0,1])))
                        marker_pose_cam.pose.orientation.x = quat[0]
                        marker_pose_cam.pose.orientation.y = quat[1]
                        marker_pose_cam.pose.orientation.z = quat[2]
                        marker_pose_cam.pose.orientation.w = quat[3]

                        try:
                            transformed = self.tf_buffer.transform(marker_pose_cam, self.local_frame, timeout=rclpy.duration.Duration(seconds=1.0))
                            x, y, z = transformed.pose.position.x, transformed.pose.position.y, transformed.pose.position.z
                            self.get_logger().info(f"Marker in ENU ({self.local_frame}): x={x:.2f}, y={y:.2f}, z={z:.2f}")
                        except (LookupException, ConnectivityException, ExtrapolationException) as e:
                            self.get_logger().warn(f"TF transform failed: {str(e)}")

            debug_msg = self.cv_bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def detect_markers(self, image):
        try:
            if self.detector is not None:
                corners, ids, rejected = self.detector.detectMarkers(image)
            else:
                corners, ids, rejected = cv2.aruco.detectMarkers(image, self.aruco_dict, parameters=self.aruco_params)
            return corners, ids, rejected
        except Exception as e:
            self.get_logger().error(f'Error in marker detection: {str(e)}')
            return [], None, []

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
