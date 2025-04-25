#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, PoseStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from transforms3d.euler import mat2euler
from tf2_ros import TransformBroadcaster, Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
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
        except AttributeError:
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.detector = None

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
        self.enu_marker_pub = self.create_publisher(PoseStamped, '/aruco/marker_pose_enu', 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Image, '/drone/down_mono', self.image_callback, 10)
        camera_info_qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(CameraInfo, '/drone/down_mono/camera_info', self.camera_info_callback, qos_profile=camera_info_qos)

    def detect_markers(self, image):
        try:
            if self.detector:
                return self.detector.detectMarkers(image)
            return cv2.aruco.detectMarkers(image, self.aruco_dict, parameters=self.aruco_params)
        except Exception as e:
            self.get_logger().error(f'Marker detection failed: {str(e)}')
            return [], None, []

    def draw_crosshair(self, image, center, size=20, color=(0, 0, 255), thickness=2):
        x, y = int(center[0]), int(center[1])
        cv2.line(image, (x - size, y), (x + size, y), color, thickness)
        cv2.line(image, (x, y - size), (x, y + size), color, thickness)
        cv2.circle(image, (x, y), 2, color, thickness)

    def image_callback(self, msg):
        if not self.calibration_received:
            self.get_logger().debug('Using default calibration')
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'mono8')
            self.frame_counter += 1
            corners, ids, _ = self.detect_markers(cv_image)
            debug_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(debug_image, corners, ids)
                for i in range(len(ids)):
                    marker_points = np.array([
                        [-self.marker_size / 2, self.marker_size / 2, 0],
                        [self.marker_size / 2, self.marker_size / 2, 0],
                        [self.marker_size / 2, -self.marker_size / 2, 0],
                        [-self.marker_size / 2, -self.marker_size / 2, 0]], dtype=np.float32)
                    success, rvec, tvec = cv2.solvePnP(marker_points, corners[i].reshape((4, 2)), self.camera_matrix, self.dist_coeffs)
                    if success:
                        marker_position = tvec.flatten()
                        rot_matrix, _ = cv2.Rodrigues(rvec)
                        euler_angles = mat2euler(rot_matrix)
                        quat = self.euler_to_quaternion(*euler_angles)

                        transform = TransformStamped()
                        transform.header.stamp = self.get_clock().now().to_msg()
                        transform.header.frame_id = 'camera_frame'
                        transform.child_frame_id = f'aruco_marker_{ids[i][0]}'
                        transform.transform.translation.x = marker_position[0]
                        transform.transform.translation.y = marker_position[1]
                        transform.transform.translation.z = marker_position[2]
                        transform.transform.rotation.x = quat[0]
                        transform.transform.rotation.y = quat[1]
                        transform.transform.rotation.z = quat[2]
                        transform.transform.rotation.w = quat[3]
                        self.tf_broadcaster.sendTransform(transform)

                        self.publish_enu_pose(ids[i][0], marker_position, quat)

                        marker_center = corners[i][0].mean(axis=0)
                        self.draw_crosshair(debug_image, marker_center)

            debug_msg = self.cv_bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f'Error in image callback: {str(e)}')

    def publish_enu_pose(self, marker_id, position, quat):
        try:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = 'camera_frame'
            pose_stamped.pose.position.x = position[0]
            pose_stamped.pose.position.y = position[1]
            pose_stamped.pose.position.z = position[2]
            pose_stamped.pose.orientation.x = quat[0]
            pose_stamped.pose.orientation.y = quat[1]
            pose_stamped.pose.orientation.z = quat[2]
            pose_stamped.pose.orientation.w = quat[3]

            transformed_pose = self.tf_buffer.transform(pose_stamped, 'odom', timeout=rclpy.duration.Duration(seconds=0.5))
            self.enu_marker_pub.publish(transformed_pose)

            self.get_logger().info(
                f"Marker {marker_id} ENU Pose -> x:{transformed_pose.pose.position.x:.2f}, y:{transformed_pose.pose.position.y:.2f}, z:{transformed_pose.pose.position.z:.2f}"
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF ENU transform failed: {str(e)}')

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        return [sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
                cr * cp * cy + sr * sp * sy]

    def camera_info_callback(self, msg):
        try:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.calibration_received = True
            self.get_logger().info('Camera calibration updated')
        except Exception as e:
            self.get_logger().error(f'Camera info error: {str(e)}')

def main():
    rclpy.init()
    node = ArucoTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
