#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import TransformStamped, Point, PoseStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from std_msgs.msg import String
from transforms3d.euler import mat2euler
import math
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
            self.get_logger().info('Using OpenCV 4.7+ ArUco API')
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
        self.global_pose_pub = self.create_publisher(PoseStamped, '/aruco/global_marker_pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Image, '/drone/down_mono', self.image_callback, 10)

        camera_info_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(CameraInfo, '/drone/down_mono/camera_info', self.camera_info_callback, qos_profile=camera_info_qos)

    def image_callback(self, msg):
        if not self.calibration_received:
            self.get_logger().debug('Using default camera calibration')

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'mono8')
            self.frame_counter += 1
            corners, ids, rejected = self.detector.detectMarkers(cv_image) if self.detector else cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)
            debug_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(debug_image, corners, ids)

                for i in range(len(ids)):
                    marker_points = np.array([
                        [-self.marker_size/2, self.marker_size/2, 0],
                        [self.marker_size/2, self.marker_size/2, 0],
                        [self.marker_size/2, -self.marker_size/2, 0],
                        [-self.marker_size/2, -self.marker_size/2, 0]], dtype=np.float32)
                    objPoints = marker_points.reshape((4,3))
                    imgPoints = corners[i].reshape((4,2))

                    success, rvec, tvec = cv2.solvePnP(objPoints, imgPoints, self.camera_matrix, self.dist_coeffs)
                    if success:
                        cv2.drawFrameAxes(debug_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_size/2)
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

                        try:
                            marker_pose_cam = PoseStamped()
                            marker_pose_cam.header.stamp = msg.header.stamp
                            marker_pose_cam.header.frame_id = 'camera_frame'
                            marker_pose_cam.pose.position.x = marker_position[0]
                            marker_pose_cam.pose.position.y = marker_position[1]
                            marker_pose_cam.pose.position.z = marker_position[2]
                            marker_pose_cam.pose.orientation.x = quat[0]
                            marker_pose_cam.pose.orientation.y = quat[1]
                            marker_pose_cam.pose.orientation.z = quat[2]
                            marker_pose_cam.pose.orientation.w = quat[3]

                            transformed_pose = self.tf_buffer.transform(marker_pose_cam, 'odom', timeout=rclpy.duration.Duration(seconds=0.5))
                            self.global_pose_pub.publish(transformed_pose)

                            self.get_logger().info(
                                f"Marker {ids[i][0]} global position → x: {transformed_pose.pose.position.x:.2f}, y: {transformed_pose.pose.position.y:.2f}, z: {transformed_pose.pose.position.z:.2f}"
                            )
                        except (LookupException, ConnectivityException, ExtrapolationException) as tf_error:
                            self.get_logger().warn(f"TF2 lookup failed: {str(tf_error)}")

                        marker_center = corners[i][0].mean(axis=0)
                        cv2.circle(debug_image, (int(marker_center[0]), int(marker_center[1])), 5, (0, 0, 255), -1)

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

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return [x, y, z, w]

def main():
    rclpy.init()
    node = ArucoTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
