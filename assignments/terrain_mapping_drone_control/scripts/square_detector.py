#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_pose
import uuid

class SquareDetector(Node):
    def __init__(self):
        super().__init__('square_detector')
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.image_sub = self.create_subscription(
            Image, '/x500_ir/ir/image_raw', self.image_callback, 10)
        self.gps_sub = self.create_subscription(
            NavSatFix, '/x500_ir/gps/fix', self.gps_callback, 10)
        
        self.pose_pub = self.create_publisher(PoseStamped, '/square_pose', 10)
        
        self.latest_gps = None
        self.camera_matrix = np.array([[525.0, 0.0, 320.0],
                                      [0.0, 525.0, 240.0],
                                      [0.0, 0.0, 1.0]], dtype=np.float32)
        self.dist_coeffs = np.zeros((4,), dtype=np.float32)

    def gps_callback(self, msg):
        self.latest_gps = msg

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            square_pose = self.detect_square(cv_image)
            
            if square_pose is not None and self.latest_gps is not None:
                self.publish_square_pose(square_pose)
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

    def detect_square(self, image):
        blurred = cv2.GaussianBlur(image, (5, 5), 0)
        _, thresh = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY)
        
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            approx = cv2.approxPolyDP(contour, 0.04 * cv2.arcLength(contour, True), True)
            
            if len(approx) == 4:
                area = cv2.contourArea(contour)
                if 1000 < area < 10000:
                    obj_points = np.array([
                        [-0.25, -0.25, 0.0],
                        [0.25, -0.25, 0.0],
                        [0.25, 0.25, 0.0],
                        [-0.25, 0.25, 0.0]
                    ], dtype=np.float32)
                    
                    img_points = approx.reshape(4, 2).astype(np.float32)
                    
                    ret, rvec, tvec = cv2.solvePnP(obj_points, img_points, 
                                                  self.camera_matrix, self.dist_coeffs)
                    if ret:
                        R, _ = cv2.Rodrigues(rvec)
                        pose = PoseStamped()
                        pose.header.frame_id = 'ir_camera_link'
                        pose.header.stamp = self.get_clock().now().to_msg()
                        pose.pose.position.x = tvec[0][0]
                        pose.pose.position.y = tvec[1][0]
                        pose.pose.position.z = tvec[2][0]
                        pose.pose.orientation.w = 1.0
                        return pose
        return None

    def publish_square_pose(self, square_pose):
        try:
            transform = self.tf_buffer.lookup_transform('world', 'ir_camera_link', 
                                                       rclpy.time.Time())
            world_pose = do_transform_pose(square_pose.pose, transform)
            
            if self.latest_gps:
                lat = self.latest_gps.latitude + (world_pose.position.y / 111000.0)
                lon = self.latest_gps.longitude + (world_pose.position.x / (111000.0 * np.cos(np.radians(self.latest_gps.latitude))))
                
                self.get_logger().info(f"Square GPS Location: Latitude={lat}, Longitude={lon}")
                
                world_pose_stamped = PoseStamped()
                world_pose_stamped.header.frame_id = 'world'
                world_pose_stamped.header.stamp = self.get_clock().now().to_msg()
                world_pose_stamped.pose = world_pose
                self.pose_pub.publish(world_pose_stamped)
                
                with open('/tmp/square_gps.txt', 'a') as f:
                    f.write(f"{str(uuid.uuid4())},{lat},{lon}\n")
                    
        except Exception as e:
            self.get_logger().error(f"Error transforming pose: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = SquareDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()