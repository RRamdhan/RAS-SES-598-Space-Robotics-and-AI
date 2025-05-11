#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math
from enum import Enum, auto
import numpy as np
import pandas as pd

from px4_msgs.msg import VehicleOdometry, OffboardControlMode, VehicleCommand, VehicleStatus, TrajectorySetpoint, VehicleGlobalPosition, VehicleLocalPosition
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo  # Added for RGB camera and camera info
import cv_bridge  # For converting ROS Image messages to OpenCV images
import cv2  # For image processing (if needed)



class LandingState(Enum):
    ARMING_AND_OFFBOARD = auto()
    MOVING_TO_SETPOINT = auto()
    LANDING = auto()

class RGBFlightNode(Node):
    def __init__(self):
        super().__init__('rgb_flight_node')

        self.ref_lat = None
        self.ref_lon = None
        self.ref_alt = None
        self.qualifying_colors = []
        self.df_position = pd.DataFrame()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        qos_default = QoSProfile(depth=10)

        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', 
            self.vehicle_odometry_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos_profile)
        self.global_position_subscriber = self.create_subscription(
            VehicleGlobalPosition, '/fmu/out/vehicle_global_position',
            self.global_position_callback, qos_profile)
        self.local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.local_position_callback, qos_profile)
        self.gazebo_pose_subscriber = self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos_default)
        self.rgb_subscriber = self.create_subscription(
            Image, '/drone/front_rgb', self.rgb_callback, qos_default)
        
        self.bridge = cv_bridge.CvBridge()
        
        self.state = LandingState.ARMING_AND_OFFBOARD
        self.phase_start_time = self.get_clock().now()
        self.waypoint_index = 0  # Track current waypoint

        self.local_position = (0.0, 0.0, 0.0)
        self.current_position = (0.0, 0.0, 0.0)
        self.global_position = (0.0, 0.0, 0.0)
        self.landing_started = False
        self.rgb_image = None
        

        self.create_timer(0.1, self.timer_callback)

    def vehicle_odometry_callback(self, msg):
        pass  # optional future use

    def local_position_callback(self, msg):
        self.local_position = (msg.x, msg.y, msg.z)
        if self.qualifying_colors:
            self.get_logger().info(
                f"Local position with qualifying colors: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}"
            )
            df_temp = pd.DataFrame({'x':[msg.x], 'y':[msg.y], 'z':[msg.z]})
            self.df_position = pd.concat([self.df_position, df_temp], ignore_index=True)

    def odom_callback(self, msg):
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z)

    def rgb_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
             # Downsample image to improve performance (optional, adjust size as needed)
            resized_image = cv2.resize(self.rgb_image, (320, 240))
            
            # Reshape image to a 2D array of pixels
            pixels = resized_image.reshape(-1, 3).astype(np.float32)
            
            # Define criteria and apply k-means clustering with K=3 clusters
            K = 3
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
            _, labels, centers = cv2.kmeans(pixels, K, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)

            # Analyze cluster centers for large RGB differences, excluding gray colors
            self.qualifying_colors = []
            for center in centers:
                b, g, r = center.astype(int)  # OpenCV uses BGR
                rgb_color = [int(r), int(g), int(b)]  # Convert to RGB
                rgb_array = np.array([r, g, b])
                
                # Calculate max difference and standard deviation
                max_diff = np.max(rgb_array) - np.min(rgb_array)
                std_dev = np.std(rgb_array)
                
                # Check if the color is not gray and has large differences
                is_not_gray = max_diff > 30 and std_dev > 20  # Exclude gray-like colors
                has_large_diff = max_diff > 100  # Require significant difference
                
                if is_not_gray and has_large_diff:
                    self.qualifying_colors.append(rgb_color)
            
            # Log and print qualifying colors
            if self.qualifying_colors:
                # self.get_logger().info("Colors with large RGB differences (non-gray):")
                # print("Colors with large RGB differences (non-gray):")
                for i, rgb_color in enumerate(self.qualifying_colors):
                    pass
                    # self.get_logger().info(f"Cluster {i+1} RGB color: {rgb_color}")
                    # print(f"Cluster {i+1} RGB color: {rgb_color}")
            else:
                pass
                # self.get_logger().info("No colors with large RGB differences (non-gray) detected")
                # print("No colors with large RGB differences (non-gray) detected")

            # # Log all cluster centers as RGB colors
            # # self.get_logger().info("Detected RGB colors in clusters:")
            # for i, center in enumerate(centers):
            #     b, g, r = center.astype(int)  # OpenCV uses BGR, convert to RGB
            #     rgb_color = [int(r), int(g), int(b)]
            #     # self.get_logger().info(f"Cluster {i+1} RGB color: {rgb_color}")
            #     if(int(r)!=int(g)):
            #        if(int(g)!=int(b)):
            #           self.get_logger().info(f"Cluster {i+1} RGB color: {rgb_color}") 

        except Exception as e:
            self.get_logger().error(f"Failed to process RGB image: {str(e)}")

    def global_position_callback(self, msg):
        a = 6378137.0
        e_sq = 6.69437999014e-3
        deg_to_rad = math.pi / 180.0
        lat_rad = msg.lat * deg_to_rad
        lon_rad = msg.lon * deg_to_rad
        alt = msg.alt

        N = a / math.sqrt(1 - e_sq * math.sin(lat_rad)**2)
        x = (N + alt) * math.cos(lat_rad) * math.cos(lon_rad)
        y = (N + alt) * math.cos(lat_rad) * math.sin(lon_rad)
        z = (N * (1 - e_sq) + alt) * math.sin(lat_rad)

        if not hasattr(self, 'ref_ecef'):
            self.ref_ecef = (x, y, z)
            self.ref_lat = msg.lat
            self.ref_lon = msg.lon
            self.ref_alt = msg.alt
            self.get_logger().info("Set reference ECEF and LLA origin")

        dx = x - self.ref_ecef[0]
        dy = y - self.ref_ecef[1]
        dz = z - self.ref_ecef[2]

        ref_lat_rad = self.ref_lat * deg_to_rad
        ref_lon_rad = self.ref_lon * deg_to_rad
        sin_lat = math.sin(ref_lat_rad)
        cos_lat = math.cos(ref_lat_rad)
        # sin\.cls
        sin_lon = math.sin(ref_lon_rad)
        cos_lon = math.cos(ref_lon_rad)

        t = [
            [-sin_lon, cos_lon, 0],
            [-sin_lat*cos_lon, -sin_lat*sin_lon, cos_lat],
            [cos_lat*cos_lon, cos_lat*sin_lon, sin_lat]
        ]

        x_enu = t[0][0]*dx + t[0][1]*dy + t[0][2]*dz
        y_enu = t[1][0]*dx + t[1][1]*dy + t[1][2]*dz
        z_enu = t[2][0]*dx + t[2][1]*dy + t[2][2]*dz
        self.global_position = (x_enu, y_enu, z_enu)

    def vehicle_status_callback(self, msg):
        pass

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("Arm command sent")

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Offboard mode command sent")

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self):
        now = self.get_clock().now()
        self.publish_offboard_control_mode()

        if self.state == LandingState.ARMING_AND_OFFBOARD:
            if (now - self.phase_start_time).nanoseconds / 1e9 > 2.0:
                self.engage_offboard_mode()
                self.arm()
                self.phase_start_time = now
                self.state = LandingState.MOVING_TO_SETPOINT

        elif self.state == LandingState.MOVING_TO_SETPOINT:
            # Define waypoints
            waypoints = [
                # (0.0, 0.0, -3.0),    # A: Start point
                # (7.5, 0.0, -3.0),    # B
                # (7.5, 12.0, -3.0),   # C
                # (2.5, 12.0, -3.0),   # D
                # (2.5, 0.0, -3.0),    # E
                # (-2.5, 0.0, -3.0),    # F
                # (-2.5, 12.0, -3.0),    # G
                # (-7.5, 12.0, -3.0),    # H
                # (-7.5, 0.0, -3.0),    # I
                # (0.0, 0.0, -3.0)     # Final point
                (0.0, 0.0, -3.0),    # A: Start point
                (2.5, 0.0, -3.0),    # B
                (7.5, 0.0, -3.0),    # C
                (7.5, 6.0, -3.0),    # D
                (7.5, 12.0, -3.0),   # E
                (5.0, 12.0, -3.0),   # F
                (2.5, 12.0, -3.0),   # G
                (2.5, 6.0, -3.0),    # H
                (2.5, 0.0, -3.0),    # I
                (0.0, 0.0, -3.0),    # J
                (-2.5, 0.0, -3.0),   # K
                (-2.5, 6.0, -3.0),   # L
                (-2.5, 12.0, -3.0),  # M
                (-5.0, 12.0, -3.0),  # N
                (-7.5, 12.0, -3.0),  # O
                (-7.5, 6.0, -3.0),   # P
                (-7.5, 0.0, -3.0),   # Q
                (-5.0, 0.0, -3.0),   # R
                (0.0, 0.0, -3.0)     # Final point
            ]
            waypoint_labels = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', \
                               'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', \
                               'Final']

            if self.waypoint_index < len(waypoints):
                # Check if enough time has passed to move to the next waypoint
                if (now - self.phase_start_time).nanoseconds / 1e9 > 10.0:
                    x_final, y_final, z_final = waypoints[self.waypoint_index]
                    print(waypoint_labels[self.waypoint_index])
                    # print(self.rgb_number)
                    self.publish_trajectory_setpoint(x=x_final, y=y_final, z=z_final, yaw=0.0)
                    self.phase_start_time = now
                    self.waypoint_index += 1
                else:
                    # Continue publishing the current setpoint
                    x_final, y_final, z_final = waypoints[self.waypoint_index]
                    self.publish_trajectory_setpoint(x=x_final, y=y_final, z=z_final, yaw=0.0)
            else:
                # All waypoints processed, move to LANDING
                self.state = LandingState.LANDING
                self.phase_start_time = now

        elif self.state == LandingState.LANDING:
            if (now - self.phase_start_time).nanoseconds / 1e9 > 10.0 and not self.landing_started:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.landing_started = True
            self.df_position.to_csv('possible_lost_hikers.csv', index=False)
def main():
    rclpy.init()
    node = RGBFlightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()