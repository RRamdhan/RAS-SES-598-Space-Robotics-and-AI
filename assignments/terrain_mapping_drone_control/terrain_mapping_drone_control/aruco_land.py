#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math
import time

from px4_msgs.msg import VehicleOdometry, OffboardControlMode, VehicleCommand, VehicleStatus, TrajectorySetpoint
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped

class ArucoLandingNode(Node):
    def __init__(self):
        super().__init__('aruco_landing_node')

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

        self.aruco_subscriber = self.create_subscription(
            PointStamped,
            '/aruco/marker_position',
            self.marker_callback,
            qos_default)

        self.marker_position = None
        self.landing_started = False
        self.offboard_setpoint_counter = 0
        self.vehicle_odometry = VehicleOdometry()

        # Create a timer to publish control commands
        self.create_timer(0.1, self.timer_callback)

    def marker_callback(self, msg):
        self.marker_position = msg.point
        self.get_logger().info(f"Received ArUco marker position: x={msg.point.x:.2f}, y={msg.point.y:.2f}, z={msg.point.z:.2f}")

    def vehicle_odometry_callback(self, msg):
        self.vehicle_odometry = msg

    def vehicle_status_callback(self, msg):
        pass  # No-op unless status needed

    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("Arm command sent")

    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Offboard mode command sent")

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
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
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        self.publish_offboard_control_mode()

        if self.marker_position:
            self.publish_trajectory_setpoint(
                x=self.marker_position.x,
                y=self.marker_position.y,
                z=0.3,
                yaw=0.0)

            if not self.landing_started:
                self.get_logger().info("Landing on ArUco marker...")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.landing_started = True
        else:
            self.get_logger().info("Waiting for ArUco marker position...")

        self.offboard_setpoint_counter += 1

def main():
    print('Starting ArUco landing node...')
    rclpy.init()
    node = ArucoLandingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
