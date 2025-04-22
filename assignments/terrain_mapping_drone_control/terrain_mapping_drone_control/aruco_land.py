#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math
import time

from px4_msgs.msg import VehicleOdometry, OffboardControlMode, VehicleCommand, VehicleStatus, TrajectorySetpoint, VehicleGlobalPosition
from std_msgs.msg import String
from nav_msgs.msg import Odometry

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

        self.global_position_subscriber = self.create_subscription(
            VehicleGlobalPosition, '/fmu/out/vehicle_global_position',
            self.global_position_callback, qos_profile)

        self.aruco_subscriber = self.create_subscription(
            String,
            '/aruco/marker_pose',
            self.marker_callback,
            qos_default)

        self.gazebo_pose_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_default)

        self.marker_position = None
        self.landing_started = False
        self.offboard_setpoint_counter = 0
        self.vehicle_odometry = VehicleOdometry()
        self.current_position = (0.0, 0.0, 0.0)
        self.global_position = (0.0, 0.0, 0.0)

        self.create_timer(0.1, self.timer_callback)

    def marker_callback(self, msg):
        try:
            parts = msg.data.strip().split(' ')
            x = float(parts[4][2:-2])
            y = float(parts[5][2:-2])
            z = float(parts[6][2:-2])
            self.marker_position = (x, y, z)
            self.get_logger().info(f"Parsed marker position: x={x:.2f}, y={y:.2f}, z={z:.2f}")
        except Exception as e:
            self.get_logger().error(f"Failed to parse marker_pose message: {str(e)}")

    def vehicle_odometry_callback(self, msg):
        self.vehicle_odometry = msg

    def global_position_callback(self, msg):
        self.global_position = (
            msg.lat / 1e7,
            msg.lon / 1e7,
            msg.alt / 1e3)
        self.get_logger().debug(f"Global position: lat={self.global_position[0]:.7f}, lon={self.global_position[1]:.7f}, alt={self.global_position[2]:.2f}")

    def vehicle_status_callback(self, msg):
        pass

    def odom_callback(self, msg):
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z)

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
            x, y, z = self.marker_position
            self.publish_trajectory_setpoint(x=x, y=y, z=0.3, yaw=0.0)
            if not self.landing_started:
                self.get_logger().info(f"Landing on marker near global position: lat={self.global_position[0]:.7f}, lon={self.global_position[1]:.7f}, alt={self.global_position[2]:.2f}")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.landing_started = True
        else:
            cx, cy, cz = self.current_position
            self.get_logger().info(f"Waiting for ArUco marker... Local pos: x={cx:.2f}, y={cy:.2f}, z={cz:.2f} | Global pos: lat={self.global_position[0]:.7f}, lon={self.global_position[1]:.7f}, alt={self.global_position[2]:.2f}")

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
