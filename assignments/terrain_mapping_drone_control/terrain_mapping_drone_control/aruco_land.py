#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, OffboardControlMode
from rclpy.qos import QoSProfile

class ArucoLandingController(Node):
    def __init__(self):
        super().__init__('aruco_landing_controller')

        qos = QoSProfile(depth=10)

        # Subscribers
        self.subscription = self.create_subscription(
            PointStamped,
            '/aruco/marker_position',
            self.marker_callback,
            qos)

        # Publishers
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)

        self.timer = self.create_timer(0.1, self.control_loop)

        # Marker target
        self.marker_position = None
        self.landing_started = False

    def marker_callback(self, msg):
        self.marker_position = msg.point
        self.get_logger().info(
            f"Received ArUco Marker Position: x={msg.point.x:.2f}, y={msg.point.y:.2f}, z={msg.point.z:.2f}")

    def control_loop(self):
        if self.marker_position is None:
            return  # wait for marker

        # Step 1: Set Offboard mode
        offboard_mode = OffboardControlMode()
        offboard_mode.position = True
        self.mode_pub.publish(offboard_mode)

        # Step 2: Fly above marker first
        landing_altitude = 0.3  # meters above ground

        sp = TrajectorySetpoint()
        sp.position = [self.marker_position.x, self.marker_position.y, landing_altitude]
        sp.yaw = 0.0
        self.trajectory_pub.publish(sp)

        # Step 3: Trigger landing once close enough
        if not self.landing_started and self.marker_position.z < 0.6:
            self.get_logger().info("Initiating landing sequence")
            self.trigger_land()
            self.landing_started = True

    def trigger_land(self):
        cmd = VehicleCommand()
        cmd.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        cmd.param1 = 0.0
        self.command_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoLandingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

