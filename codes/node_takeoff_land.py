#!/usr/bin/env python

### PROBADO, FUNCIONÓ PERO DESPUÉS NO ?? ###

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleAttitudeSetpoint, VehicleCommand, OffboardControlMode, TrajectorySetpoint
from rclpy.clock import Clock
import math

class OffboardControl(Node):

    def __init__(self):
        super().__init__('offboard_control')

        # Create publishers
        self.attitude_setpoint_pub = self.create_publisher(
            VehicleAttitudeSetpoint,
            '/fmu/in/vehicle_attitude_setpoint',
            10
        )
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        # Timer to send setpoints at 20 Hz (0.05 seconds)
        self.timer = self.create_timer(0.05, self.publish_trajectory_setpoint)
        self.offboard_set = False
        self.armed = False
        self.takeoff_completed = False
        self.start_time = None

    def publish_trajectory_setpoint(self):
        # Ensure offboard mode is set before arming
        if not self.offboard_set:
            self.get_logger().info("[DEBUG] Setting offboard mode")
            self.set_offboard_mode()
            self.offboard_set = True

        # Arm the vehicle once offboard mode is set
        if self.offboard_set and not self.armed:
            self.get_logger().info("[DEBUG] Arming the drone")
            self.arm()
            self.armed = True
            self.start_time = self.get_clock().now()

        # Send takeoff setpoint (up to 2 meters in height)
        if not self.takeoff_completed:
            takeoff_msg = TrajectorySetpoint()
            takeoff_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            takeoff_msg.position = [0.0, 0.0, -2.0]  # Z is negative for upward direction in NED
            takeoff_msg.velocity = [0.0, 0.0, 0.0]  # No lateral movement during takeoff
            takeoff_msg.yaw = 0.0  # Face forward
            self.trajectory_setpoint_pub.publish(takeoff_msg)
            self.get_logger().info("[DEBUG] Published takeoff trajectory setpoint")

            # Check if 5 seconds have passed since takeoff started
            current_time = self.get_clock().now()
            elapsed_time = (current_time - self.start_time).nanoseconds / 1e9  # Convert nanoseconds to seconds
            if elapsed_time > 5.0:
                self.takeoff_completed = True
                self.get_logger().info("[DEBUG] Takeoff completed, preparing to land")

        # Send landing setpoint if takeoff is completed
        if self.takeoff_completed:
            land_msg = TrajectorySetpoint()
            land_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            land_msg.position = [0.0, 0.0, 0.0]  # Land back at ground level (Z = 0)
            land_msg.velocity = [0.0, 0.0, 0.0]
            land_msg.yaw = 0.0  # Maintain same yaw
            self.trajectory_setpoint_pub.publish(land_msg)
            self.get_logger().info("[DEBUG] Published landing trajectory setpoint")

    def set_offboard_mode(self):
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False

        self.offboard_mode_pub.publish(offboard_msg)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

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
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)
        self.get_logger().info(f"[DEBUG] Sent vehicle command: {command} with params {param1}, {param2}")


def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
