#!/usr/bin/env python
# Modified version with enhanced debugging and a different strategy for switching to offboard mode,
# arming the drone, and moving the motors.

import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleCommand
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool


class DirectOffboard(Node):

    def __init__(self):
        super().__init__('direct_offboard')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscriptions
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)
        
        self.offboard_velocity_sub = self.create_subscription(
            Twist,
            '/offboard_velocity_cmd',
            self.offboard_velocity_callback,
            qos_profile)
        
        self.my_bool_sub = self.create_subscription(
            Bool,
            '/arm_message',
            self.arm_message_callback,
            qos_profile)

        # Create publishers
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)

        # Command loop timer
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        # Internal state variables
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_DISARMED
        self.offboardMode = False
        self.arm_message = False
        self.has_armed = False
        self.velocity = Vector3()

    def arm_message_callback(self, msg):
        self.arm_message = msg.data
        self.get_logger().info(f"[DEBUG] Received arm message: {self.arm_message}")

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.get_logger().info(f"[DEBUG] NAV_STATE: {self.nav_state}, ARM_STATE: {self.arm_state}")

    def offboard_velocity_callback(self, msg):
        # This callback is for receiving velocity commands if needed
        self.get_logger().info(f"[DEBUG] Received velocity command: {msg}")

    def cmdloop_callback(self):
        # Step 1: Switch to Offboard Mode if requested and not already in offboard mode
        if self.arm_message and not self.offboardMode:
            self.get_logger().info("[DEBUG] Attempting to switch to offboard mode")
            self.state_offboard()

        # Step 2: If in Offboard mode, attempt to arm
        if self.offboardMode and self.arm_state != VehicleStatus.ARMING_STATE_ARMED and not self.has_armed:
            self.get_logger().info("[DEBUG] Attempting to arm the drone")
            self.arm()

        # Step 3: If armed, send a small movement command
        if self.arm_state == VehicleStatus.ARMING_STATE_ARMED and not self.has_armed:
            self.get_logger().info("[DEBUG] Drone is armed, sending movement command")
            self.move_motors()
            self.has_armed = True

    def state_offboard(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
        self.offboardMode = True
        self.get_logger().info("[DEBUG] Offboard mode command sent")

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("[DEBUG] Arm command sent")

    def move_motors(self):
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        trajectory_msg.velocity[0] = 0.5  # Slight forward movement (in meters/second)
        trajectory_msg.velocity[1] = 0.0
        trajectory_msg.velocity[2] = 0.0
        trajectory_msg.yaw = 0.0
        self.publisher_trajectory.publish(trajectory_msg)
        self.get_logger().info("[DEBUG] Sent small velocity command to move motors")

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
        self.vehicle_command_publisher_.publish(msg)
        self.get_logger().info(f"[DEBUG] Published vehicle command: {command} with params {param1}, {param2}")

def main(args=None):
    rclpy.init(args=args)
    offboard_control = DirectOffboard()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
