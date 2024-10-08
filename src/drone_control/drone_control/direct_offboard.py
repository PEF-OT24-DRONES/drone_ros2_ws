#!/usr/bin/env python
# This is a modified version of the offboard_control.py script that uses the PX4 Offboard Control API
# to control the drone in offboard mode.

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
from math import pi
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
        
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
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

        # Arm timer
        arm_timer_period = 0.1  # seconds
        self.arm_timer_ = self.create_timer(arm_timer_period, self.arm_timer_callback)

        # Command loop timer
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        # Internal state variables
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_DISARMED
        self.velocity = Vector3()
        self.yaw = 0.0
        self.trueYaw = 0.0
        self.offboardMode = False
        self.arm_message = False
        self.current_state = "IDLE"
        self.last_state = self.current_state


    def arm_message_callback(self, msg):
        self.arm_message = msg.data
        self.get_logger().info(f"Arm Message: {self.arm_message}")

    def arm_timer_callback(self):
        # State machine
        match self.current_state:
            case "IDLE":
                if self.arm_message:
                    self.current_state = "OFFBOARD"
                    self.get_logger().info("Switching to Offboard Mode")

            case "OFFBOARD":
                if self.arm_state != VehicleStatus.ARMING_STATE_ARMED:
                    self.state_offboard()
                    self.get_logger().info("Setting Offboard Mode")
                else:
                    self.current_state = "ARMING"
                    self.get_logger().info("Switching to Arming")

            case "ARMING":
                if self.arm_state != VehicleStatus.ARMING_STATE_ARMED:
                    self.arm()  # Send arm command
                else:
                    self.current_state = "MOVING"
                    self.get_logger().info("Drone is armed, starting to move motors")

            case "MOVING":
                self.move_motors()
                self.get_logger().info("Moving motors slightly")

        # Log state changes
        if self.last_state != self.current_state:
            self.last_state = self.current_state
            self.get_logger().info(f"State: {self.current_state}")

    def state_offboard(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
        self.offboardMode = True

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command sent")

    def move_motors(self):
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        trajectory_msg.velocity[0] = 0.5  # Slight forward movement (in meters/second)
        trajectory_msg.velocity[1] = 0.0
        trajectory_msg.velocity[2] = 0.0
        trajectory_msg.yaw = 0.0
        self.publisher_trajectory.publish(trajectory_msg)
        self.get_logger().info("Sent small velocity command to move motors")

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

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state

    def offboard_velocity_callback(self, msg):
        self.velocity.x = -msg.linear.y
        self.velocity.y = msg.linear.x
        self.velocity.z = -msg.linear.z
        self.yaw = msg.angular.z

    def attitude_callback(self, msg):
        orientation_q = msg.q
        self.trueYaw = -np.arctan2(
            2.0 * (orientation_q[3] * orientation_q[0] + orientation_q[1] * orientation_q[2]),
            1.0 - 2.0 * (orientation_q[0] ** 2 + orientation_q[1] ** 2)
        )

    def cmdloop_callback(self):
        if self.offboardMode:
            offboard_msg = OffboardControlMode()
            offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
            offboard_msg.velocity = True
            self.publisher_offboard_mode.publish(offboard_msg)

def main(args=None):
    rclpy.init(args=args)
    offboard_control = DirectOffboard()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
