#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint
import time

class OffboardControlNode(Node):
    def __init__(self):
        super().__init__('offboard_control_node')
        self.subscription = self.create_subscription(
            Twist,
            '/drone/cmd_vel',
            self.cmd_vel_callback,
            10)

        # Publishers for PX4 Offboard control
        self.offboard_control_mode_publisher_ = self.create_publisher(
            OffboardControlMode, "/fmu/offboard_control_mode/in", 10)
        self.trajectory_setpoint_publisher_ = self.create_publisher(
            TrajectorySetpoint, "/fmu/trajectory_setpoint/in", 10)
        self.vehicle_command_publisher_ = self.create_publisher(
            VehicleCommand, "/fmu/vehicle_command/in", 10)

        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.offboard_setpoint_counter_ = 0
        self.twist = Twist()

    def cmd_vel_callback(self, msg):
        self.twist = msg

    def timer_callback(self):
        # Arm the vehicle and switch to offboard mode after sending a few setpoints
        if self.offboard_setpoint_counter_ == 10:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.get_logger().info("Offboard mode activated")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.get_logger().info("Vehicle armed")

        # Publish OffboardControlMode
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = self.get_timestamp()
        offboard_msg.position = False
        offboard_msg.velocity = True
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        self.offboard_control_mode_publisher_.publish(offboard_msg)

        # Publish TrajectorySetpoint
        traj_msg = TrajectorySetpoint()
        traj_msg.timestamp = self.get_timestamp()
        traj_msg.vx = self.twist.linear.x
        traj_msg.vy = self.twist.linear.y
        traj_msg.vz = self.twist.linear.z
        traj_msg.yaw = self.twist.angular.z
        self.trajectory_setpoint_publisher_.publish(traj_msg)

        self.offboard_setpoint_counter_ += 1

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_timestamp()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_publisher_.publish(msg)

    def get_timestamp(self):
        return int(self.get_clock().now().nanoseconds / 1000)  # PX4 timestamp is in microseconds

def main(args=None):
    rclpy.init(args=args)
    node = OffboardControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
