#!/usr/bin/env python

### PROBADO , SI FUNCIOINA ###

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleAttitudeSetpoint, VehicleCommand, OffboardControlMode
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
        self.timer = self.create_timer(0.05, self.publish_attitude_setpoint)
        self.offboard_set = False
        self.armed = False

    def publish_attitude_setpoint(self):
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

        # Send attitude setpoint to maintain level flight with slight pitch forward
        attitude_msg = VehicleAttitudeSetpoint()
        attitude_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        attitude_msg.roll_body = 0.0  # Keep level roll
        attitude_msg.pitch_body = math.radians(5.0)  # Slight forward pitch (positive value tilts the nose down)
        attitude_msg.yaw_body = 0.0  # Keep facing forward
        attitude_msg.thrust_body = [0.0, 0.0, -0.3]  # Adjust this to control thrust; negative value for lift

        self.attitude_setpoint_pub.publish(attitude_msg)
        self.get_logger().info("[DEBUG] Published attitude setpoint")

    def set_offboard_mode(self):
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = False
        offboard_msg.velocity = False
        offboard_msg.acceleration = False#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleAttitudeSetpoint, VehicleCommand, OffboardControlMode
from rclpy.clock import Clock
import math
import threading

class OffboardControl(Node):

    def __init__(self):
        super().__init__('offboard_control')

        # Create publishers
        self.attitude_setpoint_pub = self.create_publisher(
            VehicleAttitudeSetpoint,
            '/fmu/in/vehicle_attitude_setpoint',
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
        self.timer = self.create_timer(0.05, self.publish_attitude_setpoint)
        self.offboard_set = False
        self.armed = False
        self.takeoff_completed = False
        self.landing = False
        self.start_time = None

        # Start a thread to listen for a landing command
        threading.Thread(target=self.listen_for_landing).start()

    def publish_attitude_setpoint(self):
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

        # Send attitude setpoint for takeoff and hover
        attitude_msg = VehicleAttitudeSetpoint()
        attitude_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        attitude_msg.roll_body = 0.0  # Keep level roll
        attitude_msg.pitch_body = 0.0  # Keep level pitch
        attitude_msg.yaw_body = 0.0  # Maintain current yaw

        if not self.landing:
            # Thrust to maintain hover or ascend initially
            attitude_msg.thrust_body = [0.0, 0.0, -0.5] if not self.takeoff_completed else [0.0, 0.0, -0.3]
            self.get_logger().info("[DEBUG] Published attitude setpoint for hover")

            # Check if 5 seconds have passed since takeoff started
            current_time = self.get_clock().now()
            elapsed_time = (current_time - self.start_time).nanoseconds / 1e9  # Convert nanoseconds to seconds
            if elapsed_time > 5.0 and not self.takeoff_completed:
                self.takeoff_completed = True
                self.get_logger().info("[DEBUG] Takeoff completed, waiting for landing command")
        else:
            # Reduce thrust for landing
            attitude_msg.thrust_body = [0.0, 0.0, 0.1]  # Positive thrust to descend
            self.get_logger().info("[DEBUG] Published attitude setpoint for landing")

        self.attitude_setpoint_pub.publish(attitude_msg)

    def set_offboard_mode(self):
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position = False
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = True  # Control based on attitude
        offboard_msg.body_rate = False

        self.offboard_mode_pub.publish(offboard_msg)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("[DEBUG] Arm command sent")

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("[DEBUG] Disarm command sent")

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

    def listen_for_landing(self):
        # Wait for the user to press Enter to start landing
        input("Press Enter to land the drone...")
        self.landing = True
        self.get_logger().info("[DEBUG] Landing initiated")

def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

        offboard_msg.attitude = True  # Control based on attitude
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
