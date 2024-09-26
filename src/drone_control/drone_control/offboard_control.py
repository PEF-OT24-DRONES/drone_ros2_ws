#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State, PositionTarget
from std_msgs.msg import String
import math
from rclpy.qos import QoSPresetProfiles


class OffboardControlNode(Node):
    """Node for controlling the drone with the keyboard in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control')

        qos_profile = QoSPresetProfiles.SENSOR_DATA.value

        # Create publishers
        self.setpoint_publisher = self.create_publisher(
            PositionTarget, '/mavros/setpoint_raw/local', qos_profile)
        self.state_subscriber = self.create_subscription(
            State, '/mavros/state', self.state_callback, qos_profile)

        # Create subscribers for keyboard commands
        self.keyboard_subscriber = self.create_subscription(
            String, "keyboard", self.keyboard_callback, 10)

        # Create service clients for arming and setting mode
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        # Initialize variables
        self.keyboard = String()
        self.current_state = None
        self.v = 1.0  # Velocity magnitude

        # Flag to indicate when the setpoints have been initialized
        self.initialized = False

        # Create a timer to publish control commands at 20Hz (0.05 seconds interval)
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("OffboardControlNode has been started.")

    def state_callback(self, state):
        """Callback function for the /mavros/state topic."""
        self.current_state = state

    def keyboard_callback(self, keyboard):
        """Callback function for keyboard input."""
        self.keyboard = keyboard
        self.get_logger().info(f"Received keyboard command: {self.keyboard.data}")

        if self.keyboard.data == "offboard":
            self.set_offboard_mode()
        elif self.keyboard.data == "takeoff":
            self.takeoff()
        elif self.keyboard.data == "land":
            self.land()
        elif self.keyboard.data == "arm":
            self.arm(True)
        elif self.keyboard.data == "disarm":
            self.arm(False)
        elif self.keyboard.data == "return":
            self.return_to_launch()

    def send_initial_setpoints(self):
        """Send initial setpoints to allow the drone to transition to Offboard mode."""
        if not self.initialized:
            for _ in range(100):  # Send 100 setpoints
                self.publish_stabilizing_setpoint()
                self.get_logger().info("Sending initial setpoints...")
                rclpy.spin_once(self, timeout_sec=0.05)  # Delay between setpoints
            self.initialized = True  # Set flag after initial setpoints are sent

    def publish_stabilizing_setpoint(self):
        """Publish a stabilizing setpoint to hold the drone's position."""
        msg = PositionTarget()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ \
                        | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ \
                        | PositionTarget.IGNORE_YAW_RATE
        msg.velocity.x = 0.0
        msg.velocity.y = 0.0
        msg.velocity.z = 0.0
        msg.yaw = 0.0  # No rotation
        self.setpoint_publisher.publish(msg)
        self.get_logger().debug("Published stabilizing setpoint (holding position).")

    def publish_movement_setpoint(self, x, y, z, yaw):
        """Publish the trajectory setpoint for movement."""
        msg = PositionTarget()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ \
                        | PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ \
                        | PositionTarget.IGNORE_YAW_RATE
        msg.velocity.x = x
        msg.velocity.y = y
        msg.velocity.z = z
        msg.yaw = yaw
        self.setpoint_publisher.publish(msg)
        self.get_logger().info(f"Published movement setpoint: velocity=({x}, {y}, {z}), yawspeed={yaw}")

    def set_offboard_mode(self):
        """Switch to OFFBOARD mode using the service call."""
        if not self.set_mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service /mavros/set_mode not available")
            return

        # Ensure that initial setpoints are sent before attempting to switch modes
        self.send_initial_setpoints()

        req = SetMode.Request()
        req.custom_mode = "OFFBOARD"

        future = self.set_mode_client.call_async(req)
        future.add_done_callback(self.set_mode_response_callback)

    def set_mode_response_callback(self, future):
        """Handle the response from the service call."""
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info("OFFBOARD mode set successfully.")
            else:
                self.get_logger().error("Failed to set OFFBOARD mode.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def arm(self, arm_status):
        """Arm or disarm the drone."""
        if not self.arm_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service /mavros/cmd/arming not available")
            return

        # Before arming, publish at least one setpoint to ensure the FCU accepts commands
        self.publish_stabilizing_setpoint()

        req = CommandBool.Request()
        req.value = arm_status

        future = self.arm_client.call_async(req)
        future.add_done_callback(self.arm_response_callback)

    def arm_response_callback(self, future):
        """Handle the response from the arming service."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Arm status changed successfully.")
            else:
                self.get_logger().error(f"Failed to change arm status.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def land(self):
        """Send a command to land the drone."""
        if not self.set_mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service /mavros/set_mode not available")
            return

        req = SetMode.Request()
        req.custom_mode = "AUTO.LAND"

        future = self.set_mode_client.call_async(req)
        future.add_done_callback(self.land_response_callback)

    def land_response_callback(self, future):
        """Handle the response from the land service call."""
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info("Landing command sent successfully.")
            else:
                self.get_logger().error("Failed to send landing command.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def takeoff(self):
        """Send a command to take off."""
        if not self.set_mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service /mavros/set_mode not available")
            return

        req = SetMode.Request()
        req.custom_mode = "AUTO.TAKEOFF"

        future = self.set_mode_client.call_async(req)
        future.add_done_callback(self.takeoff_response_callback)

    def takeoff_response_callback(self, future):
        """Handle the response from the takeoff service call."""
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info("Takeoff command sent successfully.")
            else:
                self.get_logger().error("Failed to send takeoff command.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def return_to_launch(self):
        """Send a command to return to launch (RTL)."""
        if not self.set_mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service /mavros/set_mode not available")
            return

        req = SetMode.Request()
        req.custom_mode = "AUTO.RTL"

        future = self.set_mode_client.call_async(req)
        future.add_done_callback(self.rtl_response_callback)

    def rtl_response_callback(self, future):
        """Handle the response from the RTL service call."""
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info("Return to Launch command sent successfully.")
            else:
                self.get_logger().error("Failed to send RTL command.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def timer_callback(self) -> None:
        """This timer callback runs every 0.05 seconds to handle mode changes and setpoint publication."""
        
        # Publish setpointsHere is the rest of the correction incorporating the changes based on the PX4 ROS1 example to your original ROS2 offboard control code:

    def timer_callback(self) -> None:
        """This timer callback runs every 0.05 seconds to handle mode changes and setpoint publication."""
        
        # Publish setpoints at a consistent rate
        if self.current_state and self.current_state.mode == "OFFBOARD":
            self.publish_stabilizing_setpoint()  # Publish a setpoint to keep the drone stable

        # Handle commands from the keyboard
        if self.keyboard.data == "arm":
            self.arm(True)
        elif self.keyboard.data == "disarm":
            self.arm(False)

        # Movement commands
        if self.keyboard.data == "right":
            self.publish_movement_setpoint(self.v, 0.0, 0.0, 0.0)
        elif self.keyboard.data == "left":
            self.publish_movement_setpoint(-self.v, 0.0, 0.0, 0.0)
        elif self.keyboard.data == "front":
            self.publish_movement_setpoint(0.0, self.v, 0.0, 0.0)
        elif self.keyboard.data == "back":
            self.publish_movement_setpoint(0.0, -self.v, 0.0, 0.0)
        elif self.keyboard.data == "up":
            self.publish_movement_setpoint(0.0, 0.0, self.v, 0.0)
        elif self.keyboard.data == "down":
            self.publish_movement_setpoint(0.0, 0.0, -self.v, 0.0)
        elif self.keyboard.data == "rotate_right":
            self.publish_movement_setpoint(0.0, 0.0, 0.0, math.pi / 8)
        elif self.keyboard.data == "rotate_left":
            self.publish_movement_setpoint(0.0, 0.0, 0.0, -math.pi / 8)
        elif self.keyboard.data == "stop":
            self.publish_stabilizing_setpoint()  # Stop movement

        # Reset keyboard data after processing
        self.keyboard.data = "none"

def main(args=None) -> None:
    print('Starting offboard control script')
    rclpy.init(args=args)
    offboard_control = OffboardControlNode()
    try:
        rclpy.spin(offboard_control)
    except KeyboardInterrupt:
        pass
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
