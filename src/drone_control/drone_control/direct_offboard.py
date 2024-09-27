#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import SetMode, CommandBool
from rclpy.qos import QoSPresetProfiles

class OffboardTestNode(Node):
    def __init__(self):
        super().__init__('offboard_test_node')

        # Publishers and Subscribers
        qos_profile = QoSPresetProfiles.SENSOR_DATA.value
        self.state_subscriber = self.create_subscription(
            State, '/mavros/state', self.state_callback, qos_profile)
        self.setpoint_publisher = self.create_publisher(
            PositionTarget, '/mavros/setpoint_raw/local', qos_profile)

        # Service Clients
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        # Variables
        self.current_state = State()
        self.offboard_setpoint_counter = 0

        # Timer to publish setpoints at 20Hz
        self.setpoint_timer = self.create_timer(0.05, self.publish_setpoint)

        # Start the offboard control sequence
        self.offboard_started = False
        self.get_logger().info("OffboardTestNode has been started.")

        # Futures for service calls
        self.set_mode_future = None
        self.arm_future = None

    def state_callback(self, msg):
        self.current_state = msg
        self.get_logger().debug(f"Current state - Mode: {self.current_state.mode}, Armed: {self.current_state.armed}")

    def publish_setpoint(self):
        # Log the current state for debugging
        self.get_logger().debug(f"Publishing setpoint. Offboard started: {self.offboard_started}, Counter: {self.offboard_setpoint_counter}")

        if not self.offboard_started:
            # Send initial setpoints to allow transition to Offboard mode
            if self.offboard_setpoint_counter < 100:
                self.send_setpoint(0.0, 0.0, 0.0, 0.0)
                self.offboard_setpoint_counter += 1
                self.get_logger().debug(f"Sending initial setpoints... Count: {self.offboard_setpoint_counter}")
            elif self.set_mode_future is None:
                # After sending initial setpoints, initiate change to Offboard mode
                self.get_logger().info("Initial setpoints sent. Attempting to switch to Offboard mode.")
                self.change_to_offboard_mode()
            elif self.set_mode_future.done():
                # Process result of set_mode service call
                try:
                    response = self.set_mode_future.result()
                    if response.mode_sent:
                        self.get_logger().info("Offboard mode set successfully.")
                        self.get_logger().info("Attempting to arm the drone.")
                        self.arm_drone()
                    else:
                        self.get_logger().error("Failed to set Offboard mode.")
                    self.set_mode_future = None  # Reset future
                except Exception as e:
                    self.get_logger().error(f"SetMode service call failed: {e}")
                    self.set_mode_future = None  # Reset future
            elif self.arm_future and self.arm_future.done():
                # Process result of arm_drone service call
                try:
                    response = self.arm_future.result()
                    if response.success:
                        self.get_logger().info("Drone armed successfully.")
                        self.offboard_started = True
                        self.command_sent_time = self.get_clock().now()
                    else:
                        self.get_logger().error("Failed to arm the drone.")
                    self.arm_future = None  # Reset future
                except Exception as e:
                    self.get_logger().error(f"Arm service call failed: {e}")
                    self.arm_future = None  # Reset future
            else:
                # Continue publishing setpoints while waiting
                self.send_setpoint(0.0, 0.0, 0.0, 0.0)
        else:
            # Continue sending setpoints to keep the drone in Offboard mode
            # Send upward velocity command for a short time
            elapsed_time = (self.get_clock().now() - self.command_sent_time).nanoseconds / 1e9
            if elapsed_time < 2.0:
                # Send upward velocity command
                self.send_setpoint(0.0, 0.0, 0.5, 0.0)
                self.get_logger().info(f"Sending upward velocity command. Elapsed time: {elapsed_time:.2f}s")
            else:
                # Hold position after 2 seconds
                self.send_setpoint(0.0, 0.0, 0.0, 0.0)
                self.get_logger().info("Holding position.")

    def send_setpoint(self, vx, vy, vz, yaw_rate):
        msg = PositionTarget()
        msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
        msg.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW
        )
        msg.velocity.x = vx
        msg.velocity.y = vy
        msg.velocity.z = vz
        msg.yaw_rate = yaw_rate
        self.setpoint_publisher.publish(msg)
        self.get_logger().debug(f"Setpoint published: vx={vx}, vy={vy}, vz={vz}, yaw_rate={yaw_rate}")

    def change_to_offboard_mode(self):
        # Wait for the set_mode service to be available
        if not self.set_mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service /mavros/set_mode not available")
            return
        req = SetMode.Request()
        req.custom_mode = "OFFBOARD"
        self.set_mode_future = self.set_mode_client.call_async(req)

    def arm_drone(self):
        # Wait for the arming service to be available
        if not self.arm_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service /mavros/cmd/arming not available")
            return
        req = CommandBool.Request()
        req.value = True
        self.arm_future = self.arm_client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = OffboardTestNode()
    # Set logging level to DEBUG to see detailed logs
    rclpy.logging.set_logger_level('offboard_test_node', rclpy.logging.LoggingSeverity.DEBUG)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
