#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State, PositionTarget
from std_msgs.msg import String
import math
from rclpy.qos import QoSPresetProfiles

class OffboardControlNode(Node):
    """Node for controlling the drone in offboard mode."""

    def __init__(self):
        super().__init__('offboard_control')

        qos_profile = QoSPresetProfiles.SENSOR_DATA.value

        # Publishers and Subscribers
        self.setpoint_publisher = self.create_publisher(
            PositionTarget, '/mavros/setpoint_raw/local', qos_profile)
        self.state_subscriber = self.create_subscription(
            State, '/mavros/state', self.state_callback, qos_profile)
        self.keyboard_subscriber = self.create_subscription(
            String, "keyboard", self.keyboard_callback, 10)

        # Service Clients
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        # Variables
        self.current_state = State()
        self.keyboard = String()
        self.keyboard.data = "none"
        self.v = 1.0  # Velocity magnitude
        self.initializing = False
        self.init_setpoint_count = 0

        # Timer to publish setpoints at 20Hz
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("OffboardControlNode has been started.")

    def state_callback(self, state):
        self.current_state = state

    def keyboard_callback(self, keyboard):
        self.keyboard = keyboard
        self.get_logger().info(f"Received keyboard command: {self.keyboard.data}")

        if self.keyboard.data == "offboard":
            self.initializing = True  # Start sending initial setpoints
        elif self.keyboard.data == "arm":
            self.arm(True)
        elif self.keyboard.data == "disarm":
            self.arm(False)
        elif self.keyboard.data == "takeoff":
            self.takeoff()
        elif self.keyboard.data == "land":
            self.land()
        elif self.keyboard.data == "return":
            self.return_to_launch()
        # Movement commands are handled in timer_callback

    def set_offboard_mode(self):
        if not self.set_mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service /mavros/set_mode not available")
            return

        req = SetMode.Request()
        req.custom_mode = "OFFBOARD"
        future = self.set_mode_client.call_async(req)
        future.add_done_callback(self.set_mode_response_callback)

    def set_mode_response_callback(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info("OFFBOARD mode set successfully.")
            else:
                self.get_logger().error("Failed to set OFFBOARD mode.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def arm(self, arm_status):
        if not self.arm_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service /mavros/cmd/arming not available")
            return

        req = CommandBool.Request()
        req.value = arm_status
        future = self.arm_client.call_async(req)
        future.add_done_callback(self.arm_response_callback)

    def arm_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Arm status changed successfully.")
            else:
                self.get_logger().error("Failed to change arm status.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def timer_callback(self):
        # Si estamos inicializando, enviamos setpoints iniciales
        if self.initializing:
            if self.init_setpoint_count < 100:
                self.publish_stabilizing_setpoint()
                self.init_setpoint_count += 1
                self.get_logger().debug("Sending initial setpoints...")
            else:
                self.initializing = False
                self.init_setpoint_count = 0
                self.set_offboard_mode()  # Cambiamos a modo Offboard después de enviar los setpoints iniciales
        else:
            # Siempre publica setpoints cuando está en modo OFFBOARD
            if self.current_state.mode == "OFFBOARD":
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
                else:
                    # Si no hay comando de movimiento, mantén la posición
                    self.publish_stabilizing_setpoint()
            else:
                # Si no está en modo OFFBOARD, envía setpoints para prepararlo
                self.publish_stabilizing_setpoint()

    def publish_stabilizing_setpoint(self):
        msg = PositionTarget()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = (
            PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        msg.velocity.x = 0.0
        msg.velocity.y = 0.0
        msg.velocity.z = 0.0
        msg.yaw = 0.0
        self.setpoint_publisher.publish(msg)

    def publish_movement_setpoint(self, x, y, z, yaw):
        msg = PositionTarget()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = (
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        msg.velocity.x = x
        msg.velocity.y = y
        msg.velocity.z = z
        msg.yaw = yaw
        self.setpoint_publisher.publish(msg)

    def takeoff(self):
        # Implementa la lógica de despegue si es necesario
        pass

    def land(self):
        if not self.set_mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service /mavros/set_mode not available")
            return

        req = SetMode.Request()
        req.custom_mode = "AUTO.LAND"

        future = self.set_mode_client.call_async(req)
        future.add_done_callback(self.land_response_callback)

    def land_response_callback(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info("Landing command sent successfully.")
            else:
                self.get_logger().error("Failed to send landing command.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def return_to_launch(self):
        if not self.set_mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service /mavros/set_mode not available")
            return

        req = SetMode.Request()
        req.custom_mode = "AUTO.RTL"

        future = self.set_mode_client.call_async(req)
        future.add_done_callback(self.rtl_response_callback)

    def rtl_response_callback(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info("Return to Launch command sent successfully.")
            else:
                self.get_logger().error("Failed to send RTL command.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControlNode()
    try:
        rclpy.spin(offboard_control)
    except KeyboardInterrupt:
        pass
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
