import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class Takeoff(Node):
    """Node for controlling a drone using MAVROS in offboard mode."""

    def __init__(self) -> None:
        super().__init__('takeoff_land')

        # QoS profile for topics
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publisher for setpoint position (offboard control)
        self.local_position_publisher = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', qos_profile)

        # Create subscribers for MAVROS state and local position
        self.state_subscriber = self.create_subscription(
            State, '/mavros/state', self.state_callback, qos_profile)

        # Services for arming and setting flight mode
        self.arm_service = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_service = self.create_client(SetMode, '/mavros/set_mode')

        # Initialize variables
        self.current_state = State()
        self.target_pose = PoseStamped()
        self.mode = 0  # 0: Waiting to arm, 1: Takeoff, 2: Land

        # Publish setpoints at 20Hz (50ms)
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("Takeoff node started.")

    def state_callback(self, state_msg):
        """Callback for receiving the MAVROS state."""
        self.current_state = state_msg

    def send_arm_command(self, arm):
        """Send arming command to the drone."""
        arm_cmd = CommandBool.Request()
        arm_cmd.value = arm
        self.arm_service.call_async(arm_cmd)

    def set_offboard_mode(self):
        """Set flight mode to OFFBOARD."""
        set_mode_request = SetMode.Request()
        set_mode_request.custom_mode = "OFFBOARD"
        self.set_mode_service.call_async(set_mode_request)

    def timer_callback(self) -> None:
        """Publish setpoints and handle flight control logic."""
        if self.mode == 0 and self.current_state.mode != "OFFBOARD":
            self.get_logger().info("Setting OFFBOARD mode...")
            # Send a few position setpoints before switching to OFFBOARD mode
            self.target_pose.pose.position.x = 0.0
            self.target_pose.pose.position.y = 0.0
            self.target_pose.pose.position.z = 2.0  # Target altitude for takeoff
            self.local_position_publisher.publish(self.target_pose)
            
            # Set flight mode to OFFBOARD
            self.set_offboard_mode()

            # Try to arm the drone
            self.send_arm_command(True)
            self.mode = 1

        elif self.mode == 1 and self.current_state.armed:
            # Takeoff complete, maintaining altitude
            self.get_logger().info("Drone armed and taking off...")
            self.local_position_publisher.publish(self.target_pose)

        elif self.mode == 2:
            # Land the drone
            self.target_pose.pose.position.z = 0.0  # Set altitude to 0 to land
            self.local_position_publisher.publish(self.target_pose)
            self.get_logger().info("Landing...")

            if not self.current_state.armed:
                self.get_logger().info("Drone landed and disarmed.")
                self.mode = 3

    def on_shutdown(self):
        """Disarm the drone on shutdown."""
        self.send_arm_command(False)


def main(args=None) -> None:
    rclpy.init(args=args)
    takeoff_land = Takeoff()

    try:
        rclpy.spin(takeoff_land)
    except KeyboardInterrupt:
        takeoff_land.on_shutdown()
        takeoff_land.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
