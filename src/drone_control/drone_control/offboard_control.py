import rclpy
from rclpy.node import Node
from px4_msgs.msg import (
    VehicleCommand,
    OffboardControlMode,
    TrajectorySetpoint,
    TimesyncStatus,
)
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import time

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')
        
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            qos)

        # Subscription to TimesyncStatus with adjusted QoS
        self.timesync_sub = self.create_subscription(
            TimesyncStatus,
            '/fmu/out/timesync_status',
            self.timesync_callback,
            qos)

        self.timestamp = 0

        # Timer
        timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.offboard_setpoint_counter = 0

    def timesync_callback(self, msg):
        # Usar el timestamp proporcionado por TimesyncStatus
        self.timestamp = msg.timestamp

    def timer_callback(self):
        if self.timestamp == 0:
            return  # Esperar a tener un timestamp válido

        # Publicar los mensajes OffboardControlMode y TrajectorySetpoint
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        # Solo intenta cambiar al modo Offboard y armar después de 50 mensajes enviados
        if self.offboard_setpoint_counter == 150:
            # Después de publicar suficientes mensajes, cambiar al modo Offboard
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self.get_logger().info('Comando para cambiar a modo Offboard enviado')

        # Intenta armar después de 60 mensajes, para asegurar que el modo Offboard esté activo
        if self.offboard_setpoint_counter == 190:
            self.arm()
            self.get_logger().info('Comando de armado enviado')

        self.offboard_setpoint_counter += 1

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.timestamp
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp
        msg.position = [0.0, 0.0, -5.0]  # Mantener posición actual en XY, descender a -5m en Z
        msg.velocity = [0.0, 0.0, 0.0]  # Sin movimiento adicional
        msg.yaw = 0.0  # Orientación en yaw
        self.trajectory_setpoint_publisher.publish(msg)

    def arm(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def disarm(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.timestamp
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()

    try:
        rclpy.spin(offboard_control)
    except KeyboardInterrupt:
        pass

    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
