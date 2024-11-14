#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time

from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import AttitudeTarget, State
from std_msgs.msg import Header

class MavrosNoGPSSimpleNode(Node):
    def __init__(self):
        super().__init__('mavros_nogps_simple_node')

        # Estado actual del dron
        self.state = State()
        self.state_subscriber = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )

        # Publicador de actitud
        self.attitude_publisher = self.create_publisher(
            AttitudeTarget,
            '/mavros/setpoint_raw/attitude',
            10
        )

        # Cliente para armar/desarmar el dron
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio /mavros/cmd/arming...')

        # Cliente para cambiar el modo de vuelo
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio /mavros/set_mode...')

        # Esperar a que el dron esté conectado
        self.get_logger().info('Esperando conexión con el FCU...')
        while rclpy.ok() and not self.state.connected:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info('Conexión establecida con el FCU')

        # Iniciar la rutina de despegue y aterrizaje
        self.run_test()

    def state_callback(self, msg):
        self.state = msg

    def arm_and_set_mode(self):
        # Armar el dron
        arm_cmd = CommandBool.Request()
        arm_cmd.value = True
        future = self.arming_client.call_async(arm_cmd)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info('Dron armado con éxito')
        else:
            self.get_logger().error('Error al armar el dron')
            return False

        # Cambiar al modo GUIDED_NOGPS
        mode_cmd = SetMode.Request()
        mode_cmd.custom_mode = 'GUIDED_NOGPS'
        future = self.set_mode_client.call_async(mode_cmd)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().mode_sent:
            self.get_logger().info('Modo de vuelo cambiado a GUIDED_NOGPS')
            return True
        else:
            self.get_logger().error('Error al cambiar el modo de vuelo')
            return False

    def send_motor_commands(self):
        self.get_logger().info('Enviando comandos a los motores...')
        # Crear mensaje de actitud para activar los motores
        attitude = AttitudeTarget()
        attitude.header = Header()
        attitude.header.frame_id = 'base_link'
        attitude.type_mask = AttitudeTarget.IGNORE_ATTITUDE  # 128
        attitude.orientation.x = 0.0
        attitude.orientation.y = 0.0
        attitude.orientation.z = 0.0
        attitude.orientation.w = 1.0
        attitude.thrust = 0.6  # Ajusta este valor para controlar la potencia de los motores (0.0 a 1.0)

        # Publicar comandos durante cierto tiempo
        start_time = time.time()
        rate = self.create_rate(10)  # 10 Hz

        while time.time() - start_time < 5.0:
            attitude.header.stamp = self.get_clock().now().to_msg()
            self.attitude_publisher.publish(attitude)
            rate.sleep()

        self.get_logger().info('Comandos enviados')

    def disarm(self):
        # Desarmar el dron
        arm_cmd = CommandBool.Request()
        arm_cmd.value = False
        future = self.arming_client.call_async(arm_cmd)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info('Dron desarmado con éxito')
        else:
            self.get_logger().error('Error al desarmar el dron')

    def run_test(self):
        # Enviar algunos mensajes de actitud antes de armar y cambiar de modo
        self.get_logger().info('Enviando mensajes iniciales...')
        initial_attitude = AttitudeTarget()
        initial_attitude.header = Header()
        initial_attitude.header.frame_id = 'base_link'
        initial_attitude.type_mask = AttitudeTarget.IGNORE_ATTITUDE  # 128
        initial_attitude.orientation.x = 0.0
        initial_attitude.orientation.y = 0.0
        initial_attitude.orientation.z = 0.0
        initial_attitude.orientation.w = 1.0
        initial_attitude.thrust = 0.0

        start_time = time.time()
        rate = self.create_rate(10)  # 10 Hz
        while time.time() - start_time < 2.0:
            initial_attitude.header.stamp = self.get_clock().now().to_msg()
            self.attitude_publisher.publish(initial_attitude)
            rate.sleep()

        # Armar y configurar el modo
        if not self.arm_and_set_mode():
            self.get_logger().error('No se pudo armar y configurar el modo de vuelo')
            return

        # Enviar comandos para mover los motores
        self.send_motor_commands()

        # Desarmar el dron
        self.disarm()

def main(args=None):
    rclpy.init(args=args)
    node = MavrosNoGPSSimpleNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
