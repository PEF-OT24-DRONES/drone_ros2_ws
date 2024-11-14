#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time

from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

class MavrosNode(Node):
    def __init__(self):
        super().__init__('mavros_node')
        
        # Servicio para armar/desarmar el dron
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio /mavros/cmd/arming...')
        
        # Servicio para cambiar el modo de vuelo
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio /mavros/set_mode...')
        
        # Publicador para enviar comandos de posición
        self.pose_publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        
        # Tiempo de espera para asegurar la conexión
        self.get_logger().info('Esperando 5 segundos antes de iniciar...')
        time.sleep(5)
        
        # Iniciar la secuencia de vuelo
        self.arm_drone()
        self.takeoff()
        self.land_drone()
        
    def arm_drone(self):
        # Crear mensaje de solicitud para armar el dron
        arm_cmd = CommandBool.Request()
        arm_cmd.value = True
        
        # Llamar al servicio para armar el dron
        future = self.arming_client.call_async(arm_cmd)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.get_logger().info('Dron armado con éxito')
        else:
            self.get_logger().error('Error al armar el dron')
            return

    def takeoff(self):
        # Cambiar al modo OFFBOARD o GUIDED
        mode_cmd = SetMode.Request()
        mode_cmd.custom_mode = 'GUIDED'
        
        future = self.set_mode_client.call_async(mode_cmd)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().mode_sent:
            self.get_logger().info('Modo de vuelo cambiado a OFFBOARD')
        else:
            self.get_logger().error('Error al cambiar el modo de vuelo')
            return
        
        # Enviar comandos de posición para ascender a 2 metros
        self.get_logger().info('Despegando y ascendiendo a 2 metros...')
        pose = PoseStamped()
        pose.header = Header()
        pose.header.frame_id = 'map'
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 2.0
        pose.pose.orientation.w = 1.0
        
        # Publicar la posición deseada durante cierto tiempo
        start_time = time.time()
        while time.time() - start_time < 10.0:
            pose.header.stamp = self.get_clock().now().to_msg()
            self.pose_publisher.publish(pose)
            time.sleep(0.1)  # Publicar a 10 Hz

    def land_drone(self):
        # Comandar el aterrizaje cambiando al modo LAND
        mode_cmd = SetMode.Request()
        mode_cmd.custom_mode = 'AUTO.LAND'
        
        future = self.set_mode_client.call_async(mode_cmd)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().mode_sent:
            self.get_logger().info('Comenzando el aterrizaje...')
        else:
            self.get_logger().error('Error al cambiar al modo LAND')
            return
        
        # Esperar a que el dron aterrice
        time.sleep(10)
        
        # Desarmar el dron
        arm_cmd = CommandBool.Request()
        arm_cmd.value = False
        
        future = self.arming_client.call_async(arm_cmd)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.get_logger().info('Dron desarmado con éxito')
        else:
            self.get_logger().error('Error al desarmar el dron')

def main(args=None):
    rclpy.init(args=args)
    node = MavrosNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
