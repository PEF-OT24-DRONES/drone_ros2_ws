#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import StatusText
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class RCControlNode(Node):
    def __init__(self):
        super().__init__('rc_control')

        # Crear un perfil de QoS que sea compatible con el publicador de MAVROS
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Suscriptor al tópico de mensajes de estado del sistema con el perfil de QoS ajustado
        self.statustext_subscriber = self.create_subscription(
            StatusText,
            '/mavros/statustext/recv',
            self.status_callback,
            qos_profile
        )

        # Publicador para enviar el comando basado en el estado del relé
        self.command_publisher = self.create_publisher(Int32, 'rc_command', 10)

        self.get_logger().info('Nodo rc_control iniciado y esperando mensajes de estado del relé...')

    def status_callback(self, msg: StatusText):
        # Revisar si el mensaje contiene información sobre el estado del relé
        message_text = msg.text
        command = Int32()

        if 'RC6: Relay2 HIGH' in message_text:
            command.data = 1
            self.get_logger().info('Relé encendido (HIGH), enviando comando 1')
        elif 'RC6: Relay2 LOW' in message_text:
            command.data = 2
            self.get_logger().info('Relé apagado (LOW), enviando comando 2')
        else:
            # Ignorar otros mensajes
            return

        # Publicar el comando
        self.command_publisher.publish(command)

def main(args=None):
    rclpy.init(args=args)
    node = RCControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
