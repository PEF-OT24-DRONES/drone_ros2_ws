import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SerialMotorPublisher(Node):
    def __init__(self):
        super().__init__('serial_motor_publisher')
        self.publisher_ = self.create_publisher(String, 'serial_motor_control', 10)
        self.get_logger().info('Nodo publicador iniciado. Presiona 0, 1 o 2 para enviar comandos.')

    def run(self):
        while rclpy.ok():
            user_input = input('Ingresa 0, 1 o 2: ')
            if user_input in ['0', '1', '2']:
                msg = String()
                msg.data = user_input
                self.publisher_.publish(msg)
                self.get_logger().info(f'Publicando: "{msg.data}"')
            else:
                self.get_logger().info('Entrada no válida, ignorando.')

def main(args=None):
    rclpy.init(args=args)
    command_publisher = SerialMotorPublisher()
    command_publisher.run()
    command_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
