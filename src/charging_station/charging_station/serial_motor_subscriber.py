import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SerialMotorSubscriber(Node):
    def __init__(self):
        super().__init__('serial_motor_subscriber')
        self.subscription = self.create_subscription(
            String,
            'serial_motor_control',
            self.listener_callback,
            10)
        self.subscription  # evita que Python elimine la suscripción
        self.get_logger().info('Serial Motor Subscriber node initiated')
        # Configura la conexión serial
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 9600)  # Ajusta el puerto según sea necesario
        except serial.SerialException:
            self.get_logger().error('No se pudo abrir el puerto serial')
            exit(1)
        
        

    def listener_callback(self, msg):
        self.get_logger().info(f'Recibido: "{msg.data}"')
        if msg.data == '1':
            self.ser.write(b'1')  # Envía el byte '1' al Arduino
            self.get_logger().info('Enviando "1" al Arduino. Adelante')
        elif msg.data == '2':
            self.ser.write(b'2')  # Envía el byte '2' al Arduino
            self.get_logger().info('Enviando "2" al Arduino. Reversa')
        elif msg.data == '0':
            self.ser.write(b'0')  # Envía el byte '0' al Arduino
            self.get_logger().info('Enviando "0" al Arduino. Detener')
        else:
            self.get_logger().info('Comando desconocido, no se envía nada.')

def main(args=None):
    rclpy.init(args=args)
    serial_subscriber = SerialMotorSubscriber()
    rclpy.spin(serial_subscriber)
    serial_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
