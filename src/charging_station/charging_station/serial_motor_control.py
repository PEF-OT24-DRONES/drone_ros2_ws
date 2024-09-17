import rclpy
from rclpy.node import Node
import serial
import pyserial

class SerialMotorControl(Node):
    def __init__(self):
        super().__init__('serial_motor_control')
        self.get_logger().info('Serial Motor Control Node has been started.')
        
        # Initialize serial connection
        self.serial_port = '/dev/ttyUSB0'  # Update with your serial port
        self.baud_rate = 9600  # Update with your baud rate
        self.serial_connection = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
        
        # Example: Create a timer to send commands periodically
        self.timer_period = 1/2  # seconds
        self.timer = self.create_timer(self.timer_period, self.send_motor_command)
        
    def send_motor_command(self):
        command = '1'  # Example command
        self.serial_connection.write(command.encode())
        self.get_logger().info(f'Sent command: {command.strip()}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialMotorControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()