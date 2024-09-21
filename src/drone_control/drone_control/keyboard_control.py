#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import curses
from std_msgs.msg import String

class KeyboardControlNode(Node):

    def __init__(self):
        super().__init__("keyboard_control")

        self.screen = curses.initscr()
        curses.noecho()
        curses.cbreak()
        self.screen.keypad(True)

        # Publicador para comandos de vuelo
        self.keyboard_monitor_publisher = self.create_publisher(String, "keyboard", 10)

        # Publicador para comandos del motor
        self.motor_control_publisher = self.create_publisher(String, "serial_motor_control", 10)

        self.prev_char = None
        self.keyboard()

    def keyboard(self):
        try:
            while True:
                char = self.screen.getch()
                msg = String()

                # Modos de vuelo
                if char == ord('T'):
                    msg.data = "takeoff"
                    self.keyboard_monitor_publisher.publish(msg)
                elif char == ord('L'):
                    msg.data = "land"
                    self.keyboard_monitor_publisher.publish(msg)
                elif char == ord('A'):
                    msg.data = "arm"
                    self.keyboard_monitor_publisher.publish(msg)
                elif char == ord('O'):
                    msg.data = "offboard"
                    self.keyboard_monitor_publisher.publish(msg)
                elif char == ord('R'):
                    msg.data = "return"
                    self.keyboard_monitor_publisher.publish(msg)
                elif char == curses.KEY_DOWN:
                    msg.data = "back"
                    self.keyboard_monitor_publisher.publish(msg)
                elif char == curses.KEY_UP:
                    msg.data = "front"
                    self.keyboard_monitor_publisher.publish(msg)
                elif char == curses.KEY_RIGHT:
                    msg.data = "right"
                    self.keyboard_monitor_publisher.publish(msg)
                elif char == curses.KEY_LEFT:
                    msg.data = "left"
                    self.keyboard_monitor_publisher.publish(msg)
                elif char == ord('w'):
                    msg.data = "up"
                    self.keyboard_monitor_publisher.publish(msg)
                elif char == ord('a'):
                    msg.data = "rotate_left"
                    self.keyboard_monitor_publisher.publish(msg)
                elif char == ord('s'):
                    msg.data = "down"
                    self.keyboard_monitor_publisher.publish(msg)
                elif char == ord('d'):
                    msg.data = "rotate_right"
                    self.keyboard_monitor_publisher.publish(msg)
                elif char == 43:  # Tecla '+'
                    msg.data = "plus"
                    self.keyboard_monitor_publisher.publish(msg)
                elif char == 45:  # Tecla '-'
                    msg.data = "minus"
                    self.keyboard_monitor_publisher.publish(msg)
                elif char == ord(' '):
                    msg.data = "stop"
                    self.keyboard_monitor_publisher.publish(msg)

                # Comandos del motor
                elif char == ord('1'):
                    msg.data = "1"  # Comando para avanzar el motor
                    self.motor_control_publisher.publish(msg)
                    self.get_logger().info('Comando enviado: Adelante (1)')
                elif char == ord('2'):
                    msg.data = "2"  # Comando para reversa del motor
                    self.motor_control_publisher.publish(msg)
                    self.get_logger().info('Comando enviado: Reversa (2)')
                elif char == ord('0'):
                    msg.data = "0"  # Comando para detener el motor
                    self.motor_control_publisher.publish(msg)
                    self.get_logger().info('Comando enviado: Detener (0)')
                else:
                    continue  # Ignorar otras teclas

        finally:
            curses.nocbreak()
            self.screen.keypad(0)
            curses.echo()
            curses.endwin()

def main(args=None):
    rclpy.init(args=args)
    keyboard_listener = KeyboardControlNode()
    try:
        rclpy.spin(keyboard_listener)
    except KeyboardInterrupt:
        pass
    keyboard_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
