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

        self.keyboard_monitor_publisher = self.create_publisher(String, "keyboard", 10)
        self.prev_char = None
        self.keyboard()

    def keyboard(self):
        try:
            while True:
                char = self.screen.getch()
                msg = String()
                # Modes
                if char == ord('T'):
                    msg.data = "takeoff"
                elif char == ord('L'):
                    msg.data = "land"
                elif char == ord('A'):
                    msg.data = "arm"
                elif char == ord('O'):
                    msg.data = "offboard"
                elif char == ord('R'):
                    msg.data = "return"
                elif char == curses.KEY_DOWN:
                    msg.data = "back"
                elif char == curses.KEY_UP:
                    msg.data = "front"
                elif char == curses.KEY_RIGHT:
                    msg.data = "right"
                elif char == curses.KEY_LEFT:
                    msg.data = "left"
                elif char == ord('w'):
                    msg.data = "up"
                elif char == ord('a'):
                    msg.data = "rotate_left"
                elif char == ord('s'):
                    msg.data = "down"
                elif char == ord('d'):
                    msg.data = "rotate_right"
                elif char == 43:  # + symbol ascii
                    msg.data = "plus"
                elif char == 45:  # - symbol ascii
                    msg.data = "minus"
                elif char == ord(' '):
                    msg.data = "stop"
                else:
                    continue  # Ignorar otras teclas

                self.get_logger().info(f"Comando enviado: {msg.data}")
                self.keyboard_monitor_publisher.publish(msg)

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

