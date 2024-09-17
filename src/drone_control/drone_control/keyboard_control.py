#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import sys
import select
import termios
import tty

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher_ = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        self.get_logger().info('Keyboard control node has been started.')

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        try:
            print("Controla el dron con las teclas WASD:")
            while True:
                tty.setraw(sys.stdin.fileno())
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                if rlist:
                    key = sys.stdin.read(1)
                    twist = Twist()
                    if key == 'w':
                        twist.linear.x = 1.0
                    elif key == 's':
                        twist.linear.x = -1.0
                    elif key == 'a':
                        twist.linear.y = 1.0
                    elif key == 'd':
                        twist.linear.y = -1.0
                    elif key == 'q':
                        twist.angular.z = 1.0
                    elif key == 'e':
                        twist.angular.z = -1.0
                    elif key == '\x03':  # Ctrl+C
                        break
                    else:
                        continue
                    self.publisher_.publish(twist)
                else:
                    twist = Twist()
                    self.publisher_.publish(twist)
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControl()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
