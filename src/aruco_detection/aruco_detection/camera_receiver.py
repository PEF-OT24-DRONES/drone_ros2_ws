#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraReceiverNode(Node):
    def __init__(self):
        super().__init__('camera_receiver_node')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera_image/compressed',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.get_logger().info('Image Subscriber has been started')

    def listener_callback(self, msg):
        # Convertir los datos del mensaje en una matriz NumPy
        np_arr = np.frombuffer(msg.data, np.uint8)
        # Decodificar la imagen
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is not None:
            cv2.imshow('Received Image', frame)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CameraReceiverNode()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()