#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_publisher_node')
        self.publisher_ = self.create_publisher(CompressedImage, '/camera_image/compressed', 10)
        self.timer_ = self.create_timer(1/30, self.timer_callback)  # Publicar a 30 FPS
        self.cap_ = cv2.VideoCapture(0)
        self.bridge = CvBridge()
        self.get_logger().info('Camera Publisher has been started')

    def timer_callback(self):
        ret, frame = self.cap_.read()
        if ret:
            # Comprimir la imagen en formato JPEG
            success, compressed_image = cv2.imencode('.jpg', frame)
            if success:
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_frame'
                msg.format = 'jpeg'
                msg.data = compressed_image.tobytes()
                self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()