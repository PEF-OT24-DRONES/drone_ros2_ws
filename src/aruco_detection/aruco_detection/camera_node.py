#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
#from cv_bridge import CvBridge
import cv2

class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_publisher_node')
        self.publisher_ = self.create_publisher(CompressedImage, '/camera_image/compressed', 10)
        self.timer_ = self.create_timer(1/20, self.timer_callback)  # Publicar a 30 FPS
        self.cap_ = cv2.VideoCapture(0)

        # Establecer una resolución más baja (por ejemplo, 320x240)
        self.cap_.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap_.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        #self.bridge = CvBridge()
        self.get_logger().info('Camera Publisher has been started')

    def timer_callback(self):
        ret, frame = self.cap_.read()
        if ret:
            # Opcional: Redimensionar la imagen si la cámara no soporta la resolución baja
            # frame = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_AREA)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Comprimir la imagen en formato JPEG con calidad reducida
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]  # Calidad JPEG al 50%
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