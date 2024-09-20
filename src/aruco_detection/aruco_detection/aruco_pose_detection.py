#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

class ArucoPoseDetectionNode(Node):
    def __init__(self):
        super().__init__('aruco_pose_detection')

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Subscribe to the camera image topic
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera_image/compressed',
            self.image_callback,
            10)

        # Load camera parameters for pose estimation
        self.camera_matrix = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]])  # Example calibration data
        self.dist_coeffs = np.zeros((5, 1))  # Example assuming no lens distortion

        # Define the ArUco dictionary and parameters
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters_create()

        self.get_logger().info('Aruco Pose Detection Node has been started')

    def image_callback(self, msg):
        try:
            # Convert compressed image to a numpy array
            np_arr = np.frombuffer(msg.data, np.uint8)

            # Decode the image
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if frame is not None:
                # Convert to grayscale for ArUco detection
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                # Detect ArUco markers
                corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

                if ids is not None:
                    # Draw detected markers on the image
                    aruco.drawDetectedMarkers(frame, corners, ids)

                    # Estimate pose of each marker
                    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, self.camera_matrix, self.dist_coeffs)

                    for rvec, tvec in zip(rvecs, tvecs):
                        # Draw axis for each marker
                        aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)

                        # Log the detected marker ID and its pose
                        self.get_logger().info(f"Marker ID: {ids.flatten()}, Position: {tvec.flatten()}, Rotation: {rvec.flatten()}")

                # Display the image with the detected markers
                cv2.imshow('Aruco Detection', frame)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error processing the image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ArucoPoseDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
