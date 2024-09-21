#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import cv2.aruco as aruco
import numpy as np
import math

class ArucoPoseDetectionNode(Node):
    def __init__(self):
        super().__init__('aruco_pose_detection')

        # Subscribe to the camera image topic
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera_image/compressed',
            self.image_callback,
            10)

        # Load camera parameters for pose estimation
        self.camera_matrix = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]], dtype=np.float32)  # Example calibration data
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)  # Example assuming no lens distortion

        # Define the ArUco dictionary and parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters()

        # IDs for inner and outer markers
        self.inner_marker_id = 100
        self.outer_marker_id = 5
        self.marker_size = 0.185  # Marker size in meters

        self.get_logger().info('Aruco Pose Detection Node has been started')

    @staticmethod
    def isRotationMatrix(R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    @staticmethod
    def rotationMatrixToEulerAngles(R):
        assert (ArucoPoseDetectionNode.isRotationMatrix(R))
        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0
        return np.array([x, y, z])

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
                    # Check if inner or outer marker is detected
                    marker_id = None
                    if self.inner_marker_id in ids:
                        marker_id = self.inner_marker_id
                    elif self.outer_marker_id in ids:
                        marker_id = self.outer_marker_id

                    if marker_id is not None:
                        # Get index of the detected marker
                        index = np.where(ids == marker_id)[0][0]
                        
                        # Estimate pose of the detected marker
                        ret = aruco.estimatePoseSingleMarkers([corners[index]], self.marker_size, self.camera_matrix, self.dist_coeffs)
                        rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]

                        # Draw the marker
                        aruco.drawDetectedMarkers(frame, [corners[index]])

                        # Manually project the axes on the image
                        axis_length = 0.1  # Axis length in meters
                        axis = np.float32([[axis_length, 0, 0], [0, axis_length, 0], [0, 0, axis_length]]).reshape(-1, 3)

                        # Project 3D points to the image plane
                        imgpts, jac = cv2.projectPoints(axis, rvec, tvec, self.camera_matrix, self.dist_coeffs)
                        imgpts = np.int32(imgpts).reshape(-1, 2)

                        # Draw the axis lines on the marker
                        corner = tuple(corners[index][0].mean(axis=0).astype(int))  # Use the marker center as origin
                        frame = cv2.line(frame, corner, tuple(imgpts[0]), (255, 0, 0), 5)  # X-axis (Red)
                        frame = cv2.line(frame, corner, tuple(imgpts[1]), (0, 255, 0), 5)  # Y-axis (Green)
                        frame = cv2.line(frame, corner, tuple(imgpts[2]), (0, 0, 255), 5)  # Z-axis (Blue)

                        # Print position and attitude of the marker
                        self.get_logger().info(f"MARKER {marker_id} Position x={tvec[0]:.2f} m, y={tvec[1]:.2f} m, z={tvec[2]:.2f} m")

                        R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
                        R_tc = R_ct.T

                        roll_marker, pitch_marker, yaw_marker = self.rotationMatrixToEulerAngles(R_tc)
                        self.get_logger().info(f"MARKER {marker_id} Attitude roll={math.degrees(roll_marker):.2f} deg, pitch={math.degrees(pitch_marker):.2f} deg, yaw={math.degrees(yaw_marker):.2f} deg")

                # Display the frame with the detected markers and axes
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

