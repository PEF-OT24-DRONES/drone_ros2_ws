#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import cv2.aruco as aruco
import numpy as np
import math
from geometry_msgs.msg import PoseStamped


class ArucoPoseDetectionNode(Node):
    def __init__(self):
        super().__init__('aruco_pose_detection')

        # Subscribe to the camera image topic
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera_image/compressed',
            self.image_callback,
            10)
        
        self.publisher = self.create_publisher(CompressedImage, '/aruco_detection/compressed', 10)

        # Publisher for ArUco pose
        self.pose_publisher = self.create_publisher(PoseStamped, '/aruco_pose', 10)

        # Load camera parameters for pose estimation
        self.camera_matrix = np.array([[1.01112869e+03, 0, 6.25027187e+02], 
                                       [0, 1.00939931e+03, 3.55205263e+02], 
                                       [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.array([-0.0031836, -0.10976409, 0.00019764, 
                                     -0.001902, -0.13254417])
        
        """
        self.camera_matrix = np.array([[1.0, 0, 0.0], 
                                       [0, 1.0, 0.0], 
                                       [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.array([0.0, 0.0, 0.0, 
                                     0.0, 0.0])
        """

        # Define the ArUco dictionary and parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters()

        # IDs for inner and outer markers
        self.inner_marker_id = 100
        self.outer_marker_id = 5
        self.inner_marker_size = 0.05  # Inner marker size in meters (5 cm)
        self.outer_marker_size = 0.21  # Outer marker size in meters (21 cm)

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
    
    @staticmethod
    def eulerAnglesToQuaternion(roll, pitch, yaw):
        """
        Convert Euler angles to quaternion.
        """
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return [qx, qy, qz, qw]

    def image_callback(self, msg):
        try:
            # Convert compressed image to a numpy array
            np_arr = np.frombuffer(msg.data, np.uint8)

            # Decode the image
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Resize the image
            frame = cv2.resize(frame, (320, 240))

            # Convert image to grayscale
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Detect ArUco markers
            corners, ids, _ = aruco.detectMarkers(frame_gray, self.aruco_dict, parameters=self.aruco_params)

            if ids is not None:
                # Process each detected marker
                for i, marker_id in enumerate(ids.flatten()):
                    # Determine the size based on the marker ID
                    if marker_id == self.inner_marker_id:
                        marker_size = self.inner_marker_size
                        axis_length = self.inner_marker_size / 2
                    elif marker_id == self.outer_marker_id:
                        marker_size = self.outer_marker_size
                        axis_length = self.outer_marker_size / 2
                    else:
                        continue  # Skip any markers that are not inner or outer

                    # Get the corresponding corners
                    corners_single = [corners[i]]

                    # Estimate pose of the detected marker
                    ret = aruco.estimatePoseSingleMarkers(
                        corners_single, marker_size, self.camera_matrix, self.dist_coeffs)
                    rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]

                    # Draw the marker
                    aruco.drawDetectedMarkers(frame, corners_single)

                    # Manually project the axes on the image
                    axis = np.float32([[axis_length, 0, 0], 
                                       [0, axis_length, 0], 
                                       [0, 0, axis_length]]).reshape(-1, 3)

                    # Project 3D points to the image plane
                    imgpts, jac = cv2.projectPoints(
                        axis, rvec, tvec, self.camera_matrix, self.dist_coeffs)
                    imgpts = np.int32(imgpts).reshape(-1, 2)

                    # Draw the axis lines on the marker
                    corner = tuple(corners_single[0][0].mean(axis=0).astype(int))  # Use the marker center as origin
                    frame = cv2.line(frame, corner, tuple(imgpts[0]), (255, 0, 0), 2)  # X-axis (Red)
                    frame = cv2.line(frame, corner, tuple(imgpts[1]), (0, 255, 0), 2)  # Y-axis (Green)
                    frame = cv2.line(frame, corner, tuple(imgpts[2]), (0, 0, 255), 2)  # Z-axis (Blue)

                    # Extract rotation matrix and Euler angles
                    R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
                    R_tc = R_ct.T  # Transpose rotation matrix to change coordinate systems
                    roll_marker, pitch_marker, yaw_marker = self.rotationMatrixToEulerAngles(R_tc)

                    # Convert angles to degrees
                    roll_deg = math.degrees(roll_marker)
                    pitch_deg = math.degrees(pitch_marker)
                    yaw_deg = math.degrees(yaw_marker)

                    # Log position in meters and attitude in degrees
                    self.get_logger().info(
                        f"[Marker {marker_id}] Position (x, y, z): {tvec[0]:.3f} m, {tvec[1]:.3f} m, {tvec[2]:.3f} m"
                    )
                    self.get_logger().info(
                        f"[Marker {marker_id}] Attitude (roll, pitch, yaw): "
                        f"{roll_deg:.2f}°, {pitch_deg:.2f}°, {yaw_deg:.2f}°"
                    )

                    # Publish the pose in meters and degrees
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.header.frame_id = 'camera_frame'
                    pose_msg.pose.position.x = tvec[0]
                    pose_msg.pose.position.y = tvec[1]
                    pose_msg.pose.position.z = tvec[2]


                    pose_msg.pose.orientation.x = roll_deg
                    pose_msg.pose.orientation.y = pitch_deg
                    pose_msg.pose.orientation.z = yaw_deg
                    pose_msg.pose.orientation.w = 0.0

                    self.pose_publisher.publish(pose_msg)

                    # Calculate corrections needed to center the drone
                    x_correction = -tvec[0]  # Lateral adjustment
                    y_correction = -tvec[1]  # Vertical adjustment
                    z_correction = -tvec[2]  # Depth adjustment

                    self.get_logger().info(
                        f"[Marker {marker_id}] Corrections: Move x={x_correction:.3f}m, y={y_correction:.3f}m, z={z_correction:.3f}m"
                    )

            # Publish the image with the detected markers
            msg_out = CompressedImage()
            msg_out.header.stamp = self.get_clock().now().to_msg()
            msg_out.format = 'jpeg'
            _, img_out = cv2.imencode('.jpg', frame)  # Encode the color image
            msg_out.data = img_out.tobytes()
            self.publisher.publish(msg_out)

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
