#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from cv_bridge import CvBridge
import cv2
import numpy as np


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # Parameters
        self.declare_parameter('marker_size', 0.15)  # Marker size in meters (15cm default)
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')  # ArUco dictionary
        self.declare_parameter('camera_fx', 615.0)  # Focal length x (estimated for OAK-D-PRO)
        self.declare_parameter('camera_fy', 615.0)  # Focal length y
        self.declare_parameter('camera_cx', 640.0)  # Principal point x (image center)
        self.declare_parameter('camera_cy', 360.0)  # Principal point y

        self.marker_size = self.get_parameter('marker_size').value
        aruco_dict_name = self.get_parameter('aruco_dict').value
        self.camera_fx = self.get_parameter('camera_fx').value
        self.camera_fy = self.get_parameter('camera_fy').value
        self.camera_cx = self.get_parameter('camera_cx').value
        self.camera_cy = self.get_parameter('camera_cy').value

        # Camera matrix
        self.camera_matrix = np.array([
            [self.camera_fx, 0, self.camera_cx],
            [0, self.camera_fy, self.camera_cy],
            [0, 0, 1]
        ], dtype=np.float32)

        # Assume no distortion for simplicity (can be updated with actual calibration)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)

        # Initialize ArUco detector
        aruco_dict_map = {
            'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
            'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
            'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
            'DICT_4X4_1000': cv2.aruco.DICT_4X4_1000,
            'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
            'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
            'DICT_5X5_250': cv2.aruco.DICT_5X5_250,
            'DICT_5X5_1000': cv2.aruco.DICT_5X5_1000,
            'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
            'DICT_6X6_100': cv2.aruco.DICT_6X6_100,
            'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
            'DICT_6X6_1000': cv2.aruco.DICT_6X6_1000,
        }

        if aruco_dict_name not in aruco_dict_map:
            self.get_logger().error(f'Unknown ArUco dictionary: {aruco_dict_name}')
            aruco_dict_name = 'DICT_4X4_50'

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_map[aruco_dict_name])
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # CV Bridge
        self.bridge = CvBridge()

        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            '/rgb/color',
            self.image_callback,
            10
        )

        # Publishers
        self.detected_pub = self.create_publisher(Bool, '/aruco/detected', 10)
        self.distance_pub = self.create_publisher(Float32, '/aruco/distance', 10)

        self.get_logger().info(
            f'ArUco detector initialized - Dictionary: {aruco_dict_name}, '
            f'Marker size: {self.marker_size}m'
        )

    def estimate_distance(self, corners):
        """
        Estimate distance to ArUco marker using perspective-n-point

        Args:
            corners: ArUco marker corners in image coordinates

        Returns:
            Distance to marker in meters
        """
        # Define 3D points of the marker in marker coordinate system
        # Marker is assumed to be a square with given size
        half_size = self.marker_size / 2.0
        object_points = np.array([
            [-half_size, half_size, 0],
            [half_size, half_size, 0],
            [half_size, -half_size, 0],
            [-half_size, -half_size, 0]
        ], dtype=np.float32)

        # Image points are the detected corners
        image_points = corners.reshape((4, 2)).astype(np.float32)

        # Solve PnP to get rotation and translation vectors
        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            self.camera_matrix,
            self.dist_coeffs
        )

        if success:
            # Distance is the magnitude of the translation vector
            distance = np.linalg.norm(tvec)
            return distance
        else:
            return None

    def image_callback(self, msg):
        """Process camera image and detect ArUco markers"""
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Convert to grayscale for detection
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Detect ArUco markers
            corners, ids, rejected = self.aruco_detector.detectMarkers(gray)

            # Check if any markers were detected
            if ids is not None and len(ids) > 0:
                # Marker detected
                self.detected_pub.publish(Bool(data=True))

                # Estimate distance to the first detected marker
                distance = self.estimate_distance(corners[0])

                if distance is not None:
                    self.distance_pub.publish(Float32(data=float(distance)))
                    self.get_logger().info(
                        f'ArUco marker {ids[0][0]} detected at {distance:.3f}m'
                    )
                else:
                    self.get_logger().warn('Failed to estimate distance')
            else:
                # No marker detected
                self.detected_pub.publish(Bool(data=False))
                self.distance_pub.publish(Float32(data=0.0))

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
