"""Detect a colored square in the camera image and publish its 3D position."""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge


class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')

        # HSV range for red detection (default: bright red)
        self.declare_parameter('hsv_lower_1', [0, 120, 70])     # Red wraps in HSV
        self.declare_parameter('hsv_upper_1', [10, 255, 255])
        self.declare_parameter('hsv_lower_2', [170, 120, 70])
        self.declare_parameter('hsv_upper_2', [180, 255, 255])
        self.declare_parameter('table_height', 0.055)  # z of the table surface
        self.declare_parameter('min_contour_area', 100)

        self.hsv_lower_1 = np.array(self.get_parameter('hsv_lower_1').value, dtype=np.uint8)
        self.hsv_upper_1 = np.array(self.get_parameter('hsv_upper_1').value, dtype=np.uint8)
        self.hsv_lower_2 = np.array(self.get_parameter('hsv_lower_2').value, dtype=np.uint8)
        self.hsv_upper_2 = np.array(self.get_parameter('hsv_upper_2').value, dtype=np.uint8)
        self.table_height = self.get_parameter('table_height').value
        self.min_contour_area = self.get_parameter('min_contour_area').value

        self.bridge = CvBridge()

        # Camera intrinsics (will be set from camera_info if available)
        # Default: rough estimates for 640x480, 60deg FOV
        self.fx = 554.0
        self.fy = 554.0
        self.cx = 320.0
        self.cy = 240.0
        self.camera_info_received = False

        # Camera pose in world frame (matches tracking_world.sdf)
        # Camera at (0.2, 0, 0.7), rotated pitch=1.2 rad (looking down)
        self.cam_x = 0.2
        self.cam_y = 0.0
        self.cam_z = 0.7
        self.cam_pitch = 1.2  # rotation about Y axis

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10,
        )
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10,
        )

        # Publishers
        self.target_pub = self.create_publisher(PointStamped, '/target/position', 10)
        self.debug_pub = self.create_publisher(Image, '/target/image_debug', 10)

        self.get_logger().info('ColorDetector started')

    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.camera_info_received = True
            self.get_logger().info(
                f'Camera intrinsics: fx={self.fx:.1f} fy={self.fy:.1f} cx={self.cx:.1f} cy={self.cy:.1f}'
            )

    def image_callback(self, msg):
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Red detection (red wraps around 0/180 in HSV)
        mask1 = cv2.inRange(hsv, self.hsv_lower_1, self.hsv_upper_1)
        mask2 = cv2.inRange(hsv, self.hsv_lower_2, self.hsv_upper_2)
        mask = mask1 | mask2

        # Morphological cleanup
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area > self.min_contour_area:
                # Compute centroid
                M = cv2.moments(largest)
                if M['m00'] > 0:
                    cx_px = int(M['m10'] / M['m00'])
                    cy_px = int(M['m01'] / M['m00'])

                    # Project pixel to 3D world coordinates
                    world_point = self.pixel_to_world(cx_px, cy_px)

                    if world_point is not None:
                        # Publish target position
                        target_msg = PointStamped()
                        target_msg.header.stamp = msg.header.stamp
                        target_msg.header.frame_id = 'world'
                        target_msg.point.x = world_point[0]
                        target_msg.point.y = world_point[1]
                        target_msg.point.z = world_point[2]
                        self.target_pub.publish(target_msg)

                    # Debug visualization
                    cv2.drawContours(cv_image, [largest], -1, (0, 255, 0), 2)
                    cv2.circle(cv_image, (cx_px, cy_px), 5, (0, 255, 0), -1)
                    cv2.putText(
                        cv_image,
                        f'({world_point[0]:.3f}, {world_point[1]:.3f})' if world_point else 'N/A',
                        (cx_px + 10, cy_px - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
                    )

        # Publish debug image
        debug_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)

    def pixel_to_world(self, px, py):
        """Project a pixel coordinate to a 3D point on the table plane (z=table_height).

        Uses the known camera pose and pinhole camera model.
        Camera is at (cam_x, cam_y, cam_z) with pitch rotation about Y.
        """
        # Ray in camera optical frame (x-right, y-down, z-forward)
        dx = (px - self.cx) / self.fx
        dy = (py - self.cy) / self.fy

        # Convert to model frame (x-forward, y-left, z-up)
        # optical z-forward → model x, optical x-right → model -y, optical y-down → model -z
        ray_model = np.array([1.0, -dx, -dy])

        # Rotation matrix: pitch about Y axis (model frame to world frame)
        cp = np.cos(self.cam_pitch)
        sp = np.sin(self.cam_pitch)
        R = np.array([
            [cp, 0, sp],
            [0, 1, 0],
            [-sp, 0, cp],
        ])

        # Ray in world frame
        ray_world = R @ ray_model

        # Intersect with table plane z = table_height
        if abs(ray_world[2]) < 1e-6:
            return None  # Ray is parallel to the table

        t = (self.table_height - self.cam_z) / ray_world[2]
        if t < 0:
            return None  # Intersection is behind the camera

        x = self.cam_x + t * ray_world[0]
        y = self.cam_y + t * ray_world[1]

        return (x, y, self.table_height)


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
