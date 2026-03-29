"""Move a colored square randomly on the table in Gazebo."""

import random

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SetEntityPose
from geometry_msgs.msg import Pose, Point, Quaternion


class SquareMover(Node):
    def __init__(self):
        super().__init__('square_mover')

        # Parameters
        self.declare_parameter('move_interval', 2.0)  # seconds between moves
        self.declare_parameter('x_min', 0.05)
        self.declare_parameter('x_max', 0.40)
        self.declare_parameter('y_min', -0.20)
        self.declare_parameter('y_max', 0.20)
        self.declare_parameter('z_height', 0.055)  # just above table surface
        self.declare_parameter('entity_name', 'colored_square')

        self.move_interval = self.get_parameter('move_interval').value
        self.x_min = self.get_parameter('x_min').value
        self.x_max = self.get_parameter('x_max').value
        self.y_min = self.get_parameter('y_min').value
        self.y_max = self.get_parameter('y_max').value
        self.z_height = self.get_parameter('z_height').value
        self.entity_name = self.get_parameter('entity_name').value

        # Service client for setting entity pose in Gazebo
        self.set_pose_client = self.create_client(
            SetEntityPose,
            '/world/tracking_world/set_pose',
        )

        # Timer
        self.timer = self.create_timer(self.move_interval, self.move_square)
        self.get_logger().info(
            f'SquareMover started: moving every {self.move_interval}s '
            f'in x=[{self.x_min}, {self.x_max}], y=[{self.y_min}, {self.y_max}]'
        )

    def move_square(self):
        if not self.set_pose_client.service_is_ready():
            self.get_logger().warn('SetEntityPose service not ready, waiting...')
            return

        x = random.uniform(self.x_min, self.x_max)
        y = random.uniform(self.y_min, self.y_max)

        request = SetEntityPose.Request()
        request.entity.name = self.entity_name
        request.entity.type = 6  # MODEL
        request.pose = Pose(
            position=Point(x=x, y=y, z=self.z_height),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )

        future = self.set_pose_client.call_async(request)
        future.add_done_callback(
            lambda f: self.get_logger().debug(f'Square moved to ({x:.3f}, {y:.3f})')
        )


def main(args=None):
    rclpy.init(args=args)
    node = SquareMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
