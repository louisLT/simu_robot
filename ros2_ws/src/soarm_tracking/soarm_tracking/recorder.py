"""Record target positions and joint states to CSV files."""

import csv
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState


class Recorder(Node):
    def __init__(self):
        super().__init__('recorder')

        # Parameters
        self.declare_parameter('output_dir', '/ws/recordings')
        self.declare_parameter('flush_interval', 5.0)  # seconds

        self.output_dir = self.get_parameter('output_dir').value
        flush_interval = self.get_parameter('flush_interval').value

        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)

        # Create CSV files with timestamp in name
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.target_file = open(
            os.path.join(self.output_dir, f'target_positions_{timestamp}.csv'),
            'w', newline='',
        )
        self.joint_file = open(
            os.path.join(self.output_dir, f'joint_states_{timestamp}.csv'),
            'w', newline='',
        )

        # CSV writers
        self.target_writer = csv.writer(self.target_file)
        self.target_writer.writerow(['timestamp', 'x', 'y', 'z'])

        self.joint_writer = csv.writer(self.joint_file)
        self.joint_header_written = False

        # Subscribers
        self.target_sub = self.create_subscription(
            PointStamped, '/target/position', self.target_callback, 10,
        )
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10,
        )

        # Periodic flush
        self.flush_timer = self.create_timer(flush_interval, self.flush)

        self.get_logger().info(f'Recorder started, writing to {self.output_dir}/')

    def target_callback(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.target_writer.writerow([
            f'{t:.6f}',
            f'{msg.point.x:.6f}',
            f'{msg.point.y:.6f}',
            f'{msg.point.z:.6f}',
        ])

    def joint_callback(self, msg):
        # Write header on first message (includes joint names)
        if not self.joint_header_written:
            header = ['timestamp']
            for name in msg.name:
                header.extend([f'{name}_pos', f'{name}_vel'])
            self.joint_writer.writerow(header)
            self.joint_header_written = True

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        row = [f'{t:.6f}']
        for pos, vel in zip(msg.position, msg.velocity):
            row.extend([f'{pos:.6f}', f'{vel:.6f}'])
        self.joint_writer.writerow(row)

    def flush(self):
        self.target_file.flush()
        self.joint_file.flush()

    def destroy_node(self):
        self.target_file.close()
        self.joint_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Recorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
