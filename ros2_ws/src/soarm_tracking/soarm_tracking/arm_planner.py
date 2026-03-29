"""Plan arm movements using inverse kinematics to reach target positions."""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PointStamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# ikpy for inverse kinematics
import ikpy.chain


class ArmPlanner(Node):
    def __init__(self):
        super().__init__('arm_planner')

        # Parameters
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('move_duration', 1.5)  # seconds for each move
        self.declare_parameter('min_move_interval', 1.0)  # min time between commands

        self.move_duration = self.get_parameter('move_duration').value
        self.min_move_interval = self.get_parameter('min_move_interval').value

        # Joint names (must match URDF and controllers.yaml)
        self.joint_names = [
            'shoulder_pan',
            'shoulder_lift',
            'elbow_flex',
            'wrist_flex',
            'wrist_roll',
        ]

        # Build IK chain from URDF
        urdf_path = self.get_parameter('urdf_path').value
        if urdf_path:
            self.chain = ikpy.chain.Chain.from_urdf_file(
                urdf_path,
                base_elements=['base_link'],
                active_links_mask=[False, True, True, True, True, True, False, False],
            )
            self.get_logger().info(f'IK chain loaded from {urdf_path}')
        else:
            # Build chain manually with approximate SO-ARM101 dimensions
            self.chain = self._build_default_chain()
            self.get_logger().info('Using default IK chain (approximate dimensions)')

        # Action client for arm_controller
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
        )

        # Subscriber
        self.target_sub = self.create_subscription(
            PointStamped, '/target/position', self.target_callback, 10,
        )

        # Rate limiting
        self.last_command_time = self.get_clock().now()
        self.current_joint_positions = [0.0] * 6  # seed for IK

        self.get_logger().info('ArmPlanner started, waiting for arm_controller...')

    def _build_default_chain(self):
        """Build an approximate IK chain for the SO-ARM101."""
        import ikpy.link

        links = [
            ikpy.link.OriginLink(),
            # shoulder_pan: rotation about Z at base height
            ikpy.link.URDFLink(
                name='shoulder_pan',
                origin_translation=[0.0388, 0, 0.0624],
                origin_orientation=[0, 0, 0],
                rotation=[0, 0, 1],
                bounds=(-1.92, 1.92),
            ),
            # shoulder_lift: rotation about Y
            ikpy.link.URDFLink(
                name='shoulder_lift',
                origin_translation=[0, 0, 0.054],
                origin_orientation=[0, 0, 0],
                rotation=[0, 1, 0],
                bounds=(-1.75, 1.75),
            ),
            # elbow_flex: rotation about Y
            ikpy.link.URDFLink(
                name='elbow_flex',
                origin_translation=[0.1126, 0, 0],
                origin_orientation=[0, 0, 0],
                rotation=[0, 1, 0],
                bounds=(-1.69, 1.69),
            ),
            # wrist_flex: rotation about Y
            ikpy.link.URDFLink(
                name='wrist_flex',
                origin_translation=[0.1349, 0, 0],
                origin_orientation=[0, 0, 0],
                rotation=[0, 1, 0],
                bounds=(-1.66, 1.66),
            ),
            # wrist_roll: rotation about X (roll)
            ikpy.link.URDFLink(
                name='wrist_roll',
                origin_translation=[0.061, 0, 0],
                origin_orientation=[0, 0, 0],
                rotation=[1, 0, 0],
                bounds=(-2.74, 2.84),
            ),
            # End effector (fixed)
            ikpy.link.URDFLink(
                name='end_effector',
                origin_translation=[0.05, 0, 0],
                origin_orientation=[0, 0, 0],
                rotation=[0, 0, 0],
            ),
        ]
        return ikpy.chain.Chain(name='so101', links=links)

    def target_callback(self, msg):
        now = self.get_clock().now()
        elapsed = (now - self.last_command_time).nanoseconds / 1e9
        if elapsed < self.min_move_interval:
            return

        target = [msg.point.x, msg.point.y, msg.point.z]

        # Compute IK
        seed = [0.0] + self.current_joint_positions[:5] + [0.0]
        joint_angles = self.chain.inverse_kinematics(
            target_position=target,
            initial_position=seed,
        )

        # Extract the active joint values (skip base origin and end effector)
        active_angles = joint_angles[1:6].tolist()

        self.get_logger().info(
            f'Target: ({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}) -> '
            f'Joints: [{", ".join(f"{a:.3f}" for a in active_angles)}]'
        )

        # Send trajectory goal
        self.send_trajectory(active_angles)
        self.current_joint_positions[:5] = active_angles
        self.last_command_time = now

    def send_trajectory(self, joint_positions):
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('arm_controller action server not available')
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.velocities = [0.0] * len(joint_positions)
        secs = int(self.move_duration)
        nsecs = int((self.move_duration - secs) * 1e9)
        point.time_from_start = Duration(sec=secs, nanosec=nsecs)
        goal.trajectory.points = [point]

        self.action_client.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)
    node = ArmPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
