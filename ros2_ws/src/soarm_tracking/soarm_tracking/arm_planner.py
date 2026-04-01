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
        assert urdf_path
        # Load chain from real URDF: base_link → ... → gripper_frame_link
        # active_links_mask: only the 5 arm joints are active (not gripper, not fixed joints)
        self.chain = ikpy.chain.Chain.from_urdf_file(
            urdf_path,
            base_elements=['base_link'],
            last_link_vector=[0, 0, -0.098],  # gripper tip offset from gripper_link
            active_links_mask=[False, True, True, True, True, True, False, False],
        )
        self.get_logger().info(f'IK chain loaded from {urdf_path}')
        for i, link in enumerate(self.chain.links):
            self.get_logger().info(f'  [{i}] {link.name}')

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


    def target_callback(self, msg):
        now = self.get_clock().now()
        elapsed = (now - self.last_command_time).nanoseconds / 1e9
        if elapsed < self.min_move_interval:
            return

        target = [msg.point.x, msg.point.y, msg.point.z]

        # Compute IK
        # Seed: one value per link in the chain (origin + 5 joints + end-effector)
        n_links = len(self.chain.links)
        seed = [0.0] * n_links
        for i, link in enumerate(self.chain.links):
            if link.name in self.joint_names:
                idx = self.joint_names.index(link.name)
                seed[i] = self.current_joint_positions[idx]
        joint_angles = self.chain.inverse_kinematics(
            target_position=target,
            initial_position=seed,
        )

        # Extract active joint values by matching names
        active_angles = []
        for jname in self.joint_names:
            for i, link in enumerate(self.chain.links):
                if link.name == jname:
                    active_angles.append(joint_angles[i])
                    break

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
