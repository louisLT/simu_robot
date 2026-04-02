#!/usr/bin/env python3
"""Minimal IK test: solve IK for a static target and send to arm_controller.

Usage:
    # Terminal 1: launch Gazebo + robot
    ros2 launch soarm_gazebo gazebo.launch.py

    # Terminal 2: run this script
    python3 test_ik.py --target 0.25 0.0 0.155
"""

import argparse
import os
import subprocess
import sys
import tempfile

import ikpy.chain
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


def spawn_target_marker(target):
    """Spawn a small green sphere in Gazebo at the target position."""
    x, y, z = target
    sdf = (
        '<sdf version=\\"1.9\\">'
        '<model name=\\"ik_target_marker\\">'
        '<static>true</static>'
        f'<pose>{x} {y} {z} 0 0 0</pose>'
        '<link name=\\"link\\">'
        '<visual name=\\"v\\">'
        '<geometry><sphere><radius>0.02</radius></sphere></geometry>'
        '<material>'
        '<ambient>0 1 0 1</ambient>'
        '<diffuse>0 1 0 1</diffuse>'
        '</material>'
        '</visual>'
        '</link>'
        '</model>'
        '</sdf>'
    )

    # Remove previous marker if it exists
    subprocess.run(
        ['gz', 'service', '-s', '/world/tracking_world/remove',
         '--reqtype', 'gz.msgs.Entity',
         '--reptype', 'gz.msgs.Boolean',
         '--timeout', '2000',
         '--req', 'name: "ik_target_marker" type: MODEL'],
        capture_output=True, text=True,
    )

    result = subprocess.run(
        ['gz', 'service', '-s', '/world/tracking_world/create',
         '--reqtype', 'gz.msgs.EntityFactory',
         '--reptype', 'gz.msgs.Boolean',
         '--timeout', '5000',
         '--req', f'sdf: "{sdf}"'],
        capture_output=True, text=True,
    )
    if result.returncode == 0:
        print(f'  Target marker (green sphere) spawned at [{x:.3f}, {y:.3f}, {z:.3f}]')
    else:
        print(f'  WARNING: Could not spawn marker: {result.stderr.strip()}')


JOINT_NAMES = [
    'shoulder_pan',
    'shoulder_lift',
    'elbow_flex',
    'wrist_flex',
    'wrist_roll',
]


def generate_urdf():
    """Generate URDF from xacro using ros2 CLI."""
    result = subprocess.run(
        ['ros2', 'run', 'xacro', 'xacro',
         '/ws/ros2_ws/install/soarm_description/share/soarm_description/urdf/so101.urdf.xacro'],
        capture_output=True, text=True,
    )
    if result.returncode != 0:
        print(f'xacro failed: {result.stderr}', file=sys.stderr)
        sys.exit(1)

    fd, path = tempfile.mkstemp(suffix='.urdf')
    with os.fdopen(fd, 'w') as f:
        f.write(result.stdout)
    return path


def solve_ik(urdf_path, target):
    """Solve IK and return (active_angles, report_string)."""
    chain = ikpy.chain.Chain.from_urdf_file(
        urdf_path,
        base_elements=['base_link'],
        last_link_vector=[0, 0, -0.098],
        active_links_mask=[False, True, True, True, True, True, False, False],
    )

    # Print chain structure
    print('\n=== IK Chain ===')
    for i, link in enumerate(chain.links):
        mask = chain.active_links_mask[i] if i < len(chain.active_links_mask) else False
        print(f'  [{i}] {link.name:20s} active={mask}')

    # Solve
    seed = [0.0] * len(chain.links)
    joint_angles = chain.inverse_kinematics(target_position=target, initial_position=seed)

    # Verify with forward kinematics
    fk_matrix = chain.forward_kinematics(joint_angles)
    reached = fk_matrix[:3, 3]
    error = np.linalg.norm(np.array(target) - reached)

    # Extract active joint angles
    active_angles = []
    for jname in JOINT_NAMES:
        for i, link in enumerate(chain.links):
            if link.name == jname:
                active_angles.append(joint_angles[i])
                break

    # Report
    print('\n=== IK Result ===')
    print(f'  Target:    [{target[0]:.4f}, {target[1]:.4f}, {target[2]:.4f}]')
    print(f'  Reached:   [{reached[0]:.4f}, {reached[1]:.4f}, {reached[2]:.4f}]')
    print(f'  Error:     {error:.6f} m')
    print()
    for name, angle in zip(JOINT_NAMES, active_angles):
        print(f'  {name:20s} = {angle:+.4f} rad  ({np.degrees(angle):+.1f} deg)')

    if error > 0.01:
        print(f'\n  WARNING: IK error is {error:.4f} m — target may be out of reach!')

    return active_angles


class IKTestNode(Node):
    def __init__(self, joint_positions, move_duration=3.0):
        super().__init__('ik_test')
        self.joint_positions = joint_positions
        self.move_duration = move_duration

        self.action_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory',
        )

    def send_and_wait(self):
        print('\n=== Sending trajectory to arm_controller ===')
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            print('  ERROR: arm_controller action server not available after 10s')
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = self.joint_positions
        point.velocities = [0.0] * len(self.joint_positions)
        secs = int(self.move_duration)
        nsecs = int((self.move_duration - secs) * 1e9)
        point.time_from_start = Duration(sec=secs, nanosec=nsecs)
        goal.trajectory.points = [point]

        print(f'  Sending goal: {[f"{a:.4f}" for a in self.joint_positions]}')
        print(f'  Duration: {self.move_duration}s')

        future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            print('  ERROR: Goal rejected by arm_controller')
            return False

        print('  Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=self.move_duration + 5.0)

        result = result_future.result()
        if result:
            print(f'  Result: error_code = {result.result.error_code}')
            if result.result.error_code == 0:
                print('  SUCCESS — robot should be at target position')
            else:
                print(f'  FAILED — error string: {result.result.error_string}')
        else:
            print('  WARNING: No result received (timeout?)')

        return True


def main():
    parser = argparse.ArgumentParser(description='Minimal IK test for SO-ARM101')
    parser.add_argument('--target', type=float, nargs=3, default=[0.25, 0.0, 0.155],
                        metavar=('X', 'Y', 'Z'),
                        help='Target position in world frame (default: 0.25 0.0 0.155)')
    parser.add_argument('--duration', type=float, default=3.0,
                        help='Trajectory duration in seconds (default: 3.0)')
    parser.add_argument('--dry-run', action='store_true',
                        help='Only solve IK, do not send to robot')
    args = parser.parse_args()

    print('=== SO-ARM101 Minimal IK Test ===')
    print(f'  Target: {args.target}')

    # Step 1: Generate URDF
    print('\nGenerating URDF from xacro...')
    urdf_path = generate_urdf()
    print(f'  URDF written to {urdf_path}')

    # Step 2: Solve IK
    active_angles = solve_ik(urdf_path, args.target)

    # Step 3: Spawn visual marker at target
    print('\nSpawning target marker in Gazebo...')
    spawn_target_marker(args.target)

    if args.dry_run:
        print('\n  --dry-run: not sending to robot')
        return

    # Step 4: Send to robot
    rclpy.init()
    node = IKTestNode(active_angles, move_duration=args.duration)
    node.send_and_wait()
    node.destroy_node()
    rclpy.shutdown()

    print('\nDone. Check Gazebo to verify robot position.')


if __name__ == '__main__':
    main()
