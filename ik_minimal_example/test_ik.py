#!/usr/bin/env python3
"""Minimal IK test: solve IK for a static target and send to arm_controller.

Usage:
    # Terminal 1: launch Gazebo + robot
    ros2 launch soarm_gazebo gazebo.launch.py

    # Terminal 2: run this script
    python3 test_ik.py --target 0.25 0.0 0.155
    python3 test_ik.py --diag
"""

import argparse
import os
import subprocess
import sys
import tempfile
import time

import ikpy.chain
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration


JOINT_NAMES = [
    'shoulder_pan',
    'shoulder_lift',
    'elbow_flex',
    'wrist_flex',
    'wrist_roll',
]

BASE_LINK = 'base_link'

ALL_MARKER_NAMES = ['ik_target_marker', 'fk_marker']


# =============================================================================
# Gazebo helpers
# =============================================================================

def cleanup_markers():
    """Remove all markers from previous runs."""
    for name in ALL_MARKER_NAMES:
        subprocess.run(
            ['gz', 'service', '-s', '/world/tracking_world/remove',
             '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '2000', '--req', f'name: "{name}" type: MODEL'],
            capture_output=True, text=True,
        )


def spawn_target_marker(target, name='ik_target_marker', color='0 1 0 1'):
    """Spawn a small sphere in Gazebo at the given position."""
    x, y, z = target
    sdf = (
        '<sdf version=\\"1.9\\">'
        f'<model name=\\"{name}\\">'
        '<static>true</static>'
        f'<pose>{x} {y} {z} 0 0 0</pose>'
        '<link name=\\"link\\">'
        '<visual name=\\"v\\">'
        '<geometry><sphere><radius>0.015</radius></sphere></geometry>'
        '<material>'
        f'<ambient>{color}</ambient>'
        f'<diffuse>{color}</diffuse>'
        '</material>'
        '</visual>'
        '</link>'
        '</model>'
        '</sdf>'
    )

    # Remove previous marker with same name
    subprocess.run(
        ['gz', 'service', '-s', '/world/tracking_world/remove',
         '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
         '--timeout', '2000', '--req', f'name: "{name}" type: MODEL'],
        capture_output=True, text=True,
    )

    result = subprocess.run(
        ['gz', 'service', '-s', '/world/tracking_world/create',
         '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
         '--timeout', '5000', '--req', f'sdf: "{sdf}"'],
        capture_output=True, text=True,
    )
    if result.returncode == 0:
        print(f'  Marker "{name}" spawned at [{x:.3f}, {y:.3f}, {z:.3f}]')
    else:
        print(f'  WARNING: Could not spawn marker: {result.stderr.strip()}')


def generate_urdf():
    """Generate URDF from xacro."""
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


# =============================================================================
# IK Solver (ikpy)
# =============================================================================

def load_chain(urdf_path):
    """Load the IK chain from URDF."""
    # Discover chain structure first
    chain = ikpy.chain.Chain.from_urdf_file(
        urdf_path, base_elements=[BASE_LINK],
    )
    # Build mask: only the 5 arm joints are active
    active_joints = set(JOINT_NAMES)
    mask = [link.name in active_joints for link in chain.links]
    chain = ikpy.chain.Chain.from_urdf_file(
        urdf_path, base_elements=[BASE_LINK], active_links_mask=mask,
    )
    return chain


def solve_ik(chain, target, seed=None):
    """Solve IK and return joint angles for JOINT_NAMES."""
    n = len(chain.links)
    ik_seed = [0.0] * n
    if seed:
        for jname, val in zip(JOINT_NAMES, seed):
            for i, link in enumerate(chain.links):
                if link.name == jname:
                    ik_seed[i] = val
                    break
    angles = chain.inverse_kinematics(target_position=target, initial_position=ik_seed)
    # Extract active joints
    result = []
    for jname in JOINT_NAMES:
        for i, link in enumerate(chain.links):
            if link.name == jname:
                result.append(angles[i])
                break
    return result


def forward_ik(chain, joint_angles):
    """Return (x, y, z) end-effector position for given JOINT_NAMES angles."""
    full = [0.0] * len(chain.links)
    for jname, val in zip(JOINT_NAMES, joint_angles):
        for i, link in enumerate(chain.links):
            if link.name == jname:
                full[i] = val
                break
    fk = chain.forward_kinematics(full)
    return fk[:3, 3].tolist()


# =============================================================================
# Robot interface (ROS2)
# =============================================================================

class RobotInterface(Node):
    """Send trajectories and read joint states."""

    def __init__(self):
        super().__init__('ik_test')
        self.action_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory',
        )
        self.last_joint_state = None
        self._joint_state_updated = False
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10,
        )

    def _joint_state_cb(self, msg):
        self.last_joint_state = msg
        self._joint_state_updated = True

    def wait_for_joint_states(self, timeout=5.0):
        start = time.time()
        while self.last_joint_state is None and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.last_joint_state

    def get_current_angles(self):
        msg = self.last_joint_state
        if msg is None:
            return None
        angles = []
        for jname in JOINT_NAMES:
            if jname in msg.name:
                idx = msg.name.index(jname)
                angles.append(msg.position[idx])
            else:
                angles.append(0.0)
        return angles

    def send_trajectory(self, positions, duration=3.0):
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            print('  ERROR: arm_controller not available')
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)
        secs = int(duration)
        nsecs = int((duration - secs) * 1e9)
        point.time_from_start = Duration(sec=secs, nanosec=nsecs)
        goal.trajectory.points = [point]

        print(f'  Sending: {[f"{a:+.4f}" for a in positions]}')
        future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            print('  ERROR: Goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 5.0)
        print('  Trajectory complete')

        self._joint_state_updated = False
        deadline = time.time() + 2.0
        while not self._joint_state_updated and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return True


# =============================================================================
# Test modes
# =============================================================================

def run_diagnostic(chain, robot):
    """Send robot to known angles, compare FK prediction with actual Gazebo position."""
    print(f'\n{"=" * 60}')
    print('DIAGNOSTIC MODE')
    print('=' * 60)

    print('\n=== Chain structure ===')
    for i, link in enumerate(chain.links):
        active = chain.active_links_mask[i]
        print(f'  [{i}] {link.name:20s} active={active}')

    test_poses = [
        ('All zeros', [0.0, 0.0, 0.0, 0.0, 0.0]),
        ('Random 1', [0.2, -1.3, 0.8, -0.5, 0.3]),
        ('Random 2', [0.5, -1.3, 0.2, -0.7, 0.5]),
        ('Random 3', [0.7, -0.5, 0.3, -0.7, 0.8]),
        ('Shoulder forward', [0.0, -0.5, 0.5, 0.0, 0.0]),
        ('Reaching forward', [0.0, -0.3, 0.8, -0.5, 0.0]),
    ]

    for name, angles in test_poses:
        print(f'\n--- Test: {name} ---')
        print(f'  Commanded angles: {[f"{a:+.3f}" for a in angles]}')

        fk_pos = forward_ik(chain, angles)
        print(f'  FK end-effector:  [{fk_pos[0]:.4f}, {fk_pos[1]:.4f}, {fk_pos[2]:.4f}]')

        spawn_target_marker(fk_pos, name='fk_marker', color='0 0 1 1')

        print('  Sending to robot...')
        robot.send_trajectory(angles, duration=3.0)

        actual = robot.get_current_angles()
        if actual:
            print(f'  Actual angles:    {[f"{a:+.3f}" for a in actual]}')
            angle_errors = [abs(c - a) for c, a in zip(angles, actual)]
            print(f'  Angle errors:     {[f"{e:.4f}" for e in angle_errors]}')

        print(f'\n  >> Look at Gazebo: is the BLUE sphere at the gripper base?')
        print(f'     FK position: [{fk_pos[0]:.4f}, {fk_pos[1]:.4f}, {fk_pos[2]:.4f}]')
        input('     Press Enter to continue...')


def run_ik_test(chain, robot, target, duration):
    """Solve IK for target and send to robot."""
    print('\nSpawning target marker...')
    spawn_target_marker(target, name='ik_target_marker', color='0 1 0 1')

    # Solve IK
    active_angles = solve_ik(chain, target)

    # Verify with FK
    fk_pos = forward_ik(chain, active_angles)
    error = np.linalg.norm(np.array(target) - np.array(fk_pos))

    print('\n=== IK Result ===')
    print(f'  Target:    [{target[0]:.4f}, {target[1]:.4f}, {target[2]:.4f}]')
    print(f'  FK says:   [{fk_pos[0]:.4f}, {fk_pos[1]:.4f}, {fk_pos[2]:.4f}]')
    print(f'  IK error:  {error:.6f} m')
    for name, angle in zip(JOINT_NAMES, active_angles):
        print(f'  {name:20s} = {angle:+.4f} rad  ({np.degrees(angle):+.1f} deg)')

    # Spawn FK marker
    spawn_target_marker(fk_pos, name='fk_marker', color='0 0 1 1')
    print(f'  FK prediction (blue sphere): [{fk_pos[0]:.4f}, {fk_pos[1]:.4f}, {fk_pos[2]:.4f}]')

    # Send to robot
    print('\n=== Sending to robot ===')
    robot.send_trajectory(active_angles, duration=duration)

    actual = robot.get_current_angles()
    if actual:
        print(f'\n  Commanded: {[f"{a:+.4f}" for a in active_angles]}')
        print(f'  Actual:    {[f"{a:+.4f}" for a in actual]}')

    print('\n  >> Look at Gazebo:')
    print('     GREEN sphere = target position')
    print('     BLUE sphere  = where FK thinks gripper base is')
    print('     Compare both with actual gripper base position.')


# =============================================================================
# Main
# =============================================================================

def main():
    parser = argparse.ArgumentParser(description='Minimal IK test for SO-ARM101')
    parser.add_argument('--target', type=float, nargs=3, default=[0.25, 0.0, 0.155],
                        metavar=('X', 'Y', 'Z'))
    parser.add_argument('--duration', type=float, default=3.0)
    parser.add_argument('--dry-run', action='store_true',
                        help='Only solve IK, do not send to robot')
    parser.add_argument('--diag', action='store_true',
                        help='Diagnostic: send known angles and compare FK vs Gazebo')
    args = parser.parse_args()

    cleanup_markers()

    print('Generating URDF from xacro...')
    urdf_path = generate_urdf()

    chain = load_chain(urdf_path)

    if args.dry_run:
        active_angles = solve_ik(chain, args.target)
        fk_pos = forward_ik(chain, active_angles)
        error = np.linalg.norm(np.array(args.target) - np.array(fk_pos))
        print(f'\n=== IK Result ===')
        print(f'  Target:  {args.target}')
        print(f'  FK says: [{fk_pos[0]:.4f}, {fk_pos[1]:.4f}, {fk_pos[2]:.4f}]')
        print(f'  Error:   {error:.6f} m')
        for name, angle in zip(JOINT_NAMES, active_angles):
            print(f'  {name:20s} = {angle:+.4f} rad  ({np.degrees(angle):+.1f} deg)')
        return

    rclpy.init()
    robot = RobotInterface()
    robot.wait_for_joint_states()

    if args.diag:
        run_diagnostic(chain, robot)
    else:
        run_ik_test(chain, robot, args.target, args.duration)

    robot.destroy_node()
    rclpy.shutdown()
    print('\nDone.')


if __name__ == '__main__':
    main()
