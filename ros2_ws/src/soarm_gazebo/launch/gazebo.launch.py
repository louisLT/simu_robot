"""Launch Gazebo with the tracking world and spawn the SO-ARM101 robot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    # Package paths
    description_pkg = get_package_share_directory('soarm_description')
    gazebo_pkg = get_package_share_directory('soarm_gazebo')

    # Process XACRO to URDF
    xacro_file = os.path.join(description_pkg, 'urdf', 'so101.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # World file
    world_file = os.path.join(gazebo_pkg, 'worlds', 'tracking_world.sdf')

    # Bridge config
    bridge_config = os.path.join(gazebo_pkg, 'config', 'gz_bridge.yaml')

    # --- Nodes ---

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    # Gazebo Harmonic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'so101',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.05',
        ],
        output='screen',
    )

    # ros_gz_bridge (camera, clock)
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
        output='screen',
    )

    # ros_gz_bridge for set_pose service (to teleport models from ROS2)
    gz_set_pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/tracking_world/set_pose@ros_gz_interfaces/srv/SetEntityPose',
        ],
        output='screen',
    )

    # Controller manager: load and activate controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Chain: wait for spawn_robot to finish before starting controllers
    # This avoids "No clock received" warnings (Gazebo needs time to start publishing /clock)
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # Then wait for joint_state_broadcaster before starting arm/gripper controllers
    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner, gripper_controller_spawner],
        )
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_robot,
        gz_bridge,
        gz_set_pose_bridge,
        delay_joint_state_broadcaster,
        delay_arm_controller,
    ])
