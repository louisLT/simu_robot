"""Full simulation launch: Gazebo + robot + all tracking nodes."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('soarm_gazebo')

    # Include the base Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
        )
    )

    # Tracking nodes
    square_mover = Node(
        package='soarm_tracking',
        executable='square_mover',
        name='square_mover',
        output='screen',
        parameters=[{'use_sim_time': True, 'move_interval': 7.0}],
    )

    color_detector = Node(
        package='soarm_tracking',
        executable='color_detector',
        name='color_detector',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    arm_planner = Node(
        package='soarm_tracking',
        executable='arm_planner',
        name='arm_planner',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    recorder = Node(
        package='soarm_tracking',
        executable='recorder',
        name='recorder',
        output='screen',
        parameters=[{'use_sim_time': True, 'output_dir': '/ws/recordings'}],
    )

    return LaunchDescription([
        gazebo_launch,
        square_mover,
        color_detector,
        arm_planner,
        recorder,
    ])
