"""Full simulation launch: Gazebo + robot + all tracking nodes."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    gazebo_pkg = get_package_share_directory('soarm_gazebo')
    description_pkg = get_package_share_directory('soarm_description')

    # Generate URDF from XACRO and save to /tmp for the arm planner's IK
    xacro_file = os.path.join(description_pkg, 'urdf', 'so101.urdf.xacro')
    urdf_path = '/tmp/so101.urdf'
    with open(urdf_path, 'w') as f:
        f.write(xacro.process_file(xacro_file).toxml())

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
        parameters=[{'use_sim_time': True}],
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
        parameters=[{'use_sim_time': True, 'urdf_path': urdf_path}],
    )

    recorder = Node(
        package='soarm_tracking',
        executable='recorder',
        name='recorder',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        gazebo_launch,
        square_mover,
        color_detector,
        arm_planner,
        recorder,
    ])
