"""Launch RViz with the SO-ARM101 robot model."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    description_pkg = get_package_share_directory('soarm_description')

    xacro_file = os.path.join(description_pkg, 'urdf', 'so101.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    rviz_config = os.path.join(description_pkg, 'rviz', 'default.rviz')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])
