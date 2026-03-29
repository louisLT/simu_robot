#!/bin/bash
set -e

# Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Source workspace overlay (if built)
if [ -f /ws/ros2_ws/install/setup.bash ]; then
    source /ws/ros2_ws/install/setup.bash
fi

# Set Gazebo resource path for meshes
export GZ_SIM_RESOURCE_PATH="/ws/ros2_ws/install/soarm_description/share:${GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_RESOURCE_PATH="/ws/ros2_ws/install/soarm_gazebo/share:${GZ_SIM_RESOURCE_PATH}"

exec "$@"
