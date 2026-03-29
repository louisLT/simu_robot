# =============================================================================
# SO-ARM101 Simulation - Dockerfile
# Base: ROS2 Jazzy Desktop + Gazebo Harmonic
# =============================================================================

# ---------------------------------------------------------------------------
# Stage 1: Base image with all system dependencies
# ---------------------------------------------------------------------------
FROM osrf/ros:jazzy-desktop AS base

ENV DEBIAN_FRONTEND=noninteractive

# Gazebo Harmonic + ROS2-Gazebo integration + ros2_control
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-ros2-control \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    python3-opencv \
    python3-pip \
    curl \
    git \
    && rm -rf /var/lib/apt/lists/*

# Install uv (Python package manager)
RUN curl -LsSf https://astral.sh/uv/install.sh | sh
ENV PATH="/root/.local/bin:$PATH"
ENV UV_BREAK_SYSTEM_PACKAGES=1

# ---------------------------------------------------------------------------
# Stage 2: Development image with workspace
# ---------------------------------------------------------------------------
FROM base AS dev

# Create workspace
WORKDIR /ws

# Copy Python project config and install deps (system-wide, no venv)
COPY pyproject.toml /ws/
RUN uv pip install --system -r pyproject.toml 2>/dev/null || \
    uv pip install --system ikpy opencv-python numpy

# Copy ROS2 workspace source
COPY ros2_ws/src /ws/ros2_ws/src

# Build ROS2 workspace
WORKDIR /ws/ros2_ws
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install

# Back to workspace root
WORKDIR /ws

# Copy entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
