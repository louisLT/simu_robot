# =============================================================================
# SO-ARM101 Simulation - Makefile
# =============================================================================

COMPOSE = docker compose -f docker/docker-compose.yml

.PHONY: build up down shell rebuild sim rviz clean

# Build the Docker image
build:
	$(COMPOSE) build sim

# Start simulation container (interactive shell)
up:
	xhost +local:docker 2>/dev/null || true
	$(COMPOSE) up -d sim

# Stop all containers
down:
	$(COMPOSE) down

# Open a shell in the running sim container
shell:
	docker exec -it soarm-sim /entrypoint.sh bash

# Rebuild ROS2 workspace inside container
rebuild:
	docker exec -it soarm-sim bash -c "cd /ws/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

# Launch the full simulation
sim:
	xhost +local:docker 2>/dev/null || true
	$(COMPOSE) run --rm sim ros2 launch soarm_gazebo full_sim.launch.py

# Launch RViz only
rviz:
	xhost +local:docker 2>/dev/null || true
	$(COMPOSE) run --rm rviz

# Remove containers and images
clean:
	$(COMPOSE) down --rmi local --volumes
