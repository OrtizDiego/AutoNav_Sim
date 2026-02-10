# Makefile for AutoNav_Sim ROS 2 Project

# --- CONFIGURATION ---
SHELL := /bin/bash
CONTAINER_NAME := autonav_cpu
WS_PATH := /root/dev_ws
PACKAGE_NAME := my_bot

# Helper to run commands inside the container
EXEC := docker exec -it $(CONTAINER_NAME) bash -c
SOURCE := source /opt/ros/humble/setup.bash && source install/setup.bash

.PHONY: help build clean lint test shell sim slam nav teleop save-map ball-chaser security-guard up down

help:
	@echo "AutoNav_Sim Makefile"
	@echo "--------------------"
	@echo "Environment:"
	@echo "  up              - Start the Docker container (CPU mode)"
	@echo "  down            - Stop the Docker container"
	@echo "  shell           - Enter the running container"
	@echo ""
	@echo "Development:"
	@echo "  build           - Build the ROS 2 workspace"
	@echo "  clean           - Remove build/install/log directories"
	@echo "  lint            - Run ROS 2 linting tools"
	@echo "  test            - Run ROS 2 unit tests"
	@echo ""
	@echo "Simulation & Navigation:"
	@echo "  sim             - Launch Gazebo simulation"
	@echo "  slam            - Start SLAM for mapping"
	@echo "  nav             - Start Autonomous Navigation"
	@echo "  teleop          - Control the robot with arrow keys"
	@echo "  teleop-target   - Control the Intruder/Ball with arrow keys"
	@echo "  save-map NAME=x - Save the current map (default: my_map)"
	@echo ""
	@echo "Scripts:"
	@echo "  ball-chaser     - Run the ball chaser script"
	@echo "  security-guard  - Run the security guard state machine"

# --- DOCKER MANAGEMENT ---

up:
	docker compose up -d cpu

down:
	docker compose down

shell:
	docker exec -it $(CONTAINER_NAME) bash

# --- DEVELOPMENT ---

build:
	$(EXEC) "cd $(WS_PATH) && source /opt/ros/humble/setup.bash && colcon build --symlink-install"

clean:
	$(EXEC) "cd $(WS_PATH) && rm -rf build/ install/ log/"

lint:
	$(EXEC) "cd $(WS_PATH) && \
		source /opt/ros/humble/setup.bash && \
		colcon build --symlink-install --packages-select $(PACKAGE_NAME) --cmake-args -DBUILD_TESTING=ON && \
		source install/setup.bash && \
		colcon test --packages-select $(PACKAGE_NAME) --ctest-args -R lint && \
		colcon test-result --verbose"

test:
	$(EXEC) "cd $(WS_PATH) && \
		$(SOURCE) && \
		colcon test --packages-select $(PACKAGE_NAME) --return-code-on-test-failure && \
		colcon test-result --verbose"

# --- ROS 2 WORKFLOWS ---

sim:
	$(EXEC) "$(SOURCE) && ros2 launch $(PACKAGE_NAME) sim.launch.py"

slam:
	$(EXEC) "$(SOURCE) && ros2 launch $(PACKAGE_NAME) slam.launch.py"

nav:
	$(EXEC) "$(SOURCE) && ros2 launch $(PACKAGE_NAME) navigation.launch.py"

teleop:
	$(EXEC) "$(SOURCE) && ros2 run teleop_twist_keyboard teleop_twist_keyboard"

teleop-target:
	$(EXEC) "$(SOURCE) && ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/target_cmd_vel"

save-map:
	@MAP_NAME=$(or $(NAME),my_map); \
	$(EXEC) "$(SOURCE) && ./src/save_map.sh $$MAP_NAME"

# --- INTELLIGENT SCRIPTS ---

ball-chaser:
	$(EXEC) "$(SOURCE) && ros2 run $(PACKAGE_NAME) ball_chaser"

security-guard:
	$(EXEC) "$(SOURCE) && ros2 run $(PACKAGE_NAME) security_guard"
