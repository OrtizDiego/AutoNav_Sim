# Makefile for AutoNav_Sim ROS 2 Project

# --- CONFIGURATION ---
SHELL := /bin/bash
# Override: make build CONTAINER=autonav_gpu  (for GPU mode)
CONTAINER ?= autonav_cpu
CONTAINER_NAME := $(CONTAINER)
WS_PATH := /root/dev_ws
PACKAGE_NAME := my_bot

# Helper to run commands inside the container
EXEC := docker exec -it $(CONTAINER_NAME) bash -c
SOURCE := source /opt/ros/humble/setup.bash && source install/setup.bash

.PHONY: help build clean lint test shell sim slam nav teleop save-map \
        ball-chaser security-guard security-guard-bt \
        sensor-fusion system-monitor object-detector \
        intruder-bot obstacle-controller dynamic-sim \
        up up-gpu down

help:
	@echo "AutoNav_Sim Makefile"
	@echo "--------------------"
	@echo "Environment:"
	@echo "  up              - Start the Docker container (CPU mode)"
	@echo "  up-gpu          - Start the Docker container (GPU mode, NVIDIA WSL2)"
	@echo "  down            - Stop the Docker container"
	@echo "  shell           - Enter the running container"
	@echo "  Override: make <target> CONTAINER=autonav_gpu"
	@echo ""
	@echo "Development:"
	@echo "  build           - Build the ROS 2 workspace"
	@echo "  clean           - Remove build/install/log directories"
	@echo "  lint            - Run ROS 2 linting tools"
	@echo "  test            - Run ROS 2 unit tests"
	@echo ""
	@echo "Simulation & Navigation:"
	@echo "  sim             - Launch Gazebo simulation (room.world)"
	@echo "  dynamic-sim     - Launch Gazebo simulation (dynamic.world with moving obstacles)"
	@echo "  slam            - Start SLAM for mapping"
	@echo "  nav             - Start Autonomous Navigation"
	@echo "  teleop          - Control the robot with arrow keys"
	@echo "  teleop-target   - Control the Intruder/Ball with arrow keys"
	@echo "  save-map NAME=x - Save the current map (default: my_map)"
	@echo ""
	@echo "Behavior Nodes:"
	@echo "  ball-chaser        - Run the ball chaser (HSV + sensor fusion)"
	@echo "  security-guard     - Run the security guard (LifecycleNode)"
	@echo "  security-guard-bt  - Run the security guard (Behavior Tree)"
	@echo "  sensor-fusion      - Run LiDAR-camera fusion node"
	@echo "  system-monitor     - Run system watchdog + full security stack"
	@echo "  object-detector    - Run YOLOv8-nano ONNX object detector"
	@echo "  intruder-bot       - Run autonomous intruder random walk"
	@echo "  obstacle-controller- Run dynamic obstacle controller"

# --- DOCKER MANAGEMENT ---

up:
	docker compose up -d cpu

up-gpu:
	docker compose --profile gpu up -d

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

# --- BEHAVIOR NODES ---

ball-chaser:
	$(EXEC) "$(SOURCE) && ros2 run $(PACKAGE_NAME) ball_chaser"

security-guard:
	$(EXEC) "$(SOURCE) && ros2 run $(PACKAGE_NAME) security_guard"

security-guard-bt:
	$(EXEC) "$(SOURCE) && ros2 run $(PACKAGE_NAME) security_guard_bt"

sensor-fusion:
	$(EXEC) "$(SOURCE) && ros2 launch $(PACKAGE_NAME) sensor_fusion.launch.py"

system-monitor:
	$(EXEC) "$(SOURCE) && ros2 launch $(PACKAGE_NAME) security_guard_full.launch.py"

object-detector:
	$(EXEC) "$(SOURCE) && ros2 run $(PACKAGE_NAME) object_detector"

intruder-bot:
	$(EXEC) "$(SOURCE) && ros2 run $(PACKAGE_NAME) intruder_bot"

obstacle-controller:
	$(EXEC) "$(SOURCE) && ros2 run $(PACKAGE_NAME) obstacle_controller"

dynamic-sim:
	$(EXEC) "$(SOURCE) && ros2 launch $(PACKAGE_NAME) dynamic_sim.launch.py"
