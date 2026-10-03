# Makefile for AutoNav_Sim ROS 2 Project

# --- CONFIGURATION ---
SHELL := /bin/bash
# Override: make build CONTAINER=autonav_gpu  (for GPU mode)
CONTAINER ?= autonav_cpu
CONTAINER_NAME := $(CONTAINER)
# Compose service behind the container: autonav_cpu -> cpu, autonav_gpu -> gpu
SERVICE := $(patsubst autonav_%,%,$(CONTAINER_NAME))
WS_PATH := /root/dev_ws
PACKAGE_NAME := my_bot

# Helper to run commands inside the container
EXEC := docker exec -it $(CONTAINER_NAME) bash -c
SOURCE := source /opt/ros/humble/setup.bash && source install/setup.bash
# The YOLO scenarios stop here instead of tracking nothing when the container
# runs an image from before the model export (docker compose build alone
# leaves a running container on its old image; make image recreates it).
YOLO_MODEL := /root/models/yolov8n.onnx
NEED_YOLO := test -s $(YOLO_MODEL) || { \
	echo 'No YOLO model at $(YOLO_MODEL): $(CONTAINER_NAME) runs an image from before the model export.'; \
	echo 'Fix, on the host: make image CONTAINER=$(CONTAINER_NAME)  (rebuilds the image, recreates the container)'; \
	exit 1; };
# A scenario starts from a clean container: stop what an earlier run left
# behind (closing a terminal does not stop docker exec; see src/stop_sim.sh).
STOP_LEFTOVERS := bash src/stop_sim.sh;

.PHONY: help up up-gpu down shell image build clean lint test \
        sim nav-sim slam nav teleop save-map system-monitor \
        ball-sim teleop-ball person-sim yolo-sim perf stop

help:
	@echo "AutoNav_Sim Makefile"
	@echo "--------------------"
	@echo "Environment:"
	@echo "  up              - Start the Docker container (CPU mode)"
	@echo "  up-gpu          - Start the Docker container (GPU mode, NVIDIA WSL2)"
	@echo "  down            - Stop the Docker container"
	@echo "  shell           - Enter the running container"
	@echo "  image           - Rebuild the image and recreate the container on it"
	@echo "                    (after Dockerfile changes; also rebuilds the workspace)"
	@echo "  Override: make <target> CONTAINER=autonav_gpu"
	@echo ""
	@echo "Development:"
	@echo "  build           - Build the ROS 2 workspace"
	@echo "  clean           - Remove build/install/log directories"
	@echo "  lint            - Run ROS 2 linting tools"
	@echo "  test            - Run ROS 2 unit tests"
	@echo ""
	@echo "Scenarios (each is a single command):"
	@echo "  sim             - Robot in the museum, Gazebo + RViz, nothing else"
	@echo "  nav-sim         - sim + Nav2 (map, AMCL, planners) in one go"
	@echo "  ball-sim        - Robot chases the red ball (HSV + lidar fusion)"
	@echo "  person-sim      - Security guard: patrol + YOLO + fusion + follow (the demo)"
	@echo "  yolo-sim        - Static person in front of the robot: YOLO + fusion proof"
	@echo ""
	@echo "Tools (run next to a scenario, in a second terminal):"
	@echo "  teleop          - Drive the robot with the keyboard"
	@echo "  teleop-ball     - Drive the ball in ball-sim with the keyboard"
	@echo "  slam            - SLAM mapping (with sim + teleop)"
	@echo "  save-map NAME=x - Save the current SLAM map (default: my_map)"
	@echo "  nav             - Nav2 only (with an already running sim)"
	@echo "  system-monitor  - Sensor watchdog + /trigger_estop (built into person-sim)"
	@echo "  perf            - Real-time factor, topic rates and latencies of a running scenario"
	@echo "  stop            - Stop every scenario process in the container (each scenario"
	@echo "                    also does this first: a closed terminal leaves them running)"

# --- DOCKER MANAGEMENT ---

up:
	docker compose up -d cpu

up-gpu:
	docker compose --profile gpu up -d

down:
	docker compose down

shell:
	docker exec -it $(CONTAINER_NAME) bash

# docker compose build alone does not touch the running container, so every
# make target would keep using the old image. Recreate it, then rebuild the
# workspace (the image's install/ is a copy, make build symlinks src/).
image:
	docker compose build
	docker compose up -d --force-recreate $(SERVICE)
	$(MAKE) clean build CONTAINER=$(CONTAINER_NAME)

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

# --- SCENARIOS ---

sim:
	$(EXEC) "$(STOP_LEFTOVERS) $(SOURCE) && ros2 launch $(PACKAGE_NAME) sim.launch.py"

nav-sim:
	$(EXEC) "$(STOP_LEFTOVERS) $(SOURCE) && ros2 launch $(PACKAGE_NAME) nav_sim.launch.py"

ball-sim:
	$(EXEC) "$(STOP_LEFTOVERS) $(SOURCE) && ros2 launch $(PACKAGE_NAME) ball_sim.launch.py"

person-sim:
	$(EXEC) "$(STOP_LEFTOVERS) $(NEED_YOLO) $(SOURCE) && ros2 launch $(PACKAGE_NAME) person_sim.launch.py"

yolo-sim:
	$(EXEC) "$(STOP_LEFTOVERS) $(NEED_YOLO) $(SOURCE) && ros2 launch $(PACKAGE_NAME) yolo_sim.launch.py"

# --- TOOLS ---

teleop:
	$(EXEC) "$(SOURCE) && ros2 run teleop_twist_keyboard teleop_twist_keyboard"

teleop-ball:
	$(EXEC) "$(SOURCE) && ros2 run $(PACKAGE_NAME) ball_teleop"

slam:
	$(EXEC) "$(SOURCE) && ros2 launch $(PACKAGE_NAME) slam.launch.py"

save-map:
	@MAP_NAME=$(or $(NAME),my_map); \
	$(EXEC) "$(SOURCE) && ./src/save_map.sh $$MAP_NAME"

nav:
	$(EXEC) "$(SOURCE) && ros2 launch $(PACKAGE_NAME) navigation.launch.py"

system-monitor:
	$(EXEC) "$(SOURCE) && ros2 run $(PACKAGE_NAME) system_monitor --ros-args --params-file src/$(PACKAGE_NAME)/config/behavior_params.yaml -p use_sim_time:=true"

perf:
	$(EXEC) "$(SOURCE) && ros2 run $(PACKAGE_NAME) perf_monitor --ros-args -p use_sim_time:=true"

stop:
	$(EXEC) "$(STOP_LEFTOVERS)"
