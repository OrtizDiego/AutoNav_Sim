# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

**AutoNav Sim** is a ROS 2 Humble simulation environment for autonomous navigation and robot behavior. It combines Gazebo physics simulation, Nav2 path planning, SLAM mapping, and custom Python nodes implementing vision-based behaviors. All development occurs inside a containerized environment using Docker and Docker Compose.

## Quick Start

### Environment Setup
```bash
# Build the Docker image
docker compose build

# Start the container (CPU mode)
make up
# or: docker compose up -d cpu

# Enter the running container
make shell
# or: docker exec -it autonav_cpu bash
```

### Build & Test Workflow
Inside the Docker container, the workspace is at `/root/dev_ws`:

```bash
# Build the ROS 2 workspace (always run after code changes)
make build
# or: cd /root/dev_ws && colcon build --symlink-install

# Run linting (ament_lint_auto + pytest)
make lint

# Run all unit tests
make test

# Clean build artifacts (use if build is corrupted)
make clean
```

### Running the Simulation
```bash
# Launch full simulation (Gazebo + RViz)
make sim
# or: ros2 launch my_bot sim.launch.py

# Launch SLAM (mapping)
make slam
# or: ros2 launch my_bot slam.launch.py

# Launch Nav2 (requires pre-built map)
make nav
# or: ros2 launch my_bot navigation.launch.py

# Control robot with keyboard
make teleop

# Control target/intruder with keyboard
make teleop-target
```

### Behavior Scripts
```bash
# Run ball chaser (reactive vision-based follower)
make ball-chaser
# or: ros2 run my_bot ball_chaser

# Run security guard (patrol + intruder detection)
make security-guard
# or: ros2 run my_bot security_guard
```

## Architecture

### System Layers

**Hardware Interface (Gazebo)**
- Simulates differential drive robot with 2D Lidar and RGB camera
- Publishes: `/scan` (Lidar), `/odom` (odometry), `/camera/image_raw` (camera)
- Subscribes to: `/cmd_vel` (velocity commands)
- Multiple world files: `room.world`, `obstacles.world`, `intruder.world`

**SLAM Toolbox**
- Runs asynchronous SLAM to generate `/map` from Lidar scans and odometry
- Publishes the occupancy grid for navigation
- Configuration: `src/my_bot/config/mapper_params_online_async.yaml`

**Nav2 Stack**
- Provides `/map` → `/odom` → `/base_link` transform chain
- Global planner (A*) and local planner (DWB)
- AMCL particle filter for localization
- Configuration: `src/my_bot/config/nav2_params.yaml`

**Custom Behavior Nodes (Python)**
- **ball_chaser.py**: Subscribes to `/camera/image_raw`, uses OpenCV HSV thresholding to detect red objects, publishes velocity commands to `/cmd_vel` (proportional steering). Search-rotate behavior when no target visible.
- **security_guard.py**: ROS 2 LifecycleNode — patrol + intruder detection. Lifecycle transitions: configure → activate → deactivate → cleanup. Uses `/target_range` from sensor_fusion for accurate distance.
- **security_guard_bt.py**: py_trees BehaviorTree version of security_guard. Selector → [IntruderProtocol, PatrolProtocol]. Supports HSV + YOLO dual detection via blackboard. Publishes mission metrics to `/security_guard/metrics` and sighting markers to `/intruder_sightings`.
- **sensor_fusion.py**: Fuses LiDAR bearing lookup with camera contour detection. Publishes `/target_range` (Float32) and `/target_position` (PointStamped).
- **system_monitor.py**: Watchdog — monitors /scan and /camera heartbeats, publishes `/system_health` (DiagnosticArray), provides `/trigger_estop` service.
- **object_detector.py**: YOLOv8-nano ONNX inference node. GPU auto-select (CUDA → CPU fallback). Publishes `/detections` (MarkerArray), `/person_detected`, `/target_detected`.
- **intruder_bot.py**: Autonomous random-walk node driving the intruder sphere via `/target_cmd_vel`.
- **obstacle_controller.py**: Drives dynamic world obstacles (moving_box_1, moving_cylinder_1) with timed random walk.
- **person_controller.py**: Two-mode pedestrian FSM (WALK / RUN). Drives the Gazebo `<actor>` in `person.world` via `/gazebo/set_entity_state`. Subscribes to `/person_detected` — when seen, switches to RUN and flees from the robot's odom pose; returns to WALK after `calm_down_secs`.
- **person_tracker.py**: Hybrid YOLO + OpenCV-CSRT object tracker. Runs YOLOv8n every `redetect_every` frames to seed/correct an OpenCV CSRT tracker that propagates the box every frame. Publishes `/person_bbox` (Float32MultiArray [x,y,w,h]), `/person_track` (PointStamped centroid), `/person_detected` (Bool).
- **person_follower.py**: Stand-off follower. Reads `/person_bbox` + `/scan`, projects the bbox centroid into a lidar bearing, fetches the metric range, then runs a P-controller on (range − desired_distance) and bearing. Stops on track loss.
- **patrol.py**: Simple waypoint navigation (superseded by security_guard)

**RViz Visualization**
- Displays map, costmaps, Lidar scans, planned path, and camera feed
- Configuration: `src/my_bot/config/navigation.rviz`

### Key Files & Directories

```
src/my_bot/
├── my_bot/                    # Python nodes (behavior logic)
│   ├── ball_chaser.py         # Red ball detection & following (threaded display)
│   ├── security_guard.py      # LifecycleNode — patrol + intruder detection
│   ├── security_guard_bt.py   # py_trees BT version of security_guard
│   ├── sensor_fusion.py       # LiDAR-camera fusion → /target_range
│   ├── system_monitor.py      # Watchdog — heartbeats + /trigger_estop
│   ├── object_detector.py     # YOLOv8-nano ONNX detector (GPU/CPU)
│   ├── intruder_bot.py        # Autonomous random-walk intruder node
│   ├── obstacle_controller.py # Dynamic obstacle random-walk driver
│   ├── person_controller.py   # WALK/RUN pedestrian FSM driving the actor pose
│   ├── person_tracker.py      # YOLO + OpenCV CSRT person tracker
│   ├── person_follower.py     # Stand-off follower (proportional control)
│   ├── patrol.py              # Basic waypoint navigation
│   └── camera_test.py         # Debug camera feed
├── launch/                    # Python launch files
│   ├── sim.launch.py          # Gazebo + RViz
│   ├── slam.launch.py         # SLAM mapping
│   ├── navigation.launch.py   # Nav2 stack
│   ├── rsp.launch.py          # Robot State Publisher (Xacro)
│   ├── sensor_fusion.launch.py        # sensor_fusion node only
│   ├── security_guard_full.launch.py  # sensor_fusion + security_guard + monitor
│   ├── dynamic_sim.launch.py          # Gazebo with dynamic world + obstacles
│   └── person_sim.launch.py           # person.world + tracker + controller + follower
├── config/                    # Parameters & visualization configs
│   ├── nav2_params.yaml       # Navigation stack tuning
│   ├── mapper_params_online_async.yaml  # SLAM tuning
│   ├── behavior_params.yaml   # Centralized behavior node parameters
│   ├── navigation.rviz        # RViz layout
│   └── view_robot.rviz        # Alternative RViz layout
├── urdf/                      # Robot description (Xacro)
│   ├── robot.urdf.xacro       # Main robot description
│   ├── robot_core.xacro       # Body + differential drive
│   ├── gazebo_control.xacro   # Gazebo plugins; wheel friction mu1/mu2=1.0
│   ├── lidar.xacro            # 2D Lidar + Gaussian noise (stddev=0.01m)
│   └── camera.xacro           # RGB camera 30Hz + Gaussian noise (stddev=0.007)
├── worlds/                    # Gazebo environment files
│   ├── room.world             # Simple room environment
│   ├── obstacles.world        # Complex environment with obstacles
│   ├── intruder.world         # Room with target/intruder sphere
│   ├── dynamic.world          # intruder.world + 2 moving obstacles
│   └── person.world           # Walking/running pedestrian actor (walk.dae)
├── maps/                      # Saved occupancy grids
│   └── my_map.yaml / my_map.pgm
├── test/                      # Unit tests (69/70 pass without ROS runtime)
│   ├── test_urdf.py           # Xacro XML validation + noise/friction checks
│   ├── test_scripts.py        # Script shebang & py_compile checks
│   ├── test_behavior_params.py  # behavior_params.yaml structure validation
│   ├── test_sensor_fusion.py  # Pure-Python math tests (18 tests)
│   ├── test_behavior_tree.py  # BT leaf + tree structure tests (28 tests)
│   └── test_object_detector.py  # preprocess/postprocess tests (15 tests)
└── package.xml                # ROS 2 package manifest
```

### Development Workflow

1. **Code Changes**: Modify Python nodes in `src/my_bot/my_bot/`, launch files, or configs
2. **Rebuild**: Run `make build` inside the container (uses colcon with symlink-install for faster iteration)
3. **Test**: Run `make lint` and `make test` before committing
4. **Run**: Use appropriate launch command (sim, slam, nav) and behavior scripts
5. **Debug**: 
   - Check ROS 2 topics: `ros2 topic echo /topic_name`
   - Verify transforms: `ros2 run tf2_ros tf2_echo frame1 frame2`
   - View node lifecycle: `ros2 lifecycle get node_name`

### CI/CD Pipeline

The GitHub Actions workflow (`.github/workflows/ci.yml`) runs on every push and PR to `main`:
1. **Linting**: `ament_lint_auto` (PEP 8, naming conventions, URDF validity)
2. **URDF Validation**: Xacro parsing of `robot.urdf.xacro`
3. **Script Validation**: Checks shebangs and executability
4. **Unit Tests**: Pytest coverage report uploaded to Codecov

Run these locally with:
```bash
make lint   # Runs ament_lint_auto + pytest with coverage
make test   # Runs pytest without coverage
```

## Important Notes

### Workspace Sourcing
Every new terminal inside the container requires:
```bash
source /opt/ros/humble/setup.bash
source /root/dev_ws/install/setup.bash
```
(Already in `.bashrc`, so happens automatically in interactive shells)

### Simulation Time (`use_sim_time`)
All nodes that read time (SLAM, Nav2, AMCL) must have `use_sim_time=true`. Launch files handle this automatically. Verify with:
```bash
ros2 param get /slam_toolbox use_sim_time  # Should be "True"
```

### Map Files
Nav2 requires a pre-built map (`maps/my_map.yaml` + `maps/my_map.pgm`). To create:
1. Launch SLAM: `make slam` + `make teleop` in separate terminal
2. Drive robot to explore
3. Save map: `ros2 run nav2_map_server map_saver_cli -f /root/dev_ws/src/my_bot/maps/my_map`

### Common Issues

**AMCL cannot publish pose** → Reset Gazebo world (`Ctrl+R`) and re-initialize: 
```bash
ros2 topic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped "{ header: { frame_id: 'map' }, pose: { pose: { position: { x: 0.0, y: 0.0, z: 0.0 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } } } }"
```

**Vision nodes not detecting objects** → Check HSV thresholds in `ball_chaser.py` (lines 54-58) and `security_guard.py` (lines 66-68); tuned for red objects under current lighting

**"Node not found" errors** → Workspace may be out of sync; run `make clean && make build`

## Entry Points (Console Scripts)

Defined in `setup.py`:
- `patrol` → `my_bot.patrol:main`
- `ball_chaser` → `my_bot.ball_chaser:main`
- `security_guard` → `my_bot.security_guard:main`
- `security_guard_bt` → `my_bot.security_guard_bt:main`
- `sensor_fusion` → `my_bot.sensor_fusion:main`
- `system_monitor` → `my_bot.system_monitor:main`
- `object_detector` → `my_bot.object_detector:main`
- `intruder_bot` → `my_bot.intruder_bot:main`
- `obstacle_controller` → `my_bot.obstacle_controller:main`
- `person_controller` → `my_bot.person_controller:main`
- `person_tracker` → `my_bot.person_tracker:main`
- `person_follower` → `my_bot.person_follower:main`
- `camera_test` → `my_bot.camera_test:main`

Run with `ros2 run my_bot <script_name>` or via Make targets.

### Key Make Targets
| Target | Description |
|--------|-------------|
| `make sim` | Full Gazebo + RViz simulation |
| `make security-guard` | LifecycleNode patrol + intruder detection |
| `make security-guard-bt` | py_trees BT version |
| `make sensor-fusion` | LiDAR-camera sensor fusion node |
| `make system-monitor` | Watchdog + e-stop service |
| `make object-detector` | YOLOv8-nano ONNX inference |
| `make intruder-bot` | Autonomous random-walk intruder |
| `make dynamic-sim` | Simulation with moving obstacles |
| `make person-sim` | Full person-intruder sim (walking/running actor + YOLO tracker + follower) |
| `make person-controller` | WALK/RUN pedestrian FSM (drives the Gazebo actor) |
| `make person-tracker` | YOLO + CSRT person tracker only |
| `make person-follower` | Stand-off follower only |
| `make up-gpu` | Start GPU container (autonav_gpu) |

## Dependencies

Key ROS 2 packages (installed in Dockerfile):
- `nav2_bringup`: Navigation stack
- `slam_toolbox`: SLAM mapping
- `gazebo_ros_pkgs`: Gazebo integration
- `robot_localization`: AMCL
- `py_trees_ros`: Behavior Tree support
- `cv_bridge`: OpenCV ↔ ROS image conversion
- Standard: `geometry_msgs`, `sensor_msgs`, `diagnostic_msgs`, `visualization_msgs`, `tf2_ros`, etc.

Python: `cv2` (OpenCV), `numpy`, `onnxruntime-gpu` (CPU fallback), ROS 2 Python client library (rclpy)

### GPU Notes
- Container: `docker compose --profile gpu up -d` → starts `autonav_gpu`
- `object_detector.py` auto-selects `CUDAExecutionProvider` if NVIDIA GPU available
- YOLOv8n model downloaded to `/root/models/yolov8n.onnx` during image build
- Override container: `make build CONTAINER=autonav_gpu`

