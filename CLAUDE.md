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

### Scenarios
Each scenario is one command (Gazebo + RViz layout + every node it needs). Make targets `docker exec` into the container, so run them from the host:
```bash
make sim         # robot in the museum (room.world), nothing else
make nav-sim     # sim + Nav2 (map, AMCL with initial pose from nav2_params.yaml)
make ball-sim    # ball chase: ball_controller + sensor_fusion(hsv) + ball_chaser
make person-sim  # demo: Nav2 + person_controller + person_tracker + sensor_fusion(person)
                 #       + security_guard_bt + system_monitor
make yolo-sim    # static person ahead: person_tracker + sensor_fusion(person)
```

### Tools (second terminal)
```bash
make teleop          # drive the robot
make teleop-ball     # drive the ball in ball-sim (hold keys; robot-relative or world frame)
make slam            # SLAM (with sim + teleop), then make save-map NAME=x
make nav             # Nav2 only, next to a running sim
make system-monitor  # watchdog + /trigger_estop, /clear_estop (already in person-sim)
```

## Architecture

### System Layers

**Hardware Interface (Gazebo)**
- Simulates a differential drive robot (TurtleBot3 Waffle Pi meshes/dimensions) with 2D Lidar and RGB camera
- Publishes: `/scan` (Lidar), `/odom` (odometry), `/camera/image_raw` (camera)
- Subscribes to: `/cmd_vel` (velocity commands)
- World files: `room.world` (museum, vanilla), `ball.world` (museum + red ball), `person.world` (museum + pedestrian actor), `yolo.world` (flat ground + standing person). `sim.launch.py` takes `world:=` and `rviz_config:=`. The museum block (incl. its `<state>` pose) must stay identical across the museum worlds: `maps/my_map` was built in it and its frame equals the world frame (robot spawns at the origin).

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
- **sensor_fusion.py**: `mode` `hsv` (largest red blob in the camera) or `person` (`/person_bbox` from person_tracker). Range = low percentile of lidar beams across the box's angular span; in person mode cross-checked against a monocular estimate (person height, or feet ground contact). Image bearings are positive-right, so they are negated for ROS angles. Publishes `/target_range` (Float32, -1 unknown), `/target_bearing` (Float32 rad positive-left, NaN when no target), `/target_position` (PointStamped base_link), `/sensor_fusion/image` (annotated debug).
- **follow_control.py** (library): stand-off P-control on range + bearing with a front safety stop, used by ball_chaser and the BT.
- **ball_chaser.py**: follows the fused target at 1 m; turns toward the last-seen side when lost.
- **ball_controller.py**: drives the ball (`/ball/cmd_vel`, planar_move plugin, body frame, yaw held at 0) on a figure-eight checked against the map; flees a close robot, waits for a far one, pauses/reverses at random. `/ball/teleop` (TwistStamped, frame_id `robot`|`world`) overrides it while messages arrive.
- **ball_teleop.py**: hold-to-move keyboard teleop for the ball.
- **person_tracker.py**: YOLOv8n detection (pre/post-processing in `object_detector.py`) + OpenCV tracker (CSRT → KCF → MIL fallback) + constant-velocity Kalman filter. Publishes `/person_bbox` (Float32MultiArray [x,y,w,h]), `/person_track`, `/person_detected` (Bool), `/person_tracker/image`.
- **person_controller.py**: Pedestrian behaviour (WALK / RUN / EXHAUSTED) publishing `/person/cmd_vel` for the actor plugin. Pure `PersonBrain` steers on `clearance_map.py` (distance transform of `maps/my_map`, passed as `map_yaml`): line-of-sight wander targets, `safe_heading` fan search, speed capped to stop before walls. A `/person_detected` lock within `notice_radius` triggers a stamina-limited sprint away from the robot.
- **security_guard_bt.py**: py_trees tree Selector → [EmergencyStop, IntruderProtocol (follow at 2.5 m), SearchProtocol (turn to last-seen side), PatrolProtocol (Nav2 waypoints)]. Detector-agnostic: reads sensor_fusion topics. Publishes `/security_guard/state`, `/security_guard/metrics`, `/intruder_sightings`.
- **system_monitor.py**: heartbeats → `/system_health`; `/trigger_estop` latches `/estop` (Bool, transient local) which the BT obeys; `/clear_estop` releases.

**RViz Visualization**
- Displays map, costmaps, Lidar scans, planned path, and camera feed
- Configuration: `src/my_bot/config/navigation.rviz`

### Key Files & Directories

```
src/my_bot/
├── my_bot/                    # Python nodes + pure libraries
│   ├── sensor_fusion.py       # camera box + lidar → range/bearing (hsv | person)
│   ├── follow_control.py      # stand-off follow law (library)
│   ├── ball_chaser.py         # ball-sim follower
│   ├── ball_controller.py     # ball autopilot + teleop arbitration
│   ├── ball_teleop.py         # keyboard teleop for the ball
│   ├── person_tracker.py      # YOLO + OpenCV tracker + Kalman
│   ├── object_detector.py     # YOLOv8 preprocess/postprocess (library)
│   ├── person_controller.py   # WALK/RUN/EXHAUSTED pedestrian, map-aware
│   ├── clearance_map.py       # distance-to-wall lookups on the saved map (library)
│   ├── security_guard_bt.py   # py_trees security guard
│   └── system_monitor.py      # watchdog + e-stop
├── launch/                    # sim, nav_sim, ball_sim, person_sim, yolo_sim,
│                              # navigation, slam, rsp
├── config/
│   ├── behavior_params.yaml   # one ros__parameters section per behaviour node
│   ├── nav2_params.yaml       # Nav2 tuning (AMCL initial pose = spawn)
│   ├── mapper_params_online_async.yaml
│   └── sim.rviz / navigation.rviz / perception.rviz / person.rviz
├── meshes/                    # TurtleBot3 Waffle Pi STL meshes (Apache-2.0, see meshes/README.md)
├── urdf/                      # Xacro: robot_core, gazebo_control, lidar (noise 0.01 m), camera (noise 0.007)
├── worlds/                    # room, ball, person, yolo
├── maps/                      # my_map.yaml / my_map.pgm (museum; also used by tests)
├── test/                      # pytest; conftest.py stubs ROS so tests run without it
└── package.xml

src/person_actor_plugin/          # ament_cmake Gazebo plugin package
└── src/person_actor_plugin.cpp   # Twist-driven actor; walk/run clip switching, gait synced to distance
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

The same map drives person_controller's wall avoidance and is checked by the ball-path and patrol-waypoint tests, so re-run `make test` after re-mapping.

### Common Issues

**AMCL cannot publish pose** → Reset Gazebo world (`Ctrl+R`) and re-initialize: 
```bash
ros2 topic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped "{ header: { frame_id: 'map' }, pose: { pose: { position: { x: 0.0, y: 0.0, z: 0.0 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } } } }"
```

**Ball not detected** → Check the `sensor_fusion` HSV thresholds in `config/behavior_params.yaml`; open the "Sensor fusion" image in RViz to see what it sees

**"Node not found" errors** → Workspace may be out of sync; run `make clean && make build`

## Entry Points (Console Scripts)

Defined in `setup.py` (test_scripts.py checks they match the modules):
`ball_controller`, `ball_teleop`, `ball_chaser`, `sensor_fusion`, `person_controller`, `person_tracker`, `security_guard_bt`, `system_monitor`.

Run with `ros2 run my_bot <script_name>`, but prefer the scenario make targets, which load `behavior_params.yaml` and `use_sim_time`.

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
- `person_tracker.py` auto-selects `CUDAExecutionProvider` if an NVIDIA GPU is available
- YOLOv8n model downloaded to `/root/models/yolov8n.onnx` during image build
- Override container: `make build CONTAINER=autonav_gpu`

