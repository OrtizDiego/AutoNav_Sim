# Master Cheat Sheet

These are all the critical commands used to build, map, and navigate with the robot.

These are organized by **Workflow Phase** to easily reference them.

---

### **1. The Golden Rule (Workspace Management)**

*Run these whenever you change code, launch files, or configuration files.*

**Build the Workspace:**

```bash
cd ~/dev_ws
colcon build --symlink-install

```

**Source the Overlay (Run in EVERY new terminal):**

```bash
source install/setup.bash

```

**The "Nuclear Clean" (If things are acting weird):**

```bash
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash

```

---

### **2. Simulation Phase**

*Starts Gazebo, RViz, and the Robot State Publisher.*

**Launch Simulation:**

```bash
ros2 launch my_bot sim.launch.py

```

**Reset World (Fixes Time Sync/Odometry issues):**

* **In Gazebo:** Menu `Edit` -> `Reset World` (or press `Ctrl+R`).

---

### **3. Mapping Phase (SLAM)**

*Creates the map while you drive.*

**Start SLAM:**

```bash
ros2 launch my_bot slam.launch.py

```

**Drive the Robot (Teleop):**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard

```

**Save the Map (Run inside `src/my_bot/maps/`):**

```bash
ros2 run nav2_map_server map_saver_cli -f my_map

```

---

### **4. Navigation Phase (Nav2)**

*Loads the saved map and drives autonomously.*

**Start Navigation:**

```bash
ros2 launch my_bot navigation.launch.py

```

**The "Manual Kickstart" (Fixes "AMCL cannot publish pose"):**
*If the "2D Pose Estimate" button in RViz fails, reset the Gazebo world (`Ctrl+R`) and IMMEDIATELY run this:*

```bash
ros2 topic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped "{ header: { frame_id: 'map' }, pose: { pose: { position: { x: 0.0, y: 0.0, z: 0.0 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } } } }"

```

---

### **5. Diagnostics & Debugging (The "Why isn't it working?" Kit)**

**Check if a Node is Alive (Lifecycle):**

```bash
ros2 lifecycle get map_server
# Should say: "active"

```

**Check the Transform Tree (TF) Connections:**

```bash
# Check if Map connects to Odom (Localization)
ros2 run tf2_ros tf2_echo map odom

# Check if Odom connects to Base Link (Odometry)
ros2 run tf2_ros tf2_echo odom base_link

```

**Check if Sensors are Publishing:**

```bash
# Check Lidar
ros2 topic echo /scan --once

# Check Odometry
ros2 topic echo /odom --once

```

**Verify "Sim Time" is Active:**

```bash
ros2 param get /slam_toolbox use_sim_time
# Should say: "Boolean value is: True"

```

**Check if Files were Installed Correctly:**
*Use this if "Node not found" errors appear.*

```bash
ls ~/dev_ws/install/my_bot/share/my_bot/maps/
# Should show: my_map.pgm  my_map.yaml

```
