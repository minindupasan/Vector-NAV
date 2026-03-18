# VECTOR NAV Simulation Guide

## Architecture

The simulation is split across two machines for optimal performance:

- **Jetson (Humble)** — Runs headless Gazebo physics, controllers, and sensor bridges
- **Laptop (Jazzy)** — Runs RViz2 for visualization over the network

All ROS 2 topics are discovered automatically via DDS (CycloneDDS) multicast.

---

## Prerequisites

### Jetson

```bash
# Install CycloneDDS (one-time)
sudo apt-get install -y ros-humble-rmw-cyclonedds-cpp

# Add to ~/.bashrc (one-time)
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc
source ~/.bashrc
```

### Laptop

```bash
# Install CycloneDDS (one-time)
sudo apt-get install -y ros-jazzy-rmw-cyclonedds-cpp

# Add to ~/.bashrc (one-time)
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc
source ~/.bashrc
```

Clone and build the `vector_description` package on the laptop (needed for mesh files and RViz config).

---

## Running the Simulation

### Step 1 — Jetson: Start headless simulation

```bash
cd ~/vector_nav
colcon build --packages-select vector_description
source install/setup.bash
ros2 launch vector_description sim.launch.py
```

Optional spawn pose arguments:

```bash
ros2 launch vector_description sim.launch.py x_pose:=1.0 y_pose:=2.0
```

### Step 2 — Laptop: Start RViz2

```bash
ros2 launch vector_description viz.launch.py
```

### Step 3 — Control the robot

From either machine:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```

---

## Available Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/diff_drive_controller/cmd_vel_unstamped` | `geometry_msgs/Twist` | Velocity commands |
| `/diff_drive_controller/odom` | `nav_msgs/Odometry` | Odometry |
| `/scan` | `sensor_msgs/LaserScan` | RPLidar A1M8 (simulated) |
| `/imu` | `sensor_msgs/Imu` | MPU-6050 IMU (simulated) |
| `/tf` | `tf2_msgs/TFMessage` | Transform tree |
| `/robot_description` | `std_msgs/String` | URDF |
| `/clock` | `rosgraph_msgs/Clock` | Simulation clock |

---

## Launch Files

| File | Machine | Description |
|------|---------|-------------|
| `sim.launch.py` | Jetson | Headless Gazebo + controllers + sensor bridges |
| `viz.launch.py` | Laptop | RViz2 visualization only |
| `gazebo.launch.py` | Jetson | Gazebo with GUI (standalone, no RViz) |
| `display.launch.py` | Either | URDF viewer only |

---

## Performance Notes

- Gazebo runs in server-only mode (`-s`) on the Jetson to avoid GPU rendering overhead
- Collision geometry uses simple primitives (boxes/cylinders) instead of STL meshes for fast physics
- CycloneDDS is required on both machines — FastDDS has buffer overflow issues with the URDF size
- Close VS Code and Firefox on the Jetson before running the simulation to free RAM
- Both machines must be on the same network and use the same `ROS_DOMAIN_ID`

---

## Troubleshooting

| Issue | Fix |
|-------|-----|
| `sequence size exceeds remaining buffer` | Ensure `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` is set |
| Laptop can't see Jetson topics | Verify same `ROS_DOMAIN_ID` and same network; run `ros2 topic list` |
| Controller spawner times out | Increase timer delays in `sim.launch.py` or restart |
| Robot spins in place on forward command | Check wheel joint axes in URDF — all should be `0 1 0` |
| Simulation is laggy | Close heavy apps on Jetson; check `htop` for CPU/RAM usage |
