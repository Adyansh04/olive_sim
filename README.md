# Olive Sim — ROS 2 Jazzy Sensor-Fusion Simulation

A ROS 2 **Jazzy** / Gazebo **Harmonic** (gz-sim 8) simulation for developing and stress-testing the OLIVE multi-modal sensor-fusion stack. It ships a clean differential-drive robot with a 3D LiDAR, IMU, and RGB camera, five test worlds, and four WhyCode fiducial markers at known positions.

> **Gazebo version:** this package targets **Gazebo Harmonic** (the Jazzy pairing). The old Gazebo Fortress / Gazebo Classic files have been removed — everything here uses the `gz-sim-*-system` plugins and the `gz` CLI.

## Package contents

### 1. Robot (`olive_bot`)
- **Base:** 2-wheel **differential drive** with two frictionless ball casters (front + rear). This replaced the old 4-wheel skid-steer, which slipped on every turn and produced poor `/odom`; the diff-drive layout matches the `DiffDrive` plugin kinematics exactly for clean, low-slip wheel odometry.
- **Chassis:** 0.40 m × 0.30 m × 0.12 m, ~10 kg.
- **Sensors:**
  - **IMU** — 100 Hz, `/imu/data`
  - **3D LiDAR** — 16-beam, 10 m range, 10 Hz, `/lidar/points`
  - **RGB camera** — 640×480, 30 Hz, `/camera/image_raw` (+ `/camera/camera_info`)
- **Frames (REP-105):** `odom → base_footprint → base_link → {imu_link, lidar_link, camera_link → camera_optical_link}`. `DiffDrive` publishes `odom → base_footprint`; a separate ground-truth odometry is published on `/ground_truth` (topic only, not TF).

### 2. Test worlds (5)
| Name | File | Size | Character |
|------|------|------|-----------|
| `fusion_test` | `fusion_test_world.sdf` | 10×8 m | Original loop, textured walls, loop closure |
| `warehouse` | `warehouse_world.sdf` | 12×12 m | Corrugated metal, low light, varied reflectance |
| `maze` | `maze_world.sdf` | 16×16 m | Multi-colored surfaces, high feature density |
| `office` | `office_corridor_world.sdf` | 10×10 m | Drywall/wood, low contrast, soft lighting |
| `industrial` | `industrial_facility_world.sdf` | 18×18 m | Steel structures, sparse features |

See [worlds/README.md](worlds/README.md) for details.

### 3. YAML configuration
`config/worlds_config.yaml` holds per-world spawn poses, characteristics, and the default world. See [config/README.md](config/README.md).

### 4. Launch files
- **`simulation.launch.py`** (recommended) — YAML-driven world/spawn selection.
- **`simulation_manual.launch.py`** — manual world path + spawn via launch args.

## Building

```bash
# ROS 2 Jazzy + Gazebo Harmonic bridges
sudo apt install ros-jazzy-ros-gz

cd ~/olive_ws
colcon build --symlink-install --packages-select olive_sim
source install/setup.bash   # or setup.zsh
```

The launch files add the package's `models/` directory to `GZ_SIM_RESOURCE_PATH` automatically so the WhyCode markers resolve.

## Running

```bash
# Default world (fusion_test) with GUI + RViz
ros2 launch olive_sim simulation.launch.py

# Pick a world
ros2 launch olive_sim simulation.launch.py world_name:=maze     # fusion_test|warehouse|maze|office|industrial

# Headless (server-only, no GUI) — CI, robots, or if the Qt GUI misbehaves
ros2 launch olive_sim simulation.launch.py world_name:=warehouse headless:=true use_rviz:=false

# Override spawn pose
ros2 launch olive_sim simulation.launch.py world_name:=warehouse \
  use_config_spawn:=false x_pose:=1.0 y_pose:=2.0 z_pose:=0.1 yaw:=1.57

# Manual launch with an explicit world path
ros2 launch olive_sim simulation_manual.launch.py \
  world:=$(ros2 pkg prefix olive_sim)/share/olive_sim/worlds/maze_world.sdf
```

## Controlling the robot

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard          # keyboard teleop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"
```

## Topics

| Topic | Type | Notes |
|-------|------|-------|
| `/cmd_vel` | `geometry_msgs/Twist` | velocity command in |
| `/odom` | `nav_msgs/Odometry` | wheel odometry (50 Hz) |
| `/ground_truth` | `nav_msgs/Odometry` | perfect pose (30 Hz) |
| `/imu/data` | `sensor_msgs/Imu` | 100 Hz |
| `/lidar/points` | `sensor_msgs/PointCloud2` | 16-beam, 10 Hz |
| `/camera/image_raw` | `sensor_msgs/Image` | 640×480, 30 Hz |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | |
| `/joint_states` | `sensor_msgs/JointState` | wheel joints |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | |
| `/clock` | `rosgraph_msgs/Clock` | sim time (`use_sim_time`) |

## WhyCode markers

Four WhyCode fiducial markers sit at known corner positions (used as global anchors for the fusion stack):

| Marker | Corner | Position (x, y, z) |
|--------|--------|--------------------|
| 0 | NE | (4.5, 4.5, 1.0) |
| 1 | NW | (-4.5, 4.5, 1.0) |
| 2 | SW | (-4.5, -4.5, 1.0) |
| 3 | SE | (4.5, -4.5, 1.0) |

See [models/README.md](models/README.md) for replacing marker textures / adding markers.

## File structure

```
olive_sim/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── worlds_config.yaml            # world selection + spawn poses
│   └── README.md
├── launch/
│   ├── simulation.launch.py          # YAML-configured launch (primary)
│   └── simulation_manual.launch.py   # manual world-path launch
├── urdf/
│   └── olive_bot.urdf.xacro          # diff-drive + caster robot (Harmonic)
├── worlds/
│   ├── fusion_test_world.sdf
│   ├── warehouse_world.sdf
│   ├── maze_world.sdf
│   ├── office_corridor_world.sdf
│   ├── industrial_facility_world.sdf
│   └── README.md
├── rviz/
│   └── olive_bot.rviz
└── models/                           # WhyCode marker models
    ├── README.md
    ├── generate_markers.py
    └── whycode_0 … whycode_3/
```

## Troubleshooting

- **Gazebo doesn't start:** confirm Harmonic is installed — `gz sim --version` should report 8.x. Install the bridges with `sudo apt install ros-jazzy-ros-gz`.
- **GUI crashes with a `libpthread` / `GLIBC_PRIVATE` symbol-lookup error:** a snap environment (e.g. a snap-installed VS Code/terminal) is leaking libraries into the process. Launch from a non-snap terminal, or run `headless:=true`.
- **Robot doesn't spawn:** check the URDF — `xacro urdf/olive_bot.urdf.xacro | check_urdf /dev/stdin`.
- **No sensor data:** `ros2 topic list`; verify bridges with `ros2 node list | grep bridge`; inspect gz side with `gz topic -l`.
- **Markers missing/untextured:** ensure `GZ_SIM_RESOURCE_PATH` includes `models/`, and that the PNGs exist under `models/whycode_*/materials/textures/`.
