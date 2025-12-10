# Project Olive Sim - ROS 2 Sensor Fusion Simulation

A complete ROS 2 (Humble) simulation package for testing multi-modal sensor fusion algorithms with a 4-wheeled differential drive mobile robot in Gazebo.

## Gazebo Versions

This package supports both:
- **Gazebo Fortress (Ignition Gazebo)** - Recommended for ROS 2 Humble (files with `_fortress` suffix)
- **Gazebo Classic** - Legacy support (original files without suffix)

**⚠️ Note**: Gazebo Classic is End-of-Life (EOL). The Fortress version is recommended for all new projects.

## Package Contents

### 1. Robot (olive_bot)
- **Base:** 4-wheeled differential drive mobile robot (skid-steer configuration)
- **Dimensions:** 0.5m x 0.3m x 0.15m chassis
- **Sensors:**
  - **IMU:** Center-mounted, 100Hz update rate (`/imu/data`)
  - **3D LiDAR:** 16-layer sensor, 10m range, 10Hz update rate (`/lidar/points`)
  - **RGB Camera:** 640x480 resolution, 30Hz update rate (`/camera/image_raw`)
- **Visual Design:** Blue chassis, black wheels

### 2. Test Environments (5 Diverse Worlds)

All worlds feature PBR materials, varied textures, and WhyCode markers:

1. **fusion_test_world_fortress.sdf** - Original loop (10m × 8m)
   - Mixed textured walls, loop closure testing
   
2. **warehouse_world.sdf** - Industrial warehouse (12m × 12m)
   - Corrugated metal, low-light conditions, varied reflectance
   
3. **maze_world.sdf** - Navigation maze (16m × 16m)
   - Multi-colored surfaces, complex paths, high feature density
   
4. **office_corridor_world.sdf** - Office environment (10m × 10m)
   - Drywall, wood paneling, low-contrast, soft lighting
   
5. **industrial_facility_world.sdf** - Industrial facility (18m × 18m)
   - Steel structures, sparse features, metallic reflections

See [worlds/README.md](worlds/README.md) for detailed world documentation.

### 3. YAML Configuration System

Centralized configuration in `config/worlds_config.yaml`:
- Pre-configured optimal spawn locations for each world
- World characteristics and difficulty levels
- Simulation and visualization settings
- Extensible for custom parameters

See [config/README.md](config/README.md) for configuration guide.

### 4. Launch System
- **YAML-based launch** (recommended): Automatic spawn positioning from config
- **Standard launch**: Manual spawn with launch arguments
- Automatic robot spawning and environment setup
- RViz2 visualization preconfigured
- ROS 2 bridges for sensor data (Fortress version)

## Building the Package

```bash
# Navigate to your ROS 2 workspace
cd ~/ros2_ws/src

# Clone or copy this package
# (Assuming the package is already in the workspace)

# Install dependencies
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-ros-gz-image

# Build the package
cd ~/ros2_ws
colcon build --packages-select olive_sim

# Source the workspace
source install/setup.bash

# Set GAZEBO_MODEL_PATH for custom markers
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix olive_sim)/share/olive_sim/models
```

**For Gazebo Classic Support (Legacy):**

```bash
# Additional dependency for Gazebo Classic
sudo apt install ros-humble-gazebo-ros-pkgs
```

## Running the Simulation

### Gazebo Fortress with YAML Configuration (Recommended)

```bash
# Launch with default world (fusion_test) using config spawn
ros2 launch olive_sim simulation_fortress_yaml.launch.py

# Launch specific world with config spawn
ros2 launch olive_sim simulation_fortress_yaml.launch.py world_name:=maze
ros2 launch olive_sim simulation_fortress_yaml.launch.py world_name:=warehouse
ros2 launch olive_sim simulation_fortress_yaml.launch.py world_name:=office
ros2 launch olive_sim simulation_fortress_yaml.launch.py world_name:=industrial

# Override spawn location
ros2 launch olive_sim simulation_fortress_yaml.launch.py \
  world_name:=warehouse \
  use_config_spawn:=false \
  x_pose:=1.0 y_pose:=2.0 z_pose:=0.1 yaw:=1.57

# Without RViz
ros2 launch olive_sim simulation_fortress_yaml.launch.py \
  world_name:=maze \
  use_rviz:=false
```

### Gazebo Fortress (Standard Launch)

```bash
# Launch with default world
ros2 launch olive_sim simulation_fortress.launch.py

# Launch with specific world (manual path)
ros2 launch olive_sim simulation_fortress.launch.py \
  world:=$(ros2 pkg prefix olive_sim)/share/olive_sim/worlds/warehouse_world.sdf

# Optional: Launch without RViz
ros2 launch olive_sim simulation_fortress.launch.py use_rviz:=false

# Optional: Spawn robot at a different position
ros2 launch olive_sim simulation_fortress.launch.py x_pose:=2.0 y_pose:=2.0
```

### Gazebo Classic (Legacy)

```bash
# Launch with Gazebo Classic
ros2 launch olive_sim simulation.launch.py
```

## Controlling the Robot

```bash
# Use keyboard teleoperation (install if needed)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/cmd_vel

# Or publish velocity commands directly
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

## Available Topics

### Robot Control
- `/cmd_vel` - Velocity commands (geometry_msgs/Twist)
- `/odom` - Odometry data (nav_msgs/Odometry)

### Sensor Data
- `/imu/data` - IMU measurements (sensor_msgs/Imu)
- `/lidar/points` - 3D point cloud (sensor_msgs/PointCloud2)
- `/camera/image_raw` - RGB camera images (sensor_msgs/Image)
- `/camera/camera_info` - Camera calibration info
- `/joint_states` - Wheel joint states (sensor_msgs/JointState)

## WhyCode Marker Configuration

The simulation includes 4 configurable WhyCode fiducial markers at the corners of the environment:

- **Marker 0:** North-East corner (4.5, 4.5, 1.0)
- **Marker 1:** North-West corner (-4.5, 4.5, 1.0)
- **Marker 2:** South-West corner (-4.5, -4.5, 1.0)
- **Marker 3:** South-East corner (4.5, -4.5, 1.0)

### Customizing WhyCode Markers

See [models/README.md](models/README.md) for detailed instructions on:
- Replacing marker images with your own WhyCode patterns
- Importing markers from the `whycode-database` branch
- Generating placeholder markers for testing
- Adding additional markers to the environment

Quick setup:

1. Place your WhyCode marker PNGs in `models/whycode_X/materials/textures/marker_X.png`
2. Or generate placeholders: `cd models && python3 generate_markers.py`
3. Or copy from whycode-database branch (see models/README.md)

## Testing Sensor Fusion

This environment is designed to stress-test odometry algorithms across diverse scenarios:

### 5 Worlds for Comprehensive Testing

1. **Fusion Test** (fusion_test): Baseline loop layout
   - Featureless hallways test wheel odometry and IMU
   - Textured corners test visual odometry
   - Loop closure testing

2. **Warehouse** (warehouse): Varied material reflectance
   - Low-light industrial conditions
   - Metal, brick, concrete, plaster textures
   - Tests challenging lighting

3. **Maze** (maze): Complex navigation
   - High visual feature density
   - Multi-colored surfaces
   - Path planning and loop closure

4. **Office** (office): Low-contrast environment
   - Subtle texture detection
   - Soft diffused lighting
   - Drywall and wood paneling

5. **Industrial** (industrial): Sparse features
   - Open floor plan
   - Metallic reflections
   - Structural columns as landmarks

### Recommended Testing Sequence

Use the YAML config's recommended sequence:

```bash
# Run all worlds in recommended order
for world in fusion_test warehouse maze office industrial; do
  ros2 launch olive_sim simulation_fortress_yaml.launch.py world_name:=$world
  # Perform your sensor fusion tests here
done
```

### Testing Aspects

- **WhyCode Markers:** Use for ground truth localization at known corners
- **Loop Closure:** Drive loops to test drift accumulation
- **Varied Lighting:** Test robustness across different illumination
- **Material Diversity:** Test feature extraction on different surfaces


## File Structure

```
olive_sim/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── worlds_config.yaml                 # YAML world configuration (NEW)
│   └── README.md                          # Configuration guide (NEW)
├── launch/
│   ├── simulation.launch.py               # Gazebo Classic launch
│   ├── simulation_fortress.launch.py      # Gazebo Fortress launch
│   └── simulation_fortress_yaml.launch.py # YAML-configured launch (NEW)
├── urdf/
│   ├── olive_bot.urdf.xacro               # Robot for Gazebo Classic
│   └── olive_bot_fortress.urdf.xacro      # Robot for Gazebo Fortress (recommended)
├── worlds/
│   ├── fusion_test_world.sdf              # World for Gazebo Classic
│   ├── fusion_test_world_fortress.sdf     # Original Fortress world
│   ├── warehouse_world.sdf                # Industrial warehouse (NEW)
│   ├── maze_world.sdf                     # Navigation maze (NEW)
│   ├── office_corridor_world.sdf          # Office environment (NEW)
│   ├── industrial_facility_world.sdf      # Industrial facility (NEW)
│   └── README.md                          # World documentation (NEW)
├── rviz/
│   └── olive_bot.rviz
├── models/                                 # WhyCode marker models
│   ├── README.md                           # Marker configuration guide
│   ├── generate_markers.py                 # Placeholder marker generator
│   ├── whycode_0/                          # Marker 0 model (with real images)
│   ├── whycode_1/                          # Marker 1 model (with real images)
│   ├── whycode_2/                          # Marker 2 model (with real images)
│   └── whycode_3/                          # Marker 3 model (with real images)
└── materials/                              # Legacy material directory
    └── textures/
        └── README.md
```

## Troubleshooting

### Gazebo Fortress doesn't start
- Ensure Gazebo Fortress is installed: `ign sim --version` should show Fortress (7.x)
- Install ROS-Gazebo bridges: `sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge`

### Robot doesn't spawn
- Check URDF validity: `ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro urdf/olive_bot_fortress.urdf.xacro)"`
- Verify Gazebo is running: `ign topic -l`

### No sensor data
- Check topics: `ros2 topic list`
- Verify bridges are running (Fortress): `ros2 node list | grep bridge`
- Check Gazebo topics: `ign topic -l`

### Markers don't appear or show no texture
- Verify GAZEBO_MODEL_PATH is set correctly
- Check marker images exist: `ls models/whycode_0/materials/textures/`
- Ensure images are PNG format
- See [models/README.md](models/README.md) for detailed troubleshooting

### Migrating from whycode-database branch
See [models/README.md](models/README.md) for instructions on importing markers.
