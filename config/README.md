# World Configuration System

This directory contains YAML configuration files for managing world environments and robot spawn locations in the Olive Bot simulation.

## Configuration File: worlds_config.yaml

The `worlds_config.yaml` file provides a centralized configuration system for:

1. **World Selection**: Define available simulation worlds with metadata
2. **Spawn Locations**: Pre-configured robot spawn positions for each world
3. **World Characteristics**: Documentation of world features and difficulty
4. **Simulation Settings**: Global simulation parameters
5. **Testing Sequences**: Recommended testing progression

## Configuration Structure

### World Definition

Each world is defined with the following properties:

```yaml
world_name:
  name: "Display Name"
  description: "Brief description of the world"
  world_file: "filename.sdf"
  spawn_location:
    x: 0.0      # meters
    y: 0.0      # meters
    z: 0.1      # meters (slightly above ground)
    roll: 0.0   # radians
    pitch: 0.0  # radians
    yaw: 0.0    # radians
  characteristics:
    - "Feature 1"
    - "Feature 2"
  difficulty: "easy|medium|hard|very_hard"
  size: "XxY dimensions"
```

### Available Worlds

Current worlds configured:

1. **fusion_test** - Default loop layout (10m × 8m)
2. **warehouse** - Industrial L-shaped warehouse (12m × 12m)
3. **maze** - Complex navigation maze (16m × 16m)
4. **office** - T-shaped office corridor (10m × 10m)
5. **industrial** - Open floor plan facility (18m × 18m)


## Usage

### Using YAML Configuration (Recommended)

The new `simulation_fortress_yaml.launch.py` launch file uses the YAML configuration:

```bash
# Launch with default world (fusion_test)
ros2 launch olive_sim simulation_fortress_yaml.launch.py

# Launch specific world using config spawn location
ros2 launch olive_sim simulation_fortress_yaml.launch.py world_name:=maze

# Launch warehouse world
ros2 launch olive_sim simulation_fortress_yaml.launch.py world_name:=warehouse

# Launch office world
ros2 launch olive_sim simulation_fortress_yaml.launch.py world_name:=office

# Launch industrial world
ros2 launch olive_sim simulation_fortress_yaml.launch.py world_name:=industrial
```

### Custom Spawn Location

Override the config spawn location:

```bash
# Use custom spawn location instead of config
ros2 launch olive_sim simulation_fortress_yaml.launch.py \
  world_name:=maze \
  use_config_spawn:=false \
  x_pose:=2.0 \
  y_pose:=3.0 \
  z_pose:=0.1 \
  yaw:=1.57
```

### Without RViz

```bash
ros2 launch olive_sim simulation_fortress_yaml.launch.py \
  world_name:=warehouse \
  use_rviz:=false
```

## Additional Configuration Sections

### Simulation Settings

Global simulation parameters:
- `use_sim_time`: Use Gazebo clock (default: true)
- `physics_update_rate`: Physics timestep rate in Hz (default: 1000)
- `real_time_factor`: Target real-time factor (default: 1.0)

### Visualization Settings

RViz and display settings:
- `use_rviz`: Launch RViz automatically (default: true)
- `camera_follow_robot`: Camera tracking mode (default: false)

### Robot Settings

Robot physical parameters (informational):
- `max_linear_velocity`: 0.5 m/s
- `max_angular_velocity`: 1.0 rad/s
- `wheel_radius`: 0.05 m
- `wheel_separation`: 0.34 m

These are documented in the config for reference and can be used by other tools that parse the config.

## Recommended Testing Sequence

The config includes a recommended testing progression:

1. **fusion_test** - Establish baseline performance
2. **warehouse** - Test with varied materials
3. **maze** - Test with high feature density
4. **office** - Challenge with low contrast
5. **industrial** - Test sparse features

Run all worlds in sequence:

```bash
for world in fusion_test warehouse maze office industrial; do
  echo "Testing world: $world"
  ros2 launch olive_sim simulation_fortress_yaml.launch.py world_name:=$world
  # Add test execution here
done
```

## Extending the Configuration

### Adding a New World

1. Create your world SDF file in `worlds/` directory
2. Add world definition to `worlds_config.yaml`:

```yaml
my_custom_world:
  name: "My Custom World"
  description: "Description of my world"
  world_file: "my_world.sdf"
  spawn_location:
    x: 0.0
    y: 0.0
    z: 0.1
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
  characteristics:
    - "Custom feature 1"
    - "Custom feature 2"
  difficulty: "medium"
  size: "10m x 10m"
```

3. Launch your world:
```bash
ros2 launch olive_sim simulation_fortress_yaml.launch.py world_name:=my_custom_world
```

### Adding Custom Parameters

The YAML config can be extended with additional sections for:

- **Sensor calibration parameters**: IMU noise, camera intrinsics
- **Navigation parameters**: Planner configurations, costmap settings
- **Test scenarios**: Predefined waypoints, trajectories
- **Performance metrics**: Expected completion times, accuracy thresholds

Example extension:

```yaml
test_scenarios:
  maze_basic_navigation:
    world: "maze"
    waypoints:
      - [0, 0]
      - [5, 5]
      - [-5, 5]
    timeout: 300  # seconds
    success_criteria:
      position_error: 0.5  # meters
      orientation_error: 0.2  # radians
```

## Python API

The launch file demonstrates how to parse the config in Python:

```python
import yaml
import os
from ament_index_python.packages import get_package_share_directory

# Load configuration
pkg = get_package_share_directory('olive_sim')
config_file = os.path.join(pkg, 'config', 'worlds_config.yaml')

with open(config_file, 'r') as f:
    config = yaml.safe_load(f)

# Access world data
world_config = config['worlds']['maze']
spawn_x = world_config['spawn_location']['x']
world_file = world_config['world_file']

# Get default world
default_world = config['default_world']

# Get recommended sequence
test_sequence = config['recommended_testing_sequence']
```

## Future Enhancements

Potential additions:

- **Multiple spawn points per world**: Support for different starting positions
- **Time-based scenarios**: Different configurations for day/night
- **Dynamic obstacles**: Configuration for moving objects
- **Performance profiles**: Different simulation quality settings
- **Camera viewpoints**: Predefined camera positions for visualization
- **Sensor configurations**: Enable/disable specific sensors per world

## Troubleshooting

**World not found error**:
- Check that world name matches a key in `worlds_config.yaml`
- Ensure world_file exists in `worlds/` directory

**Spawn location issues**:
- Verify spawn coordinates are within world bounds
- Check for obstacles at spawn location
- Ensure z coordinate is slightly above ground (typically 0.1m)

**YAML parsing errors**:
- Validate YAML syntax (use online YAML validator)
- Check indentation (must use spaces, not tabs)
- Ensure all required fields are present

