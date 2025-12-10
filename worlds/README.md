# Gazebo Fortress World Files for Sensor Fusion Testing

This directory contains multiple world files designed to test different aspects of visual odometry and sensor fusion algorithms in varied environments.

## Available Worlds

### 1. fusion_test_world_fortress.sdf (Original)
**Purpose**: General sensor fusion testing with loop closure
**Layout**: 10m × 8m rectangular loop with hallways
**Features**:
- Long straight hallways for featureless odometry testing
- Corner turns for rotation accuracy
- Mixed wall textures (red brick patterns and light concrete)
- Interior dividing walls (wood texture)
- 4 WhyCode markers at corners

**Best For**: Testing wheel/IMU odometry dominance in featureless areas and basic visual features at corners

---

### 2. warehouse_world.sdf
**Purpose**: Industrial warehouse environment simulation
**Layout**: L-shaped corridor (12m × 12m) with storage areas
**Features**:
- **Corrugated metal walls** (north) - high metalness (0.6), medium roughness (0.3)
- **Brick pattern walls** (south) - low metalness (0.0), high roughness (0.9)
- **Concrete panel walls** (east) - low metalness (0.1), high roughness (0.7)
- **Painted plaster walls** (west) - very low metalness (0.05), very high roughness (0.85)
- Storage dividers with metallic and wooden textures
- Industrial warehouse lighting (dimmer ambient, focused spotlights)
- Concrete floor with PBR materials

**Best For**: Testing visual odometry with varied material reflectance properties and industrial lighting conditions

---

### 3. maze_world.sdf
**Purpose**: Complex navigation with multiple path choices
**Layout**: 16m × 16m maze with internal walls
**Features**:
- **Blue-tinted walls** (boundaries) - medium metalness (0.2), medium roughness (0.7)
- **Warm-toned walls** (east/west) - low metalness (0.1), high roughness (0.8)
- **Multi-colored internal walls** with varying PBR properties:
  - Green-tinted dividers
  - Pink-tinted dividers  
  - Blue accent walls
  - Beige dividers
- Each wall section has unique material properties for feature detection
- Bright overhead lighting
- Tiled polished floor (metalness 0.2, roughness 0.6)

**Best For**: Testing path planning, loop closure detection, and visual odometry with diverse colored surfaces

---

### 4. office_corridor_world.sdf
**Purpose**: Office building environment with T-shaped corridors
**Layout**: 10m × 10m T-shaped corridor with room dividers
**Features**:
- **Painted drywall walls** - very low metalness (0.05), very high roughness (0.88)
  - Slight color variations (off-white, light blue, light green tints)
- **Wood paneling** room dividers - low metalness (0.1), high roughness (0.75)
- **Glass-like divider** - high metalness (0.7), low roughness (0.15), semi-transparent
- **Blue accent wall** - metalness 0.15, roughness 0.7
- **Beige accent wall** - metalness 0.12, roughness 0.72
- Soft diffused ceiling lighting
- Polished tile floor (metalness 0.3, roughness 0.3)

**Best For**: Testing in low-contrast environments, testing under soft lighting, subtle texture detection

---

### 5. industrial_facility_world.sdf
**Purpose**: Large industrial facility with open floor plan
**Layout**: 18m × 18m open area with structural columns
**Features**:
- **Reinforced concrete perimeter walls** with subtle color variations
  - Metalness range: 0.10-0.15
  - Roughness range: 0.82-0.88
- **Steel I-beam columns** (4 structural columns) - high metalness (0.72-0.75), low roughness (0.35-0.38)
- **Chain-link fence dividers** - semi-transparent (0.7 alpha), metalness 0.64-0.65
- **Corrugated steel loading walls** - high metalness (0.67-0.68), medium roughness (0.42-0.43)
- Multiple point lights creating varied lighting zones
- Industrial concrete floor
- Open floor plan with fewer walls, more columns

**Best For**: Testing with sparse features, reflective metallic surfaces, and challenging lighting gradients

---

## Material Properties (PBR)

All worlds use Physically Based Rendering (PBR) materials with the following parameters:

- **Roughness**: 0.0 (mirror-like) to 1.0 (completely diffuse)
- **Metalness**: 0.0 (dielectric) to 1.0 (pure metal)
- **Ambient/Diffuse/Specular**: RGB color values for appearance

### Texture Variety Summary

| World | Primary Textures | Metalness Range | Roughness Range | Visual Complexity |
|-------|-----------------|-----------------|-----------------|-------------------|
| Fusion Test | Bricks, concrete, wood | 0.0 - 0.2 | 0.7 - 0.9 | Medium |
| Warehouse | Metal, brick, concrete, plaster | 0.0 - 0.6 | 0.3 - 0.9 | High |
| Maze | Painted surfaces (varied colors) | 0.1 - 0.25 | 0.65 - 0.85 | Very High |
| Office | Drywall, wood, glass | 0.05 - 0.7 | 0.15 - 0.88 | Low-Medium |
| Industrial | Concrete, steel, fence | 0.1 - 0.75 | 0.35 - 0.88 | Medium-High |

## Lighting Conditions

| World | Ambient Level | Primary Lighting | Shadows | Best For |
|-------|--------------|------------------|---------|----------|
| Fusion Test | Medium (0.5) | Directional sun | Yes | Standard conditions |
| Warehouse | Low (0.4) | Directional + 2 spots | Yes | Low-light testing |
| Maze | Bright (0.6) | Overhead directional | Yes | High-visibility testing |
| Office | Bright (0.7) | Soft directional + 2 ceiling | Yes | Indoor diffused light |
| Industrial | Medium-Low (0.45) | Directional + 3 area lights | Yes | Industrial gradients |

## WhyCode Marker Placement

All worlds include 4 WhyCode markers at strategic corner locations for ground truth localization:

- **Marker 0**: North-East corner (facing inward at -135°)
- **Marker 1**: North-West corner (facing inward at 135°)
- **Marker 2**: South-West corner (facing inward at 45°)
- **Marker 3**: South-East corner (facing inward at -45°)

## Usage

### Launch a Specific World

```bash
# Default world (fusion_test_world_fortress.sdf)
ros2 launch olive_sim simulation_fortress.launch.py

# Warehouse world
ros2 launch olive_sim simulation_fortress.launch.py world:=$(ros2 pkg prefix olive_sim)/share/olive_sim/worlds/warehouse_world.sdf

# Maze world
ros2 launch olive_sim simulation_fortress.launch.py world:=$(ros2 pkg prefix olive_sim)/share/olive_sim/worlds/maze_world.sdf

# Office world
ros2 launch olive_sim simulation_fortress.launch.py world:=$(ros2 pkg prefix olive_sim)/share/olive_sim/worlds/office_corridor_world.sdf

# Industrial world
ros2 launch olive_sim simulation_fortress.launch.py world:=$(ros2 pkg prefix olive_sim)/share/olive_sim/worlds/industrial_facility_world.sdf
```

### Quick Launch Aliases (Optional)

Add to your `~/.bashrc`:

```bash
alias olive_warehouse='ros2 launch olive_sim simulation_fortress.launch.py world:=$(ros2 pkg prefix olive_sim)/share/olive_sim/worlds/warehouse_world.sdf'
alias olive_maze='ros2 launch olive_sim simulation_fortress.launch.py world:=$(ros2 pkg prefix olive_sim)/share/olive_sim/worlds/maze_world.sdf'
alias olive_office='ros2 launch olive_sim simulation_fortress.launch.py world:=$(ros2 pkg prefix olive_sim)/share/olive_sim/worlds/office_corridor_world.sdf'
alias olive_industrial='ros2 launch olive_sim simulation_fortress.launch.py world:=$(ros2 pkg prefix olive_sim)/share/olive_sim/worlds/industrial_facility_world.sdf'
```

## Testing Strategy

### Recommended Testing Sequence

1. **Start Simple**: Begin with `fusion_test_world_fortress.sdf` to establish baseline performance
2. **Add Complexity**: Move to `maze_world.sdf` for more feature-rich environment
3. **Low-Light**: Test `warehouse_world.sdf` for dimmer lighting conditions
4. **Subtle Features**: Challenge with `office_corridor_world.sdf` for low-contrast environment
5. **Sparse Features**: Test `industrial_facility_world.sdf` with open floor plan and reflective surfaces

### Visual Odometry Testing

Each world provides different challenges:

- **Warehouse**: Tests feature matching with varied material reflectance
- **Maze**: Tests feature abundance and color discrimination
- **Office**: Tests subtle texture and low-contrast feature detection
- **Industrial**: Tests sparse feature environments with metallic reflections

### Sensor Fusion Testing

- **Straight hallways**: Test IMU/wheel odometry drift without visual corrections
- **Corners**: Test sensor fusion during rotations
- **Open areas**: Test with sparse visual features
- **Varied lighting**: Test robustness across different illumination conditions

## World Design Principles

All worlds follow these design principles:

1. **PBR Materials**: Physically accurate material properties for realistic rendering
2. **Varied Textures**: Different roughness and metalness values for visual feature diversity
3. **Strategic Lighting**: Different lighting schemes to test under varied conditions
4. **WhyCode Markers**: Ground truth reference points in all environments
5. **Collision Geometry**: All walls and obstacles have proper collision detection
6. **Performance Optimized**: Reasonable polygon counts for real-time simulation

## Creating Custom Worlds

To create your own world based on these templates:

1. Copy one of the existing world files
2. Modify the layout (wall positions, sizes)
3. Adjust PBR material properties (ambient, diffuse, specular, roughness, metalness)
4. Change lighting (type, position, intensity)
5. Update WhyCode marker positions
6. Validate XML: `python3 -c "import xml.etree.ElementTree as ET; ET.parse('your_world.sdf')"`

### Key Parameters to Adjust

**Material Properties**:
```xml
<material>
  <ambient>R G B A</ambient>  <!-- Base color in shadow -->
  <diffuse>R G B A</diffuse>  <!-- Main surface color -->
  <specular>R G B A</specular>  <!-- Highlight color -->
  <pbr>
    <metal>
      <roughness>0.0-1.0</roughness>  <!-- Surface roughness -->
      <metalness>0.0-1.0</metalness>  <!-- Metal vs dielectric -->
    </metal>
  </pbr>
</material>
```

**Lighting**:
```xml
<light type="directional|point|spot" name="light_name">
  <pose>x y z roll pitch yaw</pose>
  <diffuse>R G B A</diffuse>
  <specular>R G B A</specular>
  <attenuation>  <!-- For point/spot lights -->
    <range>distance</range>
    <linear>linear_falloff</linear>
    <quadratic>quadratic_falloff</quadratic>
  </attenuation>
</light>
```

## Performance Notes

- All worlds are optimized for real-time simulation
- Physics update rate: 1000 Hz
- Rendering uses Ogre2 for PBR support
- Shadows are enabled in all worlds (can be disabled for performance)

## Troubleshooting

**Markers not visible**:
- Ensure `IGN_GAZEBO_RESOURCE_PATH` includes the models directory
- The launch file sets this automatically

**Poor performance**:
- Disable shadows: Set `<shadows>false</shadows>` in world file
- Reduce light count
- Lower physics update rate

**Materials look flat**:
- Ensure using Ogre2 render engine (not Ogre 1.x)
- Check PBR parameters are within valid ranges

## Future Enhancements

Potential additions for future versions:

- Dynamic obstacles (moving objects)
- Variable lighting conditions (day/night cycles)
- Weather effects (fog, rain simulation)
- Outdoor environments
- Multi-floor layouts with ramps/elevators
- Doors and movable obstacles
