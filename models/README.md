# WhyCode Marker Models

This directory contains configurable WhyCode fiducial marker models for Gazebo simulation.

## Structure

Each WhyCode marker (whycode_0 through whycode_3) has the following structure:

```
whycode_X/
├── model.config          # Model metadata
├── model.sdf             # Model definition (0.2m x 0.2m x 0.01m box)
└── materials/
    ├── scripts/
    │   └── marker.material   # Material script referencing texture
    └── textures/
        └── marker_X.png      # Marker texture image (user-provided)
```

## Marker Positions in World

The markers are placed at the four corners of the test environment:

- **WhyCode_0**: North-East corner (4.5, 4.5, 1.0), facing inward at -135°
- **WhyCode_1**: North-West corner (-4.5, 4.5, 1.0), facing inward at 135°
- **WhyCode_2**: South-West corner (-4.5, -4.5, 1.0), facing inward at 45°
- **WhyCode_3**: South-East corner (4.5, -4.5, 1.0), facing inward at -45°

## Customizing Markers

### Replace Individual Marker Images

1. Generate or obtain your WhyCode marker images (PNG format, 200x200+ pixels recommended)
2. Name them `marker_0.png`, `marker_1.png`, `marker_2.png`, `marker_3.png`
3. Place each image in the corresponding model's textures directory:
   - `whycode_0/materials/textures/marker_0.png`
   - `whycode_1/materials/textures/marker_1.png`
   - `whycode_2/materials/textures/marker_2.png`
   - `whycode_3/materials/textures/marker_3.png`


## Adding More Markers

To add additional markers (e.g., whycode_4, whycode_5):

1. Copy an existing marker directory:
   ```bash
   cp -r whycode_0 whycode_4
   ```

2. Update the files:
   - Edit `whycode_4/model.config`: Change name to `WhyCode_4`
   - Edit `whycode_4/model.sdf`: Update model name and material URIs
   - Edit `whycode_4/materials/scripts/marker.material`: Update material name and texture filename
   - Add your `marker_4.png` image

3. Include in the world file:
   ```xml
   <include>
     <uri>model://whycode_4</uri>
     <name>whycode_marker_4</name>
     <pose>X Y Z roll pitch yaw</pose>
   </include>
   ```

## Setting GAZEBO_MODEL_PATH

For Gazebo to find these models, add the models directory to your GAZEBO_MODEL_PATH:

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix olive_sim)/share/olive_sim/models
```

Or add to your `~/.bashrc`:

```bash
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix olive_sim)/share/olive_sim/models' >> ~/.bashrc
```

## Marker Image Requirements

- **Format**: PNG (recommended) or JPEG
- **Size**: 200x200 pixels minimum, 512x512 or higher recommended
- **Pattern**: High-contrast black and white pattern
- **Content**: QR codes, ArUco tags, or WhyCode patterns
- **Uniqueness**: Each marker should have a distinct pattern for identification

## Generating WhyCode Markers

WhyCode markers can be generated using:

1. **Online QR Code Generators**: Generate unique QR codes with different IDs
2. **ArUco Marker Generators**: Use OpenCV or online tools
3. **WhyCode Library**: If you have access to WhyCode marker generation tools
4. **Custom Patterns**: Create unique black/white grid patterns

## Troubleshooting

**Markers appear as white boxes**: 
- Check that marker images are in the correct location
- Verify image format is PNG
- Ensure GAZEBO_MODEL_PATH is set correctly

**Model not found error**:
- Verify models directory is in GAZEBO_MODEL_PATH
- Check that model.config and model.sdf exist
- Ensure directory name matches URI in world file

**Markers not visible in camera**:
- Check marker orientation (should face inward)
- Verify marker size (0.2m x 0.2m)
- Ensure markers are at correct height (1.0m)
- Check lighting in the world
