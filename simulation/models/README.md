# BANTAY-SUNOG 3D Robot Models Directory

This directory is designated for custom 3D models and textures of the physical BANTAY-SUNOG robot.

## Supported 3D Formats
1. **GLTF / GLB (`.glb`, `.gltf`)** *(Recommended)*:
   - Self-contained binary mesh, materials, PBR textures, and animations.
   - Fast loading and optimal WebGL performance.
2. **Wavefront OBJ (`.obj`, `.mtl`)**:
   - Supported with accompanying `.mtl` material definitions.

---

## How to Load Your 3D Model into the Simulation

### Option 1: Direct Web Upload (No restart required)
1. Open the 3D Simulation Dashboard at `http://localhost:8000/`.
2. In the right-hand sidebar under **Robot 3D Model**, click **"Load Custom 3D Model (.glb / .gltf)"**.
3. Select your `.glb`, `.gltf`, or `.obj` file.
4. The simulation dynamically attaches your custom model to the robot's kinematic chassis and tilt turret!

### Option 2: Default Asset Slot
1. Save your 3D model file as:
   ```
   simulation/models/robot_custom.glb
   ```
2. The simulation will automatically detect and load `robot_custom.glb` on startup.

---

## Kinematic Pivot Alignment Guide
When designing or exporting your 3D model from Blender, SolidWorks, or Fusion 360:
- **Origin / Pivot Point (0,0,0)**: Ground contact plane directly centered between the 4 wheels.
- **Forward Axis**: `+Z` is forward direction, `+Y` is upward, `+X` is rightward.
- **Scale**: Real-world meters ($1.0 \text{ unit} = 1.0 \text{ meter}$). If exported in millimeters, scale factor in `asset_loader.js` can be adjusted to `0.001`.
- **Sub-assemblies (Optional for articulated joints)**:
  - If your model has named nodes:
    - `Body` or `Chassis`
    - `Tilt_Gimbal` or `Turret` (mounts camera, nozzle, and flame sensor)
    - `Wheel_FL`, `Wheel_FR`, `Wheel_RL`, `Wheel_RR`
  - The simulator can bind rotational transforms directly to these named bones/meshes.
