# BANTAY-SUNOG in NVIDIA Isaac Sim / Omniverse

This directory contains the **NVIDIA Isaac Sim / Isaac Lab** simulation assets and standalone application for the BANTAY-SUNOG autonomous firefighting robot.

---

## 1. System Requirements vs Your PC Hardware

### NVIDIA Isaac Sim Official Requirements:
| Component | Minimum Required | Recommended | Your Current Hardware | Compatible? |
| :--- | :--- | :--- | :--- | :--- |
| **GPU** | NVIDIA RTX 2070 (8GB VRAM) | RTX 3080 / RTX 4080 (12GB+ VRAM) | **Intel(R) UHD Graphics (1GB Shared VRAM)** | ❌ **No (Lacks RTX Raytracing)** |
| **CPU** | Intel Core i7 / AMD Ryzen 7 (8 cores) | Intel Core i9 / AMD Ryzen 9 (16+ cores) | **Intel Core i3-1005G1 (2 Cores, 4 Threads)** | ⚠️ **Below minimum** |
| **RAM** | 32 GB RAM | 64 GB RAM | **12 GB RAM (~2.1 GB free)** | ❌ **Insufficient** |
| **Storage** | 50 GB NVMe SSD | 100 GB NVMe SSD | ~25 GB free | ⚠️ **Tight** |
| **Vulkan Driver** | Vulkan 1.3 with Raytracing Extensions | Latest NVIDIA Studio Driver | Intel Gen11 D3D12/Vulkan driver | ❌ **Lacks VK_KHR_ray_tracing** |

> [!WARNING]
> **Isaac Sim Cannot Launch on Intel Integrated Graphics**:
> NVIDIA Isaac Sim is built directly on NVIDIA Omniverse and PhysX 5 GPU Raytracing. On computers without a dedicated NVIDIA RTX GPU, Isaac Sim exits on startup with the error:
> `[Error] No compatible NVIDIA RTX GPU found. Vulkan raytracing extensions are required.`

---

## 2. How to Run BANTAY-SUNOG in Isaac Sim (Cloud / RTX Machine)

If you have access to a machine with an NVIDIA RTX GPU (or a Cloud GPU on AWS, Azure, Google Cloud, or Lambda Labs):

### Option A: Local RTX Workstation
1. Install [NVIDIA Omniverse Launcher](https://www.nvidia.com/en-us/omniverse/).
2. Install **Isaac Sim** from the Exchange tab.
3. Open a terminal in the Isaac Sim installation directory:
   ```powershell
   # Windows:
   .\python.bat simulation/isaac_sim/isaac_sim_app.py
   
   # Linux:
   ./python.sh simulation/isaac_sim/isaac_sim_app.py
   ```

### Option B: Cloud Docker Container (NVIDIA NGC)
Run Isaac Sim headless on a cloud instance with an NVIDIA A10G, T4, or L4 GPU:
```bash
docker run --name isaac-sim --entrypoint bash -it --gpus all \
  -e "ACCEPT_EULA=Y" --rm --network=host \
  -v $(pwd):/workspace \
  nvcr.io/nvidia/isaac-sim:4.2.0
```

---

## 3. Included Isaac Sim Assets

1. **[`bantay_sunog.urdf`](./bantay_sunog.urdf)**:
   - Full robot kinematic definition matching the physical robot:
     - 4WD skid-steer wheel links and continuous joints (`wheel_fl`, `wheel_fr`, `wheel_rl`, `wheel_rr`).
     - Articulated tilt gimbal servo link (`gimbal_tilt_link`, pitch range: -45° to +45°).
     - Camera optical frame (`camera_link`) and water nozzle (`nozzle_link`).
     - 3x HC-SR04 ultrasonic range sensors (`ultrasonic_front_link`, `ultrasonic_left_link` at -45°, `ultrasonic_right_link` at +45°).
2. **[`isaac_sim_app.py`](./isaac_sim_app.py)**:
   - Standalone Isaac Sim script that builds the 9m $\times$ 9m arena, loads `bantay_sunog.urdf`, spawns obstacles, and connects `TinyMLNavPolicy` (`simulation/models/navigation_model.json`) to the robot's drive joints.

---

## 4. Why We Built the Custom Three.js Digital Twin for This Project

Because most development and field deployment laptops (and the Raspberry Pi 3B onboard) do not have a $1,500 dedicated NVIDIA RTX desktop GPU:
- The custom **Three.js 3D Digital Twin** (`run_simulation.py`) was engineered specifically for BANTAY-SUNOG:
  - Runs in real-time at **60 FPS** on **any hardware** (Intel UHD graphics, Mac, Linux, Raspberry Pi).
  - Simulates 4WD skid-steer kinematics with smoothed inertia.
  - Implements 3-ray ultrasonic raycasting against 3D oriented bounding boxes and cylindrical barrels.
  - Streams virtual camera video frames to the OpenCV / TFLite fire detector in real-time.
  - Runs the TinyML navigation model (`nav_model_runner.js`) client-side in the browser.

---

## 5. How to Revert or Switch Branches

Before creating this Isaac Sim setup, a dedicated git tag was created:
- Tag: **`v2-pre-isaac-sim`** (Commit `a5c9a47`)

To revert back at any time:
```powershell
git checkout v2-pre-isaac-sim
```
Or to return to your current branch:
```powershell
git checkout v2-navigation-model
```
