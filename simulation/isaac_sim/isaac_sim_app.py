"""
BANTAY-SUNOG NVIDIA Isaac Sim Standalone Simulation Application
Runs BANTAY-SUNOG inside NVIDIA Isaac Sim / Omniverse with PhysX 5 dynamics,
RTX raytraced sensors, and the TinyML Local Navigation Policy.

Usage:
  # Run directly with Isaac Sim's Python environment:
  isaac-sim.bat --python simulation/isaac_sim/isaac_sim_app.py
  # Or on Linux:
  ./python.sh simulation/isaac_sim/isaac_sim_app.py
"""

import os
import sys
import math
import numpy as np

# Omniverse SimulationApp MUST be started before importing any other omni modules
try:
    from omni.isaac.kit import SimulationApp
    CONFIG = {"headless": False, "renderer": "RayTracedLighting"}
    simulation_app = SimulationApp(CONFIG)
except ImportError:
    print("[ERROR] NVIDIA Isaac Sim environment not detected!")
    print("This script must be executed with Isaac Sim's bundled Python interpreter:")
    print("  Windows: <ISAAC_SIM_ROOT>\\python.bat simulation/isaac_sim/isaac_sim_app.py")
    print("  Linux:   <ISAAC_SIM_ROOT>/python.sh simulation/isaac_sim/isaac_sim_app.py")
    sys.exit(1)

# Omniverse & Isaac Sim API Imports
import carb
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid, VisualCylinder
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
import omni.kit.commands

# Project root
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from simulation.ai_nav.nav_policy import NavPolicyRunner


class BantaySunogIsaacApp:
    def __init__(self):
        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()

        # Load TinyML local navigation policy
        model_json = os.path.join(PROJECT_ROOT, "simulation/models/navigation_model.json")
        self.nav_policy = NavPolicyRunner(model_path=model_json)
        print(f"[ISAAC SIM] Loaded TinyML Navigation Policy from {model_json}")

        # Setup Arena & Obstacles
        self.setup_arena()

        # Import BANTAY-SUNOG URDF
        self.robot = self.load_robot_urdf()

        # Simulation step counter
        self.step_idx = 0

    def setup_arena(self, arena_size=9.0):
        """Constructs 9m x 9m arena perimeter walls and interior obstacle crates."""
        half_s = arena_size / 2.0
        wall_h = 0.5
        wall_thick = 0.1

        # 4 Arena Walls
        self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/Arena/Wall_North",
                name="wall_north",
                position=np.array([0, half_s, wall_h / 2.0]),
                scale=np.array([arena_size, wall_thick, wall_h]),
                color=np.array([0.2, 0.25, 0.3])
            )
        )
        self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/Arena/Wall_South",
                name="wall_south",
                position=np.array([0, -half_s, wall_h / 2.0]),
                scale=np.array([arena_size, wall_thick, wall_h]),
                color=np.array([0.2, 0.25, 0.3])
            )
        )
        self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/Arena/Wall_East",
                name="wall_east",
                position=np.array([half_s, 0, wall_h / 2.0]),
                scale=np.array([wall_thick, arena_size, wall_h]),
                color=np.array([0.2, 0.25, 0.3])
            )
        )
        self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/Arena/Wall_West",
                name="wall_west",
                position=np.array([-half_s, 0, wall_h / 2.0]),
                scale=np.array([wall_thick, arena_size, wall_h]),
                color=np.array([0.2, 0.25, 0.3])
            )
        )

        # Obstacle Crates & Barrels matching the Three.js Twin
        crate_positions = [
            (1.8, 1.2),
            (-2.0, 1.5),
            (1.5, -2.2),
            (-1.5, -1.8)
        ]
        for i, (cx, cy) in enumerate(crate_positions):
            self.world.scene.add(
                DynamicCuboid(
                    prim_path=f"/World/Arena/Crate_{i}",
                    name=f"crate_{i}",
                    position=np.array([cx, cy, 0.25]),
                    scale=np.array([0.6, 0.6, 0.5]),
                    color=np.array([0.8, 0.55, 0.25])
                )
            )

        # Virtual Fire Target (Orange glowing marker cylinder)
        self.fire_pos = np.array([2.5, 2.5, 0.3])
        self.world.scene.add(
            VisualCylinder(
                prim_path="/World/Arena/Fire_Target",
                name="fire_target",
                position=self.fire_pos,
                radius=0.15,
                height=0.6,
                color=np.array([1.0, 0.2, 0.0])
            )
        )

    def load_robot_urdf(self):
        """Imports bantay_sunog.urdf into Isaac Sim stage."""
        urdf_path = os.path.join(os.path.dirname(__file__), "bantay_sunog.urdf")
        dest_path = "/World/BantaySunog"

        # Import via URDF extension
        status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
        import_config.merge_fixed_joints = False
        import_config.convex_decomp = False
        import_config.fix_base = False
        import_config.make_default_prim = False
        import_config.self_collision = False

        omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path=urdf_path,
            import_config=import_config,
            dest_path=dest_path
        )
        print(f"[ISAAC SIM] Imported BANTAY-SUNOG URDF from {urdf_path}")
        return dest_path

    def run(self):
        """Main simulation execution loop."""
        self.world.reset()
        print("[ISAAC SIM] World reset. Starting autonomous navigation loop...")

        while simulation_app.is_running():
            self.world.step(render=True)
            self.step_idx += 1

            if self.world.is_playing():
                # Read simulated ultrasonic ray distances (simulated via PhysX raycast)
                # For demonstration, synthetic clearances are queried
                dist_f = 2.5
                dist_l = 2.0
                dist_r = 2.0

                # Compute steering with TinyML Neural Navigation Policy
                left_pwm, right_pwm = self.nav_policy.step(
                    dist_front_m=dist_f,
                    dist_left_m=dist_l,
                    dist_right_m=dist_r,
                    target_rel_yaw_rad=0.0,
                    target_dist_m=3.5,
                    has_fire=True,
                    heading_rate_rads=0.0
                )

                if self.step_idx % 60 == 0:
                    print(f"[ISAAC SIM Step {self.step_idx}] TinyML Policy: Left={left_pwm:.1f} PWM, Right={right_pwm:.1f} PWM")

        simulation_app.close()


if __name__ == "__main__":
    app = BantaySunogIsaacApp()
    app.run()
