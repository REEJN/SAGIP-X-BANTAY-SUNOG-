"""
BANTAY-SUNOG Reinforcement Learning Simulation Environment
Defines the point-based reward function:
+ Points for avoiding obstacles & keeping safe clearance
+ Points for navigating toward fire targets & heading alignment
+ Big points for reaching 45-50cm standoff distance & stopping
- Heavy penalties for collisions (-60 pts) and close proximity (-30 pts)
"""

import math
import random
from typing import Dict, Tuple
import numpy as np

from simulation.ai_nav.arena_sim import ArenaSim
from simulation.ai_nav.model import normalize_inputs, MAX_PWM


class BantayNavRLEnv:
    """
    RL Environment wrapping ArenaSim with point-based reward shaping.
    """
    def __init__(self, dt: float = 0.05, max_steps: int = 300):
        self.sim = ArenaSim()
        self.dt = dt
        self.max_steps = max_steps
        self.step_count = 0
        self.prev_dist_fire = 5.0
        self.has_fire = False

    def reset(self, randomize: bool = True) -> np.ndarray:
        self.step_count = 0
        if randomize:
            num_crates = random.randint(3, 5)
            num_barrels = random.randint(2, 4)
            # 70% chance of fire target, 30% exploration / obstacle maze
            self.has_fire = random.random() < 0.70
            self.sim.randomize_arena(num_crates=num_crates, num_barrels=num_barrels, add_fire=self.has_fire)
        else:
            self.sim.reset_robot(0, 0, 0)

        sensors = self.sim.get_sensor_readings()
        self.prev_dist_fire = sensors["target_distance"] if sensors["has_fire_target"] > 0.5 else 5.0

        return self._get_obs(sensors)

    def _get_obs(self, sensors: Dict[str, float]) -> np.ndarray:
        return normalize_inputs(
            dist_front=sensors["dist_front"],
            dist_left=sensors["dist_left"],
            dist_right=sensors["dist_right"],
            target_rel_yaw=sensors["target_rel_yaw"],
            target_distance=sensors["target_distance"],
            has_fire_target=sensors["has_fire_target"],
            heading_rate=sensors["heading_rate"]
        )

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, Dict]:
        """
        Takes action: [left_pwm, right_pwm] in [-255, 255].
        Returns: (obs, reward, done, info)
        """
        self.step_count += 1
        left_pwm = float(action[0])
        right_pwm = float(action[1])

        # Step simulation physics
        sensors, collision = self.sim.step(left_pwm, right_pwm, dt=self.dt)
        obs = self._get_obs(sensors)

        df = sensors["dist_front"]
        dl = sensors["dist_left"]
        dr = sensors["dist_right"]
        has_fire = sensors["has_fire_target"] > 0.5
        rel_yaw = sensors["target_rel_yaw"]
        dist_fire = sensors["target_distance"]

        # ---------------------------------------------------------------------
        # POINT-BASED REWARD SHAPING
        # ---------------------------------------------------------------------
        reward = 0.0

        # 1. COLLISION PENALTY: Heavy punishment for hitting walls or obstacles
        if collision:
            reward -= 60.0

        # 2. PROXIMITY PENALTY & FORWARD DRIVE RESTRICTION:
        if df < 0.55:
            # Heavy penalty if attempting to drive forward when front is blocked
            avg_fwd = (left_pwm + right_pwm) / 2.0
            if avg_fwd > 20:
                reward -= 45.0 * (avg_fwd / 255.0)

            # Bonus for pivoting in place
            if (left_pwm * right_pwm) < 0:
                reward += 20.0
            reward -= 30.0 * ((0.55 - df) / 0.55)

        if dl < 0.35:
            reward -= 15.0 * ((0.35 - dl) / 0.35)
        if dr < 0.35:
            reward -= 15.0 * ((0.35 - dr) / 0.35)

        # 3. SAFE OBSTACLE EVASION BONUS: Reward turning away when front is blocked
        if df < 0.65:
            turn_differential = abs(left_pwm - right_pwm)
            # If front is blocked, reward steering away towards the more open side
            if dl > dr and right_pwm > left_pwm:
                reward += 8.0 * (turn_differential / 255.0)  # Turning left toward open space
            elif dr > dl and left_pwm > right_pwm:
                reward += 8.0 * (turn_differential / 255.0)  # Turning right toward open space

        # 4. CRUISING EXPLORATION BONUS (When path is clear)
        if df >= 0.70 and dl >= 0.40 and dr >= 0.40:
            avg_fwd = (left_pwm + right_pwm) / 2.0
            if avg_fwd > 50:
                reward += 1.5 * (avg_fwd / 255.0)

        # 5. FIRE HOMING & SUPPRESSION POINTS
        done = False
        extinguished = False

        if has_fire:
            # Progress reward: points for reducing distance to fire
            dist_delta = self.prev_dist_fire - dist_fire
            reward += dist_delta * 20.0
            self.prev_dist_fire = dist_fire

            # Heading alignment reward: points for keeping fire centered in view
            reward += 3.0 * max(0.0, 1.0 - abs(rel_yaw) / 0.8)

            # Fire Standoff Suppression Goal: [0.45m - 0.50m]
            if dist_fire <= 0.50 or df <= 0.46:
                # Deceleration check: must slow down to extinguish!
                speed = math.hypot(self.sim.current_linear_speed, self.sim.current_angular_speed)
                if speed < 0.15:
                    # BIG SUPPRESSION REWARD!
                    reward += 100.0
                    extinguished = True
                    done = True
                else:
                    # Stood too fast or rammed into fire
                    reward += 20.0

        if self.step_count >= self.max_steps:
            done = True

        info = {
            "extinguished": extinguished,
            "collision": collision,
            "dist_front": df,
            "dist_fire": dist_fire,
            "has_fire": has_fire
        }

        return obs, reward, done, info
