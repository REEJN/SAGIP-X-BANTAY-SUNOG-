"""
BANTAY-SUNOG Synthetic Dataset Recorder
Generates optimal state-action pairs (sensor_inputs, target_pwm) across:
1. Systematic room sweep patrol
2. Dynamic continuous obstacle avoidance
3. Visual servoing homing toward fire targets
4. Standoff deceleration at 45-50cm extinguishing distance
"""

import math
import random
from typing import Dict, List, Tuple
import numpy as np

from simulation.ai_nav.arena_sim import ArenaSim
from simulation.ai_nav.model import normalize_inputs


class ExpertPolicy:
    """
    Expert supervisor that computes optimal continuous (left_pwm, right_pwm)
    for any given arena state.
    """
    def __init__(self):
        self.sweep_phase = 0.0
        self.sweep_dir = 1.0
        self.base_speed = 145.0
        self.approach_speed = 140.0
        self.turn_speed = 135.0

    def compute_action(self, sensors: Dict[str, float]) -> Tuple[float, float]:
        df = sensors["dist_front"]
        dl = sensors["dist_left"]
        dr = sensors["dist_right"]
        rel_yaw = sensors["target_rel_yaw"]
        dist_fire = sensors["target_distance"]
        has_fire = sensors["has_fire_target"] > 0.5
        heading_rate = sensors["heading_rate"]

        # ---------------------------------------------------------------------
        # 1. Fire Homing & Standoff Suppression
        # ---------------------------------------------------------------------
        if has_fire:
            # 1a. Standoff Deceleration at 45-50cm
            if dist_fire <= 0.48 or df <= 0.44:
                # Target reached! Hold position for extinguishing spray
                return 0.0, 0.0

            # Proportional approach speed
            # Dist > 1.2m -> full approach speed (140 PWM)
            # Dist 0.48m - 1.2m -> smooth deceleration down to zero
            dist_factor = float(np.clip((dist_fire - 0.48) / 0.72, 0.0, 1.0))
            fwd_speed = self.approach_speed * (dist_factor ** 1.5)

            # Visual servoing yaw correction: P-controller with heading rate damping
            kp_yaw = 120.0
            kd_yaw = 15.0
            yaw_steer = (rel_yaw * kp_yaw) - (heading_rate * kd_yaw)
            yaw_steer = np.clip(yaw_steer, -110.0, 110.0)

            # If heading error is large, reduce forward speed to prioritize turning
            if abs(rel_yaw) > 0.4:
                fwd_speed *= max(0.15, math.cos(rel_yaw))

            left_pwm = fwd_speed + yaw_steer
            right_pwm = fwd_speed - yaw_steer

            # Obstacle repulsion overlay (safety reflex while homing)
            if df < 0.55 or dl < 0.35 or dr < 0.35:
                obs_steer = (dl - dr) * 110.0
                if df < 0.40:
                    left_pwm = -self.turn_speed if dl < dr else self.turn_speed
                    right_pwm = -left_pwm
                else:
                    left_pwm = np.clip(left_pwm - obs_steer, -200.0, 200.0)
                    right_pwm = np.clip(right_pwm + obs_steer, -200.0, 200.0)

            return float(np.clip(left_pwm, -255.0, 255.0)), float(np.clip(right_pwm, -255.0, 255.0))

        # ---------------------------------------------------------------------
        # 2. Dynamic Obstacle Avoidance & Wall Repulsion
        # ---------------------------------------------------------------------
        # Critical proximity pinch (< 0.28m front): reverse pivot
        if df < 0.28:
            if dl >= dr:
                # More room on left -> pivot left (right_pwm > left_pwm)
                return -self.turn_speed, self.turn_speed
            else:
                # More room on right -> pivot right (left_pwm > right_pwm)
                return self.turn_speed, -self.turn_speed

        # Close obstacle in front (< 0.70m): smooth curving evasion
        if df < 0.70:
            urgency = (0.70 - df) / 0.70  # 0 to 1
            if dl >= dr:
                # More room on left -> steer left (right_pwm > left_pwm)
                left_pwm = self.base_speed * (1.0 - 0.7 * urgency) - 90.0 * urgency
                right_pwm = self.base_speed * (1.0 - 0.2 * urgency) + 80.0 * urgency
            else:
                # More room on right -> steer right (left_pwm > right_pwm)
                left_pwm = self.base_speed * (1.0 - 0.2 * urgency) + 80.0 * urgency
                right_pwm = self.base_speed * (1.0 - 0.7 * urgency) - 90.0 * urgency
            return float(np.clip(left_pwm, -255.0, 255.0)), float(np.clip(right_pwm, -255.0, 255.0))

        # Side corridor pinch:
        side_steer = 0.0
        if dl < 0.45:
            # Obstacle on left -> push right (left_pwm > right_pwm)
            side_steer += (0.45 - dl) * 160.0
        if dr < 0.45:
            # Obstacle on right -> push left (right_pwm > left_pwm)
            side_steer -= (0.45 - dr) * 160.0

        # ---------------------------------------------------------------------
        # 3. Systematic Room Sweep Patrol (Curving Search)
        # ---------------------------------------------------------------------
        # Smooth gentle undulation while patrolling to scan wide horizontal swath
        self.sweep_phase += 0.08
        sweep_trim = math.sin(self.sweep_phase) * 25.0

        fwd = self.base_speed
        left_pwm = fwd + side_steer + sweep_trim
        right_pwm = fwd - side_steer - sweep_trim

        return float(np.clip(left_pwm, -255.0, 255.0)), float(np.clip(right_pwm, -255.0, 255.0))


def generate_synthetic_dataset(
    num_episodes: int = 400,
    max_steps_per_episode: int = 120,
    target_samples: int = 35000,
    seed: int = 42
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Runs virtual simulation episodes across randomized arena layouts and records
    optimal state-action pairs.
    
    Returns:
        X: np.ndarray of shape (N, 7) normalized input features
        Y: np.ndarray of shape (N, 2) target PWM continuous outputs [-255, 255]
    """
    random.seed(seed)
    np.random.seed(seed)

    sim = ArenaSim()
    expert = ExpertPolicy()

    inputs_list = []
    actions_list = []

    print(f"[DATASET] Starting synthetic episode generation (target: ~{target_samples} samples)...")

    episode = 0
    total_samples = 0

    while total_samples < target_samples and episode < num_episodes * 2:
        episode += 1

        # Varied layout scenarios
        # 60% chance of fire target, 40% systematic room sweep
        has_fire_episode = (random.random() < 0.65)
        num_crates = random.randint(2, 6)
        num_barrels = random.randint(1, 4)

        sim.randomize_arena(num_crates=num_crates, num_barrels=num_barrels, add_fire=has_fire_episode)

        # Allow expert policy to steer through episode
        episode_samples = 0
        extinguished = False

        for step_idx in range(max_steps_per_episode):
            sensors = sim.get_sensor_readings()

            # Record transition
            norm_feat = normalize_inputs(
                dist_front=sensors["dist_front"],
                dist_left=sensors["dist_left"],
                dist_right=sensors["dist_right"],
                target_rel_yaw=sensors["target_rel_yaw"],
                target_distance=sensors["target_distance"],
                has_fire_target=sensors["has_fire_target"],
                heading_rate=sensors["heading_rate"]
            )

            # Compute optimal expert action
            left_pwm, right_pwm = expert.compute_action(sensors)

            # Add minor Gaussian exploration noise (3%) for policy robustness
            noisy_left = left_pwm + random.gauss(0, 3.0)
            noisy_right = right_pwm + random.gauss(0, 3.0)

            inputs_list.append(norm_feat)
            actions_list.append([left_pwm, right_pwm])
            total_samples += 1
            episode_samples += 1

            # Step physical simulation
            sim.step(noisy_left, noisy_right, dt=0.05)

            # Check if reached fire standoff distance
            if sensors["has_fire_target"] > 0.5 and sensors["target_distance"] <= 0.48:
                # Add a few standoff standstill frames
                for _ in range(5):
                    inputs_list.append(norm_feat)
                    actions_list.append([0.0, 0.0])
                    total_samples += 1
                extinguished = True
                break

        if episode % 50 == 0 or total_samples >= target_samples:
            print(f"[DATASET] Episode {episode}: Collected {total_samples} samples so far...")

    # Augment with standoff standstill examples (fire at 0.40m - 0.48m, target action [0, 0])
    print("[DATASET] Augmenting dataset with fire standoff standstill transitions...")
    for _ in range(2500):
        df_standoff = random.uniform(0.40, 0.48)
        dl_standoff = random.uniform(0.6, 2.5)
        dr_standoff = random.uniform(0.6, 2.5)
        yaw_standoff = random.uniform(-0.15, 0.15)
        dist_standoff = df_standoff
        norm_feat = normalize_inputs(
            dist_front=df_standoff,
            dist_left=dl_standoff,
            dist_right=dr_standoff,
            target_rel_yaw=yaw_standoff,
            target_distance=dist_standoff,
            has_fire_target=1.0,
            heading_rate=0.0
        )
        inputs_list.append(norm_feat)
        actions_list.append([0.0, 0.0])

    # Augment with obstacle avoidance scenarios (various clearances)
    print("[DATASET] Augmenting dataset with dedicated obstacle avoidance transitions...")
    for _ in range(4000):
        df_obs = random.uniform(0.20, 0.75)
        dl_obs = random.uniform(0.20, 3.0)
        dr_obs = random.uniform(0.20, 3.0)
        norm_feat = normalize_inputs(
            dist_front=df_obs,
            dist_left=dl_obs,
            dist_right=dr_obs,
            target_rel_yaw=0.0,
            target_distance=5.0,
            has_fire_target=0.0,
            heading_rate=0.0
        )
        lp, rp = expert.compute_action({
            "dist_front": df_obs,
            "dist_left": dl_obs,
            "dist_right": dr_obs,
            "target_rel_yaw": 0.0,
            "target_distance": 5.0,
            "has_fire_target": 0.0,
            "heading_rate": 0.0
        })
        inputs_list.append(norm_feat)
        actions_list.append([lp, rp])

    # Augment with visual fire homing at varied angles
    print("[DATASET] Augmenting dataset with visual servoing fire homing transitions...")
    for _ in range(4000):
        df_fire = random.uniform(0.8, 3.0)
        dl_fire = random.uniform(0.8, 3.0)
        dr_fire = random.uniform(0.8, 3.0)
        rel_yaw = random.uniform(-1.0, 1.0)
        dist_fire = random.uniform(0.6, 4.0)
        norm_feat = normalize_inputs(
            dist_front=df_fire,
            dist_left=dl_fire,
            dist_right=dr_fire,
            target_rel_yaw=rel_yaw,
            target_distance=dist_fire,
            has_fire_target=1.0,
            heading_rate=0.0
        )
        lp, rp = expert.compute_action({
            "dist_front": df_fire,
            "dist_left": dl_fire,
            "dist_right": dr_fire,
            "target_rel_yaw": rel_yaw,
            "target_distance": dist_fire,
            "has_fire_target": 1.0,
            "heading_rate": 0.0
        })
        inputs_list.append(norm_feat)
        actions_list.append([lp, rp])

    X = np.array(inputs_list, dtype=np.float32)
    Y = np.array(actions_list, dtype=np.float32)

    print(f"[DATASET] Successfully generated {len(X)} state-action transitions!")
    return X, Y


if __name__ == "__main__":
    X, Y = generate_synthetic_dataset(target_samples=35000)
    np.savez_compressed("simulation/ai_nav/nav_dataset.npz", X=X, Y=Y)
    print("Dataset saved to simulation/ai_nav/nav_dataset.npz")
