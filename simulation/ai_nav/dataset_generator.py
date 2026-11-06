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
            # 1a. Standoff Deceleration at 45-50cm (Only time the robot stops!)
            if dist_fire <= 0.48 or df <= 0.44:
                return 0.0, 0.0

            # 1b. Obstacle in path to fire: overtake toward clearer side while keeping fire bias
            if df < 0.60:
                if dl > dr + 0.08:
                    return 35.0, 150.0   # Overtake left
                elif dr > dl + 0.08:
                    return 150.0, 35.0   # Overtake right
                else:
                    return (150.0, 35.0) if rel_yaw >= 0 else (35.0, 150.0)

            # 1c. Fire alignment: steer toward flame
            dist_factor = float(np.clip((dist_fire - 0.48) / 1.0, 0.0, 1.0))
            fwd_speed = self.approach_speed * (dist_factor ** 1.2)
            yaw_steer = rel_yaw * 115.0
            left_pwm = fwd_speed + yaw_steer
            right_pwm = fwd_speed - yaw_steer
            return float(np.clip(left_pwm, -255.0, 255.0)), float(np.clip(right_pwm, -255.0, 255.0))

        # ---------------------------------------------------------------------
        # 2. Dynamic Obstacle Avoidance & Active Overtaking (Never Stall/Stop!)
        # ---------------------------------------------------------------------
        # Extreme close dead-end (< 0.22m): reverse backout curve to open space
        if df < 0.22:
            if dl >= dr:
                return -80.0, -140.0  # Reverse curve right
            else:
                return -140.0, -80.0  # Reverse curve left

        # Obstacle ahead (< 0.70m): execute active overtaking curve into clearer path
        if df < 0.70:
            clearance_diff = dr - dl  # > 0 -> right is clearer, < 0 -> left is clearer
            
            # Determine clear side decisively
            if clearance_diff > 0.08:
                turn_right = True
            elif clearance_diff < -0.08:
                turn_right = False
            else:
                # Symmetric clearance (e.g. wall straight ahead, both sides clear or equal)
                # Break symmetry deterministically using angular momentum or consistent right-hand rule
                if heading_rate > 0.10:
                    turn_right = True
                elif heading_rate < -0.10:
                    turn_right = False
                else:
                    turn_right = True  # Deterministic default: right-hand sweep

            # Urgency-based inner wheel speed:
            # At df = 0.65m: inner wheel = 65 PWM (gentle overtake curve)
            # At df = 0.35m: inner wheel = -10 PWM (tight overtake curve)
            urgency = (0.70 - df) / 0.48  # 0.0 to ~1.0
            inner_pwm = float(np.clip(70.0 - 90.0 * urgency, -40.0, 80.0))
            outer_pwm = 150.0

            if turn_right:
                return outer_pwm, inner_pwm
            else:
                return inner_pwm, outer_pwm

        # ---------------------------------------------------------------------
        # 3. Side Corridor Proximity Nudge
        # ---------------------------------------------------------------------
        side_steer = 0.0
        if dl < 0.45:
            side_steer += (0.45 - dl) * 150.0
        if dr < 0.45:
            side_steer -= (0.45 - dr) * 150.0

        # ---------------------------------------------------------------------
        # 4. Continuous Systematic Room Sweep Scanning Patrol
        # ---------------------------------------------------------------------
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
