"""
BANTAY-SUNOG Synthetic Dataset Recorder
Applies the verified Rule-Based SIL (Software-in-the-Loop) FSM directly
as the Expert Teacher to distill its full intelligence into the TinyML neural model.

Behaviors distilled:
1. SEARCHING: Structured Zigzag scanning pattern with yaw-based 90-degree lane shifts
2. OBSTACLE_AVOID: 4-phase escalating recovery:
   - BACKOUT: Reverse out when df < 0.18m
   - PIVOT: Yaw-based pivot toward more open side
   - WALL_FOLLOW: Forward arcing curve (140/110 PWM) away from obstacle to overtake it
   - ESCAPE: Reverse + large pivot if stuck
3. ALIGNING: Visual Servoing centering normalized flame target
4. APPROACHING: Standoff approach at speed 135 with centering trim
5. EXTINGUISHING: Standstill suppression (0, 0) at 45-50cm standoff
"""

import math
import random
from typing import Dict, List, Tuple
import numpy as np

from simulation.ai_nav.arena_sim import ArenaSim
from simulation.ai_nav.model import normalize_inputs


class RuleBasedSilExpert:
    """
    Python clone of the Rule-Based SIL FSM from simulation/frontend/js/ui_controller.js.
    Acts as the expert teacher for behavioral cloning into TinyMLNavPolicy.
    """
    def __init__(self):
        self.reset()

    def reset(self):
        self.robot_state = "SEARCHING"
        self.state_timer = 0.0
        self.phase_timer = 0.0

        # Structured Zigzag Scanning Pattern
        self.scan_phase = "SWEEP"       # 'SWEEP', 'TURN_90_OUT', 'SHIFT_LANE', 'TURN_90_IN'
        self.scan_direction = 1.0       # +1 right, -1 left
        self.target_yaw = 0.0
        self.turn_tolerance = 0.08      # ~4.5 deg

        # Multi-behavior Obstacle Avoidance
        self.avoid_phase = "NONE"       # 'NONE', 'BACKOUT', 'PIVOT', 'WALL_FOLLOW', 'ESCAPE'
        self.avoid_timer = 0.0
        self.avoid_turn_left = True
        self.avoid_max_timer = 0.0
        self.avoid_attempts = 0

        # Stuck detection
        self.prev_x = 0.0
        self.prev_z = 0.0
        self.stuck_timer = 0.0
        self.pos_check_timer = 0.0
        self.stuck_threshold = 1.5

        # Last motor PWM
        self.last_left_pwm = 0.0
        self.last_right_pwm = 0.0

    def _normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def compute_action(
        self,
        sensors: Dict[str, float],
        robot_x: float = 0.0,
        robot_z: float = 0.0,
        robot_yaw: float = 0.0,
        dt: float = 0.05
    ) -> Tuple[float, float]:
        self.state_timer += dt
        self.phase_timer += dt

        df = sensors["dist_front"]
        dl = sensors["dist_left"]
        dr = sensors["dist_right"]
        has_fire = sensors["has_fire_target"] > 0.5
        rel_yaw = sensors["target_rel_yaw"]
        dist_fire = sensors["target_distance"]

        is_suppression = (self.robot_state in ("EXTINGUISHING", "COOLDOWN"))

        # ---------------------------------------------------------------------
        # 0. Stuck Detection
        # ---------------------------------------------------------------------
        if not is_suppression:
            self.pos_check_timer += dt
            if self.pos_check_timer >= 0.25:
                self.pos_check_timer = 0.0
                moved = math.hypot(robot_x - self.prev_x, robot_z - self.prev_z)
                if moved < 0.005:
                    self.stuck_timer += 0.25
                else:
                    self.stuck_timer = 0.0
                self.prev_x = robot_x
                self.prev_z = robot_z

            if self.stuck_timer >= self.stuck_threshold and self.robot_state != "OBSTACLE_AVOID":
                self.robot_state = "OBSTACLE_AVOID"
                self.state_timer = 0.0
                self.avoid_phase = "ESCAPE"
                self.avoid_timer = 0.6
                self.avoid_max_timer = 0.0
                self.avoid_attempts += 1
                self.stuck_timer = 0.0
                self.avoid_turn_left = (random.random() > 0.5)

        # ---------------------------------------------------------------------
        # 1. High-Priority Obstacle Avoidance Trigger (matching runAutonomousSilFsm)
        # ---------------------------------------------------------------------
        if not is_suppression and self.robot_state != "OBSTACLE_AVOID":
            is_targeting_fire = (self.robot_state in ("APPROACHING", "ALIGNING")) and has_fire

            front_trigger = False
            side_trigger = False

            is_moving_fwd = (self.last_left_pwm > 50 and self.last_right_pwm > 50)
            if not is_targeting_fire:
                # 26-28cm front trigger in SIL
                front_trigger = (df < 0.28)
                side_trigger = is_moving_fwd and (dl < 0.14 or dr < 0.14)
            else:
                side_trigger = is_moving_fwd and (dl < 0.12 or dr < 0.12)

            if front_trigger or side_trigger:
                self.robot_state = "OBSTACLE_AVOID"
                self.state_timer = 0.0
                self.avoid_max_timer = 0.0

                if df < 0.18:
                    # Very close: back out first
                    self.avoid_phase = "BACKOUT"
                    self.avoid_timer = 0.50
                else:
                    # Moderate distance: pivot away toward more open side
                    self.avoid_phase = "PIVOT"
                    self.avoid_timer = 0.0
                    turn_angle = (math.pi / 2.0)
                    if dl >= dr:
                        self.target_yaw = self._normalize_angle(robot_yaw - turn_angle) # turn left
                        self.avoid_turn_left = True
                    else:
                        self.target_yaw = self._normalize_angle(robot_yaw + turn_angle) # turn right
                        self.avoid_turn_left = False

        # ---------------------------------------------------------------------
        # 2. FSM Execution
        # ---------------------------------------------------------------------
        left_pwm = 0.0
        right_pwm = 0.0

        if self.robot_state == "OBSTACLE_AVOID":
            self.avoid_max_timer += dt
            if self.avoid_max_timer > 5.0 and self.avoid_phase != "ESCAPE":
                self.avoid_phase = "ESCAPE"
                self.avoid_timer = 0.7
                self.avoid_turn_left = not self.avoid_turn_left

            if self.avoid_max_timer > 8.0:
                self.robot_state = "SEARCHING"
                self.scan_phase = "SWEEP"
                self.phase_timer = 0.0
                self.avoid_phase = "NONE"
                self.avoid_attempts = 0
                self.stuck_timer = 0.0

            if self.avoid_phase == "BACKOUT":
                left_pwm = -140.0
                right_pwm = -140.0
                self.avoid_timer -= dt
                if self.avoid_timer <= 0:
                    self.avoid_phase = "PIVOT"
                    self.avoid_timer = 0.0
                    turn_angle = (math.pi / 2.0)
                    if dl >= dr:
                        self.target_yaw = self._normalize_angle(robot_yaw - turn_angle)
                        self.avoid_turn_left = True
                    else:
                        self.target_yaw = self._normalize_angle(robot_yaw + turn_angle)
                        self.avoid_turn_left = False

            elif self.avoid_phase == "PIVOT":
                yaw_err = self._normalize_angle(self.target_yaw - robot_yaw)
                if abs(yaw_err) < self.turn_tolerance:
                    if df > 0.35:
                        self.robot_state = "SEARCHING"
                        self.scan_phase = "SWEEP"
                        self.phase_timer = 0.0
                        self.avoid_phase = "NONE"
                        self.avoid_attempts = 0
                    else:
                        self.avoid_phase = "WALL_FOLLOW"
                        self.avoid_timer = 2.0
                else:
                    speed = 145.0
                    if yaw_err > 0:
                        left_pwm, right_pwm = speed, -speed
                    else:
                        left_pwm, right_pwm = -speed, speed

            elif self.avoid_phase == "WALL_FOLLOW":
                self.avoid_timer -= dt
                if df < 0.20:
                    self.avoid_phase = "PIVOT"
                    turn_angle = (math.pi / 3.0)
                    if dl >= dr:
                        self.target_yaw = self._normalize_angle(robot_yaw - turn_angle)
                    else:
                        self.target_yaw = self._normalize_angle(robot_yaw + turn_angle)
                elif self.avoid_timer <= 0 or df > 0.60:
                    self.robot_state = "SEARCHING"
                    self.scan_phase = "SWEEP"
                    self.phase_timer = 0.0
                    self.avoid_phase = "NONE"
                    self.avoid_attempts = 0
                else:
                    # Drive forward while curving away from obstacle to overtake it!
                    if dl < dr:
                        left_pwm, right_pwm = 140.0, 110.0
                    else:
                        left_pwm, right_pwm = 110.0, 140.0

            elif self.avoid_phase == "ESCAPE":
                self.avoid_timer -= dt
                if self.avoid_timer > 0.35:
                    left_pwm, right_pwm = -155.0, -155.0
                elif self.avoid_timer > 0:
                    speed = 160.0
                    if self.avoid_turn_left:
                        left_pwm, right_pwm = -speed, speed
                    else:
                        left_pwm, right_pwm = speed, -speed
                else:
                    if df > 0.30:
                        self.robot_state = "SEARCHING"
                        self.scan_phase = "SWEEP"
                        self.phase_timer = 0.0
                        self.avoid_phase = "NONE"
                        self.avoid_attempts = 0
                    else:
                        self.avoid_phase = "ESCAPE"
                        self.avoid_timer = 0.9
                        self.avoid_turn_left = not self.avoid_turn_left

        elif self.robot_state in ("SEARCHING", "IDLE"):
            if has_fire:
                self.robot_state = "ALIGNING"
                self.state_timer = 0.0
                left_pwm, right_pwm = 0.0, 0.0
            else:
                if self.scan_phase == "SWEEP":
                    if df < 0.75:
                        # Smooth predictive obstacle bypass curve based on sector clearance
                        turn_right = (dr >= dl)
                        diff = max(0.0, 0.75 - df)
                        steer_offset = min(110.0, diff * 220.0)
                        if turn_right:
                            left_pwm = 155.0
                            right_pwm = max(45.0, 155.0 - steer_offset)
                        else:
                            right_pwm = 155.0
                            left_pwm = max(45.0, 155.0 - steer_offset)
                    else:
                        left_pwm, right_pwm = 140.0, 140.0

                    if df < 0.38 or self.phase_timer > 5.0:
                        self.scan_phase = "TURN_90_OUT"
                        self.phase_timer = 0.0
                        self.target_yaw = self._normalize_angle(robot_yaw + self.scan_direction * (math.pi / 2.0))
                elif self.scan_phase == "TURN_90_OUT":
                    yaw_err = self._normalize_angle(self.target_yaw - robot_yaw)
                    if abs(yaw_err) < self.turn_tolerance or self.phase_timer > 3.0:
                        self.scan_phase = "SHIFT_LANE"
                        self.phase_timer = 0.0
                    else:
                        speed = 135.0
                        left_pwm = speed if yaw_err > 0 else -speed
                        right_pwm = -speed if yaw_err > 0 else speed
                elif self.scan_phase == "SHIFT_LANE":
                    left_pwm, right_pwm = 135.0, 135.0
                    if df < 0.30 or self.phase_timer >= 0.80:
                        self.scan_phase = "TURN_90_IN"
                        self.phase_timer = 0.0
                        self.target_yaw = self._normalize_angle(robot_yaw + self.scan_direction * (math.pi / 2.0))
                elif self.scan_phase == "TURN_90_IN":
                    yaw_err = self._normalize_angle(self.target_yaw - robot_yaw)
                    if abs(yaw_err) < self.turn_tolerance or self.phase_timer > 3.0:
                        self.scan_direction = -self.scan_direction
                        self.scan_phase = "SWEEP"
                        self.phase_timer = 0.0
                    else:
                        speed = 135.0
                        left_pwm = speed if yaw_err > 0 else -speed
                        right_pwm = -speed if yaw_err > 0 else speed

        elif self.robot_state == "ALIGNING":
            if not has_fire:
                if self.state_timer > 2.0:
                    self.robot_state = "SEARCHING"
                    self.scan_phase = "SWEEP"
            else:
                err_x = rel_yaw / 1.13  # normalize to camera frame error
                if abs(err_x) > 0.05:
                    turn_speed = min(150.0, max(90.0, abs(err_x) * 250.0))
                    if err_x > 0:
                        left_pwm, right_pwm = turn_speed, -turn_speed
                    else:
                        left_pwm, right_pwm = -turn_speed, turn_speed
                else:
                    self.robot_state = "APPROACHING"
                    self.state_timer = 0.0

        elif self.robot_state == "APPROACHING":
            if not has_fire:
                if self.state_timer > 2.0:
                    self.robot_state = "SEARCHING"
                    self.scan_phase = "SWEEP"
            else:
                if dist_fire <= 0.50 or df <= 0.45:
                    left_pwm, right_pwm = 0.0, 0.0
                    self.robot_state = "EXTINGUISHING"
                    self.state_timer = 0.0
                else:
                    err_x = rel_yaw / 1.13
                    trim = err_x * 85.0
                    speed = 135.0
                    left_pwm = speed + trim
                    right_pwm = speed - trim

        elif self.robot_state == "EXTINGUISHING":
            left_pwm, right_pwm = 0.0, 0.0
            if self.state_timer >= 2.0:
                self.robot_state = "SEARCHING"
                self.scan_phase = "SWEEP"
                self.state_timer = 0.0

        self.last_left_pwm = left_pwm
        self.last_right_pwm = right_pwm
        return float(np.clip(left_pwm, -255.0, 255.0)), float(np.clip(right_pwm, -255.0, 255.0))


# Alias for backward compatibility
ExpertPolicy = RuleBasedSilExpert


def generate_synthetic_dataset(
    num_episodes: int = 400,
    max_steps_per_episode: int = 140,
    target_samples: int = 35000,
    seed: int = 42
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Runs virtual simulation episodes where the Rule-Based SIL FSM drives the robot,
    recording high-fidelity state-action pairs to distill the SIL behavior into TinyMLNavPolicy.
    """
    random.seed(seed)
    np.random.seed(seed)

    sim = ArenaSim()
    sil_expert = RuleBasedSilExpert()

    inputs_list = []
    actions_list = []

    print(f"[DATASET] Distilling Rule-Based SIL into dataset (target: ~{target_samples} samples)...")

    episode = 0
    total_samples = 0

    while total_samples < target_samples and episode < num_episodes * 2:
        episode += 1

        # Varied layout scenarios
        has_fire_episode = (random.random() < 0.65)
        num_crates = random.randint(2, 6)
        num_barrels = random.randint(1, 4)

        sim.randomize_arena(num_crates=num_crates, num_barrels=num_barrels, add_fire=has_fire_episode)
        sil_expert.reset()

        for step_idx in range(max_steps_per_episode):
            sensors = sim.get_sensor_readings()

            # Record input features
            norm_feat = normalize_inputs(
                dist_front=sensors["dist_front"],
                dist_left=sensors["dist_left"],
                dist_right=sensors["dist_right"],
                target_rel_yaw=sensors["target_rel_yaw"],
                target_distance=sensors["target_distance"],
                has_fire_target=sensors["has_fire_target"],
                heading_rate=sensors["heading_rate"]
            )

            # Compute Rule-Based SIL action
            left_pwm, right_pwm = sil_expert.compute_action(
                sensors,
                robot_x=sim.x,
                robot_z=sim.z,
                robot_yaw=sim.yaw,
                dt=0.05
            )

            # Small exploration noise (2%) for robust generalization
            noisy_left = left_pwm + random.gauss(0, 2.0)
            noisy_right = right_pwm + random.gauss(0, 2.0)

            inputs_list.append(norm_feat)
            actions_list.append([left_pwm, right_pwm])
            total_samples += 1

            # Step physical simulation
            sim.step(noisy_left, noisy_right, dt=0.05)

            # Check if suppression achieved
            if sil_expert.robot_state == "EXTINGUISHING":
                # Record 5 frames of standstill suppression
                for _ in range(5):
                    inputs_list.append(norm_feat)
                    actions_list.append([0.0, 0.0])
                    total_samples += 1
                break

        if episode % 50 == 0 or total_samples >= target_samples:
            print(f"[DATASET] Episode {episode}: Collected {total_samples} samples so far...")

    # 1. Augment with fire standoff standstill examples (fire at 0.40m - 0.50m, target action [0, 0])
    print("[DATASET] Augmenting dataset with Rule-Based SIL standoff suppression transitions...")
    for _ in range(3000):
        df_standoff = random.uniform(0.40, 0.50)
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

    # 2. Augment with Destination-Directed Obstacle Bypass Transitions (8,000 samples)
    # The robot encounters an obstacle in front and smoothly steers along the clear bypass angle
    # that leads toward its destination (fire target or room waypoint).
    print("[DATASET] Augmenting dataset with destination-directed obstacle bypass transitions...")
    for _ in range(8000):
        df_obs = random.uniform(0.22, 0.85)
        dl_obs = random.uniform(0.20, 2.8)
        dr_obs = random.uniform(0.20, 2.8)
        dest_yaw = random.uniform(-1.2, 1.2)
        dest_dist = random.uniform(0.8, 5.0)
        has_fire = 1.0 if random.random() < 0.40 else 0.0
        heading = random.uniform(-0.5, 0.5)

        # Determine clear bypass heading biased toward destination
        # Polar clearance evaluation:
        if dr_obs >= dl_obs:
            if dest_yaw > -0.15 or dl_obs < 0.45:
                # Clear rightward corridor aligned with destination
                clear_angle = random.uniform(0.35, 0.85)
            else:
                # Left is slightly closer to destination, but right has wider clearance
                clear_angle = random.uniform(-0.65, -0.35) if dl_obs > 0.70 else random.uniform(0.35, 0.85)
        else:
            if dest_yaw < 0.15 or dr_obs < 0.45:
                # Clear leftward corridor aligned with destination
                clear_angle = random.uniform(-0.85, -0.35)
            else:
                clear_angle = random.uniform(0.35, 0.65) if dr_obs > 0.70 else random.uniform(-0.85, -0.35)

        norm_feat = normalize_inputs(
            dist_front=df_obs,
            dist_left=dl_obs,
            dist_right=dr_obs,
            target_rel_yaw=clear_angle,
            target_distance=dest_dist,
            has_fire_target=has_fire,
            heading_rate=heading
        )

        # Smooth differential drive steering along clear bypass angle:
        # Positive forward momentum with differential trimming (no stalling!)
        if clear_angle > 0.05:
            # Turn right to bypass obstacle
            base_speed = 155.0
            steer = min(115.0, clear_angle * 140.0)
            lp = base_speed
            rp = max(40.0, base_speed - steer)
        else:
            # Turn left to bypass obstacle
            base_speed = 155.0
            steer = min(115.0, abs(clear_angle) * 140.0)
            rp = base_speed
            lp = max(40.0, base_speed - steer)

        inputs_list.append(norm_feat)
        actions_list.append([lp, rp])

    # 3. Augment with Close-Quarters Emergency Escape / Backout (3,000 samples)
    print("[DATASET] Augmenting dataset with close-quarters emergency escape transitions...")
    for _ in range(3000):
        df_close = random.uniform(0.10, 0.22)
        dl_close = random.uniform(0.12, 1.5)
        dr_close = random.uniform(0.12, 1.5)
        turn_left = (dl_close >= dr_close)
        norm_feat = normalize_inputs(
            dist_front=df_close,
            dist_left=dl_close,
            dist_right=dr_close,
            target_rel_yaw=0.0,
            target_distance=5.0,
            has_fire_target=0.0,
            heading_rate=0.0
        )
        # Reverse or reverse pivot
        if random.random() < 0.5:
            lp, rp = -150.0, -150.0
        else:
            lp = -140.0 if turn_left else 140.0
            rp = 140.0 if turn_left else -140.0
        inputs_list.append(norm_feat)
        actions_list.append([lp, rp])

    # 4. Augment with Open-Space Destination & Waypoint Tracking (6,000 samples)
    print("[DATASET] Augmenting dataset with open-space waypoint tracking transitions...")
    for _ in range(6000):
        df_open = random.uniform(0.85, 3.0)
        dl_open = random.uniform(0.80, 3.0)
        dr_open = random.uniform(0.80, 3.0)
        waypoint_yaw = random.uniform(-1.4, 1.4)
        waypoint_dist = random.uniform(1.0, 5.0)
        has_fire = 1.0 if random.random() < 0.40 else 0.0

        norm_feat = normalize_inputs(
            dist_front=df_open,
            dist_left=dl_open,
            dist_right=dr_open,
            target_rel_yaw=waypoint_yaw,
            target_distance=waypoint_dist,
            has_fire_target=has_fire,
            heading_rate=0.0
        )

        speed = 145.0
        trim = np.clip(waypoint_yaw * 85.0, -80.0, 80.0)
        lp = float(np.clip(speed + trim, -255.0, 255.0))
        rp = float(np.clip(speed - trim, -255.0, 255.0))

        inputs_list.append(norm_feat)
        actions_list.append([lp, rp])

    # 5. Augment with visual fire homing at varied angles (4,000 samples)
    print("[DATASET] Augmenting dataset with Rule-Based SIL visual servoing transitions...")
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
        dummy_expert = RuleBasedSilExpert()
        dummy_expert.robot_state = "ALIGNING" if abs(rel_yaw) > 0.15 else "APPROACHING"
        lp, rp = dummy_expert.compute_action({
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

    print(f"[DATASET] Successfully distilled {len(X)} Rule-Based SIL state-action transitions!")
    return X, Y


if __name__ == "__main__":
    X, Y = generate_synthetic_dataset(num_episodes=450, target_samples=35000, seed=42)
    output_path = "simulation/ai_nav/nav_dataset.npz"
    np.savez_compressed(output_path, X=X, Y=Y)
    print(f"Dataset saved to {output_path}")
