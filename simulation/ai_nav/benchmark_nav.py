"""
BANTAY-SUNOG Navigation Benchmark Suite
Compares Legacy Rule-Based FSM vs TinyML Neural Local Navigation Policy across:
1. Time-to-Fire-Extinguishment (seconds & success rate)
2. Total & Mean Collision Count
3. Trajectory Smoothness (Jerk / Acceleration Variance)
4. Path Efficiency & Motor Effort
"""

import math
import random
import time
from typing import Dict, List, Tuple
import numpy as np

from simulation.ai_nav.arena_sim import ArenaSim
from simulation.ai_nav.nav_policy import NavPolicyRunner


class RuleBasedFSMController:
    """
    Python implementation of the improved rule-based FSM from ui_controller.js.
    """
    def __init__(self):
        self.state = "SEARCHING"
        self.state_timer = 0.0
        self.phase_timer = 0.0
        self.scan_phase = "SWEEP"
        self.scan_dir = 1.0
        self.target_yaw = None
        self.avoid_phase = "NONE"
        self.avoid_timer = 0.0
        self.avoid_turn_left = True
        self.avoid_max_timer = 0.0

    def reset(self):
        self.state = "SEARCHING"
        self.state_timer = 0.0
        self.phase_timer = 0.0
        self.scan_phase = "SWEEP"
        self.scan_dir = 1.0
        self.target_yaw = None
        self.avoid_phase = "NONE"
        self.avoid_timer = 0.0
        self.avoid_turn_left = True
        self.avoid_max_timer = 0.0

    def step(self, sensors: Dict[str, float], robot_yaw: float, dt: float) -> Tuple[float, float, bool]:
        """
        Returns:
            (left_pwm, right_pwm, is_extinguishing)
        """
        self.state_timer += dt
        self.phase_timer += dt

        df = sensors["dist_front"] * 100.0  # m to cm
        dl = sensors["dist_left"] * 100.0
        dr = sensors["dist_right"] * 100.0
        has_fire = sensors["has_fire_target"] > 0.5
        target_yaw_err = sensors["target_rel_yaw"]
        dist_fire = sensors["target_distance"] * 100.0

        is_suppression = (self.state == "EXTINGUISHING")

        # 1. Obstacle Avoidance Trigger
        if not is_suppression and self.state != "OBSTACLE_AVOID":
            is_targeting = (self.state in ("APPROACHING", "ALIGNING")) and has_fire
            front_trigger = (df < 26.0) if not is_targeting else (df < 18.0)
            side_trigger = (dl < 12.0 or dr < 12.0)

            if front_trigger or side_trigger:
                self.state = "OBSTACLE_AVOID"
                self.state_timer = 0.0
                self.avoid_max_timer = 0.0

                if df < 18.0:
                    self.avoid_phase = "BACKOUT"
                    self.avoid_timer = 0.50
                else:
                    self.avoid_phase = "PIVOT"
                    self.avoid_timer = 0.0
                    turn_angle = math.pi / 2.0
                    if dl >= dr:
                        self.target_yaw = robot_yaw - turn_angle
                        self.avoid_turn_left = True
                    else:
                        self.target_yaw = robot_yaw + turn_angle
                        self.avoid_turn_left = False

        # Fire confirmation check
        if self.state == "SEARCHING" and has_fire:
            self.state = "ALIGNING"
            self.state_timer = 0.0

        # 2. FSM Execution
        if self.state == "OBSTACLE_AVOID":
            self.avoid_max_timer += dt
            if self.avoid_max_timer > 5.0:
                self.state = "SEARCHING"
                self.scan_phase = "SWEEP"
                self.phase_timer = 0.0
                return 140.0, 140.0, False

            if self.avoid_phase == "BACKOUT":
                self.avoid_timer -= dt
                if self.avoid_timer <= 0:
                    self.avoid_phase = "PIVOT"
                    self.target_yaw = robot_yaw - math.pi / 2.0 if dl >= dr else robot_yaw + math.pi / 2.0
                    self.avoid_turn_left = (dl >= dr)
                return -140.0, -140.0, False

            elif self.avoid_phase == "PIVOT":
                yaw_err = self.target_yaw - robot_yaw if self.target_yaw else 0.0
                yaw_err = (yaw_err + math.pi) % (2.0 * math.pi) - math.pi
                if abs(yaw_err) < 0.08 or self.phase_timer > 2.5:
                    self.state = "SEARCHING"
                    self.scan_phase = "SWEEP"
                    self.phase_timer = 0.0
                    return 0.0, 0.0, False

                speed = 135.0
                return (-speed, speed, False) if self.avoid_turn_left else (speed, -speed, False)

        elif self.state == "SEARCHING":
            if df < 40.0:
                self.state = "OBSTACLE_AVOID"
                self.avoid_phase = "PIVOT"
                self.phase_timer = 0.0
                turn_angle = math.pi / 2.0
                if dl >= dr:
                    self.target_yaw = robot_yaw - turn_angle
                    self.avoid_turn_left = True
                else:
                    self.target_yaw = robot_yaw + turn_angle
                    self.avoid_turn_left = False
                speed = 135.0
                return (-speed, speed, False) if self.avoid_turn_left else (speed, -speed, False)
            return 140.0, 140.0, False

        elif self.state == "ALIGNING":
            if not has_fire:
                if self.state_timer > 2.0:
                    self.state = "SEARCHING"
                return 0.0, 0.0, False

            if abs(target_yaw_err) > 0.08:
                turn = 120.0
                return (turn, -turn, False) if target_yaw_err > 0 else (-turn, turn, False)
            else:
                self.state = "APPROACHING"
                return 140.0, 140.0, False

        elif self.state == "APPROACHING":
            if not has_fire:
                self.state = "SEARCHING"
                return 0.0, 0.0, False

            if dist_fire <= 50.0 or df <= 46.0:
                self.state = "EXTINGUISHING"
                self.state_timer = 0.0
                return 0.0, 0.0, True

            trim = target_yaw_err * 80.0
            speed = 135.0
            return speed + trim, speed - trim, False

        elif self.state == "EXTINGUISHING":
            return 0.0, 0.0, True

        return 0.0, 0.0, False


def run_benchmark(
    num_episodes: int = 50,
    max_steps_per_episode: int = 600,
    dt: float = 0.05
) -> Dict[str, Dict[str, float]]:
    """
    Executes identical randomized arena episodes across Rule-Based FSM vs Neural Navigation Model.
    """
    print(f"\n=================================================================")
    print(f"   BANTAY-SUNOG NAVIGATION BENCHMARK (N={num_episodes} Episodes)   ")
    print(f"=================================================================")

    sim = ArenaSim()
    fsm_controller = RuleBasedFSMController()
    neural_runner = NavPolicyRunner("simulation/models/navigation_model.json")

    results = {
        "FSM": {
            "success_count": 0,
            "extinguish_times": [],
            "collision_counts": [],
            "angular_jerks": [],
            "linear_jerks": [],
            "path_lengths": []
        },
        "Neural": {
            "success_count": 0,
            "extinguish_times": [],
            "collision_counts": [],
            "angular_jerks": [],
            "linear_jerks": [],
            "path_lengths": []
        }
    }

    seeds = [1000 + i for i in range(num_episodes)]

    for mode in ["FSM", "Neural"]:
        print(f"\n>>> Running Benchmark: {mode} Controller...")
        for ep_idx, seed in enumerate(seeds):
            random.seed(seed)
            np.random.seed(seed)

            # Randomize layout
            sim.randomize_arena(num_crates=random.randint(3, 5), num_barrels=random.randint(2, 4), add_fire=True)
            if mode == "FSM":
                fsm_controller.reset()

            prev_pwm_l = 0.0
            prev_pwm_r = 0.0
            prev_ang_speed = 0.0
            total_path = 0.0
            ang_jerks = []
            lin_jerks = []
            extinguished = False
            extinguish_time = max_steps_per_episode * dt

            for step in range(max_steps_per_episode):
                sensors = sim.get_sensor_readings()
                old_x, old_z = sim.x, sim.z

                if mode == "FSM":
                    pwm_l, pwm_r, is_ext = fsm_controller.step(sensors, sim.yaw, dt)
                else:
                    # Neural Policy
                    dist_f = sensors["dist_front"]
                    dist_l = sensors["dist_left"]
                    dist_r = sensors["dist_right"]
                    has_fire = sensors["has_fire_target"] > 0.5
                    rel_yaw = sensors["target_rel_yaw"]
                    dist_fire = sensors["target_distance"]

                    if has_fire and (dist_fire <= 0.50 or dist_f <= 0.46):
                        pwm_l, pwm_r = 0.0, 0.0
                        is_ext = True
                    else:
                        pwm_l, pwm_r = neural_runner.step(
                            dist_front_m=dist_f,
                            dist_left_m=dist_l,
                            dist_right_m=dist_r,
                            target_rel_yaw_rad=rel_yaw,
                            target_dist_m=dist_fire,
                            has_fire=has_fire,
                            heading_rate_rads=sim.current_angular_speed
                        )
                        is_ext = False

                # Measure jerk (acceleration delta)
                lin_jerk = abs(pwm_l - prev_pwm_l) + abs(pwm_r - prev_pwm_r)
                ang_jerk = abs(sim.current_angular_speed - prev_ang_speed) / dt
                lin_jerks.append(lin_jerk)
                ang_jerks.append(ang_jerk)

                prev_pwm_l = pwm_l
                prev_pwm_r = pwm_r
                prev_ang_speed = sim.current_angular_speed

                # Step physics
                sim.step(pwm_l, pwm_r, dt=dt)
                total_path += math.hypot(sim.x - old_x, sim.z - old_z)

                if is_ext:
                    extinguished = True
                    extinguish_time = step * dt
                    break

            # Record episode metrics
            if extinguished:
                results[mode]["success_count"] += 1
                results[mode]["extinguish_times"].append(extinguish_time)
            results[mode]["collision_counts"].append(sim.collision_count)
            results[mode]["angular_jerks"].append(np.mean(ang_jerks) if ang_jerks else 0.0)
            results[mode]["linear_jerks"].append(np.mean(lin_jerks) if lin_jerks else 0.0)
            results[mode]["path_lengths"].append(total_path)

        print(f"Finished {num_episodes} episodes for {mode}.")

    # Summary Statistics
    summary = {}
    for mode in ["FSM", "Neural"]:
        succ = results[mode]["success_count"]
        succ_rate = (succ / num_episodes) * 100.0
        avg_time = np.mean(results[mode]["extinguish_times"]) if succ > 0 else float("inf")
        med_time = np.median(results[mode]["extinguish_times"]) if succ > 0 else float("inf")
        tot_col = sum(results[mode]["collision_counts"])
        avg_col = np.mean(results[mode]["collision_counts"])
        avg_ang_jerk = np.mean(results[mode]["angular_jerks"])
        avg_lin_jerk = np.mean(results[mode]["linear_jerks"])
        avg_path = np.mean(results[mode]["path_lengths"])

        summary[mode] = {
            "success_rate_pct": succ_rate,
            "avg_extinguish_time_s": avg_time,
            "median_extinguish_time_s": med_time,
            "total_collisions": tot_col,
            "avg_collisions_per_ep": avg_col,
            "avg_angular_jerk": avg_ang_jerk,
            "avg_linear_pwm_jerk": avg_lin_jerk,
            "avg_path_length_m": avg_path
        }

    # Print Formatted Report Table
    print("\n" + "=" * 70)
    print("                     BENCHMARK COMPARISON REPORT                     ")
    print("=" * 70)
    print(f"{'Metric':<32} | {'Rule-Based FSM':<16} | {'Neural TinyML':<16}")
    print("-" * 70)
    print(f"{'Success Rate (%)':<32} | {summary['FSM']['success_rate_pct']:>15.1f}% | {summary['Neural']['success_rate_pct']:>15.1f}%")
    print(f"{'Avg Time to Fire (s)':<32} | {summary['FSM']['avg_extinguish_time_s']:>15.2f}s | {summary['Neural']['avg_extinguish_time_s']:>15.2f}s")
    print(f"{'Median Time to Fire (s)':<32} | {summary['FSM']['median_extinguish_time_s']:>15.2f}s | {summary['Neural']['median_extinguish_time_s']:>15.2f}s")
    print(f"{'Total Collisions':<32} | {summary['FSM']['total_collisions']:>16d} | {summary['Neural']['total_collisions']:>16d}")
    print(f"{'Avg Collisions / Episode':<32} | {summary['FSM']['avg_collisions_per_ep']:>16.2f} | {summary['Neural']['avg_collisions_per_ep']:>16.2f}")
    print(f"{'Angular Jerk (rad/s^2)':<32} | {summary['FSM']['avg_angular_jerk']:>16.2f} | {summary['Neural']['avg_angular_jerk']:>16.2f}")
    print(f"{'PWM Jerk (delta PWM/step)':<32} | {summary['FSM']['avg_linear_pwm_jerk']:>16.2f} | {summary['Neural']['avg_linear_pwm_jerk']:>16.2f}")
    print(f"{'Avg Path Length (m)':<32} | {summary['FSM']['avg_path_length_m']:>15.2f}m | {summary['Neural']['avg_path_length_m']:>15.2f}m")
    print("=" * 70 + "\n")

    return summary


if __name__ == "__main__":
    run_benchmark(num_episodes=50)
