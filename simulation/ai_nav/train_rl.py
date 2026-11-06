"""
BANTAY-SUNOG Reinforcement Learning Training Engine
Optimizes the TinyMLNavPolicy directly using point-based rewards:
- Rewards: Obstacle dodging, room patrol, fire homing progress, standoff suppression (+100 pts)
- Penalties: Collisions (-60 pts), dangerous front proximity (-25 pts)
Exports policy to ESP32 C++, TorchScript/Python, and Three.js JavaScript.
"""

import copy
import json
import os
import random
import time
from typing import Dict, List, Tuple
import numpy as np
import torch
import torch.nn as nn

from simulation.ai_nav.model import TinyMLNavPolicy, MAX_PWM
from simulation.ai_nav.rl_env import BantayNavRLEnv
from simulation.ai_nav.train import (
    export_cpp_header,
    export_torchscript_and_json,
    export_js_runner
)


def get_flat_params(model: nn.Module) -> np.ndarray:
    """Flatten all model parameters into a 1D numpy array."""
    params = []
    for p in model.parameters():
        params.append(p.data.cpu().numpy().flatten())
    return np.concatenate(params)


def set_flat_params(model: nn.Module, flat_params: np.ndarray):
    """Load a 1D numpy array back into model parameters."""
    offset = 0
    for p in model.parameters():
        numel = p.numel()
        shape = p.shape
        p.data = torch.from_numpy(flat_params[offset:offset + numel].reshape(shape)).float()
        offset += numel


def evaluate_policy(
    model: nn.Module,
    env: BantayNavRLEnv,
    episodes: int = 4,
    max_steps: int = 250
) -> Tuple[float, int, int]:
    """
    Evaluates policy over multiple episodes.
    Returns: (mean_reward, total_collisions, total_extinguished)
    """
    total_reward = 0.0
    total_collisions = 0
    total_extinguished = 0

    with torch.no_grad():
        for _ in range(episodes):
            obs = env.reset(randomize=True)
            ep_reward = 0.0

            for _ in range(max_steps):
                tensor_obs = torch.tensor(obs, dtype=torch.float32).unsqueeze(0)
                action = model(tensor_obs).squeeze(0).numpy()

                obs, reward, done, info = env.step(action)
                ep_reward += reward

                if info["collision"]:
                    total_collisions += 1
                if info["extinguished"]:
                    total_extinguished += 1

                if done:
                    break

            total_reward += ep_reward

    return total_reward / episodes, total_collisions, total_extinguished


def train_rl_policy(
    num_iterations: int = 40,
    population_size: int = 14,
    sigma: float = 0.04,
    learning_rate: float = 0.02,
    warm_start_path: str = "simulation/models/navigation_model.json"
) -> TinyMLNavPolicy:
    """
    Trains the TinyML navigation model using Evolutionary Policy Optimization
    with antithetic sampling and point-based reward shaping.
    """
    print("=" * 70)
    print("      BANTAY-SUNOG REINFORCEMENT LEARNING TRAINING ENGINE      ")
    print("=" * 70)
    print(f"Algorithm: Evolutionary Policy Optimization (Antithetic ES)")
    print(f"Iterations: {num_iterations} | Population: {population_size} | Sigma: {sigma}")
    print(f"Reward Points: +100 Suppression | +8 Obstacle Dodge | -60 Collision")
    print("=" * 70)

    model = TinyMLNavPolicy()
    env = BantayNavRLEnv()

    # Warm-start from previous weights if available
    if os.path.exists(warm_start_path):
        print(f"[RL] Warm-starting policy from {warm_start_path}...")
        with open(warm_start_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        state_dict = {
            "fc1.weight": torch.tensor(data["W1"], dtype=torch.float32),
            "fc1.bias": torch.tensor(data["B1"], dtype=torch.float32),
            "fc2.weight": torch.tensor(data["W2"], dtype=torch.float32),
            "fc2.bias": torch.tensor(data["B2"], dtype=torch.float32),
            "fc3.weight": torch.tensor(data["W3"], dtype=torch.float32),
            "fc3.bias": torch.tensor(data["B3"], dtype=torch.float32),
        }
        model.load_state_dict(state_dict)

    flat_params = get_flat_params(model)
    num_params = len(flat_params)
    print(f"[RL] Trainable parameters: {num_params} floats (~{num_params * 4 / 1024:.1f} KB)")

    # Baseline evaluation
    base_reward, base_col, base_ext = evaluate_policy(model, env, episodes=5)
    print(f"[RL] Initial Policy: Reward = {base_reward:.1f} | Collisions = {base_col} | Extinguished = {base_ext}")

    best_reward = base_reward
    best_params = flat_params.copy()

    start_time = time.time()

    for it in range(1, num_iterations + 1):
        # Sample antithetic perturbations: +delta and -delta
        deltas = [np.random.randn(num_params).astype(np.float32) for _ in range(population_size)]
        rewards_pos = []
        rewards_neg = []

        # Evaluate positive perturbations
        for delta in deltas:
            candidate_pos = flat_params + sigma * delta
            set_flat_params(model, candidate_pos)
            r_pos, _, _ = evaluate_policy(model, env, episodes=2, max_steps=200)
            rewards_pos.append(r_pos)

            candidate_neg = flat_params - sigma * delta
            set_flat_params(model, candidate_neg)
            r_neg, _, _ = evaluate_policy(model, env, episodes=2, max_steps=200)
            rewards_neg.append(r_neg)

        # Standardize rewards for stable gradient estimation
        all_rewards = np.array(rewards_pos + rewards_neg)
        r_std = np.std(all_rewards)
        if r_std < 1e-6:
            r_std = 1.0

        # Compute gradient step
        grad = np.zeros(num_params, dtype=np.float32)
        for i in range(population_size):
            diff = (rewards_pos[i] - rewards_neg[i]) / r_std
            grad += diff * deltas[i]
        grad /= (2.0 * population_size)

        # Update weights
        flat_params += learning_rate * grad
        set_flat_params(model, flat_params)

        # Evaluate current policy
        if it % 5 == 0 or it == 1 or it == num_iterations:
            eval_reward, eval_col, eval_ext = evaluate_policy(model, env, episodes=5)
            elapsed = time.time() - start_time
            print(f"[RL] Iter {it:2d}/{num_iterations} ({elapsed:4.1f}s) | Avg Reward: {eval_reward:>6.1f} | Collisions: {eval_col} | Extinguished: {eval_ext}")

            if eval_reward > best_reward:
                best_reward = eval_reward
                best_params = flat_params.copy()

    print("=" * 70)
    print(f"[RL] Training complete! Best Reward: {best_reward:.1f}")
    set_flat_params(model, best_params)

    # Multi-platform exports
    print("\n[RL] Exporting optimized policy across platforms...")
    export_cpp_header(model, "bantay_sunog_esp32/navigation_model.h")
    export_cpp_header(model, "esp32_firmware/include/navigation_model.h")
    export_torchscript_and_json(model, "simulation/models")
    export_js_runner(model, "simulation/frontend/js/nav_model_runner.js")
    print("[RL] All exports updated successfully!")

    return model


if __name__ == "__main__":
    train_rl_policy(num_iterations=30, population_size=12, sigma=0.035, learning_rate=0.025)
