"""
BANTAY-SUNOG Python / Raspberry Pi Inference Runner
Loads trained model weights (TorchScript .pt or JSON weights) and provides
sub-millisecond local navigation policy inference using NumPy or PyTorch.
"""

import json
import math
import os
from typing import Dict, Optional, Tuple, Union
import numpy as np


class NavPolicyRunner:
    """
    Python inference runner for BANTAY-SUNOG TinyML Local Navigation Policy.
    Compatible with Raspberry Pi 3B (pure NumPy or PyTorch).
    """
    def __init__(
        self,
        model_path: Optional[str] = None
    ):
        # Default model location
        if model_path is None:
            # Check for JSON first (zero PyTorch dependency)
            json_candidate = "simulation/models/navigation_model.json"
            pt_candidate = "simulation/models/navigation_model.pt"
            if os.path.exists(json_candidate):
                model_path = json_candidate
            elif os.path.exists(pt_candidate):
                model_path = pt_candidate
            else:
                raise FileNotFoundError("Could not find navigation_model.json or navigation_model.pt")

        self.model_path = model_path
        self.backend = "numpy"

        if model_path.endswith(".json"):
            with open(model_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            self.W1 = np.array(data["W1"], dtype=np.float32)  # (32, 7)
            self.B1 = np.array(data["B1"], dtype=np.float32)  # (32,)
            self.W2 = np.array(data["W2"], dtype=np.float32)  # (16, 32)
            self.B2 = np.array(data["B2"], dtype=np.float32)  # (16,)
            self.W3 = np.array(data["W3"], dtype=np.float32)  # (2, 16)
            self.B3 = np.array(data["B3"], dtype=np.float32)  # (2,)
            self.backend = "numpy"
        elif model_path.endswith(".pt"):
            import torch
            self.torch_model = torch.jit.load(model_path)
            self.torch_model.eval()
            self.backend = "torch"
        else:
            raise ValueError(f"Unsupported model extension: {model_path}")

    def predict(self, norm_inputs: np.ndarray) -> Tuple[float, float]:
        """
        Forward pass with normalized inputs array of shape (7,).
        Returns:
            (left_pwm, right_pwm) in [-255.0, +255.0]
        """
        if self.backend == "numpy":
            # Layer 1: Dense(7, 32) + ReLU
            h1 = np.maximum(0.0, np.dot(self.W1, norm_inputs) + self.B1)
            # Layer 2: Dense(32, 16) + ReLU
            h2 = np.maximum(0.0, np.dot(self.W2, h1) + self.B2)
            # Output: Dense(16, 2)
            out = np.dot(self.W3, h2) + self.B3
            # Clamping
            out = np.clip(out, -255.0, 255.0)
            return float(out[0]), float(out[1])
        else:
            import torch
            with torch.no_grad():
                tensor_in = torch.tensor(norm_inputs, dtype=torch.float32).unsqueeze(0)
                preds = self.torch_model(tensor_in).squeeze(0).numpy()
            return float(preds[0]), float(preds[1])

    def step(
        self,
        dist_front_m: float,
        dist_left_m: float,
        dist_right_m: float,
        target_rel_yaw_rad: float,
        target_dist_m: float,
        has_fire: bool,
        heading_rate_rads: float
    ) -> Tuple[float, float]:
        """Convenience method accepting physical sensor readings."""
        from simulation.ai_nav.model import normalize_inputs
        norm = normalize_inputs(
            dist_front=dist_front_m,
            dist_left=dist_left_m,
            dist_right=dist_right_m,
            target_rel_yaw=target_rel_yaw_rad,
            target_distance=target_dist_m,
            has_fire_target=1.0 if has_fire else 0.0,
            heading_rate=heading_rate_rads
        )
        left_pwm, right_pwm = self.predict(norm)

        # Anti-stall obstacle overtake guard: if path blocked and policy stalled without fire, overtake!
        if not has_fire and dist_front_m < 0.65 and abs(left_pwm) < 50 and abs(right_pwm) < 50:
            turn_right = (dist_right_m >= dist_left_m)
            inner = -20.0 if dist_front_m < 0.40 else 40.0
            left_pwm = 150.0 if turn_right else inner
            right_pwm = inner if turn_right else 150.0

        return left_pwm, right_pwm
