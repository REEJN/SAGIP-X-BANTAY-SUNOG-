"""
BANTAY-SUNOG TinyML Local Navigation Policy
Architecture: Dense(7, 32) -> ReLU -> Dense(32, 16) -> ReLU -> Dense(16, 2)
Total Parameters: 818 floats (~3.2 KB)
"""

import math
import numpy as np
import torch
import torch.nn as nn

# Constants for sensor normalization
MAX_ULTRASONIC_RANGE = 3.0   # meters
MAX_FIRE_DISTANCE = 5.0      # meters
MAX_HEADING_RATE = 3.0       # rad/s
MAX_PWM = 255.0


def normalize_inputs(
    dist_front: float,
    dist_left: float,
    dist_right: float,
    target_rel_yaw: float,
    target_distance: float,
    has_fire_target: float,
    heading_rate: float
) -> np.ndarray:
    """
    Normalize raw physical sensor measurements into bounded float features.
    
    Returns:
        np.ndarray of shape (7,), dtype float32 with values generally in [-1.0, 1.0].
    """
    # 1. dist_front (meters clamped [0.0, 3.0] -> [0.0, 1.0])
    df_norm = float(np.clip(dist_front, 0.0, MAX_ULTRASONIC_RANGE) / MAX_ULTRASONIC_RANGE)
    
    # 2. dist_left (meters clamped [0.0, 3.0] -> [0.0, 1.0])
    dl_norm = float(np.clip(dist_left, 0.0, MAX_ULTRASONIC_RANGE) / MAX_ULTRASONIC_RANGE)
    
    # 3. dist_right (meters clamped [0.0, 3.0] -> [0.0, 1.0])
    dr_norm = float(np.clip(dist_right, 0.0, MAX_ULTRASONIC_RANGE) / MAX_ULTRASONIC_RANGE)
    
    # 4. target_rel_yaw (radians [-PI, +PI] -> [-1.0, 1.0])
    # Wrap to [-PI, +PI]
    yaw_err = (target_rel_yaw + math.pi) % (2 * math.pi) - math.pi
    yaw_norm = float(np.clip(yaw_err / math.pi, -1.0, 1.0))
    
    # 5. target_distance (meters [0.0, 5.0] -> [0.0, 1.0])
    td_norm = float(np.clip(target_distance, 0.0, MAX_FIRE_DISTANCE) / MAX_FIRE_DISTANCE)
    
    # 6. has_fire_target (1.0 if fire confirmed, 0.0 if searching)
    hf_norm = 1.0 if has_fire_target > 0.5 else 0.0
    
    # 7. heading_rate (rad/s [-3.0, 3.0] -> [-1.0, 1.0])
    hr_norm = float(np.clip(heading_rate, -MAX_HEADING_RATE, MAX_HEADING_RATE) / MAX_HEADING_RATE)
    
    return np.array([df_norm, dl_norm, dr_norm, yaw_norm, td_norm, hf_norm, hr_norm], dtype=np.float32)


class TinyMLNavPolicy(nn.Module):
    """
    Ultra-lightweight Multilayer Perceptron local navigation policy
    suitable for deployment on ESP32 C++ and Raspberry Pi.
    """
    def __init__(self):
        super().__init__()
        self.fc1 = nn.Linear(7, 32)
        self.relu1 = nn.ReLU()
        self.fc2 = nn.Linear(32, 16)
        self.relu2 = nn.ReLU()
        self.fc3 = nn.Linear(16, 2)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Forward pass.
        Args:
            x: Tensor of shape (B, 7) or (7,)
        Returns:
            Tensor of shape (B, 2) continuous (left_pwm, right_pwm) in [-255, +255]
        """
        h1 = self.relu1(self.fc1(x))
        h2 = self.relu2(self.fc2(h1))
        out = self.fc3(h2)
        return torch.clamp(out, -MAX_PWM, MAX_PWM)

    def get_parameter_count(self) -> int:
        return sum(p.numel() for p in self.parameters() if p.requires_grad)
