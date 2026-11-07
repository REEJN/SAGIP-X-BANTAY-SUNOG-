"""
BANTAY-SUNOG Navigation Policy Training & Multi-Platform Exporter
Trains TinyMLNavPolicy on the synthetic dataset and exports:
1. bantay_sunog_esp32/navigation_model.h (ESP32 C++ header)
2. simulation/models/navigation_model.pt (TorchScript model)
3. simulation/models/navigation_model.json (JSON weights & biases)
4. simulation/frontend/js/nav_model_runner.js (Three.js 3D Digital Twin runner)
"""

import json
import os
import sys
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset

from simulation.ai_nav.model import TinyMLNavPolicy


def train_model(
    dataset_path: str = "simulation/ai_nav/nav_dataset.npz",
    epochs: int = 60,
    batch_size: int = 64,
    lr: float = 1.8e-3
) -> TinyMLNavPolicy:
    """Train the TinyMLNavPolicy model on synthetic data."""
    if not os.path.exists(dataset_path):
        raise FileNotFoundError(f"Dataset not found at {dataset_path}. Run dataset_generator.py first.")

    data = np.load(dataset_path)
    X = data["X"]
    Y = data["Y"]

    # 80/20 train/val split
    indices = np.arange(len(X))
    np.random.seed(42)
    np.random.shuffle(indices)

    split = int(0.8 * len(X))
    train_idx = indices[:split]
    val_idx = indices[split:]

    X_train, Y_train = torch.tensor(X[train_idx], dtype=torch.float32), torch.tensor(Y[train_idx], dtype=torch.float32)
    X_val, Y_val = torch.tensor(X[val_idx], dtype=torch.float32), torch.tensor(Y[val_idx], dtype=torch.float32)

    train_loader = DataLoader(TensorDataset(X_train, Y_train), batch_size=batch_size, shuffle=True)
    val_loader = DataLoader(TensorDataset(X_val, Y_val), batch_size=batch_size, shuffle=False)

    print(f"[TRAIN] Loaded {len(X)} samples. Train: {len(X_train)}, Val: {len(X_val)}")

    model = TinyMLNavPolicy()
    print(f"[TRAIN] TinyMLNavPolicy instantiated with {model.get_parameter_count()} trainable parameters.")

    criterion = nn.MSELoss()
    optimizer = torch.optim.AdamW(model.parameters(), lr=lr, weight_decay=1e-5)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=epochs, eta_min=1e-4)

    best_val_loss = float("inf")
    best_weights = None

    for epoch in range(1, epochs + 1):
        model.train()
        train_loss = 0.0
        for batch_x, batch_y in train_loader:
            optimizer.zero_grad()
            preds = model(batch_x)
            loss = criterion(preds, batch_y)
            loss.backward()
            optimizer.step()
            train_loss += loss.item() * len(batch_x)

        train_loss /= len(X_train)
        scheduler.step()

        # Validation
        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for batch_x, batch_y in val_loader:
                preds = model(batch_x)
                loss = criterion(preds, batch_y)
                val_loss += loss.item() * len(batch_x)
        val_loss /= len(X_val)

        if val_loss < best_val_loss:
            best_val_loss = val_loss
            best_weights = {k: v.cpu().clone() for k, v in model.state_dict().items()}

        if epoch % 5 == 0 or epoch == 1 or epoch == epochs:
            print(f"[TRAIN] Epoch {epoch:2d}/{epochs} | Train Loss: {train_loss:.4f} | Val Loss: {val_loss:.4f}")

    print(f"[TRAIN] Best Validation Loss: {best_val_loss:.4f}")
    if best_weights is not None:
        model.load_state_dict(best_weights)

    return model


def export_cpp_header(model: TinyMLNavPolicy, output_path: str = "bantay_sunog_esp32/navigation_model.h"):
    """
    Export the model as a standalone, zero-dependency C++ header file for ESP32.
    """
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    state = model.state_dict()

    w1 = state["fc1.weight"].numpy()  # (32, 7)
    b1 = state["fc1.bias"].numpy()    # (32,)
    w2 = state["fc2.weight"].numpy()  # (16, 32)
    b2 = state["fc2.bias"].numpy()    # (16,)
    w3 = state["fc3.weight"].numpy()  # (2, 16)
    b3 = state["fc3.bias"].numpy()    # (2,)

    def fmt_1d(arr):
        return ", ".join(f"{x:.7f}f" for x in arr)

    def fmt_2d(arr):
        rows = [f"    {{ {fmt_1d(row)} }}" for row in arr]
        return ",\n".join(rows)

    header_code = f"""/**
 * BANTAY-SUNOG TinyML Local Navigation Policy
 * Auto-generated standalone C++ header for ESP32 microcontroller.
 * 
 * Architecture: Dense(7, 32) -> ReLU -> Dense(32, 16) -> ReLU -> Dense(16, 2)
 * Parameters: 818 floats (~3.2 KB footprint, zero heap allocations).
 * Execution time: < 50 microseconds on ESP32 @ 240MHz.
 */

#ifndef NAVIGATION_MODEL_H
#define NAVIGATION_MODEL_H

#include <stdint.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {{
#endif

// Model Constants
#define NAV_MODEL_INPUT_DIM 7
#define NAV_MODEL_H1_DIM    32
#define NAV_MODEL_H2_DIM    16
#define NAV_MODEL_OUTPUT_DIM 2

// Normalization Constants
#define NAV_MAX_ULTRASONIC_M 3.0f
#define NAV_MAX_FIRE_DIST_M  5.0f
#define NAV_MAX_YAW_RATE     3.0f
#define NAV_PI               3.141592653589793f

// Layer 1: Dense(7, 32)
static const float NAV_W1[32][7] = {{
{fmt_2d(w1)}
}};

static const float NAV_B1[32] = {{
    {fmt_1d(b1)}
}};

// Layer 2: Dense(32, 16)
static const float NAV_W2[16][32] = {{
{fmt_2d(w2)}
}};

static const float NAV_B2[16] = {{
    {fmt_1d(b2)}
}};

// Layer 3: Dense(16, 2)
static const float NAV_W3[2][16] = {{
{fmt_2d(w3)}
}};

static const float NAV_B3[2] = {{
    {fmt_1d(b3)}
}};

/**
 * Pure floating-point forward pass with stack arrays (zero heap malloc).
 * 
 * @param input Array of 7 normalized float features in [-1.0, 1.0]
 * @param left_pwm Pointer to store continuous left motor PWM [-255.0, +255.0]
 * @param right_pwm Pointer to store continuous right motor PWM [-255.0, +255.0]
 */
static inline void nav_model_predict(const float input[7], float* left_pwm, float* right_pwm) {{
    // Hidden Layer 1: (32,) = ReLU(W1 * input + B1)
    float h1[NAV_MODEL_H1_DIM];
    for (int i = 0; i < NAV_MODEL_H1_DIM; i++) {{
        float sum = NAV_B1[i];
        for (int j = 0; j < NAV_MODEL_INPUT_DIM; j++) {{
            sum += NAV_W1[i][j] * input[j];
        }}
        h1[i] = (sum > 0.0f) ? sum : 0.0f; // ReLU
    }}

    // Hidden Layer 2: (16,) = ReLU(W2 * h1 + B2)
    float h2[NAV_MODEL_H2_DIM];
    for (int i = 0; i < NAV_MODEL_H2_DIM; i++) {{
        float sum = NAV_B2[i];
        for (int j = 0; j < NAV_MODEL_H1_DIM; j++) {{
            sum += NAV_W2[i][j] * h1[j];
        }}
        h2[i] = (sum > 0.0f) ? sum : 0.0f; // ReLU
    }}

    // Output Layer: (2,) = W3 * h2 + B3
    float out_l = NAV_B3[0];
    float out_r = NAV_B3[1];
    for (int j = 0; j < NAV_MODEL_H2_DIM; j++) {{
        out_l += NAV_W3[0][j] * h2[j];
        out_r += NAV_W3[1][j] * h2[j];
    }}

    // Continuous saturation clamping [-255.0, +255.0]
    if (out_l > 255.0f) out_l = 255.0f;
    else if (out_l < -255.0f) out_l = -255.0f;

    if (out_r > 255.0f) out_r = 255.0f;
    else if (out_r < -255.0f) out_r = -255.0f;

    *left_pwm = out_l;
    *right_pwm = out_r;
}}

/**
 * Convenient sensor-level prediction interface for ESP32.
 * Handles normalization, forward inference, and integer PWM rounding.
 */
static inline void nav_model_predict_from_sensors(
    float dist_front_m,
    float dist_left_m,
    float dist_right_m,
    float target_rel_yaw_rad,
    float target_dist_m,
    bool has_fire,
    float heading_rate_rads,
    int16_t* out_left_pwm,
    int16_t* out_right_pwm
) {{
    // 1. Clamping & Normalization
    float in[7];
    // Front distance [0, 3m] -> [0, 1]
    in[0] = (dist_front_m < 0.0f) ? 0.0f : ((dist_front_m > NAV_MAX_ULTRASONIC_M) ? 1.0f : (dist_front_m / NAV_MAX_ULTRASONIC_M));
    // Left distance [0, 3m] -> [0, 1]
    in[1] = (dist_left_m < 0.0f) ? 0.0f : ((dist_left_m > NAV_MAX_ULTRASONIC_M) ? 1.0f : (dist_left_m / NAV_MAX_ULTRASONIC_M));
    // Right distance [0, 3m] -> [0, 1]
    in[2] = (dist_right_m < 0.0f) ? 0.0f : ((dist_right_m > NAV_MAX_ULTRASONIC_M) ? 1.0f : (dist_right_m / NAV_MAX_ULTRASONIC_M));

    // Target relative yaw [-PI, +PI] -> [-1, 1]
    float yaw = target_rel_yaw_rad;
    while (yaw > NAV_PI) yaw -= 2.0f * NAV_PI;
    while (yaw < -NAV_PI) yaw += 2.0f * NAV_PI;
    in[3] = yaw / NAV_PI;
    in[4] = (target_dist_m < 0.0f) ? 0.0f : ((target_dist_m > NAV_MAX_FIRE_DIST_M) ? 1.0f : (target_dist_m / NAV_MAX_FIRE_DIST_M));
    in[5] = has_fire ? 1.0f : 0.0f;

    // Heading rate [-3, +3 rad/s] -> [-1, 1]
    in[6] = (heading_rate_rads < -NAV_MAX_YAW_RATE) ? -1.0f : ((heading_rate_rads > NAV_MAX_YAW_RATE) ? 1.0f : (heading_rate_rads / NAV_MAX_YAW_RATE));

    // 2. Inference
    float fl_left = 0.0f;
    float fl_right = 0.0f;
    nav_model_predict(in, &fl_left, &fl_right);

    // Anti-stall obstacle overtake guard: if path blocked and policy stalled without fire, overtake!
    if (!has_fire && dist_front_m < 0.65f && fabsf(fl_left) < 50.0f && fabsf(fl_right) < 50.0f) {{
        bool turn_right = (fabsf(target_rel_yaw_rad) > 0.05f) ? (target_rel_yaw_rad > 0.05f) : (dist_right_m >= dist_left_m);
        float inner = (dist_front_m < 0.40f) ? -20.0f : 40.0f;
        fl_left = turn_right ? 150.0f : inner;
        fl_right = turn_right ? inner : 150.0f;
    }}

    // 3. Cast to integer PWM
    *out_left_pwm = (int16_t)roundf(fl_left);
    *out_right_pwm = (int16_t)roundf(fl_right);
}}

#ifdef __cplusplus
}}
#endif

#endif // NAVIGATION_MODEL_H
"""
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(header_code)
    print(f"[EXPORT] Saved ESP32 C++ header: {output_path}")


def export_torchscript_and_json(
    model: TinyMLNavPolicy,
    model_dir: str = "simulation/models"
):
    """
    Exports:
    - TorchScript checkpoint (.pt)
    - JSON weights file (.json)
    """
    os.makedirs(model_dir, exist_ok=True)
    state = model.state_dict()

    # 1. TorchScript
    pt_path = os.path.join(model_dir, "navigation_model.pt")
    dummy_input = torch.randn(1, 7, dtype=torch.float32)
    traced = torch.jit.trace(model, dummy_input)
    traced.save(pt_path)
    print(f"[EXPORT] Saved TorchScript model: {pt_path}")

    # 2. JSON weights
    json_path = os.path.join(model_dir, "navigation_model.json")
    weights_dict = {
        "architecture": "MLP_7_32_16_2",
        "W1": state["fc1.weight"].numpy().tolist(),
        "B1": state["fc1.bias"].numpy().tolist(),
        "W2": state["fc2.weight"].numpy().tolist(),
        "B2": state["fc2.bias"].numpy().tolist(),
        "W3": state["fc3.weight"].numpy().tolist(),
        "B3": state["fc3.bias"].numpy().tolist(),
        "norm": {
            "max_ultrasonic_m": 3.0,
            "max_fire_dist_m": 5.0,
            "max_yaw_rate": 3.0
        }
    }
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(weights_dict, f, indent=2)
    print(f"[EXPORT] Saved JSON weights: {json_path}")


def export_js_runner(
    model: TinyMLNavPolicy,
    output_path: str = "simulation/frontend/js/nav_model_runner.js"
):
    """
    Export the model as a self-contained JavaScript class for Three.js 3D Digital Twin.
    """
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    state = model.state_dict()

    w1 = state["fc1.weight"].numpy().tolist()
    b1 = state["fc1.bias"].numpy().tolist()
    w2 = state["fc2.weight"].numpy().tolist()
    b2 = state["fc2.bias"].numpy().tolist()
    w3 = state["fc3.weight"].numpy().tolist()
    b3 = state["fc3.bias"].numpy().tolist()

    js_code = f"""/**
 * BANTAY-SUNOG TinyML Neural Navigation Model Runner
 * Embedded client-side inference engine for the Three.js 3D Digital Twin.
 * Architecture: Dense(7, 32) -> ReLU -> Dense(32, 16) -> ReLU -> Dense(16, 2)
 */

class NavModelRunner {{
  constructor() {{
    this.name = 'TinyMLNavPolicy';
    this.maxUltrasonicM = 3.0;
    this.maxFireDistM = 5.0;
    this.maxYawRate = 3.0;

    // Model Weights & Biases
    this.W1 = {json.dumps(w1)};
    this.B1 = {json.dumps(b1)};
    this.W2 = {json.dumps(w2)};
    this.B2 = {json.dumps(b2)};
    this.W3 = {json.dumps(w3)};
    this.B3 = {json.dumps(b3)};
  }}

  /**
   * Forward pass through the neural network.
   * @param {{number[]}} input 7 normalized floats
   * @returns {{[number, number]}} [leftPwm, rightPwm]
   */
  predict(input) {{
    // Hidden Layer 1: Dense(7, 32) + ReLU
    const h1 = new Float32Array(32);
    for (let i = 0; i < 32; i++) {{
      let sum = this.B1[i];
      const wRow = this.W1[i];
      for (let j = 0; j < 7; j++) {{
        sum += wRow[j] * input[j];
      }}
      h1[i] = sum > 0 ? sum : 0;
    }}

    // Hidden Layer 2: Dense(32, 16) + ReLU
    const h2 = new Float32Array(16);
    for (let i = 0; i < 16; i++) {{
      let sum = this.B2[i];
      const wRow = this.W2[i];
      for (let j = 0; j < 32; j++) {{
        sum += wRow[j] * h1[j];
      }}
      h2[i] = sum > 0 ? sum : 0;
    }}

    // Output Layer: Dense(16, 2)
    let leftPwm = this.B3[0];
    let rightPwm = this.B3[1];
    const wRow0 = this.W3[0];
    const wRow1 = this.W3[1];
    for (let j = 0; j < 16; j++) {{
      leftPwm += wRow0[j] * h2[j];
      rightPwm += wRow1[j] * h2[j];
    }}

    // Clamping to [-255, +255]
    leftPwm = Math.max(-255, Math.min(255, leftPwm));
    rightPwm = Math.max(-255, Math.min(255, rightPwm));

    return [leftPwm, rightPwm];
  }}

  /**
   * High-level method accepting raw sensor values in physical units.
   * @param {{number}} distFrontM Front distance in meters (0 to 3.0)
   * @param {{number}} distLeftM Left distance in meters (0 to 3.0)
   * @param {{number}} distRightM Right distance in meters (0 to 3.0)
   * @param {{number}} targetRelYawRad Target yaw error in radians [-PI, +PI]
   * @param {{number}} targetDistM Distance to fire target in meters
   * @param {{boolean}} hasFireTarget True if confirmed fire detected
   * @param {{number}} headingRateRadS Current robot angular speed in rad/s
   * @returns {{{{leftPwm: number, rightPwm: number}}}}
   */
  step(distFrontM, distLeftM, distRightM, targetRelYawRad, targetDistM, hasFireTarget, headingRateRadS) {{
    const norm = new Float32Array(7);

    // Ultrasonic distances [0, 3m] -> [0, 1]
    norm[0] = Math.max(0, Math.min(1.0, distFrontM / this.maxUltrasonicM));
    norm[1] = Math.max(0, Math.min(1.0, distLeftM / this.maxUltrasonicM));
    norm[2] = Math.max(0, Math.min(1.0, distRightM / this.maxUltrasonicM));

    // Relative yaw [-PI, +PI] -> [-1, 1]
    let yaw = targetRelYawRad || 0.0;
    while (yaw > Math.PI) yaw -= 2 * Math.PI;
    while (yaw < -Math.PI) yaw += 2 * Math.PI;
    norm[3] = Math.max(-1.0, Math.min(1.0, yaw / Math.PI));
    norm[4] = Math.max(0, Math.min(1.0, (targetDistM !== undefined ? targetDistM : 5.0) / this.maxFireDistM));
    norm[5] = hasFireTarget ? 1.0 : 0.0;

    // Heading rate [-3, +3] -> [-1, 1]
    norm[6] = Math.max(-1.0, Math.min(1.0, (headingRateRadS || 0.0) / this.maxYawRate));

    let [leftPwm, rightPwm] = this.predict(norm);

    // Obstacle overtaking & stall-prevention guard:
    // If path in front is blocked and policy predicts low/stopped PWM without a fire target,
    // actively steer into the clearer side to overtake the obstacle!
    if (!hasFireTarget && distFrontM < 0.65 && Math.abs(leftPwm) < 50 && Math.abs(rightPwm) < 50) {{
      const turnRight = (Math.abs(targetRelYawRad || 0) > 0.05) ? (targetRelYawRad > 0.05) : (distRightM >= distLeftM);
      const inner = (distFrontM < 0.40) ? -20 : 40;
      leftPwm = turnRight ? 150 : inner;
      rightPwm = turnRight ? inner : 150;
    }}

    return {{ leftPwm, rightPwm }};
  }}
}}

// Expose globally for browser environment
if (typeof window !== 'undefined') {{
  window.NavModelRunner = NavModelRunner;
}}
"""
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(js_code)
    print(f"[EXPORT] Saved JavaScript runner: {output_path}")


if __name__ == "__main__":
    trained_model = train_model()
    export_cpp_header(trained_model)
    # Also sync to esp32_firmware/include
    import shutil
    os.makedirs("esp32_firmware/include", exist_ok=True)
    shutil.copyfile("bantay_sunog_esp32/navigation_model.h", "esp32_firmware/include/navigation_model.h")
    print("[SYNC] Copied C++ header to esp32_firmware/include/navigation_model.h")
    export_torchscript_and_json(trained_model)
    export_js_runner(trained_model)
    print("[ALL DONE] Model trained and exported across all target platforms!")
