# BANTAY-SUNOG: Fire Detection Models Guide

This directory holds the lightweight machine learning models for real-time fire detection on the Raspberry Pi 3 Model B (BCM2837, 1 GB RAM).

---

## 1. Supported Architecture: Quantized SSD MobileNet V2

For the Raspberry Pi 3B CPU, large neural networks (such as YOLOv8 640x640) will achieve less than 0.5 FPS and can easily exhaust the 1 GB shared RAM.

We strongly recommend an **INT8 Post-Training Quantized SSD MobileNet V2** (or SSDLite MobileNet V2):
- **Model format:** TensorFlow Lite (`.tflite`)
- **Input resolution:** `300x300` pixels (or `224x224` for ultra-light models)
- **Quantization:** Full INT8 quantization using representative dataset calibration
- **Inference engine:** ARM NEON-accelerated XNNPACK delegate via `tflite-runtime` or `ai-edge-litert`
- **Expected CPU Latency on Pi 3B (4 threads):** ~150–350 ms per frame (approx. 3–6 FPS)
- **RAM Footprint:** ~30–50 MB

---

## 2. Expected Files in this Directory

Place your trained and quantized files here:

```
models/
├── fire_detect.tflite      # The INT8 quantized model file
├── labels.txt              # Class labels (one per line)
└── README.md
```

### Example `labels.txt`:
```
fire
```

---

## 3. Training & Conversion Workflow (Cloud / PC)

Do not attempt to train models on the Raspberry Pi 3B. Train on Google Colab or a GPU workstation:

### Step 1: Dataset
Collect or use public flame/fire datasets with bounding box annotations:
- BowFire Dataset
- D-Fire Dataset (Fire and Smoke)
- Roboflow Universe Fire Datasets

### Step 2: Training
Train an SSD MobileNet V2 object detector using TensorFlow 2 Object Detection API or PyTorch (exporting via ONNX to TFLite).

### Step 3: Full INT8 Quantization
Convert the exported `saved_model` to INT8 TFLite using a representative dataset:

```python
import tensorflow as tf

converter = tf.lite.TFLiteConverter.from_saved_model("saved_model_dir")
converter.optimizations = [tf.lite.Optimize.DEFAULT]

def representative_dataset():
    for image in calibration_images_generator():
        yield [image]

converter.representative_dataset = representative_dataset
converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
converter.inference_input_type = tf.int8
converter.inference_output_type = tf.int8

tflite_quant_model = converter.convert()
with open("models/fire_detect.tflite", "wb") as f:
    f.write(tflite_quant_model)
```

---

## 4. Fallback Detector (No Model File Required)

If no `.tflite` file is present in `models/`, BANTAY-SUNOG automatically falls back to:
- **`hsv_fallback`**: Dual-band OpenCV HSV colour-space thresholding combined with MOG2 background subtraction motion/flicker filtering.
  - *Status:* Experimental fallback.
  - *Latency:* ~30–60 ms (~15 FPS).
  - *Accuracy disclaimer:* Susceptible to false alarms on orange/red reflective surfaces, halogen lights, and sunlight.
