"""
BANTAY-SUNOG Raspberry Pi 3B Telemetry, Camera & UART Bridge Service
=====================================================================
Runs on Raspberry Pi 3B (192.168.137.200).
- Opens hardware UART /dev/serial0 (connected to ESP32 RX/TX pins) at 115200 baud
- Gathers real Pi system metrics (CPU, thermal sensor temp, RAM)
- Serves a lightweight TCP stream server on port 8888 for the laptop dashboard
- Acquires real-time frames from physical USB webcam (/dev/video0) at 640x360
- Runs Vision AI / HSV flame detector with bounding box targeting
- Serves high-speed MJPEG video stream on port 5000 (/video_feed)
- Enables bidirectional communication: ESP32 <-> Pi <-> Wi-Fi <-> Laptop Dashboard
"""

import asyncio
import json
import logging
import os
import sys
import threading
import time
from typing import Optional, Set

# Suppress flask / werkzeug verbose request logs
logging.getLogger("werkzeug").setLevel(logging.ERROR)

# Ensure parent path is in sys.path
_CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
_PARENT_DIR = os.path.dirname(_CURRENT_DIR)
for p in [_CURRENT_DIR, _PARENT_DIR]:
    if p not in sys.path:
        sys.path.insert(0, p)

try:
    import cv2
except ImportError:
    cv2 = None

try:
    from flask import Flask, Response, jsonify
    _HAS_FLASK = True
except ImportError:
    _HAS_FLASK = False

try:
    import serial
except ImportError:
    serial = None

# Optional fire detector import
fire_detector = None
confirmation_tracker = None
try:
    from bantay_sunog.detection.model_loader import load_model
    from bantay_sunog.detection.fire_detector import FireDetector
    from bantay_sunog.detection.confirmation import ConfirmationTracker
    model_info = load_model("hsv_fallback")
    fire_detector = FireDetector(model_info)
    confirmation_tracker = ConfirmationTracker()
    print("[Vision AI] Loaded HSV fallback fire detector.")
except Exception as e:
    print(f"[Vision AI] Running without detector model: {e}")

UART_PORT = os.environ.get("BANTAY_UART_PORT", "/dev/serial0")
UART_BAUD = int(os.environ.get("BANTAY_UART_BAUD", "115200"))
TCP_PORT = int(os.environ.get("BANTAY_TCP_PORT", "8888"))
VIDEO_PORT = int(os.environ.get("BANTAY_VIDEO_PORT", "5000"))

connected_clients: Set[asyncio.StreamWriter] = set()
ser_conn = None
last_esp_packet_time = 0.0

# -----------------------------------------------------------------------------
# Video Streaming & Camera State (10 FPS Optimized Pacing)
# -----------------------------------------------------------------------------
CAMERA_FPS = int(os.environ.get("BANTAY_CAMERA_FPS", "10"))
FRAME_INTERVAL = 1.0 / max(1, CAMERA_FPS)

latest_jpeg_frame: Optional[bytes] = None
frame_id: int = 0
frame_cond = threading.Condition()

camera_status = {
    "connected": False,
    "fps": 0.0,
    "target_fps": CAMERA_FPS,
    "device": "/dev/video0",
    "resolution": "640x360",
    "format": "MJPG",
    "flame_detected": False,
    "target": None,
    "backend": "HSV_Fallback"
}


def get_pi_metrics():
    metrics = {"cpu": "--", "temp": "--", "ram": "--"}
    try:
        import psutil
        metrics["cpu"] = f"{psutil.cpu_percent(interval=None):.1f}%"
        mem = psutil.virtual_memory()
        metrics["ram"] = f"{int(mem.used / 1048576)}MB / {int(mem.total / 1048576)}MB"
    except Exception:
        pass

    thermal_path = "/sys/class/thermal/thermal_zone0/temp"
    if os.path.exists(thermal_path):
        try:
            with open(thermal_path, "r") as f:
                c = int(f.read().strip()) / 1000.0
                metrics["temp"] = f"{c:.1f}°C"
        except Exception:
            pass
    return metrics


async def broadcast_to_clients(data_str: str):
    if not connected_clients:
        return
    payload = (data_str.strip() + "\n").encode("utf-8")
    dead = []
    for writer in list(connected_clients):
        try:
            writer.write(payload)
            await writer.drain()
        except Exception:
            dead.append(writer)
    for d in dead:
        connected_clients.discard(d)


def camera_capture_worker():
    """Continuously captures frames from USB webcam at stable 10 FPS with zero glitching or tearing."""
    global latest_jpeg_frame, frame_id, camera_status
    if cv2 is None:
        print("[CAM] OpenCV not installed, camera capture disabled.")
        return

    print(f"[CAM] Starting background USB webcam acquisition worker (Target: {CAMERA_FPS} FPS)...")
    cap = None
    frame_count = 0
    fps_start = time.time()

    while True:
        try:
            loop_start = time.time()

            # 1. Open camera if not opened
            if cap is None or not cap.isOpened():
                camera_status["connected"] = False
                for dev in ["/dev/video0", 0, "/dev/video1", 1]:
                    # Prefer MJPG for hardware compression (reduces USB bus traffic by 95% & prevents tearing)
                    for fmt in ["MJPG", "YUYV"]:
                        try:
                            c = cv2.VideoCapture(dev, cv2.CAP_V4L2)
                            if c.isOpened():
                                c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fmt))
                                c.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                                c.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
                                c.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
                                c.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                                test_ret, _ = c.read()
                                if test_ret:
                                    cap = c
                                    camera_status["connected"] = True
                                    camera_status["device"] = str(dev)
                                    camera_status["resolution"] = "640x360"
                                    camera_status["format"] = fmt
                                    print(f"[CAM] Successfully opened USB webcam on {dev} at 640x360 ({fmt} @ {CAMERA_FPS} FPS)")
                                    break
                                else:
                                    c.release()
                        except Exception:
                            continue
                    if cap and cap.isOpened():
                        break

                if cap is None or not cap.isOpened():
                    time.sleep(2.0)
                    continue

            # 2. Read frame
            ret, frame = cap.read()
            if not ret or frame is None:
                camera_status["connected"] = False
                try:
                    cap.release()
                except Exception:
                    pass
                cap = None
                time.sleep(1.0)
                continue

            # 3. FPS calculation
            frame_count += 1
            elapsed = time.time() - fps_start
            if elapsed >= 1.0:
                camera_status["fps"] = round(frame_count / elapsed, 1)
                frame_count = 0
                fps_start = time.time()

            # 4. Fire Detection (if model loaded)
            best_target = None
            if fire_detector is not None:
                try:
                    detections = fire_detector.detect(frame)
                    if confirmation_tracker is not None:
                        state, _, best_det = confirmation_tracker.update(detections)
                        if best_det is not None:
                            best_target = {
                                "x": round(best_det.normalized_x, 3),
                                "y": round(best_det.normalized_y, 3),
                                "confidence": round(best_det.confidence, 2),
                                "bbox": [best_det.x, best_det.y, best_det.width, best_det.height]
                            }
                            camera_status["flame_detected"] = True
                            camera_status["target"] = best_target
                        else:
                            camera_status["flame_detected"] = False
                            camera_status["target"] = None
                except Exception:
                    pass

            # 5. Draw detection overlay onto live frame if flame detected
            if best_target and "bbox" in best_target:
                bx, by, bw, bh = best_target["bbox"]
                cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), (0, 0, 255), 2)
                tag = f"FLAME {int(best_target['confidence'] * 100)}%"
                cv2.putText(frame, tag, (bx, max(18, by - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                cx = bx + bw // 2
                cy = by + bh // 2
                cv2.drawMarker(frame, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 16, 1)

            # 6. Compress to JPEG with optimized quality 70 (crystal clear, low bandwidth)
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]
            success, buffer = cv2.imencode(".jpg", frame, encode_param)
            if success:
                with frame_cond:
                    latest_jpeg_frame = buffer.tobytes()
                    frame_id += 1
                    frame_cond.notify_all()

            # 7. Sleep to lock FPS cleanly at target rate (drastically reduces CPU load & tearing)
            work_time = time.time() - loop_start
            sleep_time = max(0.01, FRAME_INTERVAL - work_time)
            time.sleep(sleep_time)

        except Exception as e:
            time.sleep(1.0)


def create_flask_app():
    """Create Flask application for MJPEG video streaming."""
    app = Flask("bantay_video_stream")

    @app.route("/")
    def index():
        return f"BANTAY-SUNOG Video Streamer Online at /video_feed ({CAMERA_FPS} FPS)"

    @app.route("/video_feed")
    def video_feed():
        def generate():
            last_id = -1
            while True:
                with frame_cond:
                    if frame_id == last_id:
                        frame_cond.wait(timeout=0.25)
                    jpeg = latest_jpeg_frame
                    curr_id = frame_id

                if jpeg is not None and curr_id != last_id:
                    last_id = curr_id
                    yield (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"
                    )
                else:
                    time.sleep(0.02)

        response = Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")
        response.headers["Access-Control-Allow-Origin"] = "*"
        response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
        response.headers["Pragma"] = "no-cache"
        response.headers["Expires"] = "0"
        return response

    @app.route("/snapshot")
    def snapshot():
        with frame_cond:
            jpeg = latest_jpeg_frame
        if jpeg is not None:
            response = Response(jpeg, mimetype="image/jpeg")
            response.headers["Access-Control-Allow-Origin"] = "*"
            return response
        return ("No frame captured yet", 404)

    @app.route("/api/status")
    def api_status():
        data = {
            "camera": camera_status,
            "metrics": get_pi_metrics()
        }
        response = jsonify(data)
        response.headers["Access-Control-Allow-Origin"] = "*"
        return response

    return app


async def uart_reader_task():
    global ser_conn, last_esp_packet_time
    print(f"[UART] Opening {UART_PORT} at {UART_BAUD} baud...")
    while True:
        try:
            if ser_conn is None or not ser_conn.is_open:
                ser_conn = serial.Serial(UART_PORT, UART_BAUD, timeout=1.0)
                ser_conn.reset_input_buffer()
                print(f"[UART] Connected to physical ESP32 on {UART_PORT}!")
                log_pkt = json.dumps({
                    "type": "PI_LOG",
                    "data": {
                        "source": "UART",
                        "level": "INFO",
                        "message": f"Connected to ESP32 on {UART_PORT} at {UART_BAUD} baud (Pins 8/10 TX/RX)",
                        "timestamp": time.time()
                    }
                })
                await broadcast_to_clients(log_pkt)

            loop = asyncio.get_running_loop()
            line = await loop.run_in_executor(None, ser_conn.readline)
            if line:
                decoded = line.decode("utf-8", errors="replace").strip()
                if decoded:
                    last_esp_packet_time = time.time()
                    if decoded.startswith("{") and decoded.endswith("}"):
                        await broadcast_to_clients(decoded)
                    else:
                        # Forward as ESP32 serial console log
                        esp_log = json.dumps({
                            "type": "ESP32_LOG",
                            "data": {
                                "source": "ESP32",
                                "level": "INFO",
                                "message": decoded,
                                "timestamp": time.time(),
                                "is_physical": True
                            }
                        })
                        await broadcast_to_clients(esp_log)
        except Exception as e:
            print(f"[UART] Error on {UART_PORT}: {e}")
            if ser_conn:
                try:
                    ser_conn.close()
                except Exception:
                    pass
                ser_conn = None
            await asyncio.sleep(2.0)


async def periodic_status_task():
    global ser_conn, last_esp_packet_time
    while True:
        await asyncio.sleep(1.0)
        now = time.time()
        esp_alive = bool(ser_conn and ser_conn.is_open and (now - last_esp_packet_time < 3.0))
        metrics = get_pi_metrics()
        status_pkt = json.dumps({
            "type": "HARDWARE_STATUS",
            "connected": esp_alive,
            "port": f"Raspberry Pi 3B (192.168.137.200 -> {UART_PORT})",
            "baudrate": UART_BAUD,
            "available_ports": [UART_PORT],
            "pi_metrics": metrics,
            "camera": camera_status,
            "source": "PI_WIFI_BRIDGE",
            "esp_alive": esp_alive,
            "last_esp_packet_age": round(now - last_esp_packet_time, 2) if last_esp_packet_time > 0 else 999.0
        })
        await broadcast_to_clients(status_pkt)


async def handle_client(reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
    peer = writer.get_extra_info("peername")
    print(f"[TCP] Dashboard client connected from {peer}")
    connected_clients.add(writer)

    welcome_log = json.dumps({
        "type": "PI_LOG",
        "data": {
            "source": "NETWORK",
            "level": "INFO",
            "message": f"Laptop Dashboard connected over Wi-Fi from {peer}. Streaming live ESP32 telemetry from {UART_PORT}.",
            "timestamp": time.time()
        }
    })
    writer.write((welcome_log + "\n").encode("utf-8"))
    await writer.drain()

    metrics = get_pi_metrics()
    esp_alive = bool(ser_conn and ser_conn.is_open and (time.time() - last_esp_packet_time < 3.0))
    status_pkt = json.dumps({
        "type": "HARDWARE_STATUS",
        "connected": esp_alive,
        "port": f"Raspberry Pi 3B (192.168.137.200 -> {UART_PORT})",
        "baudrate": UART_BAUD,
        "available_ports": [UART_PORT],
        "pi_metrics": metrics,
        "camera": camera_status,
        "source": "PI_WIFI_BRIDGE"
    })
    writer.write((status_pkt + "\n").encode("utf-8"))
    await writer.drain()

    try:
        while True:
            line = await reader.readline()
            if not line:
                break
            cmd_str = line.decode("utf-8", errors="replace").strip()
            if cmd_str and ser_conn and ser_conn.is_open:
                try:
                    ser_conn.write((cmd_str + "\n").encode("utf-8"))
                    ser_conn.flush()
                except Exception as write_err:
                    print(f"[UART TX ERR] {write_err}")
    except Exception as e:
        print(f"[TCP] Client {peer} connection error: {e}")
    finally:
        print(f"[TCP] Client {peer} disconnected.")
        connected_clients.discard(writer)
        try:
            writer.close()
            await writer.wait_closed()
        except Exception:
            pass


async def main():
    # 1. Start background camera acquisition worker
    cam_thread = threading.Thread(target=camera_capture_worker, daemon=True, name="CameraCapture")
    cam_thread.start()

    # 2. Start background Flask video streaming server on port 5000
    if _HAS_FLASK:
        flask_app = create_flask_app()
        flask_thread = threading.Thread(
            target=lambda: flask_app.run(host="0.0.0.0", port=VIDEO_PORT, threaded=True, use_reloader=False),
            daemon=True,
            name="VideoStreamer"
        )
        flask_thread.start()
        print(f"[HTTP] Video streaming server active on http://0.0.0.0:{VIDEO_PORT}/video_feed")

    # 3. Start TCP bridge on port 8888
    server = await asyncio.start_server(handle_client, "0.0.0.0", TCP_PORT)
    print(f"============================================================")
    print(f"   BANTAY-SUNOG Raspberry Pi 3B Telemetry & Video Bridge")
    print(f"   Listening on TCP 0.0.0.0:{TCP_PORT}")
    print(f"   MJPEG Stream: http://0.0.0.0:{VIDEO_PORT}/video_feed")
    print(f"   Hardware UART: {UART_PORT} @ {UART_BAUD} baud")
    print(f"============================================================")

    async with server:
        await asyncio.gather(
            server.serve_forever(),
            uart_reader_task(),
            periodic_status_task()
        )


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nShutdown bridge.")
