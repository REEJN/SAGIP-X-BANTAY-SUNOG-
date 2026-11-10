"""
BANTAY-SUNOG Digital Twin Sync Server
Unified Bridge Server:
- Serves 3D Dashboard HTTP frontend on port 8000
- Manages real-time bidirectional WebSockets on port 8765
- Interfaces with physical ESP32 microcontroller or mock simulator
"""

import argparse
import asyncio
import http.server
import json
import logging
import os
import sys
import threading
from pathlib import Path
from typing import Optional, Set

import websockets

# Add project root to sys.path
PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import base64
import time
import numpy as np
import cv2

from simulation.bridge.serial_connector import SerialConnector
from bantay_sunog.detection.model_loader import load_model
from bantay_sunog.detection.fire_detector import FireDetector
from bantay_sunog.detection.confirmation import ConfirmationTracker

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] [%(levelname)s] [%(name)s] %(message)s",
    datefmt="%H:%M:%S"
)
logger = logging.getLogger("SyncServer")

FRONTEND_DIR = Path(__file__).resolve().parent.parent / "frontend"

# Initialize BANTAY-SUNOG Vision AI Model
try:
    model_backend = os.environ.get("BANTAY_DETECTOR", "hsv_fallback")
    vision_model_info = load_model(model_backend)
    fire_detector = FireDetector(vision_model_info)
    confirmation_tracker = ConfirmationTracker()
    VISION_AI_ENABLED = True
    logger.info(f"[Vision AI] Loaded {model_backend} detector and ConfirmationTracker.")
except Exception as e:
    VISION_AI_ENABLED = False
    fire_detector = None
    confirmation_tracker = None
    logger.warning(f"[Vision AI] Failed to initialize detector: {e}")


class StaticHttpHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(FRONTEND_DIR), **kwargs)

    def do_GET(self):
        # REST API Endpoints
        if self.path == "/api/status":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            status_data = {
                "server": "BANTAY-SUNOG Sync Server",
                "status": "online",
                "connected_clients": len(SyncBridge.connected_clients),
                "serial_port": SyncBridge.serial_conn.port if SyncBridge.serial_conn else None,
                "serial_connected": SyncBridge.serial_conn.ser.is_open if (SyncBridge.serial_conn and SyncBridge.serial_conn.ser) else False,
                "mock_mode": SyncBridge.serial_conn.mock_mode if SyncBridge.serial_conn else True
            }
            self.wfile.write(json.dumps(status_data).encode("utf-8"))
            return

        elif self.path == "/api/ports":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            ports = SerialConnector.list_ports()
            self.wfile.write(json.dumps({"ports": ports}).encode("utf-8"))
            return

        super().do_GET()

    def log_message(self, format, *args):
        # Quieten normal asset requests
        pass


class SyncBridge:
    connected_clients: Set = set()
    serial_conn: Optional[SerialConnector] = None
    loop: Optional[asyncio.AbstractEventLoop] = None

    @classmethod
    def set_loop(cls, loop: asyncio.AbstractEventLoop):
        cls.loop = loop

    @classmethod
    def on_serial_telemetry(cls, telemetry: dict):
        """Called when ESP32 transmits a telemetry line."""
        if cls.loop and cls.connected_clients:
            msg = json.dumps(telemetry)
            asyncio.run_coroutine_threadsafe(cls.broadcast_ws(msg), cls.loop)

    @classmethod
    def on_serial_log(cls, log_entry: dict):
        """Called when ESP32 emits a log line."""
        if cls.loop and cls.connected_clients:
            msg = json.dumps({"type": "ESP32_LOG", "data": log_entry})
            asyncio.run_coroutine_threadsafe(cls.broadcast_ws(msg), cls.loop)

    @classmethod
    def on_hardware_status_packet(cls, status_data: dict):
        """Called when Wi-Fi bridge emits a hardware status packet."""
        if cls.loop and cls.connected_clients:
            msg = json.dumps(status_data)
            asyncio.run_coroutine_threadsafe(cls.broadcast_ws(msg), cls.loop)

    @classmethod
    def on_pi_log_packet(cls, log_entry: dict):
        """Called when Wi-Fi bridge emits a Raspberry Pi system log."""
        if cls.loop and cls.connected_clients:
            msg = json.dumps({"type": "PI_LOG", "data": log_entry})
            asyncio.run_coroutine_threadsafe(cls.broadcast_ws(msg), cls.loop)

    @classmethod
    def emit_pi_log(cls, message: str, level: str = "INFO", source: str = "VISION"):
        """Emit a Raspberry Pi 3B log message to all connected dashboards."""
        log_entry = {
            "source": source,
            "level": level,
            "message": message,
            "timestamp": time.time()
        }
        if cls.loop and cls.connected_clients:
            msg = json.dumps({"type": "PI_LOG", "data": log_entry})
            asyncio.run_coroutine_threadsafe(cls.broadcast_ws(msg), cls.loop)

    @classmethod
    async def broadcast_ws(cls, message: str):
        if not cls.connected_clients:
            return
        dead = []
        for ws in list(cls.connected_clients):
            try:
                await ws.send(message)
            except Exception:
                dead.append(ws)
        for d in dead:
            cls.connected_clients.discard(d)

    @classmethod
    def get_real_metrics(cls) -> dict:
        """Collects real hardware metrics from the host (Pi) without hardcoded numbers."""
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

    @classmethod
    def get_hardware_status_dict(cls) -> dict:
        is_phys = cls.serial_conn.is_physical_connected() if cls.serial_conn else False
        ports = SerialConnector.list_ports()
        cached = getattr(cls.serial_conn, 'cached_pi_metrics', None)
        has_real_pi = bool(cached and cached.get("cpu") != "--")
        return {
            "type": "HARDWARE_STATUS",
            "connected": is_phys,
            "port": cls.serial_conn.port if (cls.serial_conn and is_phys) else None,
            "baudrate": cls.serial_conn.baudrate if cls.serial_conn else 115200,
            "available_ports": [p for p in ports if not p.startswith("MOCK")] + (["Raspberry Pi 3B (/dev/serial0)"] if is_phys else []),
            "pi_metrics": cached if has_real_pi else cls.get_real_metrics()
        }

    @classmethod
    async def broadcast_hardware_status(cls):
        status_pkt = json.dumps(cls.get_hardware_status_dict())
        await cls.broadcast_ws(status_pkt)


async def periodic_hardware_status():
    """Periodically probes for physical ESP32 connection and broadcasts status."""
    while True:
        await asyncio.sleep(2.0)
        if SyncBridge.serial_conn and not SyncBridge.serial_conn.is_physical_connected():
            SyncBridge.serial_conn.auto_scan_and_connect()
        await SyncBridge.broadcast_hardware_status()


async def handle_ws(websocket):
    SyncBridge.connected_clients.add(websocket)
    client_ip = websocket.remote_address[0] if websocket.remote_address else "local"
    logger.info(f"[WS] Client connected from {client_ip}. Active clients: {len(SyncBridge.connected_clients)}")

    # Send initial hardware connection status immediately
    await websocket.send(json.dumps(SyncBridge.get_hardware_status_dict()))

    # Emit real network connection event to Pi console
    SyncBridge.emit_pi_log(f"[Wi-Fi] Laptop Dashboard connected from {client_ip}", "INFO", "NETWORK")

    try:
        async for message in websocket:
            try:
                data = json.loads(message)
                msg_type = data.get("type", "")
                sync_mode = data.get("sync_mode", "")

                # -------------------------------------------------------------
                # Hardware Port Scan / Connect Commands
                # -------------------------------------------------------------
                if msg_type == "SCAN_PORTS":
                    if SyncBridge.serial_conn:
                        SyncBridge.serial_conn.auto_scan_and_connect()
                    await SyncBridge.broadcast_hardware_status()
                    continue

                if msg_type == "CONNECT_PORT":
                    target_port = data.get("port")
                    if target_port and SyncBridge.serial_conn:
                        SyncBridge.serial_conn.connect_physical(target_port)
                    await SyncBridge.broadcast_hardware_status()
                    continue

                # -------------------------------------------------------------
                # Bench Test Command Dispatch
                # -------------------------------------------------------------
                if msg_type == "BENCH_COMMAND" and SyncBridge.serial_conn:
                    cmd = data.get("cmd", "")
                    SyncBridge.serial_conn.send_line(cmd)
                    SyncBridge.emit_pi_log(f"[Bench Test] Dispatched user command '{cmd}' to ESP32", "INFO", "UART")
                    continue

                # -------------------------------------------------------------
                # Process Virtual Camera Frame with BANTAY-SUNOG Detection Model
                # -------------------------------------------------------------
                if msg_type == "FRAME_INFERENCE_REQUEST" and VISION_AI_ENABLED:
                    img_data = data.get("image", "")
                    if "," in img_data:
                        img_data = img_data.split(",", 1)[1]

                    t0 = time.perf_counter()
                    try:
                        raw_bytes = base64.b64decode(img_data)
                        np_arr = np.frombuffer(raw_bytes, np.uint8)
                        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                        if frame is not None:
                            # 1. Run real detection algorithm (HSV or TFLite)
                            detections = fire_detector.detect(frame)

                            # 2. Update temporal confirmation state machine
                            state, event, best_det = confirmation_tracker.update(detections)
                            infer_ms = (time.perf_counter() - t0) * 1000.0

                            # 3. Format response to 3D simulator
                            resp = {
                                "type": "INFERENCE_RESULT",
                                "state": state,
                                "event": event,
                                "inference_ms": round(infer_ms, 1),
                                "backend": fire_detector._backend,
                                "detections": [
                                    {
                                        "class_name": d.class_name,
                                        "confidence": round(float(d.confidence), 2),
                                        "x": int(d.x),
                                        "y": int(d.y),
                                        "w": int(d.width),
                                        "h": int(d.height),
                                        "norm_x": round(float(d.normalized_x), 3),
                                        "norm_y": round(float(d.normalized_y), 3),
                                    } for d in detections
                                ],
                                "best_target": {
                                    "x": round(float(best_det.normalized_x), 3),
                                    "y": round(float(best_det.normalized_y), 3),
                                    "confidence": round(float(best_det.confidence), 2),
                                    "bbox": [int(best_det.x), int(best_det.y), int(best_det.width), int(best_det.height)],
                                    "frame_dims": [frame.shape[1], frame.shape[0]],
                                } if best_det else None
                            }
                            await websocket.send(json.dumps(resp))

                            if best_det and event == "FIRE_CONFIRMED":
                                SyncBridge.emit_pi_log(
                                    f"[Vision AI] FIRE CONFIRMED (Conf: {best_det.confidence:.2f}, Target: [{best_det.normalized_x:.2f}, {best_det.normalized_y:.2f}])",
                                    level="WARN",
                                    source="VISION"
                                )
                    except Exception as e:
                        logger.error(f"[Vision AI Error] {e}")

                # Handle Sim -> Hardware command dispatch
                if sync_mode in ("sim2hw", "teleop") and SyncBridge.serial_conn:
                    # Translate simulated movement or spray to physical ESP32
                    left_pwm = data.get("left_pwm", 0)
                    right_pwm = data.get("right_pwm", 0)
                    solenoid = data.get("solenoid_active", False)
                    tilt = data.get("tilt_angle", 90)

                    # Send structured serial command or test tokens to ESP32
                    if solenoid:
                        SyncBridge.serial_conn.send_line("f") # Pulse solenoid
                    elif left_pwm > 80 and right_pwm > 80:
                        SyncBridge.serial_conn.send_line("w") # Forward
                    elif left_pwm < -80 and right_pwm < -80:
                        SyncBridge.serial_conn.send_line("s") # Reverse
                    elif left_pwm < -80 and right_pwm > 80:
                        SyncBridge.serial_conn.send_line("a") # Pivot Left
                    elif left_pwm > 80 and right_pwm < -80:
                        SyncBridge.serial_conn.send_line("d") # Pivot Right
                    else:
                        SyncBridge.serial_conn.send_line("x") # Stop

                    # Forward vision detections if flame is spotted in virtual camera
                    target = data.get("target")
                    if target:
                        vision_pkt = {
                            "event": "FIRE_CONFIRMED",
                            "confidence": target.get("confidence", 0.9),
                            "x": target.get("x", 0.5),
                            "y": target.get("y", 0.5),
                            "bbox": target.get("bbox", [180, 90, 120, 160]),
                            "frame": [640, 480],
                            "timestamp": int(asyncio.get_event_loop().time())
                        }
                        SyncBridge.serial_conn.send_json(vision_pkt)
                        SyncBridge.emit_pi_log(
                            f"[UART TX] Dispatched FIRE_CONFIRMED packet to ESP32: x={vision_pkt['x']} y={vision_pkt['y']} conf={vision_pkt['confidence']}",
                            level="INFO",
                            source="UART"
                        )

            except json.JSONDecodeError:
                pass
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        SyncBridge.connected_clients.discard(websocket)
        logger.info(f"[WS] Client disconnected. Active clients: {len(SyncBridge.connected_clients)}")


def start_http_server(host: str, port: int):
    server = http.server.ThreadingHTTPServer((host, port), StaticHttpHandler)
    logger.info(f"[HTTP] Dashboard live at http://{host}:{port}/")
    server.serve_forever()


async def main_async(args):
    loop = asyncio.get_running_loop()
    SyncBridge.set_loop(loop)

    # Initialize Serial Connector
    SyncBridge.serial_conn = SerialConnector(
        port=args.serial_port,
        baudrate=args.baud,
        mock_mode=args.mock
    )
    SyncBridge.serial_conn.on_telemetry_received = SyncBridge.on_serial_telemetry
    SyncBridge.serial_conn.on_log_received = SyncBridge.on_serial_log
    SyncBridge.serial_conn.on_hardware_status_received = SyncBridge.on_hardware_status_packet
    SyncBridge.serial_conn.on_pi_log_received = SyncBridge.on_pi_log_packet

    # Connect to serial, Wi-Fi bridge, or mock
    SyncBridge.serial_conn.connect(args.serial_port)

    # Start HTTP Static Server in a background thread
    http_thread = threading.Thread(
        target=start_http_server,
        args=(args.host, args.http_port),
        daemon=True
    )
    http_thread.start()

    # Start periodic hardware status task
    asyncio.create_task(periodic_hardware_status())

    # Start WebSocket Server
    logger.info(f"[WS] WebSocket server listening on ws://{args.host}:{args.ws_port}/ws/telemetry")
    async with websockets.serve(handle_ws, args.host, args.ws_port):
        await asyncio.Future() # Run forever


def main():
    parser = argparse.ArgumentParser(description="BANTAY-SUNOG 3D Digital Twin Sync Server")
    parser.add_argument("--host", default="0.0.0.0", help="Host interface (default: 0.0.0.0)")
    parser.add_argument("--http-port", type=int, default=8000, help="HTTP Dashboard port (default: 8000)")
    parser.add_argument("--ws-port", type=int, default=8765, help="WebSocket port (default: 8765)")
    parser.add_argument("--serial-port", default=None, help="ESP32 COM/serial port (e.g. COM3 or /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baudrate (default: 115200)")
    parser.add_argument("--mock", action="store_true", default=True, help="Run in mock loopback mode if no ESP32")
    args = parser.parse_args()

    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        logger.info("[Server] Shutting down.")
        if SyncBridge.serial_conn:
            SyncBridge.serial_conn.disconnect()


if __name__ == "__main__":
    main()
