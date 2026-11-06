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


async def handle_ws(websocket):
    SyncBridge.connected_clients.add(websocket)
    logger.info(f"[WS] Client connected. Active clients: {len(SyncBridge.connected_clients)}")

    try:
        async for message in websocket:
            try:
                data = json.loads(message)
                # -------------------------------------------------------------
                # Process Virtual Camera Frame with BANTAY-SUNOG Detection Model
                # -------------------------------------------------------------
                if data.get("type") == "FRAME_INFERENCE_REQUEST" and VISION_AI_ENABLED:
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

    # Connect to serial or mock
    SyncBridge.serial_conn.connect(args.serial_port)

    # Start HTTP Static Server in a background thread
    http_thread = threading.Thread(
        target=start_http_server,
        args=(args.host, args.http_port),
        daemon=True
    )
    http_thread.start()

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
