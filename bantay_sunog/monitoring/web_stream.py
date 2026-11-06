"""
BANTAY-SUNOG Web Preview Streaming Server

Allows viewing the live camera feed with detection overlays, target crosshairs,
bounding boxes, and real-time telemetry from any laptop or browser on the local network.

Endpoint:
- http://<pi-ip>:5000/           -> Full Dark-Themed Live Dashboard
- http://<pi-ip>:5000/video_feed -> Direct MJPEG Video Stream
- http://<pi-ip>:5000/api/status -> JSON Telemetry Status
"""

import logging
import threading
import time
from typing import Any, Dict, Optional

import cv2
import numpy as np

# Suppress standard werkzeug request logging to keep console clean for UART TX
log = logging.getLogger("werkzeug")
log.setLevel(logging.ERROR)

try:
    from flask import Flask, Response, jsonify, render_template_string
    _HAS_FLASK = True
except ImportError:
    _HAS_FLASK = False

from bantay_sunog import config
from bantay_sunog.monitoring.logger import log_system


_DASHBOARD_HTML = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>BANTAY-SUNOG | Live Fire Detection Preview</title>
    <style>
        :root {
            --bg-primary: #0f141c;
            --bg-secondary: #1a2230;
            --bg-card: #222d3f;
            --accent-fire: #ff4747;
            --accent-warning: #ffaa00;
            --accent-ok: #00e676;
            --accent-info: #29b6f6;
            --text-main: #f0f4f8;
            --text-muted: #8b9bb4;
            --border: #2e3d55;
        }

        * {
            box-sizing: border-box;
            margin: 0;
            padding: 0;
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
        }

        body {
            background-color: var(--bg-primary);
            color: var(--text-main);
            padding: 20px;
        }

        header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding-bottom: 20px;
            border-bottom: 1px solid var(--border);
            margin-bottom: 25px;
        }

        .title-group h1 {
            font-size: 24px;
            font-weight: 700;
            letter-spacing: 0.5px;
            display: flex;
            align-items: center;
            gap: 10px;
        }

        .title-group h1 span {
            color: var(--accent-fire);
        }

        .subtitle {
            font-size: 13px;
            color: var(--text-muted);
            margin-top: 4px;
        }

        .badge-live {
            background-color: rgba(0, 230, 118, 0.15);
            color: var(--accent-ok);
            border: 1px solid var(--accent-ok);
            padding: 6px 14px;
            border-radius: 20px;
            font-size: 12px;
            font-weight: 600;
            display: flex;
            align-items: center;
            gap: 8px;
        }

        .badge-live::before {
            content: "";
            width: 8px;
            height: 8px;
            background-color: var(--accent-ok);
            border-radius: 50%;
            display: inline-block;
            animation: pulse 1.5s infinite;
        }

        @keyframes pulse {
            0% { opacity: 1; transform: scale(1); }
            50% { opacity: 0.4; transform: scale(0.85); }
            100% { opacity: 1; transform: scale(1); }
        }

        .grid {
            display: grid;
            grid-template-columns: 2fr 1fr;
            gap: 25px;
        }

        @media (max-width: 900px) {
            .grid {
                grid-template-columns: 1fr;
            }
        }

        .card {
            background-color: var(--bg-secondary);
            border: 1px solid var(--border);
            border-radius: 12px;
            overflow: hidden;
            box-shadow: 0 8px 24px rgba(0,0,0,0.3);
        }

        .card-header {
            padding: 14px 18px;
            background-color: var(--bg-card);
            border-bottom: 1px solid var(--border);
            font-size: 14px;
            font-weight: 600;
            color: var(--text-main);
            display: flex;
            justify-content: space-between;
            align-items: center;
        }

        .video-container {
            position: relative;
            background-color: #000;
            display: flex;
            justify-content: center;
            align-items: center;
            min-height: 420px;
        }

        .video-container img {
            width: 100%;
            height: auto;
            max-height: 600px;
            object-fit: contain;
            display: block;
        }

        .telemetry-panel {
            display: flex;
            flex-direction: column;
            gap: 16px;
            padding: 18px;
        }

        .state-banner {
            padding: 16px;
            border-radius: 8px;
            text-align: center;
            font-weight: 700;
            font-size: 16px;
            letter-spacing: 0.5px;
            transition: all 0.3s ease;
        }

        .state-IDLE, .state-NO_FIRE {
            background-color: rgba(0, 230, 118, 0.12);
            color: var(--accent-ok);
            border: 1px solid var(--accent-ok);
        }

        .state-FIRE_CANDIDATE {
            background-color: rgba(255, 170, 0, 0.15);
            color: var(--accent-warning);
            border: 1px solid var(--accent-warning);
        }

        .state-FIRE_CONFIRMED {
            background-color: rgba(255, 71, 71, 0.2);
            color: var(--accent-fire);
            border: 1px solid var(--accent-fire);
            animation: pulse-border 1s infinite alternate;
        }

        .state-FIRE_LOST, .state-CAMERA_ERROR, .state-SERIAL_ERROR {
            background-color: rgba(255, 71, 71, 0.15);
            color: #ff7676;
            border: 1px solid #ff7676;
        }

        @keyframes pulse-border {
            from { box-shadow: 0 0 10px rgba(255, 71, 71, 0.4); }
            to { box-shadow: 0 0 20px rgba(255, 71, 71, 0.8); }
        }

        .metric-row {
            display: flex;
            justify-content: space-between;
            padding: 10px 0;
            border-bottom: 1px solid var(--border);
            font-size: 13px;
        }

        .metric-row:last-child {
            border-bottom: none;
        }

        .metric-label {
            color: var(--text-muted);
        }

        .metric-value {
            font-weight: 600;
            color: var(--text-main);
            font-family: "Courier New", Courier, monospace;
        }

        .uart-box {
            background-color: #0b0f15;
            border: 1px solid var(--border);
            border-radius: 6px;
            padding: 12px;
            font-family: "Courier New", Courier, monospace;
            font-size: 12px;
            color: #79c0ff;
            max-height: 140px;
            overflow-y: auto;
            white-space: pre-wrap;
            word-break: break-all;
        }
    </style>
</head>
<body>
    <header>
        <div class="title-group">
            <h1><span>BANTAY-SUNOG</span> Live Video Preview</h1>
            <div class="subtitle">Raspberry Pi 3B Autonomous Firefighting Subsystem</div>
        </div>
        <div class="badge-live">STREAM ONLINE</div>
    </header>

    <div class="grid">
        <!-- Live Video Card -->
        <div class="card">
            <div class="card-header">
                <span>Real-Time Annotated Feed</span>
                <span id="fps-counter" style="color: var(--accent-info);">-- FPS</span>
            </div>
            <div class="video-container">
                <img src="/video_feed" alt="Live Stream">
            </div>
        </div>

        <!-- Telemetry & Controls Card -->
        <div class="card">
            <div class="card-header">
                <span>Subsystem Telemetry</span>
                <span id="serial-badge" style="color: var(--text-muted);">UART: --</span>
            </div>
            <div class="telemetry-panel">
                <div id="state-box" class="state-banner state-NO_FIRE">
                    NO_FIRE
                </div>

                <div class="metric-row">
                    <span class="metric-label">Target Center (X, Y):</span>
                    <span class="metric-value" id="coords-val">None</span>
                </div>
                <div class="metric-row">
                    <span class="metric-label">Confidence:</span>
                    <span class="metric-value" id="conf-val">0.0%</span>
                </div>
                <div class="metric-row">
                    <span class="metric-label">CPU Utilization:</span>
                    <span class="metric-value" id="cpu-val">--%</span>
                </div>
                <div class="metric-row">
                    <span class="metric-label">Memory Used:</span>
                    <span class="metric-value" id="ram-val">-- MB</span>
                </div>
                <div class="metric-row">
                    <span class="metric-label">SoC Temperature:</span>
                    <span class="metric-value" id="temp-val">--°C</span>
                </div>
                <div class="metric-row">
                    <span class="metric-label">UART Messages Sent:</span>
                    <span class="metric-value" id="uart-count">0</span>
                </div>

                <div style="margin-top: 10px;">
                    <span class="metric-label" style="font-size: 11px; text-transform: uppercase;">Latest Outgoing UART Packet:</span>
                    <div id="uart-preview" class="uart-box">No packets sent yet</div>
                </div>

                <!-- ESP32 Live Telemetry -->
                <div style="margin-top: 15px; padding-top: 15px; border-top: 1px solid var(--border);">
                    <div style="font-size: 13px; font-weight: 700; color: var(--accent-info); margin-bottom: 8px;">
                        ESP32 ROBOT CONTROLLER
                    </div>
                    <div class="metric-row">
                        <span class="metric-label">Robot State:</span>
                        <span class="metric-value" id="esp32-state" style="color: var(--accent-warning);">OFFLINE</span>
                    </div>
                    <div class="metric-row">
                        <span class="metric-label">Front Distance:</span>
                        <span class="metric-value" id="esp32-dist-front">-- cm</span>
                    </div>
                    <div class="metric-row">
                        <span class="metric-label">Left / Right Dist:</span>
                        <span class="metric-value" id="esp32-dist-sides">-- / -- cm</span>
                    </div>
                    <div class="metric-row">
                        <span class="metric-label">Hardware Flame Sensor:</span>
                        <span class="metric-value" id="esp32-flame">--</span>
                    </div>
                    <div class="metric-row">
                        <span class="metric-label">Tilt Angle:</span>
                        <span class="metric-value" id="esp32-tilt">--°</span>
                    </div>
                    <div class="metric-row">
                        <span class="metric-label">Suppression Solenoid:</span>
                        <span class="metric-value" id="esp32-solenoid">STANDBY</span>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <script>
        function updateTelemetry() {
            fetch('/api/status')
                .then(r => r.json())
                .then(data => {
                    // Update state badge
                    const stateBox = document.getElementById('state-box');
                    stateBox.textContent = data.state;
                    stateBox.className = 'state-banner state-' + data.state;

                    // Update metrics
                    document.getElementById('fps-counter').textContent = data.fps.toFixed(1) + ' FPS';
                    document.getElementById('serial-badge').textContent = 'UART: ' + data.serial_status;
                    document.getElementById('serial-badge').style.color = (data.serial_status === 'Connected') ? 'var(--accent-ok)' : 'var(--accent-fire)';
                    
                    if (data.target_x !== null) {
                        document.getElementById('coords-val').textContent = '(' + data.target_x.toFixed(3) + ', ' + data.target_y.toFixed(3) + ')';
                    } else {
                        document.getElementById('coords-val').textContent = 'None';
                    }

                    document.getElementById('conf-val').textContent = (data.confidence * 100).toFixed(1) + '%';
                    document.getElementById('cpu-val').textContent = (data.cpu !== null ? data.cpu.toFixed(1) + '%' : 'N/A');
                    document.getElementById('ram-val').textContent = data.ram_used + ' / ' + data.ram_total + ' MB';
                    document.getElementById('temp-val').textContent = (data.temp !== null ? data.temp.toFixed(1) + '°C' : 'N/A');
                    document.getElementById('uart-count').textContent = data.uart_sent;

                    if (data.last_uart_tx) {
                        document.getElementById('uart-preview').textContent = JSON.stringify(data.last_uart_tx, null, 2);
                    }

                    // ESP32 Telemetry Update
                    if (data.esp32_telemetry) {
                        const esp = data.esp32_telemetry;
                        const espStateEl = document.getElementById('esp32-state');
                        espStateEl.textContent = esp.robot_state || 'ONLINE';
                        espStateEl.style.color = (esp.robot_state === 'EXTINGUISHING') ? 'var(--accent-fire)' : 'var(--accent-ok)';

                        document.getElementById('esp32-dist-front').textContent = (esp.dist_front !== undefined ? esp.dist_front + ' cm' : '--');
                        document.getElementById('esp32-dist-sides').textContent = (esp.dist_left !== undefined ? esp.dist_left : '--') + ' / ' + (esp.dist_right !== undefined ? esp.dist_right : '--') + ' cm';
                        
                        const flameEl = document.getElementById('esp32-flame');
                        if (esp.flame_detected) {
                            flameEl.textContent = 'DETECTED (ACTIVE)';
                            flameEl.style.color = 'var(--accent-fire)';
                        } else {
                            flameEl.textContent = 'CLEAR';
                            flameEl.style.color = 'var(--accent-ok)';
                        }

                        document.getElementById('esp32-tilt').textContent = (esp.tilt_angle !== undefined ? esp.tilt_angle + '°' : '--');
                        
                        const solEl = document.getElementById('esp32-solenoid');
                        if (esp.solenoid_active) {
                            solEl.textContent = 'SPRAYING (VALVE OPEN)';
                            solEl.style.color = 'var(--accent-fire)';
                        } else {
                            solEl.textContent = 'STANDBY';
                            solEl.style.color = 'var(--accent-muted)';
                        }
                    }
                })
                .catch(err => console.error(err));
        }

        setInterval(updateTelemetry, 1000);
        updateTelemetry();
    </script>
</body>
</html>
"""


class WebStreamServer:
    """
    Lightweight MJPEG streaming server for Raspberry Pi.
    """

    def __init__(self, host: str = "0.0.0.0", port: int = 5000) -> None:
        self.host = host
        self.port = port
        self.running = False

        self._frame_lock = threading.Lock()
        self._current_jpeg: Optional[bytes] = None
        self._telemetry: Dict[str, Any] = {
            "state": "IDLE",
            "fps": 0.0,
            "target_x": None,
            "target_y": None,
            "confidence": 0.0,
            "cpu": 0.0,
            "ram_used": 0.0,
            "ram_total": 0.0,
            "temp": 0.0,
            "serial_status": "Disconnected",
            "uart_sent": 0,
            "last_uart_tx": None,
        }

        self._app: Optional[Flask] = None
        self._thread: Optional[threading.Thread] = None

        if _HAS_FLASK:
            self._setup_app()

    def _setup_app(self) -> None:
        app = Flask("bantay_sunog_stream")

        @app.route("/")
        def index():
            return render_template_string(_DASHBOARD_HTML)

        @app.route("/video_feed")
        def video_feed():
            return Response(
                self._generate_stream(),
                mimetype="multipart/x-mixed-replace; boundary=frame",
            )

        @app.route("/api/status")
        def api_status():
            with self._frame_lock:
                return jsonify(self._telemetry)

        self._app = app

    def _generate_stream(self):
        """MJPEG generator."""
        while self.running:
            with self._frame_lock:
                jpeg = self._current_jpeg

            if jpeg is not None:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"
                )
            time.sleep(0.04)  # Cap at ~25 FPS to save Pi bandwidth

    def update_frame(self, frame_bgr: np.ndarray, telemetry: Dict[str, Any]) -> None:
        """
        Compress frame to JPEG and update live state buffer.
        """
        if not self.running or frame_bgr is None:
            return

        try:
            # Compress to JPEG with configured quality
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), config.WEB_STREAM_JPEG_QUALITY]
            success, buffer = cv2.imencode(".jpg", frame_bgr, encode_param)
            if success:
                jpeg_bytes = buffer.tobytes()
                with self._frame_lock:
                    self._current_jpeg = jpeg_bytes
                    self._telemetry.update(telemetry)
        except Exception:
            pass

    def start(self) -> bool:
        """Start streaming server in background daemon thread."""
        if not _HAS_FLASK:
            log_system("Flask is not installed. Web stream preview unavailable.")
            return False

        if self.running:
            return True

        self.running = True
        self._thread = threading.Thread(
            target=lambda: self._app.run(
                host=self.host,
                port=self.port,
                threaded=True,
                use_reloader=False,
            ),
            daemon=True,
        )
        self._thread.start()
        log_system(f"Web Preview Stream active: http://{self.host}:{self.port}")
        return True

    def stop(self) -> None:
        """Stop streaming server."""
        self.running = False
