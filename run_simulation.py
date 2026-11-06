"""
BANTAY-SUNOG 3D Simulation Launcher
Starts the Digital Twin Sync Server and opens the 3D Web Dashboard.
"""

import argparse
import sys
import time
import webbrowser
from pathlib import Path

# Add project root to sys.path
PROJECT_ROOT = Path(__file__).resolve().parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from simulation.bridge.sync_server import main

BANNER = """
================================================================================
           [+] BANTAY-SUNOG 3D DIGITAL TWIN SIMULATION PLATFORM [+]             
================================================================================
 [3D DASHBOARD]   : http://localhost:{port}/
 [WEBSOCKET SYNC] : ws://localhost:{ws_port}/ws/telemetry
 [SERIAL LINK]    : {serial_status}

 [CONTROLS & SHORTCUTS]
   * W / S / A / D : Drive Forward / Reverse / Pivot Left / Pivot Right
   * U / J / H     : Tilt Gimbal Up (+10 deg) / Down (-10 deg) / Level Home (90 deg)
   * F             : Extinguisher Solenoid Spray Pulse
   * Orbit View    : Left-click drag to rotate, Right-click to pan, Scroll to zoom
   * Environment   : Click 'Add Fire' or 'Add Crate/Wall' and click floor to place
   * 3D Model Swap : Click 'Load Custom 3D Model' to drop your .glb/.gltf file

 Press Ctrl+C to stop the simulation server.
================================================================================
"""

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Launch BANTAY-SUNOG 3D Simulation")
    parser.add_argument("--port", type=int, default=8000, help="HTTP dashboard port (default: 8000)")
    parser.add_argument("--ws-port", type=int, default=8765, help="WebSocket port (default: 8765)")
    parser.add_argument("--serial-port", default=None, help="ESP32 COM port (e.g. COM3). Omit for auto/mock.")
    parser.add_argument("--no-browser", action="store_true", help="Do not automatically open web browser")
    args = parser.parse_args()

    serial_status = f"Connecting to {args.serial_port}" if args.serial_port else "Mock Loopback (Autonomous SIL)"
    print(BANNER.format(port=args.port, ws_port=args.ws_port, serial_status=serial_status))

    if not args.no_browser:
        # Open browser shortly after server starts
        import threading
        def open_browser():
            time.sleep(1.0)
            webbrowser.open(f"http://localhost:{args.port}/")
        threading.Thread(target=open_browser, daemon=True).start()

    sys.argv = [
        sys.argv[0],
        "--http-port", str(args.port),
        "--ws-port", str(args.ws_port),
        "--mock"
    ]
    if args.serial_port:
        sys.argv.extend(["--serial-port", args.serial_port])

    main()
