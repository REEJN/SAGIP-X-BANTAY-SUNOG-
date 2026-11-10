"""
BANTAY-SUNOG Raspberry Pi 3B Telemetry & UART Bridge Service
============================================================
Runs on Raspberry Pi 3B (192.168.137.200).
- Opens hardware UART /dev/serial0 (connected to ESP32 RX/TX pins) at 115200 baud
- Gathers real Pi system metrics (CPU, thermal sensor temp, RAM)
- Serves a lightweight TCP stream server on port 8888 for the laptop dashboard
- Enables bidirectional communication: ESP32 <-> Pi <-> Wi-Fi <-> Laptop Dashboard
"""

import asyncio
import json
import os
import sys
import time
from typing import Set

try:
    import serial
except ImportError:
    serial = None

UART_PORT = os.environ.get("BANTAY_UART_PORT", "/dev/serial0")
UART_BAUD = int(os.environ.get("BANTAY_UART_BAUD", "115200"))
TCP_PORT = int(os.environ.get("BANTAY_TCP_PORT", "8888"))

connected_clients: Set[asyncio.StreamWriter] = set()
ser_conn = None
last_esp_packet_time = 0.0


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


async def uart_reader_task():
    global ser_conn, last_esp_packet_time
    print(f"[UART] Opening {UART_PORT} at {UART_BAUD} baud...")
    while True:
        try:
            if ser_conn is None or not ser_conn.is_open:
                ser_conn = serial.Serial(UART_PORT, UART_BAUD, timeout=1.0)
                ser_conn.reset_input_buffer()
                print(f"[UART] Connected to physical ESP32 on {UART_PORT}!")
                # Announce to clients
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

            # Read line asynchronously in executor
            loop = asyncio.get_running_loop()
            line = await loop.run_in_executor(None, ser_conn.readline)
            if line:
                decoded = line.decode("utf-8", errors="replace").strip()
                if decoded.startswith("{") and decoded.endswith("}"):
                    last_esp_packet_time = time.time()
                    # Broadcast raw ESP32 packet
                    await broadcast_to_clients(decoded)
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
            "source": "PI_WIFI_BRIDGE",
            "esp_alive": esp_alive,
            "last_esp_packet_age": round(now - last_esp_packet_time, 2) if last_esp_packet_time > 0 else 999.0
        })
        await broadcast_to_clients(status_pkt)


async def handle_client(reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
    peer = writer.get_extra_info("peername")
    print(f"[TCP] Dashboard client connected from {peer}")
    connected_clients.add(writer)

    # Send initial welcome and status immediately
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

    # Immediate status
    metrics = get_pi_metrics()
    esp_alive = bool(ser_conn and ser_conn.is_open and (time.time() - last_esp_packet_time < 3.0))
    status_pkt = json.dumps({
        "type": "HARDWARE_STATUS",
        "connected": esp_alive,
        "port": f"Raspberry Pi 3B (192.168.137.200 -> {UART_PORT})",
        "baudrate": UART_BAUD,
        "available_ports": [UART_PORT],
        "pi_metrics": metrics,
        "source": "PI_WIFI_BRIDGE"
    })
    writer.write((status_pkt + "\n").encode("utf-8"))
    await writer.drain()

    try:
        while True:
            line = await reader.readline()
            if not line:
                break
            # Handle incoming commands from dashboard to ESP32
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
    server = await asyncio.start_server(handle_client, "0.0.0.0", TCP_PORT)
    print(f"============================================================")
    print(f"   BANTAY-SUNOG Raspberry Pi 3B Wi-Fi Telemetry Bridge")
    print(f"   Listening on TCP 0.0.0.0:{TCP_PORT}")
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
