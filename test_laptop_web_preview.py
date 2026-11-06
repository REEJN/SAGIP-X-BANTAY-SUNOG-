"""
Test web preview streaming from laptop to Raspberry Pi with connection retry.
"""

import json
import time
import urllib.request
import paramiko

PI_HOST = "192.168.137.200"
PI_USER = "dan"
PI_PASS = "123"

def main():
    print(f"Connecting to Raspberry Pi at {PI_HOST}...")
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(PI_HOST, username=PI_USER, password=PI_PASS, timeout=10)

    # Launch main.py with --web and --simulate for 120 frames (~10 seconds)
    cmd = "PYTHONPATH=/home/dan python3 -u -m bantay_sunog.main --simulate --web --max-frames 120"
    print(f"Starting web stream on Pi: {cmd}")
    stdin, stdout, stderr = client.exec_command(cmd)

    url = f"http://{PI_HOST}:5000/"
    api_url = f"http://{PI_HOST}:5000/api/status"

    print("\nWaiting for Flask to bind on port 5000 (polling up to 10s)...")
    connected = False
    for attempt in range(1, 15):
        time.sleep(1.0)
        try:
            req = urllib.request.Request(url)
            with urllib.request.urlopen(req, timeout=2) as resp:
                if resp.status == 200:
                    html = resp.read().decode()
                    print(f"[Laptop Client] SUCCESS! Connected on attempt {attempt}. HTTP {resp.status} OK!")
                    print(f"[Laptop Client] Received {len(html)} bytes of Dashboard HTML.")
                    assert "BANTAY-SUNOG" in html
                    connected = True
                    break
        except Exception as e:
            print(f"[Attempt {attempt}] Waiting... ({e})")

    if connected:
        print(f"\n[Laptop Client] Requesting live telemetry API: {api_url}")
        try:
            with urllib.request.urlopen(api_url, timeout=3) as resp:
                data = json.loads(resp.read().decode())
                print("[Laptop Client] Live Telemetry JSON received:")
                print(json.dumps(data, indent=2))
                assert "state" in data
                assert "fps" in data
                print(f"\n>>> [VERIFIED] State: {data['state']} | FPS: {data['fps']:.1f} | CPU: {data['cpu']}% | Temp: {data['temp']}C")
        except Exception as e:
            print(f"[Laptop Client] Telemetry fetch error: {e}")

    # Read Pi stdout to show completion
    print("\n[Pi Console Output]:")
    out = stdout.read().decode()
    print(out)
    client.close()

if __name__ == "__main__":
    main()
