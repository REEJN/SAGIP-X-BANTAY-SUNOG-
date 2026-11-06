"""
Run live BANTAY-SUNOG application on Raspberry Pi with unbuffered output.
"""

import time
import paramiko

PI_HOST = "192.168.137.200"
PI_USER = "dan"
PI_PASS = "123"

def main():
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(PI_HOST, username=PI_USER, password=PI_PASS, timeout=10)

    # Use python3 -u for unbuffered stdout
    cmd = "timeout --signal=INT 6s python3 -u -m bantay_sunog.main --simulate"
    print(f"Executing on Pi: {cmd}\n")
    stdin, stdout, stderr = client.exec_command(f"cd /home/dan && {cmd}")

    while True:
        line = stdout.readline()
        if not line:
            break
        print(line, end="")

    err = stderr.read().decode()
    if err:
        print("\n=== STDERR ===")
        print(err)

    client.close()

if __name__ == "__main__":
    main()
