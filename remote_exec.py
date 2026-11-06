"""
Remote executor utility to communicate with Raspberry Pi over SSH.
"""

import sys
import paramiko

PI_HOST = "192.168.137.200"
PI_USER = "dan"
PI_PASS = "123"

def execute_remote(commands):
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        client.connect(PI_HOST, username=PI_USER, password=PI_PASS, timeout=5)
    except Exception as e:
        print(f"Failed to connect: {e}")
        return

    for cmd in commands:
        print(f"\n>>> {cmd}")
        stdin, stdout, stderr = client.exec_command(cmd)
        out = stdout.read().decode()
        err = stderr.read().decode()
        if out:
            print(out.strip())
        if err:
            print(f"[STDERR]\n{err.strip()}")

    client.close()

if __name__ == "__main__":
    cmds = sys.argv[1:] if len(sys.argv) > 1 else [
        "free -h",
        "lscpu | head -n 15",
        "lsusb",
        "ls -l /dev/video* 2>/dev/null || echo 'No video devices found'",
        "ls -l /dev/serial* 2>/dev/null || echo 'No serial symlinks found'",
        "ls -l /dev/ttyAMA* /dev/ttyS* 2>/dev/null || echo 'No ttyAMA/ttyS found'",
        "python3 --version",
        "dpkg -l | grep -E 'python3-opencv|python3-serial|python3-numpy|python3-psutil|v4l' || echo 'None matched'",
    ]
    execute_remote(cmds)
