"""
Check Python environment on Pi.
"""

import paramiko

PI_HOST = "192.168.137.200"
PI_USER = "dan"
PI_PASS = "123"

def main():
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(PI_HOST, username=PI_USER, password=PI_PASS, timeout=10)

    for cmd in [
        "python3 -c 'import numpy; print(numpy.__file__, numpy.__version__)'",
        "python3 -c 'import cv2; print(cv2.__file__, cv2.__version__)'",
        "pip list",
    ]:
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
    main()
