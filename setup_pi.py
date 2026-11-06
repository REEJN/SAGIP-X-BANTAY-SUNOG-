"""
Configure packages and system on Raspberry Pi.
"""

import paramiko

PI_HOST = "192.168.137.200"
PI_USER = "dan"
PI_PASS = "123"

def run_cmd(client, cmd):
    print(f"\n>>> {cmd}")
    stdin, stdout, stderr = client.exec_command(cmd)
    out = stdout.read().decode()
    err = stderr.read().decode()
    if out:
        print(out.strip())
    if err:
        print(f"[STDERR]\n{err.strip()}")

def main():
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(PI_HOST, username=PI_USER, password=PI_PASS, timeout=10)

    # Clean pycache
    run_cmd(client, "find /home/dan/bantay_sunog -name '*.pyc' -delete")
    run_cmd(client, "find /home/dan/bantay_sunog -type d -name '__pycache__' -exec rm -rf {} +")

    # Install packages
    run_cmd(client, "echo 123 | sudo -S apt-get update -qq")
    run_cmd(client, "echo 123 | sudo -S apt-get install -y python3-serial python3-psutil")

    # Verify python packages
    run_cmd(client, "python3 -c 'import cv2, numpy, serial, psutil; print(\"ALL CORE IMPORTS OK!\")'")

    client.close()

if __name__ == "__main__":
    main()
