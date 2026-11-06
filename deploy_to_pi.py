"""
Deploy bantay_sunog project directory to Raspberry Pi 3B via SFTP.
"""

import os
import paramiko

PI_HOST = "192.168.137.200"
PI_USER = "dan"
PI_PASS = "123"
LOCAL_DIR = r"C:\Users\danda\Documents\antigravity\proud-babbage\bantay_sunog"
REMOTE_DIR = "/home/dan/bantay_sunog"

def sftp_mkdirs(sftp, remote_path):
    dirs = []
    head, tail = os.path.split(remote_path)
    while head and head != "/":
        dirs.append(head)
        head, _ = os.path.split(head)
    for d in reversed(dirs):
        try:
            sftp.mkdir(d)
        except IOError:
            pass
    try:
        sftp.mkdir(remote_path)
    except IOError:
        pass

def deploy():
    print(f"Connecting to {PI_HOST} as {PI_USER}...")
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(PI_HOST, username=PI_USER, password=PI_PASS, timeout=10)
    sftp = client.open_sftp()

    print(f"Creating remote directory {REMOTE_DIR}...")
    sftp_mkdirs(sftp, REMOTE_DIR)

    total_files = 0
    for root, dirs, files in os.walk(LOCAL_DIR):
        rel_path = os.path.relpath(root, LOCAL_DIR)
        remote_root = REMOTE_DIR if rel_path == "." else os.path.join(REMOTE_DIR, rel_path).replace("\\", "/")
        sftp_mkdirs(sftp, remote_root)

        for f in files:
            local_file = os.path.join(root, f)
            remote_file = os.path.join(remote_root, f).replace("\\", "/")
            print(f"Uploading: {rel_path}/{f} -> {remote_file}")
            sftp.put(local_file, remote_file)
            total_files += 1

    sftp.close()
    print(f"Successfully deployed {total_files} files to {REMOTE_DIR}!")

    # Verify remote file listing
    stdin, stdout, stderr = client.exec_command(f"find {REMOTE_DIR} -type f")
    print("\n--- Remote Files on Pi ---")
    print(stdout.read().decode())
    client.close()

if __name__ == "__main__":
    deploy()
