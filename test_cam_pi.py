"""
Test clean YUYV frame capture without MJPG corruption.
"""

import paramiko

PI_HOST = "192.168.137.200"
PI_USER = "dan"
PI_PASS = "123"

def main():
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(PI_HOST, username=PI_USER, password=PI_PASS, timeout=10)

    code = """
import cv2
cap = cv2.VideoCapture('/dev/video1', cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
for i in range(10):
    ret, frame = cap.read()
print('Read result:', ret, 'Shape:', frame.shape if ret else None)
cap.release()
"""
    cmd = f"python3 -c \"{code}\""
    stdin, stdout, stderr = client.exec_command(cmd)
    print("STDOUT:\n", stdout.read().decode())
    print("STDERR:\n", stderr.read().decode())
    client.close()

if __name__ == "__main__":
    main()
