"""
Run automated test suite on Raspberry Pi 3B.
"""

import paramiko

PI_HOST = "192.168.137.200"
PI_USER = "dan"
PI_PASS = "123"

def main():
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(PI_HOST, username=PI_USER, password=PI_PASS, timeout=10)

    cmd = "PYTHONPATH=/home/dan python3 -m unittest discover -s /home/dan/bantay_sunog/tests -v"
    print(f"\nRunning on Pi: {cmd}\n")
    stdin, stdout, stderr = client.exec_command(cmd)
    
    out = stdout.read().decode()
    err = stderr.read().decode()
    if out:
        print(out)
    if err:
        print(err)

    client.close()

if __name__ == "__main__":
    main()
