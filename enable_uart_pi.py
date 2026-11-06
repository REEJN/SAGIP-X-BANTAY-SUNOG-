"""
Configure UART in /boot/firmware/config.txt on Raspberry Pi.
"""

import paramiko

PI_HOST = "192.168.137.200"
PI_USER = "dan"
PI_PASS = "123"

def main():
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(PI_HOST, username=PI_USER, password=PI_PASS, timeout=10)

    # Check if enable_uart is already set
    stdin, stdout, stderr = client.exec_command("grep -n 'enable_uart' /boot/firmware/config.txt")
    out = stdout.read().decode()
    print("Current enable_uart setting in /boot/firmware/config.txt:")
    print(out if out else "None found")

    # Add enable_uart=1 and dtoverlay=disable-bt under [all]
    cmds = [
        "echo 123 | sudo -S bash -c 'echo \"enable_uart=1\" >> /boot/firmware/config.txt'",
        "echo 123 | sudo -S bash -c 'echo \"dtoverlay=disable-bt\" >> /boot/firmware/config.txt'",
        "echo 123 | sudo -S systemctl disable hciuart 2>/dev/null || true",
        "tail -n 6 /boot/firmware/config.txt",
    ]
    for cmd in cmds:
        print(f"\n>>> {cmd}")
        stdin, stdout, stderr = client.exec_command(cmd)
        print(stdout.read().decode().strip())

    client.close()

if __name__ == "__main__":
    main()
