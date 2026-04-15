#!/usr/bin/env python3
"""
Display the robot's IP addresses for connecting from phone/laptop.
"""

import socket
import subprocess

def get_ip_addresses():
    """Get all IP addresses for this machine."""
    ips = []

    # Method 1: Using socket
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ips.append(("Primary", s.getsockname()[0]))
        s.close()
    except:
        pass

    # Method 2: Using hostname
    try:
        hostname = socket.gethostname()
        ip = socket.gethostbyname(hostname)
        if ip not in [i[1] for i in ips]:
            ips.append(("Hostname", ip))
    except:
        pass

    # Method 3: Parse ip addr output
    try:
        result = subprocess.run(['ip', 'addr'], capture_output=True, text=True)
        lines = result.stdout.split('\n')
        for line in lines:
            if 'inet ' in line and '127.0.0.1' not in line:
                parts = line.strip().split()
                ip = parts[1].split('/')[0]
                if ip not in [i[1] for i in ips]:
                    ips.append(("Interface", ip))
    except:
        pass

    return ips

def main():
    print("\n" + "=" * 50)
    print("Robot IP Addresses")
    print("=" * 50)

    ips = get_ip_addresses()

    if not ips:
        print("\nCould not determine IP address.")
        print("Make sure you're connected to a network.")
    else:
        print("\nConnect to the robot using one of these URLs:\n")
        for label, ip in ips:
            print(f"  http://{ip}:5000")

    print("\n" + "=" * 50)
    print("Make sure the Flask server is running:")
    print("  python app.py")
    print("=" * 50 + "\n")

if __name__ == "__main__":
    main()
