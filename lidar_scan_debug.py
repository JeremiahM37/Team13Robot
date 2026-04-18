#!/usr/bin/env python3
"""
Quick LIDAR debug: prints minimum distance in each 30-degree sector
so we can see where walls actually appear.
Place the robot next to a wall on its RIGHT side, then run this.
"""
import signal, sys, time
from rplidar import RPLidar

lidar = None

def cleanup(*args):
    if lidar:
        try:
            lidar.stop(); lidar.stop_motor(); lidar.disconnect()
        except: pass
    sys.exit(0)

signal.signal(signal.SIGINT, cleanup)

lidar = RPLidar('/dev/ttyUSB0')
lidar.clean_input()
print(f"Connected: {lidar.get_info()['model']}")
print("\nPlace robot with RIGHT side along a wall.")
print("Each sector shows the minimum distance detected.\n")
time.sleep(1)

scan_count = 0
for scan in lidar.iter_scans():
    # Group into 30-degree sectors
    sectors = {}
    for i in range(0, 360, 30):
        sectors[i] = float('inf')

    for quality, angle, distance in scan:
        if quality == 0 or distance == 0 or distance < 200:
            continue
        sector = int(angle // 30) * 30
        sectors[sector] = min(sectors.get(sector, float('inf')), distance)

    # Print all sectors
    parts = []
    for start in sorted(sectors):
        end = start + 30
        d = sectors[start]
        dist_str = f"{int(d):5d}" if d != float('inf') else "  inf"
        # Highlight sectors that show a close wall
        marker = " <<< WALL?" if d < 800 else ""
        parts.append(f"  {start:3d}-{end:3d}°: {dist_str}mm{marker}")

    scan_count += 1
    if scan_count % 3 == 0:  # Print every 3rd scan to reduce spam
        print(f"\n--- Scan {scan_count} ---")
        for p in parts:
            print(p)
