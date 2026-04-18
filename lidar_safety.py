"""
Lidar Safety Module

Continuously scans using the RPLIDAR and sets front/rear blocked flags.
Runs in a background thread so it doesn't block the Flask server.

LIDAR Orientation (left/right mirrored from standard):
  - 0 degrees   = directly in FRONT of the robot
  - 90 degrees  = RIGHT side of the robot
  - 180 degrees = directly BEHIND the robot
  - 270 degrees = LEFT side of the robot

Detection Zones (60-degree cones):
  - Front danger zone: 330-360 and 0-30 degrees
    This is a 60-degree cone centered on 0 (straight ahead).
    Only blocks forward motion (positive Y joystick).
    Turning in place and sideways motion are NOT blocked.

  - Rear danger zone: 150-210 degrees
    This is a 60-degree cone centered on 180 (straight behind).
    Only blocks backward motion (negative Y joystick).
    Turning in place and sideways motion are NOT blocked.

Stopping distance: 800mm
  Any LIDAR reading within the danger zone closer than 800mm
  will set the corresponding blocked flag.

Safety behavior:
  - Forward blocked: joystick Y is zeroed, X (turning) still works
  - Rear blocked: joystick Y is zeroed, X (turning) still works
  - Both blocked: only turning in place is allowed
  - Obstacles that enter/leave range are detected dynamically

Thread safety: front_blocked and rear_blocked are simple booleans,
which are atomic in CPython. A lock is used for the scan data.
"""

import threading
import time
from rplidar import RPLidar

# Configuration - tune these for your robot
LIDAR_PORT = '/dev/ttyUSB0'
STOP_DISTANCE_MM = 600         # Stop if obstacle closer than this

# Front zone: 30 degrees each side of 0 = 330 to 30 (60 degree cone)
FRONT_ZONE = (330, 30)

# Rear zone: 14 degrees each side of 180 = 166 to 194 (28 degree cone)
# Narrower than front to avoid seeing robot's own body
REAR_ZONE = (166, 194)


def angle_in_zone(angle, zone_start, zone_end):
    """Check if an angle is within a zone, handling wraparound at 360."""
    if zone_start <= zone_end:
        return zone_start <= angle <= zone_end
    else:
        # Wraps around 360 (e.g., 330 to 30)
        return angle >= zone_start or angle <= zone_end


class LidarSafety:
    """
    Reads RPLIDAR data in a background thread and maintains
    front_blocked / rear_blocked flags for the drive system.
    """

    def __init__(self, port=LIDAR_PORT, stop_distance=STOP_DISTANCE_MM):
        self.port = port
        self.stop_distance = stop_distance
        self.front_blocked = False
        self.rear_blocked = False
        self._running = False
        self._thread = None
        self._lidar = None
        self._lock = threading.Lock()
        self._scan_callbacks = []  # List of callables to receive raw scan data
        # Store latest scan data for debugging
        self._front_min_dist = float('inf')
        self._rear_min_dist = float('inf')

    def start(self):
        """Start the background LIDAR scanning thread."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._scan_loop, daemon=True)
        self._thread.start()
        print(f"[LIDAR] Started scanning (stop distance: {self.stop_distance}mm)")
        print(f"[LIDAR] Front zone: {FRONT_ZONE[0]}-360 and 0-{FRONT_ZONE[1]} degrees (60 deg cone)")
        print(f"[LIDAR] Rear zone: {REAR_ZONE[0]}-{REAR_ZONE[1]} degrees (60 deg cone)")

    def add_scan_callback(self, callback):
        """Register a callback to receive raw scan data. Called on every scan."""
        self._scan_callbacks.append(callback)

    def remove_scan_callback(self, callback):
        """Remove a previously registered callback."""
        self._scan_callbacks = [cb for cb in self._scan_callbacks if cb != callback]

    def stop(self):
        """Stop the LIDAR scanning thread."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        if self._lidar:
            try:
                self._lidar.stop()
                self._lidar.stop_motor()
                self._lidar.disconnect()
            except:
                pass
        print("[LIDAR] Stopped")

    def get_status(self):
        """Get current safety status for API/debug."""
        with self._lock:
            return {
                'front_blocked': self.front_blocked,
                'rear_blocked': self.rear_blocked,
                'front_min_distance_mm': round(self._front_min_dist) if self._front_min_dist != float('inf') else -1,
                'rear_min_distance_mm': round(self._rear_min_dist) if self._rear_min_dist != float('inf') else -1,
                'stop_distance_mm': self.stop_distance,
            }

    def _scan_loop(self):
        """Main scanning loop - runs in background thread."""
        import serial as _pyserial
        while self._running:
            try:
                # Hard serial reset first to clear any stale data
                try:
                    _s = _pyserial.Serial(self.port, 115200, timeout=0.5)
                    _s.write(b'\xa5\x25')  # RPLIDAR stop command
                    time.sleep(0.1)
                    _s.flushInput()
                    _s.flushOutput()
                    _s.close()
                    time.sleep(0.3)
                except Exception:
                    pass

                self._lidar = RPLidar(self.port)
                self._lidar.clean_input()
                time.sleep(0.3)
                self._lidar.clean_input()
                info = self._lidar.get_info()
                print(f"[LIDAR] Connected: model {info['model']}, firmware {info['firmware']}")
                health = self._lidar.get_health()
                print(f"[LIDAR] Health: {health[0]}")

                for scan in self._lidar.iter_scans():
                    if not self._running:
                        break
                    self._process_scan(scan)

            except Exception as e:
                # Only print first error and state changes, not every retry
                if not getattr(self, '_last_error_msg', None) == str(e):
                    print(f"[LIDAR] Error: {e}")
                    self._last_error_msg = str(e)

                if self._lidar:
                    try:
                        self._lidar.stop()
                        self._lidar.stop_motor()
                        self._lidar.disconnect()
                    except:
                        pass
                    self._lidar = None

                if self._running:
                    time.sleep(1.0)

    def _process_scan(self, scan):
        """
        Process one complete scan and update blocked flags.
        Also forwards raw scan to any registered callbacks (wall follower, greeter).

        Each scan is a list of (quality, angle, distance) tuples.
        Quality 0 means invalid reading - skip those.
        Distance 0 also means invalid.
        Readings under IGNORE_BELOW_MM are ignored (robot's own body).
        """
        # Forward scan to callbacks
        for cb in self._scan_callbacks:
            try:
                cb(scan)
            except Exception as e:
                print(f"[LIDAR] Callback error: {e}")

        front_min = float('inf')
        rear_min = float('inf')

        for quality, angle, distance in scan:
            if quality == 0 or distance == 0:
                continue
            # Ignore readings under 200mm - these are the robot's own body
            if distance < 200:
                continue

            # Check front zone (330-360 and 0-30)
            if angle_in_zone(angle, FRONT_ZONE[0], FRONT_ZONE[1]):
                front_min = min(front_min, distance)

            # Check rear zone (150-210)
            if angle_in_zone(angle, REAR_ZONE[0], REAR_ZONE[1]):
                rear_min = min(rear_min, distance)

        front_was_blocked = self.front_blocked
        rear_was_blocked = self.rear_blocked

        self.front_blocked = front_min < self.stop_distance
        self.rear_blocked = rear_min < self.stop_distance

        with self._lock:
            self._front_min_dist = front_min
            self._rear_min_dist = rear_min

        # Print debug when status changes
        if self.front_blocked != front_was_blocked:
            if self.front_blocked:
                print(f"[LIDAR] FRONT BLOCKED - obstacle at {front_min:.0f}mm")
            else:
                print(f"[LIDAR] Front clear (nearest: {front_min:.0f}mm)")

        if self.rear_blocked != rear_was_blocked:
            if self.rear_blocked:
                print(f"[LIDAR] REAR BLOCKED - obstacle at {rear_min:.0f}mm")
            else:
                print(f"[LIDAR] Rear clear (nearest: {rear_min:.0f}mm)")


# Quick standalone test
if __name__ == "__main__":
    import signal

    lidar = LidarSafety()

    def cleanup(*args):
        lidar.stop()
        exit(0)

    signal.signal(signal.SIGINT, cleanup)

    lidar.start()
    print("\nWaiting for first scan...")
    time.sleep(2)
    print("Watching for obstacles... Press Ctrl+C to stop.\n")

    while True:
        status = lidar.get_status()
        front = "BLOCKED" if status['front_blocked'] else "clear"
        rear = "BLOCKED" if status['rear_blocked'] else "clear"
        print(f"  Front: {front} ({status['front_min_distance_mm']}mm)  |  "
              f"Rear: {rear} ({status['rear_min_distance_mm']}mm)", end='\r')
        time.sleep(0.2)
