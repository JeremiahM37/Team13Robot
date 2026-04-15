#!/usr/bin/env python3
"""
Autonomous Wall Follower - Project 4

The robot drives autonomously while maintaining a target distance from a wall
on one side (left or right). Uses LIDAR zones to detect walls and obstacles.

LIDAR Orientation:
  - 0 degrees   = directly in front of the robot
  - 90 degrees  = left side of the robot
  - 180 degrees = directly behind the robot
  - 270 degrees = right side of the robot

LIDAR Zones (for right-wall following):
  - FRONT:       340-360 and 0-20 degrees (40 deg cone, collision avoidance)
  - FRONT-RIGHT: 290-340 degrees          (leading edge of wall tracking)
  - RIGHT:       250-290 degrees          (primary wall distance measurement)
  - BACK-RIGHT:  210-250 degrees          (trailing edge of wall tracking)

  For left-wall following, zones are mirrored:
  - FRONT-LEFT:  20-70 degrees
  - LEFT:        70-110 degrees
  - BACK-LEFT:   110-150 degrees

Behavior Cases:
  Case 1 - Obstacle in front:  Stop forward, turn away from wall until clear.
  Case 2 - Wall too close:     Steer away from wall gently.
  Case 3 - Wall too far:       Steer toward wall gently.
  Case 4 - Wall lost:          Gently turn toward wall side to re-acquire.

Thresholds:
  - WALL_TARGET_MM:    Target distance from wall (default 500mm)
  - WALL_TOO_CLOSE_MM: Inner threshold (default 350mm) - steer away
  - WALL_TOO_FAR_MM:   Outer threshold (default 650mm) - steer toward
  - FRONT_STOP_MM:     Front obstacle stop distance (default 400mm)
  - WALL_LOST_MM:      Wall considered lost above this (default 1500mm)

  The gap between TOO_CLOSE and TOO_FAR creates a dead band around the
  target distance. Inside this band, the robot drives straight without
  correcting, preventing oscillation/spasms.

Usage (standalone):
    python wall_follower.py              # Right wall follow (default)
    python wall_follower.py --left       # Left wall follow
    python wall_follower.py --sim        # Simulation mode (no hardware)
    python wall_follower.py --speed 0.4  # Set forward speed (0.0-1.0)

Can also be started/stopped from the Flask web interface via app.py.
"""

import sys
import signal
import time
import threading
from rplidar import RPLidar
from robot_control import RobotController

# ==================== CONFIGURATION ====================

LIDAR_PORT = '/dev/ttyUSB0'

# Wall following parameters (mm)
WALL_TARGET_MM = 500       # Ideal distance from wall
WALL_TOO_CLOSE_MM = 350    # Steer away below this
WALL_TOO_FAR_MM = 650      # Steer toward above this
WALL_LOST_MM = 1500        # Wall considered lost above this
FRONT_STOP_MM = 400        # Stop if obstacle closer than this in front

# Speed settings (0.0 to 1.0)
FORWARD_SPEED = 0.35       # Base forward speed
TURN_SPEED = 0.4           # Turn-in-place speed when front is blocked
STEER_CORRECTION = 0.15    # How much to adjust steering for wall corrections
SEARCH_TURN_SPEED = 0.2    # Gentle turn speed when searching for lost wall

# LIDAR zones (degrees) - right wall following
ZONES_RIGHT = {
    'front':       (340, 20),
    'front_wall':  (290, 340),
    'wall':        (250, 290),
    'back_wall':   (210, 250),
}

ZONES_LEFT = {
    'front':       (340, 20),
    'front_wall':  (20, 70),
    'wall':        (70, 110),
    'back_wall':   (110, 150),
}

LOOP_INTERVAL = 0.1


# ==================== HELPER ====================

def angle_in_zone(angle, zone_start, zone_end):
    if zone_start <= zone_end:
        return zone_start <= angle <= zone_end
    else:
        return angle >= zone_start or angle <= zone_end


def safe_round(val):
    return round(val) if val != float('inf') else -1


# ==================== WALL FOLLOWER ====================

class WallFollower:
    """
    Autonomous wall following using LIDAR and differential drive.

    Can run standalone (with its own LIDAR thread) or integrated into
    app.py (receiving scan data from a shared LIDAR instance).
    """

    def __init__(self, robot, side='right', speed=FORWARD_SPEED):
        """
        Args:
            robot: RobotController instance (shared with app.py)
            side: 'right' or 'left' - which side the wall is on
            speed: base forward speed (0.0 to 1.0)
        """
        self.robot = robot
        self.side = side
        self.zones = ZONES_RIGHT if side == 'right' else ZONES_LEFT
        self.forward_speed = speed

        self._lock = threading.Lock()
        self.distances = {name: float('inf') for name in self.zones}
        self.state = 'IDLE'
        self._running = False
        self._thread = None

    @property
    def active(self):
        return self._running

    def start(self):
        """Start wall following in a background thread."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()
        print(f"[WALL] Started - {self.side} wall, speed={self.forward_speed}")

    def stop(self):
        """Stop wall following."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        self.robot.stop_wheels()
        self.state = 'IDLE'
        print("[WALL] Stopped")

    def update_scan(self, scan):
        """
        Receive a LIDAR scan from an external source (shared LIDAR).
        Called by the LIDAR thread in app.py or by our own LIDAR thread.
        """
        zone_mins = {name: float('inf') for name in self.zones}
        for quality, angle, distance in scan:
            if quality == 0 or distance == 0:
                continue
            for zone_name, (start, end) in self.zones.items():
                if angle_in_zone(angle, start, end):
                    zone_mins[zone_name] = min(zone_mins[zone_name], distance)
        with self._lock:
            self.distances = zone_mins

    def get_status(self):
        """Get current wall follower status."""
        with self._lock:
            d = self.distances.copy()
        return {
            'active': self._running,
            'state': self.state,
            'side': self.side,
            'front': safe_round(d.get('front', -1)),
            'wall': safe_round(d.get('wall', -1)),
        }

    def _get_distances(self):
        with self._lock:
            return self.distances.copy()

    def _control_loop(self):
        """Main control loop."""
        prev_state = None

        while self._running:
            dist = self._get_distances()
            front = dist['front']
            wall = dist['wall']

            left_speed = 0.0
            right_speed = 0.0

            # Case 1: Obstacle in front - turn away from wall
            if front < FRONT_STOP_MM:
                self.state = 'FRONT_BLOCKED'
                if self.side == 'right':
                    left_speed = -TURN_SPEED
                    right_speed = TURN_SPEED
                else:
                    left_speed = TURN_SPEED
                    right_speed = -TURN_SPEED

            # Case 4: Wall lost - gently turn toward wall
            elif wall > WALL_LOST_MM:
                self.state = 'WALL_LOST'
                if self.side == 'right':
                    left_speed = self.forward_speed
                    right_speed = self.forward_speed - SEARCH_TURN_SPEED
                else:
                    left_speed = self.forward_speed - SEARCH_TURN_SPEED
                    right_speed = self.forward_speed

            # Case 2: Wall too close - steer away
            elif wall < WALL_TOO_CLOSE_MM:
                self.state = 'TOO_CLOSE'
                if self.side == 'right':
                    left_speed = self.forward_speed - STEER_CORRECTION
                    right_speed = self.forward_speed + STEER_CORRECTION
                else:
                    left_speed = self.forward_speed + STEER_CORRECTION
                    right_speed = self.forward_speed - STEER_CORRECTION

            # Case 3: Wall too far - steer toward
            elif wall > WALL_TOO_FAR_MM:
                self.state = 'TOO_FAR'
                if self.side == 'right':
                    left_speed = self.forward_speed + STEER_CORRECTION
                    right_speed = self.forward_speed - STEER_CORRECTION
                else:
                    left_speed = self.forward_speed - STEER_CORRECTION
                    right_speed = self.forward_speed + STEER_CORRECTION

            # Default: in dead band, drive straight
            else:
                self.state = 'FOLLOWING'
                left_speed = self.forward_speed
                right_speed = self.forward_speed

            self.robot.drive(left_speed, right_speed)

            if self.state != prev_state:
                print(f"[WALL] {self.state:15s} | Front: {safe_round(front):5d}mm | Wall: {safe_round(wall):5d}mm")
                prev_state = self.state

            time.sleep(LOOP_INTERVAL)

        self.robot.stop_wheels()


# ==================== STANDALONE MODE ====================

def main():
    """Run wall follower as a standalone script with its own LIDAR."""
    side = 'left' if '--left' in sys.argv else 'right'
    simulation_mode = '--sim' in sys.argv

    speed = FORWARD_SPEED
    if '--speed' in sys.argv:
        idx = sys.argv.index('--speed')
        if idx + 1 < len(sys.argv):
            try:
                speed = max(0.1, min(1.0, float(sys.argv[idx + 1])))
            except ValueError:
                pass

    robot = RobotController(simulation_mode=simulation_mode)
    follower = WallFollower(robot, side=side, speed=speed)

    # Own LIDAR thread for standalone mode
    lidar = None
    running = True

    def shutdown(*args):
        nonlocal running
        running = False
        follower.stop()
        if lidar:
            try:
                lidar.stop()
                lidar.stop_motor()
                lidar.disconnect()
            except:
                pass
        robot.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    print(f"\n{'=' * 55}")
    print(f"  WALL FOLLOWER - {side.upper()} side (standalone)")
    print(f"{'=' * 55}")

    # Start LIDAR
    lidar = RPLidar(LIDAR_PORT)
    print(f"[LIDAR] Connected: {lidar.get_info()['model']}")

    # Start follower
    follower.start()

    # Feed scans to follower
    try:
        for scan in lidar.iter_scans():
            if not running:
                break
            follower.update_scan(scan)
    except Exception as e:
        print(f"[LIDAR] Error: {e}")
    finally:
        shutdown()


if __name__ == '__main__':
    main()
