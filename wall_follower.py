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
from collections import deque
from robot_control import RobotController

# ==================== CONFIGURATION ====================

LIDAR_PORT = '/dev/ttyUSB0'

# Wall following parameters (mm)
WALL_TARGET_MM = 750       # Ideal distance from wall (~60cm, good for 2m hallway)
WALL_TOO_CLOSE_MM = 730    # Steer away below this
WALL_TOO_FAR_MM = 780      # Steer toward above this
WALL_LOST_MM = 2000        # Wall considered lost above this
FRONT_STOP_MM = 600        # Stop if obstacle closer than this in front

# Speed settings (0.0 to 1.0)
FORWARD_SPEED = 0.4         # Base forward speed (was 0.2)
TURN_SPEED = 0.1            # Turn-in-place speed when front is blocked (was 0.3)
MAX_STEER_CORRECTION = 0.35  # Cap on PD steering magnitude
SEARCH_TURN_SPEED = 0.2    # Turn speed when searching for lost wall

# PD gains for two-ray wall following. Error terms are in mm; gains convert
# to the 0-1 speed scale. Increase for snappier tracking, decrease if oscillating.
WALL_KP = 0.0008            # Distance error gain (wall - target) (was 0.0008)
WALL_KD = 0.0006            # Angle error gain (forward_ray - back_ray) (was 0.0006)

# Smoothing: median of last N scans per zone (rejects outliers)
SCAN_HISTORY = 1

# LIDAR: 0°=front, 90°=RIGHT, 180°=rear, 270°=LEFT
# Zones are narrow, symmetric rays around perpendicular. The two off-perpendicular
# rays (front_wall, back_wall) compare lengths to infer the wall's angle relative
# to the robot — the D-term of the PD controller.
#   front_wall (forward ray)   hits wall ahead of perpendicular
#   wall       (perpendicular) gives distance to wall
#   back_wall  (rearward ray)  hits wall behind perpendicular
# When parallel: front_ray ≈ back_ray. Angled into wall: front < back. Angled away: front > back.
ZONES_RIGHT = {
    'front':       (340, 20),     # front cone (collision avoidance)
    'front_wall':  (45, 75),      # forward-right ray  (centered 60°)
    'wall':        (75, 105),     # perpendicular right (centered 90°)
    'back_wall':   (105, 135),    # back-right ray     (centered 120°)
}

ZONES_LEFT = {
    'front':       (340, 20),     # front cone
    'front_wall':  (285, 315),    # forward-left ray   (centered 300°)
    'wall':        (255, 285),    # perpendicular left (centered 270°)
    'back_wall':   (225, 255),    # back-left ray      (centered 240°)
}

LOOP_INTERVAL = 0.1


# ==================== HELPER ====================

def angle_in_zone(angle, zone_start, zone_end):
    if zone_start <= zone_end:
        return zone_start <= angle <= zone_end
    else:
        return angle >= zone_start or angle <= zone_end


def is_body_return(angle, distance):
    """
    True if a scan point should be rejected as a reflection off the robot's
    own body. The right side protrudes further than the left, so the cutoff
    is larger in the right-side arc (45°-135°).
    """
    if 45 <= angle <= 135:
        return distance < 200
    return distance < 100


def safe_round(val):
    return round(val) if val != float('inf') else 9999


# ==================== WALL FOLLOWER ====================

class WallFollower:
    """
    Autonomous wall following using LIDAR and differential drive.

    Can run standalone (with its own LIDAR thread) or integrated into
    app.py (receiving scan data from a shared LIDAR instance).
    """

    def __init__(self, robot, side='auto', speed=FORWARD_SPEED):
        """
        Args:
            robot: RobotController instance (shared with app.py)
            side: 'right', 'left', or 'auto' (detect closest wall)
            speed: base forward speed (0.0 to 1.0)
        """
        self.robot = robot
        self.side = side
        
        self._auto_detect = (side == 'auto')
        self._side_detected = False
        # Default to right zones until auto-detect picks a side
        self.zones = ZONES_RIGHT if side != 'left' else ZONES_LEFT
        self.forward_speed = speed

        self._lock = threading.Lock()
        self.distances = {name: float('inf') for name in self.zones}
        # Rolling history of last SCAN_HISTORY readings per zone (for smoothing)
        self._history = {name: deque(maxlen=SCAN_HISTORY) for name in self.zones}
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
        # If auto-detecting, collect readings from both sides over several scans
        if self._auto_detect and not self._side_detected:
            left_min = float('inf')
            right_min = float('inf')
            for quality, angle, distance in scan:
                if quality == 0 or distance == 0:
                    continue
                if is_body_return(angle, distance):
                    continue
                if angle_in_zone(angle, 60, 120):
                    right_min = min(right_min, distance)
                if angle_in_zone(angle, 240, 300):
                    left_min = min(left_min, distance)

            # Track best readings across scans
            if not hasattr(self, '_auto_left_best'):
                self._auto_left_best = float('inf')
                self._auto_right_best = float('inf')
                self._auto_scan_count = 0

            if left_min != float('inf'):
                self._auto_left_best = min(self._auto_left_best, left_min)
            if right_min != float('inf'):
                self._auto_right_best = min(self._auto_right_best, right_min)
            self._auto_scan_count += 1

            # Decide after 5 scans (gives LIDAR time for full rotations)
            if self._auto_scan_count >= 5:
                if self._auto_left_best <= self._auto_right_best:
                    self.side = 'left'
                    self.zones = ZONES_LEFT
                else:
                    self.side = 'right'
                    self.zones = ZONES_RIGHT
                self._side_detected = True
                self._history = {name: deque(maxlen=SCAN_HISTORY) for name in self.zones}
                self.distances = {name: float('inf') for name in self.zones}
                print(f"[WALL] Auto-detected: following {self.side.upper()} wall "
                      f"(L:{safe_round(self._auto_left_best)}mm R:{safe_round(self._auto_right_best)}mm)")

        self._last_scan_time = time.time()
        zone_mins = {name: float('inf') for name in self.zones}
        for quality, angle, distance in scan:
            if quality == 0 or distance == 0:
                continue
            if is_body_return(angle, distance):
                continue
            for zone_name, (start, end) in self.zones.items():
                if angle_in_zone(angle, start, end):
                    zone_mins[zone_name] = min(zone_mins[zone_name], distance)

        # Smoothing: store this scan, return median across last N scans
        with self._lock:
            for name, val in zone_mins.items():
                # Only store valid readings to avoid pulling everything to inf
                if val != float('inf'):
                    self._history[name].append(val)
                # Use median of history (rejects outliers); fall back to inf if empty
                if self._history[name]:
                    sorted_vals = sorted(self._history[name])
                    self.distances[name] = sorted_vals[len(sorted_vals) // 2]
                else:
                    self.distances[name] = float('inf')

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
        self._last_scan_time = time.time()

        while self._running:
            dist = self._get_distances()
            front = dist['front']
            wall = dist['wall']

            # Safety: if no new scan data for 0.5s (LIDAR disconnected), stop
            if time.time() - self._last_scan_time > 0.5:
                self.robot.stop_wheels()
                self.state = 'NO_LIDAR'
                if self.state != prev_state:
                    print(f"\n[WALL] NO LIDAR DATA - stopped")
                    prev_state = self.state
                time.sleep(LOOP_INTERVAL)
                continue

            left_speed = 0.0
            right_speed = 0.0
            front_wall = dist.get('front_wall', float('inf'))
            back_wall = dist.get('back_wall', float('inf'))

            # Case 1: Obstacle in front - STOP, turn in place away from wall
            if front < FRONT_STOP_MM:
                self.state = 'FRONT_BLOCKED'
                if self.side == 'right':
                    left_speed = -TURN_SPEED
                    right_speed = TURN_SPEED
                else:
                    left_speed = TURN_SPEED
                    right_speed = -TURN_SPEED

            # Case 4: Wall lost - drive forward slowly, slight turn toward wall
            elif wall > WALL_LOST_MM:
                self.state = 'WALL_LOST'
                search = 0.4
                if self.side == 'right':
                    left_speed = self.forward_speed
                    right_speed = self.forward_speed - search
                else:
                    left_speed = self.forward_speed - search
                    right_speed = self.forward_speed

            # Two-ray PD wall following.
            # Distance error: + means too far from wall, - means too close.
            # Angle error (front_ray - back_ray): + means heading away from wall,
            # - means heading into wall. Combined, they give symmetric micro-adjustments.
            else:
                distance_error = wall - WALL_TARGET_MM

                if front_wall != float('inf') and back_wall != float('inf'):
                    angle_error = front_wall - back_wall
                else:
                    angle_error = 0.0  # fall back to P-only when a ray is missing

                # Steering sign convention: positive = turn TOWARD the followed wall.
                turn_toward_wall = WALL_KP * distance_error + WALL_KD * angle_error
                turn_toward_wall = max(-MAX_STEER_CORRECTION,
                                       min(MAX_STEER_CORRECTION, turn_toward_wall)) 

                if abs(distance_error) < 50:
                    self.state = 'FOLLOWING'
                elif distance_error < 0:
                    self.state = 'TOO_CLOSE'
                else:
                    self.state = 'TOO_FAR'

                # Right-wall: toward-wall = rotate clockwise = left faster than right.
                # Left-wall:  toward-wall = rotate counter-clockwise = right faster.
                turn = turn_toward_wall if self.side == 'right' else -turn_toward_wall
                left_speed = self.forward_speed + turn
                right_speed = self.forward_speed - turn

            # Clamp: never go backward except FRONT_BLOCKED
            if self.state != 'FRONT_BLOCKED':
                left_speed = max(0.0, left_speed) #left_speed
                right_speed = max(0.0, right_speed) #right_speed

            self.robot.drive(left_speed, right_speed) #TODO: Fix turn function

            # Debug: always print distances so we can see if data is flowing
            front_wall = dist.get('front_wall', float('inf'))
            back_wall = dist.get('back_wall', float('inf'))
            if self.state != prev_state:
                print(f"\n[WALL] STATE: {self.state:15s} | Front: {safe_round(front):5d}mm | "
                      f"Wall: {safe_round(wall):5d}mm | FW: {safe_round(front_wall):5d}mm | "
                      f"BW: {safe_round(back_wall):5d}mm | L={left_speed:+.2f} R={right_speed:+.2f}")
                prev_state = self.state
            else:
                print(f"[WALL] {self.state:15s} | F:{safe_round(front):5d} W:{safe_round(wall):5d} "
                      f"FW:{safe_round(front_wall):5d} BW:{safe_round(back_wall):5d} "
                      f"L={left_speed:+.2f} R={right_speed:+.2f}", end='\r')

            time.sleep(LOOP_INTERVAL)

        self.robot.stop_wheels()


# ==================== STANDALONE MODE ====================

def main():
    """Run wall follower as a standalone script with its own LIDAR."""
    if '--left' in sys.argv:
        side = 'left'
    elif '--auto' in sys.argv:
        side = 'auto'
    else:
        side = 'right'
    simulation_mode = '--sim' in sys.argv

    speed = FORWARD_SPEED
    if '--speed' in sys.argv:
        idx = sys.argv.index('--speed')
        if idx + 1 < len(sys.argv):
            try:
                speed = max(0.0, min(1.0, float(sys.argv[idx + 1])))
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

    # Start LIDAR (retry if connection is flaky)
    lidar = None
    for attempt in range(5):
        try:
            lidar = RPLidar(LIDAR_PORT)
            lidar.clean_input()
            time.sleep(0.5)
            lidar.clean_input()
            print(f"[LIDAR] Connected: {lidar.get_info()['model']}")
            break
        except Exception as e:
            print(f"[LIDAR] Attempt {attempt+1} failed: {e}")
            if lidar:
                try:
                    lidar.disconnect()
                except:
                    pass
                lidar = None
            time.sleep(1)
    if lidar is None:
        print("[LIDAR] Could not connect after 5 attempts")
        robot.close()
        sys.exit(1)

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
