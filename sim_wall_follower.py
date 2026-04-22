#!/usr/bin/env python3
"""
Wall follower simulator.

Runs the real WallFollower against a virtual robot in a virtual environment
(one straight wall to the right). No hardware needed. The simulator:

  1. Builds a synthetic 360 deg LIDAR scan from the virtual robot's pose.
  2. Hands the scan to WallFollower.update_scan().
  3. Waits a control tick and reads the drive() command the follower issued.
  4. Integrates differential-drive kinematics to advance the virtual pose.
  5. Prints a top-down ASCII view so you can watch the robot converge.

If the PD control logic is correct, the robot should settle near
WALL_TARGET_MM (default 600mm) from the wall and track it from there.

Usage:
    python sim_wall_follower.py                 # uses FORWARD_SPEED from wall_follower.py
    python sim_wall_follower.py --speed 0.2     # override for testing
    python sim_wall_follower.py --start 200     # start 200mm from origin (1300mm from wall)
    python sim_wall_follower.py --angle 20      # start heading 20 deg into the wall
    python sim_wall_follower.py --left          # follow a left wall instead
"""

import math
import sys
import time
import threading

from wall_follower import (
    WallFollower,
    WALL_TARGET_MM,
    FORWARD_SPEED,
    LOOP_INTERVAL,
)


# -------- World / vehicle parameters (tweak if your hardware differs) --------
WALL_X = 1500.0           # right wall: infinite line at x=1500mm
MAX_RANGE_MM = 4000.0
WHEEL_BASE_MM = 200.0     # left-to-right wheel spacing
MAX_WHEEL_SPEED = 250.0   # mm/s when wheel command = 1.0
SIM_DT = LOOP_INTERVAL + 0.05  # slightly longer than follower loop for safety
SIM_DURATION = 30.0       # seconds


class MockRobot:
    """Records drive() commands from the follower so the sim can integrate pose."""

    def __init__(self):
        self._lock = threading.Lock()
        self._left = 0.0
        self._right = 0.0
        self.lidar = None  # some control paths check this attribute

    def drive(self, left_speed, right_speed):
        with self._lock:
            self._left = max(-1.0, min(1.0, float(left_speed)))
            self._right = max(-1.0, min(1.0, float(right_speed)))
        return True

    def stop_wheels(self):
        with self._lock:
            self._left = 0.0
            self._right = 0.0

    def get_commanded(self):
        with self._lock:
            return self._left, self._right


def generate_scan(robot_x, robot_y, heading_rad, wall_x=WALL_X):
    """
    Build a 360 deg LIDAR scan from the robot's pose against an infinite wall
    at x = wall_x (parallel to y-axis).

    LIDAR angle convention (matches wall_follower.py and lidar_safety.py):
      0 deg  = forward
      90 deg = right   (clockwise from forward)
      180    = behind
      270    = left

    For ray angle `a` and robot heading `h` (radians, CCW from +x),
    world-frame ray direction = (cos(h - a), sin(h - a)).
    """
    scan = []
    for angle_deg in range(360):
        a_rad = math.radians(angle_deg)
        dx = math.cos(heading_rad - a_rad)
        if abs(dx) < 1e-6:
            continue  # ray parallel to wall, no hit
        t = (wall_x - robot_x) / dx
        if t <= 0 or t > MAX_RANGE_MM:
            continue
        scan.append((15, float(angle_deg), float(t)))
    return scan


def generate_scan_left_wall(robot_x, robot_y, heading_rad, wall_x):
    """Mirror for left-wall tests: wall at negative x."""
    return generate_scan(robot_x, robot_y, heading_rad, wall_x=wall_x)


def ascii_view(robot_x, wall_x, target_dist, side, width=50):
    """One-line top-down view: '|' = wall, '.' = target, 'R' = robot."""
    if side == 'right':
        lo, hi = 0.0, wall_x + 100.0
        wall_pos = width - 1
    else:
        lo, hi = wall_x - 100.0, max(robot_x, 0.0) + 500.0
        wall_pos = 0
    span = hi - lo
    scale = (width - 1) / span
    cells = [' '] * width
    cells[wall_pos] = '|'
    if side == 'right':
        tgt = (wall_x - target_dist) - lo
    else:
        tgt = (wall_x + target_dist) - lo
    tp = int(tgt * scale)
    if 0 <= tp < width:
        cells[tp] = '.'
    rp = int((robot_x - lo) * scale)
    if 0 <= rp < width:
        cells[rp] = 'R' if cells[rp] == ' ' else '*'
    return '[' + ''.join(cells) + ']'


def parse_args():
    opts = {
        'speed': FORWARD_SPEED,
        'start_x': 300.0,
        'angle_deg': 0.0,
        'side': 'right',
        'duration': SIM_DURATION,
    }
    it = iter(sys.argv[1:])
    for arg in it:
        if arg == '--speed':
            opts['speed'] = float(next(it))
        elif arg == '--start':
            opts['start_x'] = float(next(it))
        elif arg == '--angle':
            opts['angle_deg'] = float(next(it))
        elif arg == '--left':
            opts['side'] = 'left'
        elif arg == '--duration':
            opts['duration'] = float(next(it))
        elif arg in ('-h', '--help'):
            print(__doc__)
            sys.exit(0)
    return opts


def main():
    opts = parse_args()
    side = opts['side']

    # For left-wall: wall at x = -WALL_X (mirrors the right-wall scenario)
    wall_x = WALL_X if side == 'right' else -WALL_X
    # Starting pose: robot on the "near" side of wall, facing +y (along wall)
    robot_x = opts['start_x'] if side == 'right' else -opts['start_x']
    robot_y = 0.0
    # Heading: pi/2 (+y). Positive angle tilts TOWARD the wall.
    tilt = math.radians(opts['angle_deg'])
    heading_rad = (math.pi / 2) - tilt if side == 'right' else (math.pi / 2) + tilt

    print("=" * 78)
    print("  WALL FOLLOWER SIMULATION")
    print("=" * 78)
    print(f"  Wall (side={side:<5}):  x = {wall_x:.0f} mm")
    print(f"  Target distance:     {WALL_TARGET_MM} mm  "
          f"(robot should settle at x = {wall_x - WALL_TARGET_MM if side == 'right' else wall_x + WALL_TARGET_MM:.0f})")
    print(f"  Start:               x = {robot_x:.0f}, heading tilt = {opts['angle_deg']:+.1f} deg")
    print(f"  Forward speed:       {opts['speed']}  "
          f"{'(WARNING: likely too slow to visibly converge)' if opts['speed'] < 0.05 else ''}")
    print(f"  Duration:            {opts['duration']:.0f} s @ dt = {SIM_DT:.2f} s")
    print("=" * 78)
    print(f"{'t':>5} {'x':>6} {'y':>7} {'hdg':>6} {'dist':>6} "
          f"{'L':>6} {'R':>6} {'state':<13} map")
    print("-" * 78)

    robot = MockRobot()
    follower = WallFollower(robot, side=side, speed=opts['speed'])

    # Seed the follower with one scan before starting the thread so its first
    # tick has valid data (otherwise it starts in WALL_LOST).
    follower.update_scan(generate_scan(robot_x, robot_y, heading_rad, wall_x))
    follower.start()

    n_ticks = int(opts['duration'] / SIM_DT)
    for tick in range(n_ticks):
        follower.update_scan(generate_scan(robot_x, robot_y, heading_rad, wall_x))
        time.sleep(SIM_DT)  # let control thread produce a command

        v_left_cmd, v_right_cmd = robot.get_commanded()
        v_l = v_left_cmd * MAX_WHEEL_SPEED
        v_r = v_right_cmd * MAX_WHEEL_SPEED
        v = (v_l + v_r) / 2.0
        omega = (v_r - v_l) / WHEEL_BASE_MM  # rad/s, positive = CCW

        robot_x += v * math.cos(heading_rad) * SIM_DT
        robot_y += v * math.sin(heading_rad) * SIM_DT
        heading_rad += omega * SIM_DT

        if tick % max(1, int(0.5 / SIM_DT)) == 0:
            dist = abs(wall_x - robot_x)
            hdg = math.degrees(heading_rad) % 360
            view = ascii_view(robot_x, wall_x, WALL_TARGET_MM, side)
            print(f"{tick * SIM_DT:5.1f} {robot_x:6.0f} {robot_y:7.0f} "
                  f"{hdg:6.1f} {dist:6.0f} {v_left_cmd:+.2f} {v_right_cmd:+.2f} "
                  f"{follower.state:<13} {view}")

    follower.stop()

    # Summary: did it converge?
    final_dist = abs(wall_x - robot_x)
    err = abs(final_dist - WALL_TARGET_MM)
    verdict = "CONVERGED" if err < 100 else ("CLOSE" if err < 200 else "DRIFTED")
    print("-" * 78)
    print(f"  Final distance to wall: {final_dist:.0f} mm  "
          f"(target {WALL_TARGET_MM}, error {err:+.0f})  [{verdict}]")
    print(f"  Total forward travel:   {robot_y:.0f} mm")
    print("=" * 78)


if __name__ == '__main__':
    main()
