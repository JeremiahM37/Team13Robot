#!/usr/bin/env python3
"""
Wall follower simulator.

Runs the real WallFollower against a virtual robot in a virtual environment
(one straight wall to the right). No hardware needed. The simulator:

  1. Builds a synthetic 360 deg LIDAR scan from the virtual robot's pose.
  2. Hands the scan to WallFollower.update_scan().
  3. Waits a control tick and reads the drive() command the follower issued.
  4. Integrates differential-drive kinematics to advance the virtual pose.
  5. Either prints each tick as a log line (default) or redraws a live
     top-down ASCII map each tick (--graphic) so you can watch the robot
     make micro-adjustments in real time.

Usage:
    python sim_wall_follower.py                 # line-by-line log (default)
    python sim_wall_follower.py --graphic       # live top-down view (recommended)
    python sim_wall_follower.py --graphic --speed 0.2
    python sim_wall_follower.py --start 200     # start 200mm from origin
    python sim_wall_follower.py --angle 20      # start heading 20 deg into wall
    python sim_wall_follower.py --left          # follow a left wall instead
    python sim_wall_follower.py --duration 60   # run longer

Tip: if FORWARD_SPEED in wall_follower.py is very small (e.g. 0.005), the
     sim will show little motion over 30s. Pass --speed 0.2 to override for
     watching; the control logic is independent of the base speed.
"""

import io
import math
import sys
import time
import threading
from collections import deque

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
SIM_DT = LOOP_INTERVAL + 0.05
SIM_DURATION = 30.0


# -------- ANSI escape codes for the live graphic mode --------
ANSI_CLEAR = "\033[2J"
ANSI_HOME = "\033[H"
ANSI_HIDE_CUR = "\033[?25l"
ANSI_SHOW_CUR = "\033[?25h"
ANSI_ALT_SCR = "\033[?1049h"
ANSI_REG_SCR = "\033[?1049l"
ANSI_RESET = "\033[0m"
ANSI_DIM = "\033[2m"
ANSI_BOLD = "\033[1m"
ANSI_CYAN = "\033[36m"
ANSI_GREEN = "\033[32m"
ANSI_YELLOW = "\033[33m"
ANSI_RED = "\033[31m"
ANSI_BLUE = "\033[34m"


class MockRobot:
    """Records drive() commands from the follower so the sim can integrate pose."""

    def __init__(self):
        self._lock = threading.Lock()
        self._left = 0.0
        self._right = 0.0
        self.lidar = None

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


def generate_scan(robot_x, robot_y, heading_rad, wall_x):
    """360 deg scan against an infinite wall at x = wall_x (parallel to y-axis)."""
    scan = []
    for angle_deg in range(360):
        a_rad = math.radians(angle_deg)
        dx = math.cos(heading_rad - a_rad)
        if abs(dx) < 1e-6:
            continue
        t = (wall_x - robot_x) / dx
        if t <= 0 or t > MAX_RANGE_MM:
            continue
        scan.append((15, float(angle_deg), float(t)))
    return scan


# --------------------------------------------------------------------------
# Live top-down graphic
# --------------------------------------------------------------------------

MAP_W = 62
MAP_H = 20


class LiveGraphic:
    """
    Renders a top-down view of the simulation that refreshes in place.

    Coordinate convention:
      - World: +y is robot's initial forward direction, +x is to the robot's right.
      - Screen: +y = up, +x = right. Robot is drawn near screen center and the
        view rolls with it, so the wall appears as a fixed vertical line on the
        right (or left, for left-wall following) and the trail falls behind.
    """

    def __init__(self, wall_x, target_dist, side, out):
        self.wall_x = wall_x
        self.target_dist = target_dist
        self.side = side
        self.out = out
        self.trail = deque(maxlen=200)
        # X range: always show origin (0) to a bit past the wall
        if side == 'right':
            self.x_min, self.x_max = -100.0, wall_x + 100.0
        else:
            self.x_min, self.x_max = wall_x - 100.0, 100.0
        # mm per character row (y axis on screen)
        self.y_per_row = 120.0

    def setup(self):
        self.out.write(ANSI_ALT_SCR + ANSI_HIDE_CUR + ANSI_CLEAR + ANSI_HOME)
        self.out.flush()

    def teardown(self):
        self.out.write(ANSI_SHOW_CUR + ANSI_REG_SCR + ANSI_RESET)
        self.out.flush()

    def _col(self, x):
        return int((x - self.x_min) / (self.x_max - self.x_min) * (MAP_W - 1))

    def _row(self, y, robot_y):
        # Robot sits ~60% down the screen so past trail is below, future above.
        robot_row = int(MAP_H * 0.6)
        return robot_row - int((y - robot_y) / self.y_per_row)

    def render(self, rx, ry, hdg_rad, state, dist, cmds, t, travel):
        self.trail.append((rx, ry))

        # Build an empty grid of colored characters
        cell = [[' '] * MAP_W for _ in range(MAP_H)]
        color = [[''] * MAP_W for _ in range(MAP_H)]

        # Horizontal reference lines every Y_MARK_INTERVAL mm of world-y.
        # These scroll downward on screen as the robot moves in +y (forward),
        # giving unambiguous visual feedback that the robot IS traveling along
        # the wall even when the rolling viewport keeps the robot centered.
        Y_MARK_INTERVAL = 500
        robot_row = int(MAP_H * 0.6)
        y_top_screen = ry + robot_row * self.y_per_row
        y_bot_screen = ry - (MAP_H - 1 - robot_row) * self.y_per_row
        mark = (int(y_bot_screen) // Y_MARK_INTERVAL) * Y_MARK_INTERVAL
        while mark <= y_top_screen + Y_MARK_INTERVAL:
            r = self._row(mark, ry)
            if 0 <= r < MAP_H:
                for c in range(MAP_W):
                    if cell[r][c] == ' ':
                        cell[r][c] = '-'
                        color[r][c] = ANSI_DIM
                # Label the mark with its y-value on the side OPPOSITE the wall
                # so it doesn't collide with the wall/target columns.
                label = f"y={mark}"
                label_start = 1 if self.side == 'right' else (MAP_W - len(label) - 1)
                for i, ch in enumerate(label):
                    c = label_start + i
                    if 0 <= c < MAP_W:
                        cell[r][c] = ch
                        color[r][c] = ANSI_DIM
            mark += Y_MARK_INTERVAL

        # Wall (vertical line) - drawn after marks so it stays solid
        wall_col = self._col(self.wall_x)
        if 0 <= wall_col < MAP_W:
            for r in range(MAP_H):
                cell[r][wall_col] = '|'
                color[r][wall_col] = ANSI_RED

        # Target line (robot should settle here)
        if self.side == 'right':
            tgt_x = self.wall_x - self.target_dist
        else:
            tgt_x = self.wall_x + self.target_dist
        tgt_col = self._col(tgt_x)
        if 0 <= tgt_col < MAP_W:
            for r in range(0, MAP_H, 2):
                if cell[r][tgt_col] in (' ', '-'):
                    cell[r][tgt_col] = ':'
                    color[r][tgt_col] = ANSI_GREEN

        # Trail
        for x, y in self.trail:
            rr = self._row(y, ry)
            cc = self._col(x)
            if 0 <= rr < MAP_H and 0 <= cc < MAP_W and cell[rr][cc] == ' ':
                cell[rr][cc] = '.'
                color[rr][cc] = ANSI_DIM

        # Robot + heading arrow
        rr = self._row(ry, ry)
        cc = self._col(rx)
        if 0 <= rr < MAP_H and 0 <= cc < MAP_W:
            cell[rr][cc] = 'R'
            color[rr][cc] = ANSI_BOLD + ANSI_CYAN
            # Arrow in the next cell in the heading direction (rough 8-way)
            hdeg = math.degrees(hdg_rad) % 360
            arrows = [
                (0,   '>', 0, 1),   # +x
                (45,  '/', -1, 1),  # +x+y (up-right)
                (90,  '^', -1, 0),  # +y (up)
                (135, '\\', -1, -1),# -x+y (up-left)
                (180, '<', 0, -1),  # -x
                (225, '/', 1, -1),  # -x-y (down-left)
                (270, 'v', 1, 0),   # -y (down)
                (315, '\\', 1, 1),  # +x-y (down-right)
            ]
            # pick closest arrow
            best = min(arrows, key=lambda a: min(abs(hdeg - a[0]), 360 - abs(hdeg - a[0])))
            _, arrow, dr, dc = best
            ar, ac = rr + dr, cc + dc
            if 0 <= ar < MAP_H and 0 <= ac < MAP_W:
                cell[ar][ac] = arrow
                color[ar][ac] = ANSI_BOLD + ANSI_CYAN

        # --- Assemble frame ---
        buf = [ANSI_HOME]
        buf.append(f"{ANSI_BOLD}  WALL FOLLOWER — live view{ANSI_RESET}\n")
        buf.append(f"  {ANSI_DIM}wall = | (red)   target line = : (green)   robot = R (cyan)   trail = .{ANSI_RESET}\n\n")

        # Status strip
        err = dist - self.target_dist
        err_color = ANSI_GREEN if abs(err) < 50 else (ANSI_YELLOW if abs(err) < 200 else ANSI_RED)
        buf.append(f"  t = {t:5.1f}s   state = {state:<13s}   travel = {travel:5.0f} mm\n")
        buf.append(f"  dist to wall = {err_color}{dist:5.0f}{ANSI_RESET} mm   "
                   f"target = {self.target_dist} mm   "
                   f"error = {err_color}{err:+5.0f}{ANSI_RESET} mm\n")
        buf.append(f"  wheels  L = {cmds[0]:+.3f}   R = {cmds[1]:+.3f}   "
                   f"heading = {math.degrees(hdg_rad) % 360:5.1f}°   pos = ({rx:5.0f}, {ry:5.0f})\n")
        buf.append("\n")

        # Map box
        buf.append("  ┌" + "─" * MAP_W + "┐\n")
        for r in range(MAP_H):
            buf.append("  │")
            for c in range(MAP_W):
                if color[r][c]:
                    buf.append(color[r][c] + cell[r][c] + ANSI_RESET)
                else:
                    buf.append(cell[r][c])
            buf.append("│\n")
        buf.append("  └" + "─" * MAP_W + "┘\n")
        buf.append(f"  {ANSI_DIM}Ctrl-C to stop early{ANSI_RESET}\n")

        self.out.write(''.join(buf))
        self.out.flush()


# --------------------------------------------------------------------------
# CLI + main
# --------------------------------------------------------------------------

def parse_args():
    opts = {
        'speed': FORWARD_SPEED,
        'start_x': 300.0,
        'angle_deg': 0.0,
        'side': 'right',
        'duration': SIM_DURATION,
        'graphic': False,
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
        elif arg in ('--graphic', '-g'):
            opts['graphic'] = True
        elif arg in ('-h', '--help'):
            print(__doc__)
            sys.exit(0)
    return opts


def run_sim(opts, graphic=None):
    """Run the simulation. `graphic` is a LiveGraphic or None."""
    side = opts['side']
    wall_x = WALL_X if side == 'right' else -WALL_X
    robot_x = opts['start_x'] if side == 'right' else -opts['start_x']
    robot_y = 0.0
    tilt = math.radians(opts['angle_deg'])
    heading_rad = (math.pi / 2) - tilt if side == 'right' else (math.pi / 2) + tilt

    robot = MockRobot()
    follower = WallFollower(robot, side=side, speed=opts['speed'])
    follower.update_scan(generate_scan(robot_x, robot_y, heading_rad, wall_x))
    follower.start()

    n_ticks = int(opts['duration'] / SIM_DT)
    start_ry = robot_y

    if graphic is None:
        # Line-by-line log header
        print(f"{'t':>5} {'x':>6} {'y':>7} {'hdg':>6} {'dist':>6} "
              f"{'L':>7} {'R':>7} {'state':<13} map")
        print("-" * 80)

    try:
        for tick in range(n_ticks):
            follower.update_scan(generate_scan(robot_x, robot_y, heading_rad, wall_x))
            time.sleep(SIM_DT)

            vlc, vrc = robot.get_commanded()
            v_l = vlc * MAX_WHEEL_SPEED
            v_r = vrc * MAX_WHEEL_SPEED
            v = (v_l + v_r) / 2.0
            omega = (v_r - v_l) / WHEEL_BASE_MM

            robot_x += v * math.cos(heading_rad) * SIM_DT
            robot_y += v * math.sin(heading_rad) * SIM_DT
            heading_rad += omega * SIM_DT

            dist = abs(wall_x - robot_x)
            t = tick * SIM_DT
            travel = robot_y - start_ry

            if graphic is not None:
                # ~20 fps is plenty for this update rate
                graphic.render(robot_x, robot_y, heading_rad, follower.state,
                               dist, (vlc, vrc), t, travel)
            else:
                if tick % max(1, int(0.5 / SIM_DT)) == 0:
                    hdg = math.degrees(heading_rad) % 360
                    view = _inline_view(robot_x, wall_x, WALL_TARGET_MM, side)
                    print(f"{t:5.1f} {robot_x:6.0f} {robot_y:7.0f} "
                          f"{hdg:6.1f} {dist:6.0f} {vlc:+.3f} {vrc:+.3f} "
                          f"{follower.state:<13} {view}")
    except KeyboardInterrupt:
        pass
    finally:
        follower.stop()

    # Summary
    final_dist = abs(wall_x - robot_x)
    err = abs(final_dist - WALL_TARGET_MM)
    verdict = "CONVERGED" if err < 100 else ("CLOSE" if err < 200 else "DRIFTED")
    return {
        'final_dist': final_dist,
        'err': err,
        'verdict': verdict,
        'travel_y': robot_y - start_ry,
    }


def _inline_view(rx, wall_x, target, side, width=50):
    """Compact one-line map for the non-graphic log mode."""
    if side == 'right':
        lo, hi = 0.0, wall_x + 100.0
    else:
        lo, hi = wall_x - 100.0, max(rx, 0.0) + 500.0
    scale = (width - 1) / (hi - lo)
    cells = [' '] * width
    cells[0 if side == 'left' else -1] = '|'
    tgt = (wall_x - target) if side == 'right' else (wall_x + target)
    tp = int((tgt - lo) * scale)
    if 0 <= tp < width:
        cells[tp] = '.'
    rp = int((rx - lo) * scale)
    if 0 <= rp < width:
        cells[rp] = 'R' if cells[rp] == ' ' else '*'
    return '[' + ''.join(cells) + ']'


def main():
    opts = parse_args()
    side = opts['side']
    wall_x = WALL_X if side == 'right' else -WALL_X

    header = (
        "=" * 78 + "\n"
        "  WALL FOLLOWER SIMULATION\n"
        + "=" * 78 + "\n"
        f"  Wall (side={side:<5}):  x = {wall_x:.0f} mm\n"
        f"  Target distance:     {WALL_TARGET_MM} mm\n"
        f"  Start:               x = "
        + f"{opts['start_x'] if side == 'right' else -opts['start_x']:.0f}, "
        f"heading tilt = {opts['angle_deg']:+.1f} deg\n"
        f"  Forward speed:       {opts['speed']}"
    )
    if opts['speed'] < 0.05:
        header += "  <- WARNING: very slow, robot will barely move in 30s. Pass --speed 0.2 to see motion."
    header += f"\n  Duration:            {opts['duration']:.0f} s @ dt = {SIM_DT:.2f} s\n"
    header += "=" * 78

    if opts['graphic']:
        # In graphic mode: (1) redirect wall_follower's debug prints to a buffer
        # so they don't corrupt the live view, (2) set up alternate screen, run
        # the sim, tear down, (3) print the header + summary on the normal screen.
        real_stdout = sys.stdout
        silenced = io.StringIO()
        graphic = LiveGraphic(wall_x, WALL_TARGET_MM, side, real_stdout)
        sys.stdout = silenced
        graphic.setup()
        try:
            result = run_sim(opts, graphic=graphic)
        finally:
            graphic.teardown()
            sys.stdout = real_stdout

        print(header)
        print(
            f"  Final distance to wall: {result['final_dist']:.0f} mm  "
            f"(target {WALL_TARGET_MM}, error +{result['err']:.0f})  [{result['verdict']}]"
        )
        print(f"  Total forward travel:   {result['travel_y']:.0f} mm")
        print("=" * 78)
    else:
        print(header)
        result = run_sim(opts, graphic=None)
        print("-" * 78)
        print(
            f"  Final distance to wall: {result['final_dist']:.0f} mm  "
            f"(target {WALL_TARGET_MM}, error +{result['err']:.0f})  [{result['verdict']}]"
        )
        print(f"  Total forward travel:   {result['travel_y']:.0f} mm")
        print("=" * 78)


if __name__ == '__main__':
    main()
