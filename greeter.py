#!/usr/bin/env python3
"""
Autonomous Robot Greeter - CSCI 455 Final Project

The robot acts as a hallway greeter assistant using a Finite State Machine.
It detects a human, greets them, accepts a voice command (bathroom or robot lab),
then navigates autonomously using LIDAR wall-following to guide them.

FSM States:
  WAITING         - Idle, scanning LIDAR for approaching human
  GREETING        - Human detected, robot speaks greeting
  LISTENING       - Waiting for voice command (bathroom or lab)
  TURNING_AROUND  - Rotate 180 degrees to face hallway
  WALL_FOLLOWING  - Hand off to WallFollower (left for bathroom, right for lab)
  TURNING_TO_DESTINATION          TODO: (Turns in place in the intersection.
  FINAL_APPROACH                  TODO: (Final approach to the destination. This is time-based.)
  STOPPED         - Announce arrival, done

LIDAR Zone:
  - Front: 340-20 degrees (only used here for human detection;
           wall_follower defines its own zones for navigation)

Speech Recognition:
  Uses Google Speech Recognition (requires internet via hotspot/MSU-Guest).
  Falls back to keyboard input with --keyboard flag (minus 15 points).

Usage (standalone):
    python greeter.py                # Full mode with speech recognition
    python greeter.py --keyboard     # Keyboard input (no speech)
    python greeter.py --sim          # Simulation mode (no hardware)

Can also be started/stopped from the Flask web interface via app.py.
"""

import sys
import signal
import time
import threading
import subprocess
from collections import deque
from rplidar import RPLidar
from robot_control import RobotController
from wall_follower import WallFollower

# ==================== CONFIGURATION ====================

LIDAR_PORT = '/dev/ttyUSB0'

# Human detection
HUMAN_DETECT_MM = 1000

# Speed/timing for the 180° turn-around step
TURN_SPEED = 0.7
TURN_180_DURATION = 2.5    # Tune this on real robot!

# Wall-follow durations per destination (seconds). The timer starts AFTER the
# robot has turned the corner into the doorway, so this is just the post-corner
# drive distance — bathroom and lab are both close to their doorways.
BATHROOM_FOLLOW_DURATION = 5.0
LAB_FOLLOW_DURATION = 1.0

# Corner-turn detection (uses perpendicular wall distance from WallFollower).
# WALL_PRESENT_MM:    wall is "acquired" once perpendicular ray is below this
# WALL_OPENING_MM:    doorway opening declared once perpendicular ray exceeds this
# CORNER_TURN_TIMEOUT: max seconds to wait for the corner turn before assuming
#                     it completed (fallback when wall isn't reacquired in the
#                     destination room because it has no near-side wall)
WALL_PRESENT_MM = 1200
WALL_OPENING_MM = 1500
CORNER_TURN_TIMEOUT = 4.0

LOOP_INTERVAL = 0.1

# Smoothing: median of last N scans for human detection
SCAN_HISTORY = 1

# Only the front cone is used here (human detection). Wall-follow zones
# live in wall_follower.py.
ZONES = {
    'front': (340, 20),  # LIDAR 0° = forward
}


# ==================== HELPERS ====================

def angle_in_zone(angle, zone_start, zone_end):
    if zone_start <= zone_end:
        return zone_start <= angle <= zone_end
    else:
        return angle >= zone_start or angle <= zone_end


def safe_round(val):
    return round(val) if val != float('inf') else -1


def speak_and_wait(text):
    """Speak text and wait for it to finish."""
    print(f"[SPEAK] {text}")
    try:
        subprocess.run(
            ['espeak', '-v', 'en-us', '-s', '150', text],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            timeout=10
        )
    except Exception as e:
        print(f"[SPEAK ERROR] {e}")


def listen_for_command(use_keyboard=False):
    """
    Listen for a voice command and parse it.
    Returns 'bathroom' or 'lab' or None if not understood.

    Option A (full credit): Google Speech Recognition (needs internet).
    Option B (minus 15):    Keyboard input with --keyboard flag.
    """
    if use_keyboard:
        print("\n[INPUT] Type 'bathroom' or 'lab': ", end='', flush=True)
        try:
            text = input().strip().lower()
        except (EOFError, KeyboardInterrupt):
            return None
        if 'bathroom' in text or 'restroom' in text:
            return 'bathroom'
        elif 'lab' in text or 'robot' in text:
            return 'lab'
        else:
            print(f"[INPUT] Did not understand: '{text}'")
            return None

    # Speech recognition (Option A) - uses PulseAudio (device_index=5) for Bluetooth mic
    try:
        import speech_recognition as sr
        recognizer = sr.Recognizer()
        with sr.Microphone(device_index=5) as source:
            print("[LISTEN] Adjusting for ambient noise...")
            recognizer.adjust_for_ambient_noise(source, duration=1)
            print("[LISTEN] Listening... say 'bathroom' or 'robot lab'")
            audio = recognizer.listen(source, timeout=10, phrase_time_limit=5)

        print("[LISTEN] Processing speech...")
        text = recognizer.recognize_google(audio).lower()
        print(f"[LISTEN] Heard: '{text}'")

        if 'bathroom' in text or 'restroom' in text:
            return 'bathroom'
        elif 'lab' in text or 'robot' in text:
            return 'lab'
        else:
            print(f"[LISTEN] Could not parse destination from: '{text}'")
            return None

    except Exception as e:
        print(f"[LISTEN] Speech recognition error: {e}")
        return None


# ==================== GREETER FSM ====================

class RobotGreeter:
    """
    Finite State Machine for the autonomous greeter robot.

    State transitions:
        WAITING -> GREETING            (human detected in front)
        GREETING -> LISTENING          (after greeting spoken)
        LISTENING -> TURNING_AROUND    (valid command received)
        TURNING_AROUND -> WALL_FOLLOWING (180 turn complete; spawn WallFollower)
        TURNING_TO_DESTINATION          TODO: (Turns in place in the intersection.
        FINAL_APPROACH                  TODO: (Final approach to the destination. This is time-based.)
        WALL_FOLLOWING -> STOPPED      (per-destination duration elapsed)
    """

    WAITING = 'WAITING'
    GREETING = 'GREETING'
    LISTENING = 'LISTENING'
    TURNING_AROUND = 'TURNING_AROUND'
    WALL_FOLLOWING = 'WALL_FOLLOWING'
    TURNING_TO_DESTINATION = 'TURNING_TO_DESTINATION'
    FINAL_APPROACH = 'FINAL_APPROACH'
    STOPPED = 'STOPPED'

    def __init__(self, robot, use_keyboard=False):
        """
        Args:
            robot: RobotController instance (shared with app.py)
            use_keyboard: if True, use keyboard instead of speech recognition
        """
        self.robot = robot
        self.use_keyboard = use_keyboard

        self.state = self.WAITING
        self.destination = None
        self.state_start_time = time.time()
        self._wall_follower = None
        # Corner-detection bookkeeping for WALL_FOLLOWING. We watch the
        # perpendicular-wall distance reported by the WallFollower:
        #   1) wall acquired — perpendicular < WALL_PRESENT_MM (we're tracking)
        #   2) opening seen  — perpendicular > WALL_OPENING_MM (doorway visible)
        #   3) corner turned — perpendicular drops back below WALL_PRESENT_MM
        #                       OR CORNER_TURN_TIMEOUT seconds elapse since (2)
        # The destination drive timer only starts after step 3.
        self._wf_acquired = False
        self._wf_opened = False
        self._opening_time = None
        self._corner_turn_time = None

        self._lock = threading.Lock()
        self.distances = {name: float('inf') for name in ZONES}
        # Rolling history of last SCAN_HISTORY readings per zone (smoothing)
        self._history = {name: deque(maxlen=SCAN_HISTORY) for name in ZONES}
        self._running = False
        self._thread = None

    @property
    def active(self):
        return self._running

    def start(self):
        """Start greeter FSM in a background thread."""
        if self._running:
            return
        self._running = True
        self.state = self.WAITING
        self.destination = None
        self._thread = threading.Thread(target=self._fsm_loop, daemon=True)
        self._thread.start()
        print("[GREETER] Started - waiting for human")

    def stop(self):
        """Stop greeter FSM."""
        self._running = False
        if self._wall_follower:
            self._wall_follower.stop()
            self._wall_follower = None
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        self.robot.stop_wheels()
        self.state = 'IDLE'
        print("[GREETER] Stopped")

    def update_scan(self, scan):
        """Receive a LIDAR scan from an external source. Applies median smoothing."""
        # Forward to internal wall follower if running
        if self._wall_follower and self._wall_follower.active:
            self._wall_follower.update_scan(scan)

        zone_mins = {name: float('inf') for name in ZONES}
        for quality, angle, distance in scan:
            if quality == 0 or distance == 0:
                continue
            # Ignore readings under 200mm - robot's own body
            if distance < 200:
                continue
            for zone_name, (start, end) in ZONES.items():
                if angle_in_zone(angle, start, end):
                    zone_mins[zone_name] = min(zone_mins[zone_name], distance)

        # Smoothing: store this scan, expose median across last N scans
        with self._lock:
            for name, val in zone_mins.items():
                if val != float('inf'):
                    self._history[name].append(val)
                if self._history[name]:
                    sorted_vals = sorted(self._history[name])
                    self.distances[name] = sorted_vals[len(sorted_vals) // 2]
                else:
                    self.distances[name] = float('inf')

    def get_status(self):
        """Get current greeter status."""
        with self._lock:
            d = self.distances.copy()
        return {
            'active': self._running,
            'state': self.state,
            'destination': self.destination,
            'front': safe_round(d.get('front', -1)),
        }

    def _get_distances(self):
        with self._lock:
            return self.distances.copy()

    def _set_state(self, new_state):
        print(f"\n[FSM] {self.state} -> {new_state}")
        self.state = new_state
        self.state_start_time = time.time()

    def _state_elapsed(self):
        return time.time() - self.state_start_time

    # ==================== FSM LOOP ====================

    def _fsm_loop(self):
        """Main FSM loop."""
        while self._running:
            dist = self._get_distances()

            if self.state == self.WAITING:
                self._handle_waiting(dist)
            elif self.state == self.GREETING:
                self._handle_greeting(dist)
            elif self.state == self.LISTENING:
                self._handle_listening(dist)
            elif self.state == self.TURNING_AROUND:
                self._handle_turning_around(dist)
            elif self.state == self.WALL_FOLLOWING:
                self._handle_wall_following(dist)
            elif self.state == self.STOPPED:
                self._handle_stopped(dist)

            time.sleep(LOOP_INTERVAL)

    # ==================== STATE HANDLERS ====================

    def _handle_waiting(self, dist):
        """WAITING: Scan for human approaching from front."""
        if dist['front'] < HUMAN_DETECT_MM:
            print(f"\n[GREETER] Human detected at {safe_round(dist['front'])}mm!")
            self._set_state(self.GREETING)

    def _handle_greeting(self, dist):
        """GREETING: Speak greeting, transition to LISTENING."""
        speak_and_wait("Hello, how can I help you?")
        self._set_state(self.LISTENING)

    def _handle_listening(self, dist):
        """LISTENING: Wait for voice/keyboard command."""
        command = listen_for_command(use_keyboard=self.use_keyboard)
        if command is not None:
            self.destination = command
            print(f"\n[GREETER] Destination: {self.destination}")
            speak_and_wait("Follow me.")
            self._set_state(self.TURNING_AROUND)
        else:
            speak_and_wait("Sorry, I did not understand. Please say bathroom or robot lab.")

    def _handle_turning_around(self, dist):
        """TURNING_AROUND: Rotate 180 degrees, then start the wall follower."""
        if self._state_elapsed() < TURN_180_DURATION:
            self.robot.drive(-TURN_SPEED, TURN_SPEED)
        else:
            self.robot.stop_wheels()
            # bathroom is on the left, robot lab is on the right
            side = 'left' if self.destination == 'bathroom' else 'right'
            print(f"[GREETER] Starting wall follower on {side.upper()} side")
            self._wall_follower = WallFollower(self.robot, side=side)
            self._wall_follower.start()
            self._wf_acquired = False
            self._wf_opened = False
            self._opening_time = None
            self._corner_turn_time = None
            self._set_state(self.WALL_FOLLOWING)

    def _handle_wall_following(self, dist):
        """WALL_FOLLOWING: WallFollower owns the drive loop. We watch the
        perpendicular-wall distance to detect the corner turn at the doorway:
          acquired (< WALL_PRESENT_MM) -> opening (> WALL_OPENING_MM)
          -> corner turned (back below WALL_PRESENT_MM, OR timeout)
        Then we count down the per-destination duration and stop.
        """
        if not self._wall_follower:
            return

        wall = self._wall_follower.distances.get('wall', float('inf'))

        # Step 1: confirm we acquired a wall before looking for an opening
        if not self._wf_acquired:
            if wall < WALL_PRESENT_MM:
                self._wf_acquired = True
                print(f"[GREETER] Wall acquired at {wall:.0f}mm - watching for doorway")
            return

        # Step 2: detect doorway opening (perpendicular jumps far)
        if not self._wf_opened:
            if wall > WALL_OPENING_MM:
                self._wf_opened = True
                self._opening_time = time.time()
                print(f"[GREETER] Doorway opening detected (wall at {wall:.0f}mm) - turning corner")
            return

        # Step 3: wait for corner turn to complete:
        #   wall reacquired (back below WALL_PRESENT_MM), OR
        #   CORNER_TURN_TIMEOUT seconds elapsed (doorway has no near wall)
        if self._corner_turn_time is None:
            elapsed = time.time() - self._opening_time
            reacquired = wall < WALL_PRESENT_MM
            if reacquired or elapsed >= CORNER_TURN_TIMEOUT:
                self._corner_turn_time = time.time()
                duration = (BATHROOM_FOLLOW_DURATION if self.destination == 'bathroom'
                            else LAB_FOLLOW_DURATION)
                reason = f"wall reacquired at {wall:.0f}mm" if reacquired else f"timeout {CORNER_TURN_TIMEOUT}s"
                print(f"[GREETER] Corner turned ({reason}) - driving {duration}s more then stopping")
            return

        # Step 4: count down the post-corner drive duration
        duration = (BATHROOM_FOLLOW_DURATION if self.destination == 'bathroom'
                    else LAB_FOLLOW_DURATION)
        if time.time() - self._corner_turn_time >= duration:
            print(f"[GREETER] Drive duration ({duration}s) elapsed - stopping")
            self._wall_follower.stop()
            self._wall_follower = None
            self.robot.stop_wheels()
            self._set_state(self.STOPPED)

    def _handle_stopped(self, dist):
        """STOPPED: Announce arrival."""
        if self.destination == 'bathroom':
            speak_and_wait("We have arrived at the bathroom.")
        else:
            speak_and_wait("We have arrived at the robot lab.")

        print(f"\n[GREETER] MISSION COMPLETE - arrived at {self.destination}")
        self._running = False


# ==================== STANDALONE MODE ====================

def main():
    """Run greeter as a standalone script with its own LIDAR."""
    use_keyboard = '--keyboard' in sys.argv
    simulation_mode = '--sim' in sys.argv

    robot = RobotController(simulation_mode=simulation_mode)
    greeter = RobotGreeter(robot, use_keyboard=use_keyboard)

    lidar = None
    running = True

    def shutdown(*args):
        nonlocal running
        running = False
        greeter.stop()
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
    print(f"  AUTONOMOUS ROBOT GREETER (standalone)")
    print(f"  Speech: {'KEYBOARD' if use_keyboard else 'VOICE'}")
    print(f"{'=' * 55}")

    lidar = RPLidar(LIDAR_PORT)
    lidar.clean_input()
    print(f"[LIDAR] Connected: {lidar.get_info()['model']}")

    greeter.start()

    try:
        for scan in lidar.iter_scans():
            if not running or not greeter.active:
                break
            greeter.update_scan(scan)
    except Exception as e:
        print(f"[LIDAR] Error: {e}")
    finally:
        shutdown()


if __name__ == '__main__':
    main()
