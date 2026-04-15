#!/usr/bin/env python3
"""
Autonomous Robot Greeter - CSCI 455 Final Project

The robot acts as a hallway greeter assistant using a Finite State Machine.
It detects a human, greets them, accepts a voice command (bathroom or robot lab),
then navigates autonomously using LIDAR wall-following to guide them.

FSM States:
  WAITING              - Idle, scanning LIDAR for approaching human
  GREETING             - Human detected, robot speaks greeting
  LISTENING            - Waiting for voice command (bathroom or lab)
  TURNING_AROUND       - Rotate 180 degrees to face hallway
  ALIGNING_TO_HALLWAY  - Move forward until walls detected on both sides
  MOVING_TO_T          - Wall-follow down hallway toward T-intersection
  TURNING_TO_DEST      - Turn left (lab) or right (bathroom) at T
  FINAL_MOVEMENT       - Drive straight for ~5 seconds after turn
  STOPPED              - Announce arrival, done

LIDAR Zones:
  - Front:       340-20 degrees   (obstacle detection + human detection)
  - Front-left:  20-70 degrees    (left wall leading edge)
  - Left:        70-110 degrees   (left wall distance)
  - Front-right: 290-340 degrees  (right wall leading edge)
  - Right:       250-290 degrees  (right wall distance)

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
from rplidar import RPLidar
from robot_control import RobotController

# ==================== CONFIGURATION ====================

LIDAR_PORT = '/dev/ttyUSB0'

# Human detection
HUMAN_DETECT_MM = 1000

# Wall following parameters (mm)
WALL_TARGET_MM = 500
WALL_TOO_CLOSE_MM = 350
WALL_TOO_FAR_MM = 650
FRONT_STOP_MM = 400

# T-intersection detection
T_FRONT_WALL_MM = 500
T_SIDE_OPEN_MM = 1200

# Speed settings
FORWARD_SPEED = 0.35
TURN_SPEED = 0.4
STEER_CORRECTION = 0.15
TURN_180_DURATION = 3.0    # Tune this on real robot!
TURN_90_DURATION = 1.5     # Tune this on real robot!
FINAL_DRIVE_DURATION = 5.0

LOOP_INTERVAL = 0.1

# LIDAR zones
ZONES = {
    'front':       (340, 20),
    'front_left':  (20, 70),
    'left':        (70, 110),
    'front_right': (290, 340),
    'right':       (250, 290),
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

    # Speech recognition (Option A)
    try:
        import speech_recognition as sr
        recognizer = sr.Recognizer()
        with sr.Microphone() as source:
            print("[LISTEN] Adjusting for ambient noise...")
            recognizer.adjust_for_ambient_noise(source, duration=1)
            print("[LISTEN] Listening for command...")
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
        WAITING -> GREETING           (human detected in front)
        GREETING -> LISTENING         (after greeting spoken)
        LISTENING -> TURNING_AROUND   (valid command received)
        TURNING_AROUND -> ALIGNING    (180 turn complete)
        ALIGNING -> MOVING_TO_T       (walls on both sides detected)
        MOVING_TO_T -> TURNING_TO_DEST (T-intersection detected)
        TURNING_TO_DEST -> FINAL_MOVE (turn complete)
        FINAL_MOVE -> STOPPED         (5 seconds elapsed)
    """

    WAITING = 'WAITING'
    GREETING = 'GREETING'
    LISTENING = 'LISTENING'
    TURNING_AROUND = 'TURNING_AROUND'
    ALIGNING = 'ALIGNING_TO_HALLWAY'
    MOVING_TO_T = 'MOVING_TO_T'
    TURNING_TO_DEST = 'TURNING_TO_DESTINATION'
    FINAL_MOVE = 'FINAL_MOVEMENT'
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

        self._lock = threading.Lock()
        self.distances = {name: float('inf') for name in ZONES}
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
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        self.robot.stop_wheels()
        self.state = 'IDLE'
        print("[GREETER] Stopped")

    def update_scan(self, scan):
        """Receive a LIDAR scan from an external source."""
        zone_mins = {name: float('inf') for name in ZONES}
        for quality, angle, distance in scan:
            if quality == 0 or distance == 0:
                continue
            for zone_name, (start, end) in ZONES.items():
                if angle_in_zone(angle, start, end):
                    zone_mins[zone_name] = min(zone_mins[zone_name], distance)
        with self._lock:
            self.distances = zone_mins

    def get_status(self):
        """Get current greeter status."""
        with self._lock:
            d = self.distances.copy()
        return {
            'active': self._running,
            'state': self.state,
            'destination': self.destination,
            'front': safe_round(d.get('front', -1)),
            'left': safe_round(d.get('left', -1)),
            'right': safe_round(d.get('right', -1)),
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
            elif self.state == self.ALIGNING:
                self._handle_aligning(dist)
            elif self.state == self.MOVING_TO_T:
                self._handle_moving_to_t(dist)
            elif self.state == self.TURNING_TO_DEST:
                self._handle_turning_to_dest(dist)
            elif self.state == self.FINAL_MOVE:
                self._handle_final_move(dist)
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
        """TURNING_AROUND: Rotate 180 degrees."""
        if self._state_elapsed() < TURN_180_DURATION:
            self.robot.drive(-TURN_SPEED, TURN_SPEED)
        else:
            self.robot.stop_wheels()
            self._set_state(self.ALIGNING)

    def _handle_aligning(self, dist):
        """ALIGNING: Move forward until walls on both sides detected."""
        left = dist['left']
        right = dist['right']

        if left < WALL_TOO_FAR_MM * 2 and right < WALL_TOO_FAR_MM * 2:
            print(f"\n[GREETER] Hallway aligned - L:{safe_round(left)} R:{safe_round(right)}")
            self._set_state(self.MOVING_TO_T)
            return

        if dist['front'] < FRONT_STOP_MM:
            self.robot.stop_wheels()
        else:
            self.robot.drive(FORWARD_SPEED * 0.5, FORWARD_SPEED * 0.5)

    def _handle_moving_to_t(self, dist):
        """MOVING_TO_T: Wall-follow, detect T-intersection."""
        front = dist['front']
        left = dist['left']
        right = dist['right']

        # Obstacle avoidance
        if front < FRONT_STOP_MM:
            self.robot.stop_wheels()
            # Check for T-intersection
            if front < T_FRONT_WALL_MM and (left > T_SIDE_OPEN_MM or right > T_SIDE_OPEN_MM):
                print(f"\n[GREETER] T-intersection! F:{safe_round(front)} L:{safe_round(left)} R:{safe_round(right)}")
                self._set_state(self.TURNING_TO_DEST)
            return

        # Wall following - stay centered
        left_speed = FORWARD_SPEED
        right_speed = FORWARD_SPEED

        if left < WALL_TOO_CLOSE_MM:
            left_speed += STEER_CORRECTION
            right_speed -= STEER_CORRECTION
        elif left > WALL_TOO_FAR_MM and left < WALL_TOO_FAR_MM * 2:
            left_speed -= STEER_CORRECTION
            right_speed += STEER_CORRECTION

        if right < WALL_TOO_CLOSE_MM:
            left_speed -= STEER_CORRECTION
            right_speed += STEER_CORRECTION
        elif right > WALL_TOO_FAR_MM and right < WALL_TOO_FAR_MM * 2:
            left_speed += STEER_CORRECTION
            right_speed -= STEER_CORRECTION

        self.robot.drive(left_speed, right_speed)

    def _handle_turning_to_dest(self, dist):
        """TURNING_TO_DEST: Turn right (bathroom) or left (lab)."""
        if self._state_elapsed() < TURN_90_DURATION:
            if self.destination == 'bathroom':
                self.robot.drive(TURN_SPEED, -TURN_SPEED)
            else:
                self.robot.drive(-TURN_SPEED, TURN_SPEED)
        else:
            self.robot.stop_wheels()
            self._set_state(self.FINAL_MOVE)

    def _handle_final_move(self, dist):
        """FINAL_MOVEMENT: Drive straight for FINAL_DRIVE_DURATION seconds."""
        if self._state_elapsed() >= FINAL_DRIVE_DURATION:
            self.robot.stop_wheels()
            self._set_state(self.STOPPED)
            return

        if dist['front'] < FRONT_STOP_MM:
            self.robot.stop_wheels()
        else:
            self.robot.drive(FORWARD_SPEED, FORWARD_SPEED)

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
