"""
Robot Control Layer
Independent of Flask - can be tested directly from Python.

This module provides safe, validated control of:
- Drive wheels (differential drive)
- Head tilt and pan
- Waist rotation

All commands are validated and constrained to safe limits.
"""

import time
import threading
from config import SERVO_CHANNELS, SERVO_LIMITS, MAESTRO_PORT

# Try to import maestro - will fail gracefully if not available
try:
    import maestro
    MAESTRO_AVAILABLE = True
except ImportError:
    MAESTRO_AVAILABLE = False
    print("WARNING: maestro module not available. Running in simulation mode.")


class RobotController:
    """
    Main robot control class.
    Provides safe, validated control of all robot actuators.
    Integrates LIDAR safety to block unsafe forward/backward motion.
    """

    def __init__(self, simulation_mode=False, lidar=None):
        """
        Initialize the robot controller.

        Args:
            simulation_mode: If True, don't connect to real hardware
            lidar: Optional LidarSafety instance for obstacle detection
        """
        self.simulation_mode = simulation_mode or not MAESTRO_AVAILABLE
        self.servo = None
        self.lock = threading.Lock()
        self.lidar = lidar

        # Track current positions for all servos
        self.current_positions = {
            name: limits['center']
            for name, limits in SERVO_LIMITS.items()
        }

        # Safety: last command timestamp
        self.last_command_time = time.time()

        if not self.simulation_mode:
            try:
                self.servo = maestro.Controller(MAESTRO_PORT)
                print(f"Connected to Maestro on {MAESTRO_PORT}")
                self.stop_all()  # Start in safe state
            except Exception as e:
                print(f"ERROR: Could not connect to Maestro: {e}")
                print("Falling back to simulation mode.")
                self.simulation_mode = True
        else:
            print("Running in SIMULATION MODE - no hardware commands sent")

    def _clamp(self, value, min_val, max_val):
        """Clamp a value to the specified range."""
        return max(min_val, min(max_val, value))

    def _speed_to_servo(self, wheel_name, speed):
        """
        Convert a -1.0 to 1.0 speed to a servo value, skipping the deadband.

        Speed 0 = center (stopped).
        Speed 0.01..1.0 = forward (maps to forward_min..forward_max, going BELOW center).
        Speed -0.01..-1.0 = reverse (maps to reverse_min..reverse_max, going ABOVE center).
        """
        limits = SERVO_LIMITS[wheel_name]
        center = limits['center']

        if abs(speed) < 0.01:
            return center

        abs_speed = abs(speed)

        if speed > 0:
            # Forward: map 0..1 to forward_min..forward_max (values decrease)
            fwd_min = limits['forward_min']  # 5000 (slowest forward)
            fwd_max = limits['forward_max']  # 4000 (fastest forward)
            return fwd_min + abs_speed * (fwd_max - fwd_min)
        else:
            # Reverse: map 0..1 to reverse_min..reverse_max (values increase)
            rev_min = limits['reverse_min']  # 6900 (slowest reverse)
            rev_max = limits['reverse_max']  # 7000 (fastest reverse)
            return rev_min + abs_speed * (rev_max - rev_min)

    def _set_servo(self, channel_name, value):
        """
        Set a servo to a specific value with safety checks.

        Args:
            channel_name: Name from SERVO_CHANNELS
            value: Target value in Maestro units

        Returns:
            bool: True if command was executed, False otherwise
        """
        if channel_name not in SERVO_CHANNELS:
            print(f"ERROR: Unknown channel: {channel_name}")
            return False

        limits = SERVO_LIMITS[channel_name]
        low = min(limits['min'], limits['max'])
        high = max(limits['min'], limits['max'])
        clamped_value = self._clamp(int(value), low, high)

        if clamped_value != value:
            print(f"WARNING: {channel_name} value {value} clamped to {clamped_value}")

        with self.lock:
            self.current_positions[channel_name] = clamped_value
            self.last_command_time = time.time()

            if not self.simulation_mode and self.servo:
                try:
                    channel = SERVO_CHANNELS[channel_name]
                    self.servo.setTarget(channel, clamped_value)
                    return True
                except Exception as e:
                    # Silent during shutdown (port closed) - not a real error
                    if 'port that is not open' not in str(e):
                        print(f"ERROR setting {channel_name}: {e}")
                    return False
            else:
                print(f"[SIM] {channel_name} -> {clamped_value}")
                return True

    def stop_all(self):
        """
        Emergency stop - set all actuators to neutral/safe state.
        This is the STOP / neutral state required by the spec.
        """
        print("STOP ALL - Setting neutral state")

        # Center all servos
        for name, limits in SERVO_LIMITS.items():
            self._set_servo(name, limits['center'])

        return True

    def stop_wheels(self):
        """Stop only the drive wheels."""
        self._set_servo('left_wheel', SERVO_LIMITS['left_wheel']['center'])
        self._set_servo('right_wheel', SERVO_LIMITS['right_wheel']['center'])
        return True

    # ==================== DRIVE CONTROL ====================

    def drive(self, left_speed, right_speed):
        """
        Set wheel speeds for differential drive.

        Args:
            left_speed: -1.0 to 1.0 (negative = backward)
            right_speed: -1.0 to 1.0 (negative = backward)

        Returns:
            bool: True if command was valid and executed
        """
        # Validate inputs
        if not isinstance(left_speed, (int, float)) or not isinstance(right_speed, (int, float)):
            print("ERROR: Invalid speed values (must be numbers)")
            return False

        # Clamp to valid range
        left_speed = self._clamp(float(left_speed), -1.0, 1.0)
        right_speed = self._clamp(float(right_speed), -1.0, 1.0)

        # Convert -1 to 1 range to servo units, skipping the deadband
        left_value = self._speed_to_servo('left_wheel', left_speed)
        right_value = self._speed_to_servo('right_wheel', right_speed)

        self._set_servo('left_wheel', left_value)
        self._set_servo('right_wheel', right_value)

        return True

    def drive_joystick(self, x, y):
        """
        Drive using joystick-style input (for web interface).
        Converts x,y coordinates to differential drive.

        Args:
            x: -1.0 to 1.0 (left/right)
            y: -1.0 to 1.0 (forward/backward, positive = forward)

        Returns:
            bool: True if command was valid and executed
        """
        # Validate inputs
        if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
            print("ERROR: Invalid joystick values (must be numbers)")
            return False

        x = self._clamp(float(x), -1.0, 1.0)
        y = self._clamp(float(y), -1.0, 1.0)

        # Joystick dead zone: ignore small x when mostly pushing forward/back
        # This prevents accidental turning from slight diagonal input
        if abs(y) > 0.3 and abs(x) < 0.15:
            x = 0

        # LIDAR safety check: block forward/backward if obstacle detected
        # Any y component beyond the dead zone counts as forward/backward intent
        SAFETY_DEADZONE = 0.05
        if self.lidar:
            if y > SAFETY_DEADZONE and self.lidar.front_blocked:
                print(f"[SAFETY] Forward BLOCKED by obstacle - ignoring forward (y={y:.2f})")
                y = 0  # Kill forward component, turning still allowed
            if y < -SAFETY_DEADZONE and self.lidar.rear_blocked:
                print(f"[SAFETY] Reverse BLOCKED by obstacle - ignoring backward (y={y:.2f})")
                y = 0  # Kill backward component, turning still allowed

        # Convert joystick to differential drive
        # Forward/backward is y, turning is x
        left_speed = y + x
        right_speed = y - x

        # Normalize if needed (keep proportions but limit to -1 to 1)
        max_magnitude = max(abs(left_speed), abs(right_speed))
        if max_magnitude > 1.0:
            left_speed /= max_magnitude
            right_speed /= max_magnitude

        return self.drive(left_speed, right_speed)

    # ==================== HEAD CONTROL ====================

    def set_head_tilt(self, position):
        """
        Set head tilt position.

        Args:
            position: 0.0 to 1.0 (0 = min, 1 = max)

        Returns:
            bool: True if command was valid and executed
        """
        if not isinstance(position, (int, float)):
            print("ERROR: Invalid tilt position (must be number)")
            return False

        position = self._clamp(float(position), 0.0, 1.0)
        limits = SERVO_LIMITS['head_tilt']
        value = limits['min'] + (position * (limits['max'] - limits['min']))

        return self._set_servo('head_tilt', value)

    def set_head_pan(self, position):
        """
        Set head pan position.

        Args:
            position: 0.0 to 1.0 (0 = min/left, 1 = max/right)

        Returns:
            bool: True if command was valid and executed
        """
        if not isinstance(position, (int, float)):
            print("ERROR: Invalid pan position (must be number)")
            return False

        position = self._clamp(float(position), 0.0, 1.0)
        limits = SERVO_LIMITS['head_pan']
        value = limits['min'] + (position * (limits['max'] - limits['min']))

        return self._set_servo('head_pan', value)

    # ==================== WAIST CONTROL ====================

    def set_waist(self, position):
        """
        Set waist rotation position.

        Args:
            position: 0.0 to 1.0 (0 = min/left, 1 = max/right)

        Returns:
            bool: True if command was valid and executed
        """
        if not isinstance(position, (int, float)):
            print("ERROR: Invalid waist position (must be number)")
            return False

        position = self._clamp(float(position), 0.0, 1.0)
        limits = SERVO_LIMITS['waist']
        value = limits['min'] + (position * (limits['max'] - limits['min']))

        return self._set_servo('waist', value)

    # ==================== STATUS ====================

    def get_status(self):
        """Get current robot status."""
        return {
            'simulation_mode': self.simulation_mode,
            'positions': self.current_positions.copy(),
            'last_command_age_ms': int((time.time() - self.last_command_time) * 1000),
        }

    def close(self):
        """Clean up resources."""
        self.stop_all()
        if self.servo:
            try:
                self.servo.close()
            except:
                pass


# ==================== SAFETY WATCHDOG ====================

class SafetyWatchdog:
    """
    Monitors for communication loss and stops robot if no commands received.
    """

    def __init__(self, robot, timeout_ms=500):
        self.robot = robot
        self.timeout_sec = timeout_ms / 1000.0
        self.running = False
        self.thread = None

    def start(self):
        """Start the watchdog thread."""
        self.running = True
        self.thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self.thread.start()
        print(f"Safety watchdog started (timeout: {self.timeout_sec}s)")

    def stop(self):
        """Stop the watchdog thread."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)

    def _watchdog_loop(self):
        """Main watchdog loop - stops robot if commands timeout."""
        while self.running:
            time.sleep(0.1)  # Check every 100ms

            age = time.time() - self.robot.last_command_time
            if age > self.timeout_sec:
                # Only stop wheels - don't reset head/waist position
                self.robot.stop_wheels()


# ==================== DIRECT TESTING ====================

if __name__ == "__main__":
    """
    Direct test of robot control layer.
    Run this script directly to test with real hardware.
    All movements are short pulses that auto-stop.
    Press Ctrl+C to emergency stop.
    """
    import signal

    robot = None

    def emergency_stop(*args):
        print("\n\nEMERGENCY STOP")
        if robot:
            robot.stop_all()
            robot.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, emergency_stop)
    signal.signal(signal.SIGTERM, emergency_stop)

    print("=" * 50)
    print("Robot Control Layer - Hardware Test")
    print("=" * 50)
    print("Short 0.8s pulses. Ctrl+C to emergency stop.\n")

    robot = RobotController(simulation_mode=False)

    input("Press ENTER to test FORWARD (both wheels)...")
    robot.drive(0.5, 0.5)
    time.sleep(0.8)
    robot.stop_wheels()
    print("  Stopped.\n")

    input("Press ENTER to test BACKWARD (both wheels)...")
    robot.drive(-0.5, -0.5)
    time.sleep(0.8)
    robot.stop_wheels()
    print("  Stopped.\n")

    input("Press ENTER to test TURN RIGHT...")
    robot.drive(0.5, -0.5)
    time.sleep(0.8)
    robot.stop_wheels()
    print("  Stopped.\n")

    input("Press ENTER to test TURN LEFT...")
    robot.drive(-0.5, 0.5)
    time.sleep(0.8)
    robot.stop_wheels()
    print("  Stopped.\n")

    input("Press ENTER to test HEAD TILT...")
    robot.set_head_tilt(0.0)
    time.sleep(0.8)
    robot.set_head_tilt(1.0)
    time.sleep(0.8)
    robot.set_head_tilt(0.5)
    print("  Done.\n")

    input("Press ENTER to test HEAD PAN...")
    robot.set_head_pan(0.0)
    time.sleep(0.8)
    robot.set_head_pan(1.0)
    time.sleep(0.8)
    robot.set_head_pan(0.5)
    print("  Done.\n")

    input("Press ENTER to test WAIST...")
    robot.set_waist(0.0)
    time.sleep(0.8)
    robot.set_waist(1.0)
    time.sleep(0.8)
    robot.set_waist(0.5)
    print("  Done.\n")

    robot.stop_all()
    robot.close()

    print("=" * 50)
    print("All tests complete!")
    print("=" * 50)
