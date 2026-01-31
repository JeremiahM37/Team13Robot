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
    """

    def __init__(self, simulation_mode=False):
        """
        Initialize the robot controller.

        Args:
            simulation_mode: If True, don't connect to real hardware
        """
        self.simulation_mode = simulation_mode or not MAESTRO_AVAILABLE
        self.servo = None
        self.lock = threading.Lock()

        # Track current positions
        self.current_positions = {
            'left_wheel': SERVO_LIMITS['left_wheel']['center'],
            'right_wheel': SERVO_LIMITS['right_wheel']['center'],
            'head_tilt': SERVO_LIMITS['head_tilt']['center'],
            'head_pan': SERVO_LIMITS['head_pan']['center'],
            'waist': SERVO_LIMITS['waist']['center'],
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
        clamped_value = self._clamp(int(value), limits['min'], limits['max'])

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

        # Stop wheels (set to center = no rotation)
        self._set_servo('left_wheel', SERVO_LIMITS['left_wheel']['center'])
        self._set_servo('right_wheel', SERVO_LIMITS['right_wheel']['center'])

        # Center head and waist
        self._set_servo('head_tilt', SERVO_LIMITS['head_tilt']['center'])
        self._set_servo('head_pan', SERVO_LIMITS['head_pan']['center'])
        self._set_servo('waist', SERVO_LIMITS['waist']['center'])

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

        # Convert -1 to 1 range to servo units
        left_limits = SERVO_LIMITS['left_wheel']
        right_limits = SERVO_LIMITS['right_wheel']

        # Map speed to servo range
        # Note: You may need to invert one wheel depending on mounting orientation
        left_value = left_limits['center'] + (left_speed * (left_limits['max'] - left_limits['center']))
        right_value = right_limits['center'] + (right_speed * (right_limits['max'] - right_limits['center']))

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
    Run this script directly to test without Flask.
    """
    print("=" * 50)
    print("Robot Control Layer - Direct Test")
    print("=" * 50)

    # Create controller
    robot = RobotController(simulation_mode=True)  # Set False to test real hardware

    print("\nTesting drive functions...")
    robot.drive(0.5, 0.5)   # Forward
    time.sleep(0.5)
    robot.drive(-0.5, -0.5) # Backward
    time.sleep(0.5)
    robot.drive(0.5, -0.5)  # Turn right
    time.sleep(0.5)
    robot.stop_wheels()

    print("\nTesting joystick drive...")
    robot.drive_joystick(0, 0.5)   # Forward
    time.sleep(0.5)
    robot.drive_joystick(0.5, 0)   # Turn right
    time.sleep(0.5)
    robot.drive_joystick(0.3, 0.7) # Forward and slight right
    time.sleep(0.5)
    robot.stop_wheels()

    print("\nTesting head control...")
    robot.set_head_tilt(0.0)  # Min
    time.sleep(0.3)
    robot.set_head_tilt(1.0)  # Max
    time.sleep(0.3)
    robot.set_head_tilt(0.5)  # Center

    robot.set_head_pan(0.0)   # Left
    time.sleep(0.3)
    robot.set_head_pan(1.0)   # Right
    time.sleep(0.3)
    robot.set_head_pan(0.5)   # Center

    print("\nTesting waist control...")
    robot.set_waist(0.0)
    time.sleep(0.3)
    robot.set_waist(1.0)
    time.sleep(0.3)
    robot.set_waist(0.5)

    print("\nTesting stop all...")
    robot.stop_all()

    print("\nStatus:", robot.get_status())

    print("\n" + "=" * 50)
    print("Direct test complete!")
    print("=" * 50)
