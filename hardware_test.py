#!/usr/bin/env python3
"""
Hardware Bring-Up & Verification Script (Step A Deliverable)

This script individually tests each hardware component:
- Drive wheels (left and right)
- Head tilt
- Head pan
- Waist rotation

Run this BEFORE attempting web control to identify hardware issues.

Usage:
    python hardware_test.py           # Run all tests
    python hardware_test.py wheels    # Test only wheels
    python hardware_test.py head      # Test only head
    python hardware_test.py waist     # Test only waist
    python hardware_test.py --sim     # Simulation mode (no hardware)
"""

import sys
import time
from config import SERVO_CHANNELS, SERVO_LIMITS, MAESTRO_PORT

# Try to import maestro
try:
    import maestro
    MAESTRO_AVAILABLE = True
except ImportError:
    MAESTRO_AVAILABLE = False
    print("WARNING: maestro module not found!")


class HardwareTester:
    """
    Tests individual hardware components and reports results.
    """

    def __init__(self, simulation_mode=False):
        self.simulation_mode = simulation_mode or not MAESTRO_AVAILABLE
        self.servo = None
        self.test_results = {}

        if not self.simulation_mode:
            try:
                self.servo = maestro.Controller(MAESTRO_PORT)
                print(f"✓ Connected to Maestro on {MAESTRO_PORT}")
            except Exception as e:
                print(f"✗ ERROR: Could not connect to Maestro: {e}")
                self.simulation_mode = True

        if self.simulation_mode:
            print("Running in SIMULATION MODE")

    def _set_servo(self, channel, value, name=""):
        """Set servo and wait briefly."""
        if self.simulation_mode:
            print(f"  [SIM] Channel {channel} ({name}) -> {value}")
        else:
            try:
                self.servo.setTarget(channel, value)
            except Exception as e:
                print(f"  ✗ ERROR on channel {channel}: {e}")
                return False
        return True

    def _prompt_user(self, message):
        """Ask user to confirm hardware behavior."""
        response = input(f"\n  {message} (y/n/s to skip): ").strip().lower()
        return response

    def test_wheels(self):
        """Test both drive wheels."""
        print("\n" + "=" * 50)
        print("TESTING DRIVE WHEELS")
        print("=" * 50)

        left_ch = SERVO_CHANNELS['left_wheel']
        right_ch = SERVO_CHANNELS['right_wheel']
        left_limits = SERVO_LIMITS['left_wheel']
        right_limits = SERVO_LIMITS['right_wheel']

        print(f"\nLeft wheel: Channel {left_ch}")
        print(f"  Limits: {left_limits['min']} - {left_limits['max']}")
        print(f"  Center (stop): {left_limits['center']}")

        print(f"\nRight wheel: Channel {right_ch}")
        print(f"  Limits: {right_limits['min']} - {right_limits['max']}")
        print(f"  Center (stop): {right_limits['center']}")

        # Test left wheel
        print("\n--- LEFT WHEEL ---")
        print("  Setting to STOP (center)...")
        self._set_servo(left_ch, left_limits['center'], "left stop")
        self._set_servo(right_ch, right_limits['center'], "right stop")
        time.sleep(0.5)

        print("  Testing LEFT wheel FORWARD...")
        forward_val = left_limits.get('forward_min', 5000)
        self._set_servo(left_ch, forward_val, "left forward")
        time.sleep(1.5)

        result = self._prompt_user("Did LEFT wheel rotate FORWARD?")
        if result == 's':
            self.test_results['left_wheel_forward'] = 'SKIPPED'
        else:
            self.test_results['left_wheel_forward'] = 'PASS' if result == 'y' else 'FAIL'

        print("  Testing LEFT wheel BACKWARD...")
        backward_val = left_limits.get('reverse_min', 6900)
        self._set_servo(left_ch, backward_val, "left backward")
        time.sleep(1.5)

        result = self._prompt_user("Did LEFT wheel rotate BACKWARD?")
        if result == 's':
            self.test_results['left_wheel_backward'] = 'SKIPPED'
        else:
            self.test_results['left_wheel_backward'] = 'PASS' if result == 'y' else 'FAIL'

        # Stop left, test right
        print("  Stopping left wheel...")
        self._set_servo(left_ch, left_limits['center'], "left stop")
        time.sleep(0.3)

        print("\n--- RIGHT WHEEL ---")
        print("  Testing RIGHT wheel FORWARD...")
        forward_val = right_limits.get('forward_min', 5000)
        self._set_servo(right_ch, forward_val, "right forward")
        time.sleep(1.5)

        result = self._prompt_user("Did RIGHT wheel rotate FORWARD?")
        if result == 's':
            self.test_results['right_wheel_forward'] = 'SKIPPED'
        else:
            self.test_results['right_wheel_forward'] = 'PASS' if result == 'y' else 'FAIL'

        print("  Testing RIGHT wheel BACKWARD...")
        backward_val = right_limits.get('reverse_min', 6900)
        self._set_servo(right_ch, backward_val, "right backward")
        time.sleep(1.5)

        result = self._prompt_user("Did RIGHT wheel rotate BACKWARD?")
        if result == 's':
            self.test_results['right_wheel_backward'] = 'SKIPPED'
        else:
            self.test_results['right_wheel_backward'] = 'PASS' if result == 'y' else 'FAIL'

        # Stop both wheels
        print("  Stopping all wheels...")
        self._set_servo(left_ch, left_limits['center'], "left stop")
        self._set_servo(right_ch, right_limits['center'], "right stop")

    def test_head(self):
        """Test head tilt and pan."""
        print("\n" + "=" * 50)
        print("TESTING HEAD SERVOS")
        print("=" * 50)

        tilt_ch = SERVO_CHANNELS['head_tilt']
        pan_ch = SERVO_CHANNELS['head_pan']
        tilt_limits = SERVO_LIMITS['head_tilt']
        pan_limits = SERVO_LIMITS['head_pan']

        print(f"\nHead tilt: Channel {tilt_ch}")
        print(f"  Limits: {tilt_limits['min']} - {tilt_limits['max']}")
        print(f"  Center: {tilt_limits['center']}")

        print(f"\nHead pan: Channel {pan_ch}")
        print(f"  Limits: {pan_limits['min']} - {pan_limits['max']}")
        print(f"  Center: {pan_limits['center']}")

        # Test tilt
        print("\n--- HEAD TILT ---")
        print("  Moving to CENTER...")
        self._set_servo(tilt_ch, tilt_limits['center'], "tilt center")
        time.sleep(0.5)

        print("  Moving to MIN (look down)...")
        self._set_servo(tilt_ch, tilt_limits['min'], "tilt min")
        time.sleep(1)

        print("  Moving to MAX (look up)...")
        self._set_servo(tilt_ch, tilt_limits['max'], "tilt max")
        time.sleep(1)

        print("  Returning to CENTER...")
        self._set_servo(tilt_ch, tilt_limits['center'], "tilt center")
        time.sleep(0.5)

        result = self._prompt_user("Did head TILT (up/down) correctly?")
        if result == 's':
            self.test_results['head_tilt'] = 'SKIPPED'
        else:
            self.test_results['head_tilt'] = 'PASS' if result == 'y' else 'FAIL'

        # Test pan
        print("\n--- HEAD PAN ---")
        print("  Moving to CENTER...")
        self._set_servo(pan_ch, pan_limits['center'], "pan center")
        time.sleep(0.5)

        print("  Moving to MIN (look left)...")
        self._set_servo(pan_ch, pan_limits['min'], "pan min")
        time.sleep(1)

        print("  Moving to MAX (look right)...")
        self._set_servo(pan_ch, pan_limits['max'], "pan max")
        time.sleep(1)

        print("  Returning to CENTER...")
        self._set_servo(pan_ch, pan_limits['center'], "pan center")
        time.sleep(0.5)

        result = self._prompt_user("Did head PAN (left/right) correctly?")
        if result == 's':
            self.test_results['head_pan'] = 'SKIPPED'
        else:
            self.test_results['head_pan'] = 'PASS' if result == 'y' else 'FAIL'

    def test_waist(self):
        """Test waist rotation."""
        print("\n" + "=" * 50)
        print("TESTING WAIST ROTATION")
        print("=" * 50)

        waist_ch = SERVO_CHANNELS['waist']
        waist_limits = SERVO_LIMITS['waist']

        print(f"\nWaist: Channel {waist_ch}")
        print(f"  Limits: {waist_limits['min']} - {waist_limits['max']}")
        print(f"  Center: {waist_limits['center']}")

        print("\n--- WAIST ---")
        print("  Moving to CENTER...")
        self._set_servo(waist_ch, waist_limits['center'], "waist center")
        time.sleep(0.5)

        print("  Moving to MIN (rotate left)...")
        self._set_servo(waist_ch, waist_limits['min'], "waist min")
        time.sleep(1)

        print("  Moving to MAX (rotate right)...")
        self._set_servo(waist_ch, waist_limits['max'], "waist max")
        time.sleep(1)

        print("  Returning to CENTER...")
        self._set_servo(waist_ch, waist_limits['center'], "waist center")
        time.sleep(0.5)

        result = self._prompt_user("Did waist rotate correctly?")
        if result == 's':
            self.test_results['waist'] = 'SKIPPED'
        else:
            self.test_results['waist'] = 'PASS' if result == 'y' else 'FAIL'

    def print_summary(self):
        """Print test results summary."""
        print("\n" + "=" * 50)
        print("TEST RESULTS SUMMARY")
        print("=" * 50)

        if not self.test_results:
            print("No tests were run.")
            return

        passed = 0
        failed = 0
        skipped = 0

        for test, result in self.test_results.items():
            status_symbol = "✓" if result == "PASS" else ("○" if result == "SKIPPED" else "✗")
            print(f"  {status_symbol} {test}: {result}")
            if result == "PASS":
                passed += 1
            elif result == "FAIL":
                failed += 1
            else:
                skipped += 1

        print("\n" + "-" * 50)
        print(f"  PASSED: {passed}")
        print(f"  FAILED: {failed}")
        print(f"  SKIPPED: {skipped}")

        if failed > 0:
            print("\n⚠️  Some tests FAILED! Check hardware before proceeding.")
            print("  Common issues:")
            print("  - Wrong channel assignments in config.py")
            print("  - Loose connections")
            print("  - Incorrect servo limits")
            print("  - Power issues")
        elif passed > 0:
            print("\n✓ All tested components working!")

    def run_all_tests(self):
        """Run complete hardware test suite."""
        print("\n" + "=" * 50)
        print("ROBOT HARDWARE VERIFICATION")
        print("=" * 50)
        print("\nThis script will test each component individually.")
        print("Watch the robot and answer the prompts.")
        print("\nPress Ctrl+C at any time to abort.\n")

        input("Press Enter to begin testing...")

        try:
            self.test_wheels()
            self.test_head()
            self.test_waist()
        except KeyboardInterrupt:
            print("\n\nTest aborted by user.")
        finally:
            # Make sure everything is stopped
            if self.servo:
                for name, ch in SERVO_CHANNELS.items():
                    center = SERVO_LIMITS[name]['center']
                    self._set_servo(ch, center, f"{name} stop")

            self.print_summary()

    def close(self):
        """Clean up."""
        if self.servo:
            self.servo.close()


def main():
    """Main entry point."""
    simulation_mode = '--sim' in sys.argv

    tester = HardwareTester(simulation_mode=simulation_mode)

    try:
        # Check for specific test arguments
        if 'wheels' in sys.argv:
            tester.test_wheels()
        elif 'head' in sys.argv:
            tester.test_head()
        elif 'waist' in sys.argv:
            tester.test_waist()
        else:
            tester.run_all_tests()

        tester.print_summary()
    finally:
        tester.close()


if __name__ == "__main__":
    main()
