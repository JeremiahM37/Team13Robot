#!/usr/bin/env python3
"""
Arm Calibration Script

Tests different values for each arm servo to find the correct
positions for "arms down", "arms up", "wave", etc.
Short pulses, press ENTER between each test.
"""

import sys
import signal
import time
from config import SERVO_CHANNELS, SERVO_LIMITS, MAESTRO_PORT
import maestro

servo = None

def emergency_stop(*args):
    print("\n\nEMERGENCY STOP")
    if servo:
        for name in SERVO_CHANNELS:
            if 'shoulder' in name or 'elbow' in name or 'wrist' in name or 'gripper' in name:
                try:
                    servo.setTarget(SERVO_CHANNELS[name], SERVO_LIMITS[name]['center'])
                except:
                    pass
        try:
            servo.close()
        except:
            pass
    sys.exit(0)

signal.signal(signal.SIGINT, emergency_stop)
signal.signal(signal.SIGTERM, emergency_stop)


def test_servo(name):
    ch = SERVO_CHANNELS[name]
    limits = SERVO_LIMITS[name]
    center = limits['center']
    lo = min(limits['min'], limits['max'])
    hi = max(limits['min'], limits['max'])

    print(f"\n{'=' * 50}")
    print(f"Testing: {name} (channel {ch})")
    print(f"  Range: {lo} - {hi}, Center: {center}")
    print(f"{'=' * 50}")

    # Test a range of values
    test_values = [lo, lo + (center - lo) // 2, center, center + (hi - center) // 2, hi]
    test_values = sorted(set(test_values))

    for val in test_values:
        input(f"\n  Press ENTER to move {name} to {val}...")
        servo.setTarget(ch, val)
        time.sleep(1)
        print(f"    At {val} - what position is this? (remember it)")

    # Return to center
    print(f"\n  Returning {name} to center ({center})")
    servo.setTarget(ch, center)
    time.sleep(0.5)


def main():
    global servo

    print("ARM CALIBRATION TOOL")
    print("Press Ctrl+C to emergency stop at any time.\n")

    servo = maestro.Controller(MAESTRO_PORT)
    print(f"Connected to Maestro on {MAESTRO_PORT}\n")

    # Test right arm servos
    right_arm = ['right_shoulder1', 'right_shoulder2', 'right_elbow',
                 'right_wrist_bend', 'right_wrist_rotate', 'right_gripper']

    left_arm = ['left_shoulder1', 'left_shoulder2', 'left_elbow',
                'left_wrist_bend', 'left_wrist_rotate', 'left_gripper']

    print("Which arm to calibrate?")
    print("  1) Right arm")
    print("  2) Left arm")
    print("  3) Both")
    choice = input("Choice: ").strip()

    servos_to_test = []
    if choice == '1':
        servos_to_test = right_arm
    elif choice == '2':
        servos_to_test = left_arm
    elif choice == '3':
        servos_to_test = right_arm + left_arm
    else:
        print("Invalid choice")
        emergency_stop()

    for name in servos_to_test:
        test_servo(name)

    print("\n" + "=" * 50)
    print("CALIBRATION COMPLETE")
    print("=" * 50)
    print("Write down the values for each position (down, up, wave, etc.)")
    print("Then update config.py with the correct center values.")

    servo.close()


if __name__ == '__main__':
    main()
