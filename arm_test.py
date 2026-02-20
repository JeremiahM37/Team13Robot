#!/usr/bin/env python3
"""
Arm Servo Test Script

Tests each arm servo one at a time with small, safe movements.
Press Enter to advance to the next servo. Ctrl+C to emergency stop.

Left arm:  channels 5-10
Right arm: channels 11-16
"""

import sys
import signal
import time

from config import MAESTRO_PORT

try:
    import maestro
except ImportError:
    print("ERROR: maestro module not found.")
    sys.exit(1)

# Arm channel assignments
# ORDER: Shoulder forward, Shoulder side, Elbow, Wrist up, wrist rotate, grip
RIGHT_ARM_CHANNELS = [5, 6, 7, 8, 9, 10]
LEFT_ARM_CHANNELS = [11, 12, 13, 14, 15, 16]

SERVO_NAMES = [
    "shoulder 1", #raise forward
    "shoulder 2", #raise to side
    "elbow",
    "wrist bend",
    "wrist rotate",
    "gripper",
]

# Safe defaults — adjust after finding your actual ranges
CENTER = 6000
MOVE_AMOUNT = 400  # How far from center to test (small = safe)

controller = None


def emergency_stop(*args):
    print("\n\nEMERGENCY STOP — centering all arm servos")
    if controller:
        for ch in LEFT_ARM_CHANNELS + RIGHT_ARM_CHANNELS:
            try:
                controller.setTarget(ch, CENTER)
            except:
                pass
        controller.close()
    sys.exit(0)


signal.signal(signal.SIGINT, emergency_stop)
signal.signal(signal.SIGTERM, emergency_stop)


def test_servo(channel, name):
    """Test a single servo with small movements around center."""
    print(f"\n  Channel {channel}: {name}")
    print(f"    Center ({CENTER})...")
    controller.setTarget(channel, CENTER)
    time.sleep(0.5)

    low = CENTER - MOVE_AMOUNT
    high = CENTER + MOVE_AMOUNT

    print(f"    Low ({low})...")
    controller.setTarget(channel, low)
    time.sleep(0.8)

    print(f"    High ({high})...")
    controller.setTarget(channel, high)
    time.sleep(0.8)

    print(f"    Back to center ({CENTER})")
    controller.setTarget(channel, CENTER)
    time.sleep(0.5)


def main():
    global controller

    print("=" * 50)
    print("Arm Servo Test")
    print("=" * 50)
    print(f"Left arm:  channels {LEFT_ARM_CHANNELS}")
    print(f"Right arm: channels {RIGHT_ARM_CHANNELS}")
    print(f"Move range: {CENTER} +/- {MOVE_AMOUNT}")
    print("Ctrl+C to emergency stop at any time.\n")

    controller = maestro.Controller(MAESTRO_PORT)
    print(f"Connected to Maestro on {MAESTRO_PORT}\n")

    # Center all arm servos first
    print("Centering all arm servos...")
    for ch in LEFT_ARM_CHANNELS + RIGHT_ARM_CHANNELS:
        controller.setTarget(ch, CENTER)
    time.sleep(1)

    # Test left arm
    input("Press ENTER to test LEFT ARM...")
    print("\n--- LEFT ARM ---")
    for i, ch in enumerate(LEFT_ARM_CHANNELS):
        test_servo(ch, SERVO_NAMES[i])
        input(f"    Press ENTER for next servo...")

    # Test right arm
    input("\nPress ENTER to test RIGHT ARM...")
    print("\n--- RIGHT ARM ---")
    for i, ch in enumerate(RIGHT_ARM_CHANNELS):
        test_servo(ch, SERVO_NAMES[i])
        input(f"    Press ENTER for next servo...")

    # Center everything at the end
    print("\nCentering all arm servos...")
    for ch in LEFT_ARM_CHANNELS + RIGHT_ARM_CHANNELS:
        controller.setTarget(ch, CENTER)
    time.sleep(0.5)

    controller.close()
    print("\n" + "=" * 50)
    print("Arm test complete!")
    print("=" * 50)


if __name__ == "__main__":
    main()
