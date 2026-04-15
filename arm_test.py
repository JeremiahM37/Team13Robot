#!/usr/bin/env python3
"""
Arm Servo Calibration Script

Interactive tool to find working values for each arm servo.
For each servo you can type values (e.g. 5000, 6500) and see what happens,
then record the min, max, and center that work.

Right arm: channels 5-10
Left arm:  channels 11-16

Usage:
    python arm_test.py
    python arm_test.py right   # Test only right arm
    python arm_test.py left    # Test only left arm

Ctrl+C to emergency stop at any time.
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
# ORDER: Shoulder forward, Shoulder side, Elbow, Wrist bend, Wrist rotate, Grip
RIGHT_ARM_CHANNELS = [5, 6, 7, 8, 9, 10]
LEFT_ARM_CHANNELS = [11, 12, 13, 14, 15, 16]

SERVO_NAMES = [
    "shoulder 1 (raise forward)",
    "shoulder 2 (raise to side)",
    "elbow",
    "wrist bend",
    "wrist rotate",
    "gripper",
]

DEFAULT_CENTER = 6000

# Per-channel center overrides (channel: center value)
CHANNEL_CENTERS = {
    5: 9000,   # Right shoulder 1
    6: 7000,   # Right shoulder 2
    7: 7000,   # Right elbow
    8: 7000,   # Right wrist bend
    9: 6000,   # Right wrist rotate
}

def get_center(channel):
    return CHANNEL_CENTERS.get(channel, DEFAULT_CENTER)

controller = None

# Store discovered values
results = {}


def emergency_stop(*args):
    print("\n\nEMERGENCY STOP - centering all arm servos")
    if controller:
        for ch in RIGHT_ARM_CHANNELS + LEFT_ARM_CHANNELS:
            try:
                controller.setTarget(ch, get_center(ch))
            except:
                pass
        controller.close()
    print_results()
    sys.exit(0)


signal.signal(signal.SIGINT, emergency_stop)
signal.signal(signal.SIGTERM, emergency_stop)


def print_results():
    """Print all discovered values so far."""
    if not results:
        return
    print("\n" + "=" * 50)
    print("DISCOVERED VALUES - paste into config.py")
    print("=" * 50)
    for key, vals in results.items():
        print(f"  '{key}': {{")
        print(f"      'min': {vals['min']},")
        print(f"      'max': {vals['max']},")
        print(f"      'center': {vals['center']},")
        print(f"  }},")


def calibrate_servo(channel, name, arm_label):
    """Interactive calibration for a single servo."""
    center = get_center(channel)
    full_name = f"{arm_label} {name}"
    print(f"\n{'─' * 50}")
    print(f"  Channel {channel}: {full_name}")
    print(f"{'─' * 50}")
    print(f"  Setting to center ({center})...")
    controller.setTarget(channel, center)
    time.sleep(0.3)

    print(f"\n  Type a value (3000-10000) and press Enter to move the servo.")
    print(f"  Commands:")
    print(f"    <number>  - move to that value (e.g. 5000)")
    print(f"    c         - return to center ({center})")
    print(f"    s         - skip this servo")
    print(f"    d         - done, record min/max/center and move on")
    print()

    low_seen = center
    high_seen = center
    current = center

    while True:
        try:
            raw = input(f"  [{full_name} @ {current}] > ").strip().lower()
        except EOFError:
            break

        if raw == '':
            continue
        elif raw == 's':
            print(f"  Skipping {full_name}")
            controller.setTarget(channel, center)
            return
        elif raw == 'c':
            current = center
            controller.setTarget(channel, center)
            print(f"  -> {center} (center)")
            continue
        elif raw == 'd':
            # Ask for final values
            print(f"\n  Lowest value you tested:  {low_seen}")
            print(f"  Highest value you tested: {high_seen}")

            min_val = input(f"  Enter MIN value [{low_seen}]: ").strip()
            min_val = int(min_val) if min_val else low_seen

            max_val = input(f"  Enter MAX value [{high_seen}]: ").strip()
            max_val = int(max_val) if max_val else high_seen

            center_val = input(f"  Enter CENTER value [{center}]: ").strip()
            center_val = int(center_val) if center_val else center

            config_key = f"{arm_label.lower()}_{name.split(' ')[0]}_{channel}"
            results[config_key] = {
                'min': min_val,
                'max': max_val,
                'center': center_val,
            }
            print(f"  Saved: min={min_val}, max={max_val}, center={center_val}")
            controller.setTarget(channel, center_val)
            return
        else:
            try:
                value = int(raw)
                if value < 3000 or value > 10000:
                    print(f"  Value out of safe range (3000-10000)")
                    continue
                controller.setTarget(channel, value)
                current = value
                if value < low_seen:
                    low_seen = value
                if value > high_seen:
                    high_seen = value
                print(f"  -> {value}")
            except ValueError:
                print(f"  Unknown command. Type a number, c, s, or d.")


def test_arm(channels, arm_label):
    """Run calibration for one arm."""
    print(f"\n{'=' * 50}")
    print(f"  {arm_label} ARM - channels {channels}")
    print(f"{'=' * 50}")

    # Center all servos on this arm first
    for ch in channels:
        controller.setTarget(ch, get_center(ch))
    time.sleep(0.5)

    for i, ch in enumerate(channels):
        calibrate_servo(ch, SERVO_NAMES[i], arm_label)

    # Center arm when done
    for ch in channels:
        controller.setTarget(ch, get_center(ch))


def main():
    global controller

    print("=" * 50)
    print("  ARM SERVO CALIBRATION")
    print("=" * 50)
    print(f"Right arm: channels {RIGHT_ARM_CHANNELS}")
    print(f"Left arm:  channels {LEFT_ARM_CHANNELS}")
    print(f"Ctrl+C to emergency stop at any time.\n")

    controller = maestro.Controller(MAESTRO_PORT)
    print(f"Connected to Maestro on {MAESTRO_PORT}\n")

    # Center all arm servos
    print("Centering all arm servos...")
    for ch in RIGHT_ARM_CHANNELS + LEFT_ARM_CHANNELS:
        controller.setTarget(ch, get_center(ch))
    time.sleep(1)

    # Check which arms to test
    test_right = True
    test_left = True
    if 'right' in sys.argv:
        test_left = False
    elif 'left' in sys.argv:
        test_right = False

    try:
        if test_right:
            input("Press ENTER to start RIGHT ARM calibration...")
            test_arm(RIGHT_ARM_CHANNELS, "Right")

        if test_left:
            input("\nPress ENTER to start LEFT ARM calibration...")
            test_arm(LEFT_ARM_CHANNELS, "Left")

    except KeyboardInterrupt:
        emergency_stop()

    # Center everything at the end
    print("\nCentering all arm servos...")
    for ch in RIGHT_ARM_CHANNELS + LEFT_ARM_CHANNELS:
        controller.setTarget(ch, get_center(ch))
    time.sleep(0.5)

    controller.close()
    print_results()

    print("\n" + "=" * 50)
    print("  Calibration complete!")
    print("=" * 50)


if __name__ == "__main__":
    main()
