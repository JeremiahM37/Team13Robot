#!/usr/bin/env python3
"""
Wheel Diagnostic Script (SAFE version)

Every movement is a short pulse that ALWAYS stops automatically.
No user input during movement - wheels stop first, then we ask questions.
"""

import sys
import time
import signal
from config import SERVO_CHANNELS, MAESTRO_PORT

try:
    import maestro
except ImportError:
    print("ERROR: maestro module not found")
    sys.exit(1)

servo = None

def emergency_stop(*args):
    """Stop everything on Ctrl+C or exit."""
    print("\n\nEMERGENCY STOP - killing all channels")
    if servo:
        for ch in range(6):
            try:
                servo.setTarget(ch, 6000)
            except:
                pass
        try:
            servo.close()
        except:
            pass
    sys.exit(0)

# Register emergency stop
signal.signal(signal.SIGINT, emergency_stop)
signal.signal(signal.SIGTERM, emergency_stop)


def safe_pulse(channel, value, duration=0.8):
    """Send a value for a short duration, then ALWAYS return to 6000 (stop)."""
    servo.setTarget(channel, value)
    time.sleep(duration)
    servo.setTarget(channel, 6000)
    time.sleep(0.3)


def main():
    global servo

    print("=" * 50)
    print("WHEEL DIAGNOSTIC (SAFE VERSION)")
    print("=" * 50)
    print("All movements are short 0.8s pulses.")
    print("Press Ctrl+C at any time to emergency stop.\n")

    try:
        servo = maestro.Controller(MAESTRO_PORT)
        print(f"Connected to Maestro on {MAESTRO_PORT}")
    except Exception as e:
        print(f"ERROR: Could not connect: {e}")
        sys.exit(1)

    # Make sure everything is stopped first
    for ch in range(6):
        servo.setTarget(ch, 6000)
    time.sleep(0.5)

    left_ch = SERVO_CHANNELS['left_wheel']
    right_ch = SERVO_CHANNELS['right_wheel']

    print(f"\nLeft wheel = channel {left_ch}")
    print(f"Right wheel = channel {right_ch}")

    # --- STEP 1: Quick channel scan ---
    print("\n" + "=" * 50)
    print("STEP 1: Channel scan (0.8s pulse each)")
    print("=" * 50)
    print("Watch what moves on each channel.\n")

    for ch in range(6):
        input(f"Press ENTER to pulse channel {ch}...")
        print(f"  Pulsing channel {ch} with value 7000...")
        safe_pulse(ch, 7000)
        print(f"  Stopped. What moved? (remember for later)\n")

    # --- STEP 2: Test wheel speeds ---
    print("=" * 50)
    print("STEP 2: Which value made the wheel spin?")
    print("=" * 50)

    response = input("\nDid any channel spin a wheel? (y/n): ").strip().lower()
    if response != 'y':
        print("\nNo wheel movement detected. Possible issues:")
        print("  - Wheels not connected to channels 0-5")
        print("  - Wheels need different signal range")
        print("  - Wiring issue")
        emergency_stop()

    which = input("Which channel number spun a wheel? ").strip()
    try:
        wheel_ch = int(which)
    except ValueError:
        print("Invalid channel number.")
        emergency_stop()

    print(f"\nTesting channel {wheel_ch} with different speeds (short pulses):\n")

    test_values = [
        (5000, "5000 (reverse fast)"),
        (5500, "5500 (reverse slow)"),
        (6000, "6000 (should be stop)"),
        (6500, "6500 (forward slow)"),
        (7000, "7000 (forward fast)"),
    ]

    for value, label in test_values:
        input(f"Press ENTER to test {label}...")
        print(f"  Pulsing...")
        safe_pulse(wheel_ch, value)
        print(f"  Stopped.\n")

    # --- Done ---
    print("=" * 50)
    print("DIAGNOSTIC COMPLETE")
    print("=" * 50)
    print("Use the values that worked to update config.py")

    emergency_stop()


if __name__ == "__main__":
    main()
