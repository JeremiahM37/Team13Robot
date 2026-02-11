#!/usr/bin/env python3
"""
Wheel Diagnostic Part 2 - Find the actual movement range.
Tests values above AND below center with bigger jumps.
"""

import sys
import time
import signal
from config import SERVO_CHANNELS, MAESTRO_PORT
import maestro

servo = None

def emergency_stop(*args):
    print("\n\nEMERGENCY STOP")
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

signal.signal(signal.SIGINT, emergency_stop)
signal.signal(signal.SIGTERM, emergency_stop)


def safe_pulse(channel, value, duration=0.8):
    servo.setTarget(channel, value)
    time.sleep(duration)
    servo.setTarget(channel, 6000)
    time.sleep(0.3)


def main():
    global servo
    servo = maestro.Controller(MAESTRO_PORT)

    for ch in range(6):
        servo.setTarget(ch, 6000)
    time.sleep(0.5)

    print("=" * 50)
    print("WHEEL RANGE FINDER")
    print("=" * 50)
    print("We know 7000 works. Now let's find reverse")
    print("and the minimum speed that works.\n")
    print("Press Ctrl+C to emergency stop.\n")

    # Test BELOW center for reverse
    print("--- Testing REVERSE (below 6000) on channel 0 (left) ---\n")
    reverse_tests = [4000, 4500, 5000, 5500]
    for val in reverse_tests:
        input(f"Press ENTER to test {val} on LEFT wheel...")
        print(f"  Pulsing {val}...")
        safe_pulse(0, val)
        print(f"  Stopped.\n")

    # Test ABOVE center to find minimum forward speed
    print("--- Testing FORWARD threshold on channel 0 (left) ---\n")
    forward_tests = [6600, 6700, 6800, 6900, 7000]
    for val in forward_tests:
        input(f"Press ENTER to test {val} on LEFT wheel...")
        print(f"  Pulsing {val}...")
        safe_pulse(0, val)
        print(f"  Stopped.\n")

    # Same for right wheel
    print("--- Testing channel 1 (right) - reverse ---\n")
    for val in reverse_tests:
        input(f"Press ENTER to test {val} on RIGHT wheel...")
        print(f"  Pulsing {val}...")
        safe_pulse(1, val)
        print(f"  Stopped.\n")

    print("--- Testing channel 1 (right) - forward ---\n")
    for val in forward_tests:
        input(f"Press ENTER to test {val} on RIGHT wheel...")
        print(f"  Pulsing {val}...")
        safe_pulse(1, val)
        print(f"  Stopped.\n")

    print("DONE! Tell me which values made each wheel spin.")
    emergency_stop()


if __name__ == "__main__":
    main()
