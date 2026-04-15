#!/usr/bin/env python3
"""
Quick script to find the right center value for a servo.
Type values to move the servo, Ctrl+C to stop.
"""

import sys
import signal
import time
from config import MAESTRO_PORT

import maestro

controller = None

def stop(*args):
    if controller:
        controller.setTarget(CHANNEL, 6000)
        controller.close()
    sys.exit(0)

signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)

# Default to left shoulder 1 (channel 11), or pass a channel as argument
CHANNEL = int(sys.argv[1]) if len(sys.argv) > 1 else 11

controller = maestro.Controller(MAESTRO_PORT)
print(f"Connected. Testing channel {CHANNEL}")
print(f"Currently at 6000. Type a value to move, q to quit.\n")

current = 6000
controller.setTarget(CHANNEL, current)

while True:
    raw = input(f"  [ch {CHANNEL} @ {current}] > ").strip()
    if raw in ('q', 'quit'):
        controller.setTarget(CHANNEL, 6000)
        controller.close()
        break
    try:
        val = int(raw)
        if val < 3000 or val > 10000:
            print("  Stay between 3000-10000")
            continue
        controller.setTarget(CHANNEL, val)
        current = val
        print(f"  -> {val}")
    except ValueError:
        print("  Type a number or q")
