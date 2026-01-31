#!/usr/bin/env python3
"""
Flask Control Server for Robot

This server:
- Serves the HTML/JavaScript control interface
- Accepts control commands from the webpage
- Validates all commands before forwarding to robot control layer
- Handles safety (timeout, connection loss, rate limiting)

Usage:
    python app.py              # Run with real hardware
    python app.py --sim        # Run in simulation mode
"""

import os
import sys
import time
import subprocess
from threading import Lock
from flask import Flask, render_template, request, jsonify

from config import (
    FLASK_HOST, FLASK_PORT, COMMAND_TIMEOUT_MS,
    MAX_COMMANDS_PER_SECOND, VOICE_PHRASES
)
from robot_control import RobotController, SafetyWatchdog

# Initialize Flask app
app = Flask(__name__)

# Global robot controller (initialized in main)
robot = None
watchdog = None

# Rate limiting
command_times = []
rate_limit_lock = Lock()


def check_rate_limit():
    """
    Check if we're receiving too many commands.
    Returns True if command should be allowed, False if rate limited.
    """
    global command_times

    with rate_limit_lock:
        now = time.time()
        # Remove commands older than 1 second
        command_times = [t for t in command_times if now - t < 1.0]

        if len(command_times) >= MAX_COMMANDS_PER_SECOND:
            return False

        command_times.append(now)
        return True


def validate_float(value, min_val, max_val, name):
    """
    Validate and convert a value to float within range.
    Returns (value, error_message) tuple.
    """
    try:
        val = float(value)
        if val < min_val or val > max_val:
            return None, f"{name} out of range [{min_val}, {max_val}]"
        return val, None
    except (TypeError, ValueError):
        return None, f"Invalid {name} value"


# ==================== ROUTES ====================

@app.route('/')
def index():
    """Serve the main control page."""
    return render_template('index.html', phrases=VOICE_PHRASES)


@app.route('/api/drive', methods=['POST'])
def drive():
    """
    Handle joystick drive commands.

    Expected JSON: { "x": float, "y": float }
    x: -1 to 1 (left/right)
    y: -1 to 1 (forward/backward)
    """
    if not check_rate_limit():
        return jsonify({'success': False, 'error': 'Rate limited'}), 429

    data = request.get_json()
    if not data:
        return jsonify({'success': False, 'error': 'No data provided'}), 400

    # Validate x
    x, error = validate_float(data.get('x'), -1.0, 1.0, 'x')
    if error:
        return jsonify({'success': False, 'error': error}), 400

    # Validate y
    y, error = validate_float(data.get('y'), -1.0, 1.0, 'y')
    if error:
        return jsonify({'success': False, 'error': error}), 400

    # Execute command
    success = robot.drive_joystick(x, y)
    return jsonify({'success': success})


@app.route('/api/head/tilt', methods=['POST'])
def head_tilt():
    """
    Handle head tilt commands.

    Expected JSON: { "position": float }
    position: 0 to 1
    """
    if not check_rate_limit():
        return jsonify({'success': False, 'error': 'Rate limited'}), 429

    data = request.get_json()
    if not data:
        return jsonify({'success': False, 'error': 'No data provided'}), 400

    position, error = validate_float(data.get('position'), 0.0, 1.0, 'position')
    if error:
        return jsonify({'success': False, 'error': error}), 400

    success = robot.set_head_tilt(position)
    return jsonify({'success': success})


@app.route('/api/head/pan', methods=['POST'])
def head_pan():
    """
    Handle head pan commands.

    Expected JSON: { "position": float }
    position: 0 to 1
    """
    if not check_rate_limit():
        return jsonify({'success': False, 'error': 'Rate limited'}), 429

    data = request.get_json()
    if not data:
        return jsonify({'success': False, 'error': 'No data provided'}), 400

    position, error = validate_float(data.get('position'), 0.0, 1.0, 'position')
    if error:
        return jsonify({'success': False, 'error': error}), 400

    success = robot.set_head_pan(position)
    return jsonify({'success': success})


@app.route('/api/waist', methods=['POST'])
def waist():
    """
    Handle waist rotation commands.

    Expected JSON: { "position": float }
    position: 0 to 1
    """
    if not check_rate_limit():
        return jsonify({'success': False, 'error': 'Rate limited'}), 429

    data = request.get_json()
    if not data:
        return jsonify({'success': False, 'error': 'No data provided'}), 400

    position, error = validate_float(data.get('position'), 0.0, 1.0, 'position')
    if error:
        return jsonify({'success': False, 'error': error}), 400

    success = robot.set_waist(position)
    return jsonify({'success': success})


@app.route('/api/speak', methods=['POST'])
def speak():
    """
    Handle voice output commands.

    Expected JSON: { "phrase_index": int }
    """
    data = request.get_json()
    if not data:
        return jsonify({'success': False, 'error': 'No data provided'}), 400

    try:
        index = int(data.get('phrase_index', -1))
    except (TypeError, ValueError):
        return jsonify({'success': False, 'error': 'Invalid phrase index'}), 400

    if index < 0 or index >= len(VOICE_PHRASES):
        return jsonify({'success': False, 'error': 'Phrase index out of range'}), 400

    phrase = VOICE_PHRASES[index]

    # Use espeak for text-to-speech (common on Raspberry Pi)
    try:
        subprocess.Popen(
            ['espeak', '-v', 'en', phrase],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        print(f"Speaking: {phrase}")
        return jsonify({'success': True, 'phrase': phrase})
    except FileNotFoundError:
        # espeak not installed, just log it
        print(f"[No TTS] Would say: {phrase}")
        return jsonify({'success': True, 'phrase': phrase, 'note': 'TTS not available'})
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/api/stop', methods=['POST'])
def stop():
    """Emergency stop - stop all motion."""
    robot.stop_wheels()
    return jsonify({'success': True})


@app.route('/api/heartbeat', methods=['POST'])
def heartbeat():
    """
    Heartbeat endpoint - client should call this regularly.
    Updates the last command time to prevent watchdog timeout.
    """
    robot.last_command_time = time.time()
    return jsonify({'success': True, 'status': robot.get_status()})


@app.route('/api/status', methods=['GET'])
def status():
    """Get current robot status."""
    return jsonify(robot.get_status())


# ==================== ERROR HANDLERS ====================

@app.errorhandler(404)
def not_found(e):
    return jsonify({'error': 'Not found'}), 404


@app.errorhandler(500)
def server_error(e):
    # Safety: stop robot on server error
    if robot:
        robot.stop_wheels()
    return jsonify({'error': 'Internal server error'}), 500


# ==================== MAIN ====================

def main():
    global robot, watchdog

    simulation_mode = '--sim' in sys.argv

    print("=" * 50)
    print("Robot Control Server")
    print("=" * 50)

    # Initialize robot controller
    robot = RobotController(simulation_mode=simulation_mode)

    # Start safety watchdog
    watchdog = SafetyWatchdog(robot, timeout_ms=COMMAND_TIMEOUT_MS)
    watchdog.start()

    # Print access information
    print(f"\nServer starting on http://{FLASK_HOST}:{FLASK_PORT}")
    print(f"Connect from your phone/laptop to control the robot.")
    print("\nPress Ctrl+C to stop the server.\n")

    try:
        # Run Flask server
        app.run(
            host=FLASK_HOST,
            port=FLASK_PORT,
            debug=False,  # Set True for development
            threaded=True
        )
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        if watchdog:
            watchdog.stop()
        if robot:
            robot.stop_all()
            robot.close()
        print("Server stopped.")


if __name__ == '__main__':
    main()
