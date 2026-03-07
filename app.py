#!/usr/bin/env python3
"""
Flask Control Server for Robot - Project 2: Dialog Engine

This server:
- Serves the HTML/JavaScript control interface
- Accepts control commands from the webpage
- Runs the TangoChat dialog engine with action tags
- Handles safety (timeout, connection loss, rate limiting)

Usage:
    python app.py                          # Run with real hardware
    python app.py --sim                    # Run in simulation mode
    python app.py --script myscript.txt    # Use custom script file
    python app.py --seed 42               # Deterministic random choices
"""

import os
import sys
import time
import subprocess

from threading import Lock, Thread
from flask import Flask, render_template, request, jsonify

from config import (
    FLASK_HOST, FLASK_PORT, COMMAND_TIMEOUT_MS,
    MAX_COMMANDS_PER_SECOND, VOICE_PHRASES
)
from robot_control import RobotController, SafetyWatchdog
from dialog_engine import DialogEngine
from action_runner import ActionRunner

# Initialize Flask app
app = Flask(__name__)

# Piper voice model path

# Global state (initialized in main)
robot = None
watchdog = None
dialog_engine = None
action_runner = None

# Rate limiting
command_times = []
rate_limit_lock = Lock()


def speak_text(text):
    """Speak text using espeak in a background thread."""
    def _speak():
        try:
            subprocess.Popen(
                ['espeak', '-v', 'en-us', '-s', '160', text],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
        except Exception as e:
            print(f"[TTS ERROR] {e}")

    Thread(target=_speak, daemon=True).start()
    print(f"[SPEAK] {text}")


def check_rate_limit():
    global command_times
    with rate_limit_lock:
        now = time.time()
        command_times = [t for t in command_times if now - t < 1.0]
        if len(command_times) >= MAX_COMMANDS_PER_SECOND:
            return False
        command_times.append(now)
        return True


def validate_float(value, min_val, max_val, name):
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
    return render_template('index.html', phrases=VOICE_PHRASES)


@app.route('/api/drive', methods=['POST'])
def drive():
    if not check_rate_limit():
        return jsonify({'success': False, 'error': 'Rate limited'}), 429
    data = request.get_json()
    if not data:
        return jsonify({'success': False, 'error': 'No data provided'}), 400
    x, error = validate_float(data.get('x'), -1.0, 1.0, 'x')
    if error:
        return jsonify({'success': False, 'error': error}), 400
    y, error = validate_float(data.get('y'), -1.0, 1.0, 'y')
    if error:
        return jsonify({'success': False, 'error': error}), 400
    success = robot.drive_joystick(x, y)
    return jsonify({'success': success})


@app.route('/api/head/tilt', methods=['POST'])
def head_tilt():
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
    speak_text(phrase)
    return jsonify({'success': True, 'phrase': phrase})


@app.route('/api/dialog', methods=['POST'])
def dialog():
    data = request.get_json()
    if not data:
        return jsonify({'success': False, 'error': 'No data provided'}), 400

    user_input = data.get('text', '').strip()
    if not user_input:
        return jsonify({'success': False, 'error': 'Empty input'}), 400

    print(f"\n[INPUT] User: {user_input}")

    # Process through dialog engine
    response, actions = dialog_engine.process_input(user_input)

    # Speak the response via TTS
    if response:
        speak_text(response)

    # Enqueue actions for background execution (never blocks Flask)
    if actions:
        action_runner.enqueue(actions)

    return jsonify({
        'success': True,
        'response': response or '',
        'actions': [a for a in actions if not a.startswith('__')],
        'state': dialog_engine.get_state(),
        'scope_depth': dialog_engine.get_scope_depth(),
    })


@app.route('/api/stop', methods=['POST'])
def stop():
    robot.stop_wheels()
    if action_runner:
        action_runner.interrupt()
    return jsonify({'success': True})


@app.route('/api/heartbeat', methods=['POST'])
def heartbeat():
    robot.last_command_time = time.time()
    return jsonify({
        'success': True,
        'status': robot.get_status(),
        'engine_state': dialog_engine.get_state() if dialog_engine else 'N/A',
    })


@app.route('/api/status', methods=['GET'])
def status():
    result = robot.get_status()
    if dialog_engine:
        result['engine_state'] = dialog_engine.get_state()
        result['scope_depth'] = dialog_engine.get_scope_depth()
    return jsonify(result)


# ==================== ERROR HANDLERS ====================

@app.errorhandler(404)
def not_found(e):
    return jsonify({'error': 'Not found'}), 404


@app.errorhandler(500)
def server_error(e):
    if robot:
        robot.stop_wheels()
    return jsonify({'error': 'Internal server error'}), 500


# ==================== MAIN ====================

def main():
    global robot, watchdog, dialog_engine, action_runner

    simulation_mode = '--sim' in sys.argv

    # Parse --script argument
    script_file = 'test_script.txt'
    if '--script' in sys.argv:
        idx = sys.argv.index('--script')
        if idx + 1 < len(sys.argv):
            script_file = sys.argv[idx + 1]

    # Parse --seed argument
    seed = None
    if '--seed' in sys.argv:
        idx = sys.argv.index('--seed')
        if idx + 1 < len(sys.argv):
            try:
                seed = int(sys.argv[idx + 1])
            except ValueError:
                print("WARNING: Invalid seed value, using random")

    print("=" * 50)
    print("Robot Control Server - Project 2: Dialog Engine")
    print("=" * 50)

    # Initialize robot controller
    robot = RobotController(simulation_mode=simulation_mode)

    # Initialize dialog engine
    dialog_engine = DialogEngine(seed=seed)
    if not dialog_engine.load_script(script_file):
        print(f"\nFATAL: Could not load script '{script_file}'. Exiting.")
        sys.exit(1)

    # Initialize action runner
    action_runner = ActionRunner(robot)
    action_runner.start()

    # Start safety watchdog
    watchdog = SafetyWatchdog(robot, timeout_ms=COMMAND_TIMEOUT_MS)
    watchdog.start()

    print(f"\nServer starting on http://{FLASK_HOST}:{FLASK_PORT}")
    print(f"Script: {script_file}")
    if seed is not None:
        print(f"Deterministic mode: seed={seed}")
    print(f"Connect from your phone/laptop to control the robot.")
    print("\nPress Ctrl+C to stop the server.\n")

    try:
        app.run(
            host=FLASK_HOST,
            port=FLASK_PORT,
            debug=False,
            threaded=True
        )
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        if action_runner:
            action_runner.stop()
        if watchdog:
            watchdog.stop()
        if robot:
            robot.stop_all()
            robot.close()
        print("Server stopped.")


if __name__ == '__main__':
    main()
