"""
Configuration file for robot control system.
Adjust these values based on your specific hardware setup.
"""

# Flask server settings
FLASK_HOST = '0.0.0.0'  # Listen on all interfaces
FLASK_PORT = 5000       # Change if needed

# Maestro servo controller settings
MAESTRO_PORT = '/dev/ttyACM0'  # Adjust based on your setup

# Servo channel assignments (adjust to match your wiring)
SERVO_CHANNELS = {
    'left_wheel': 0,
    'right_wheel': 1,
    'head_tilt': 2,
    'head_pan': 3,
    'waist': 4,
}

# Servo limits (in Maestro units: typically 4000-8000, center ~6000)
# IMPORTANT: Adjust these after testing your specific servos!
SERVO_LIMITS = {
    'left_wheel': {
        'min': 4000,
        'max': 8000,
        'center': 6000,  # Neutral/stop position for continuous rotation
    },
    'right_wheel': {
        'min': 4000,
        'max': 8000,
        'center': 6000,
    },
    'head_tilt': {
        'min': 4500,
        'max': 7500,
        'center': 6000,
    },
    'head_pan': {
        'min': 4500,
        'max': 7500,
        'center': 6000,
    },
    'waist': {
        'min': 4500,
        'max': 7500,
        'center': 6000,
    },
}

# Safety settings
COMMAND_TIMEOUT_MS = 500      # Stop robot if no command received in this time
HEARTBEAT_INTERVAL_MS = 200   # How often client should send heartbeat
MAX_COMMANDS_PER_SECOND = 20  # Rate limiting

# Voice phrases (customize as desired)
VOICE_PHRASES = [
    "Hello, Hunter.",
    "Hunter is so cool.",
    "Please do not touch my wheels.",
    "Hunter is the greatest.",
]
