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
    'head_tilt': 3,
    'head_pan': 4,
    'waist': 2,
}

# Servo limits (in Maestro units: typically 4000-8000, center ~6000)
# IMPORTANT: Adjust these after testing your specific servos!
SERVO_LIMITS = {
    'left_wheel': {
        'min': 4000,
        'max': 7500,
        'center': 6000,  # Neutral/stop position for continuous rotation
        'forward_min': 5000,  # At or below this = forward (lower = faster)
        'forward_max': 4500,  # Max forward speed
        'reverse_min': 7000,  # At or above this = reverse (higher = faster)
        'reverse_max': 7750,  # Max reverse speed (bumped for torque)
    },
    'right_wheel': {
        'min': 4500,
        'max': 7750,
        'center': 6000,
        'forward_min': 7000,
        'forward_max': 7750,
        'reverse_min': 5000,
        'reverse_max': 4500,
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
    "Hello, Jeremiah.",
    "I need more oil.",
    "Please do not touch my wheels.",
    "I am very appreciative of Jeremiah Mackey.",
]
