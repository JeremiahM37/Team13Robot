"""
Configuration file for robot control system.
Adjust these values based on your specific hardware setup.
"""

# Flask server settings
FLASK_HOST = '0.0.0.0'  # Listen on all interfaces
FLASK_PORT = 8080       # Using 8080 since 5000 may be blocked by network

# Maestro servo controller settings
MAESTRO_PORT = '/dev/ttyACM0'  # Adjust based on your setup

# Servo channel assignments (adjust to match your wiring)
SERVO_CHANNELS = {
    'left_wheel': 0,
    'right_wheel': 1,
    'head_tilt': 3,
    'head_pan': 4,
    'waist': 2,
    # Right arm channels 5-10
    'right_shoulder1': 5,
    'right_shoulder2': 6,
    'right_elbow': 7,
    'right_wrist_bend': 8,
    'right_wrist_rotate': 9,
    'right_gripper': 10,
    # Left arm channels 11-16
    'left_shoulder1': 11,
    'left_shoulder2': 12,
    'left_elbow': 13,
    'left_wrist_bend': 14,
    'left_wrist_rotate': 15,
    'left_gripper': 16,
}

# Servo limits (in Maestro units: typically 4000-8000, center ~6000)
# IMPORTANT: Adjust these after testing your specific servos!
SERVO_LIMITS = {
    'left_wheel': {
        'min': 4000,
        'max': 7500,
        'center': 6000,  # Neutral/stop position for continuous rotation
        'forward_min': 5000,  # At or below this = forward (lower = faster)
        'forward_max': 4800,  # Max forward speed (was 4500)
        'reverse_min': 7000,  # At or above this = reverse (higher = faster)
        'reverse_max': 7200,  # Max reverse speed (was 7750)
    },
    'right_wheel': {
        'min': 4500,
        'max': 7750,
        'center': 6000,
        'forward_min': 7000,
        'forward_max': 7200,  # Max forward speed (was 7750)
        'reverse_min': 5000,
        'reverse_max': 4800,  # Max reverse speed (was 4500)
    },
    'head_tilt': {
        'min': 7500,
        'max': 4500,
        'center': 6000,
    },
    'head_pan': {
        'min': 7500,
        'max': 4500,
        'center': 6000,
    },
    'waist': {
        'min': 7500,
        'max': 4500,
        'center': 6000,
    },
    # Right arm - calibrated values
    'right_shoulder1': {
        'min': 3000,
        'max': 10000,
        'center': 9000,
    },
    'right_shoulder2': {
        'min': 3000,
        'max': 10000,
        'center': 7000,
    },
    'right_elbow': {
        'min': 3000,
        'max': 10000,
        'center': 7000,
    },
    'right_wrist_bend': {
        'min': 3000,
        'max': 10000,
        'center': 7000,
    },
    'right_wrist_rotate': {
        'min': 3000,
        'max': 10000,
        'center': 6000,
    },
    'right_gripper': {
        'min': 3000,
        'max': 10000,
        'center': 6000,
    },
    # Left arm - defaults, calibrate later
    'left_shoulder1': {
        'min': 3000,
        'max': 10000,
        'center': 6000,
    },
    'left_shoulder2': {
        'min': 3000,
        'max': 10000,
        'center': 6000,
    },
    'left_elbow': {
        'min': 3000,
        'max': 10000,
        'center': 6000,
    },
    'left_wrist_bend': {
        'min': 3000,
        'max': 10000,
        'center': 6000,
    },
    'left_wrist_rotate': {
        'min': 3000,
        'max': 10000,
        'center': 6000,
    },
    'left_gripper': {
        'min': 3000,
        'max': 10000,
        'center': 6000,
    },
}

# Safety settings
COMMAND_TIMEOUT_MS = 500      # Stop robot if no command received in this time
HEARTBEAT_INTERVAL_MS = 200   # How often client should send heartbeat
MAX_COMMANDS_PER_SECOND = 20  # Rate limiting

# Voice phrases (customize as desired)
VOICE_PHRASES = [
    "Hello world.",
    "All systems functional.",
    "Waiting for command.",
    "Task complete.",
]
