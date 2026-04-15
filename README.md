# Robot Control System

Browser-based control interface for a physical robot.

## Project Structure

```
robot/
├── app.py              # Flask server (main entry point)
├── robot_control.py    # Robot control layer (hardware abstraction)
├── hardware_test.py    # Hardware verification script (Step A)
├── config.py           # Configuration (servo limits, ports, etc.)
├── requirements.txt    # Python dependencies
├── templates/
│   └── index.html      # Web interface HTML
└── static/
    ├── style.css       # Styling
    ├── joystick.js     # Virtual joystick implementation
    └── control.js      # Main control logic
```

## Quick Start

### 1. Install Dependencies
```bash
pip install flask
# Maestro library should already be installed
```

### 2. Test Hardware (Step A - Required First!)
```bash
python hardware_test.py        # Full test
python hardware_test.py wheels # Test only wheels
python hardware_test.py --sim  # Simulation mode
```

### 3. Run the Server
```bash
python app.py          # With real hardware
python app.py --sim    # Simulation mode (for testing)
```

### 4. Connect from Browser
Open `http://<robot-ip>:5000` on your phone or laptop.

## Configuration

Edit `config.py` to adjust:
- Servo channel assignments
- Servo limits (min/max/center values)
- Flask port
- Safety timeouts
- Voice phrases

## API Endpoints

| Endpoint | Method | Data | Description |
|----------|--------|------|-------------|
| `/` | GET | - | Main control page |
| `/api/drive` | POST | `{x, y}` | Joystick drive (-1 to 1) |
| `/api/head/tilt` | POST | `{position}` | Head tilt (0 to 1) |
| `/api/head/pan` | POST | `{position}` | Head pan (0 to 1) |
| `/api/waist` | POST | `{position}` | Waist rotation (0 to 1) |
| `/api/speak` | POST | `{phrase_index}` | Speak phrase |
| `/api/stop` | POST | `{}` | Emergency stop |
| `/api/heartbeat` | POST | `{}` | Keep-alive |
| `/api/status` | GET | - | Robot status |

## Safety Features

1. **Watchdog Timer**: Stops wheels if no commands received in 500ms
2. **Page Visibility**: Stops robot when browser tab hidden
3. **Page Unload**: Stops robot when page refreshed/closed
4. **Rate Limiting**: Prevents command flooding (20/sec max)
5. **Input Validation**: All commands validated server-side
6. **Safe Limits**: Servo positions clamped to safe range

## Testing Without Hardware

Use `--sim` flag to run in simulation mode:
```bash
python robot_control.py  # Direct control layer test
python app.py --sim      # Full server test
```

## Troubleshooting

- **Can't connect to Maestro**: Check USB connection, port in config.py
- **Servos not moving**: Run hardware_test.py to verify channels
- **Wrong direction**: May need to swap min/max or invert in config
- **Page not loading**: Check Flask is running, firewall allows port
