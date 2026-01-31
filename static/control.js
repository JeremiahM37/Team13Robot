/**
 * Robot Control Interface
 * Handles communication with Flask server and UI updates.
 *
 * DATA FLOW:
 * 1. User interacts with joystick/sliders/buttons
 * 2. JavaScript validates input locally
 * 3. JSON data sent to Flask API endpoints
 * 4. Flask validates data again (server-side)
 * 5. Valid commands forwarded to robot control layer
 * 6. Response returned to browser
 */

// Configuration
const API_BASE = '';  // Same origin
const HEARTBEAT_INTERVAL = 200;  // ms
const DRIVE_UPDATE_INTERVAL = 50;  // ms - how often to send drive commands

// State
let connected = false;
let lastDriveCommand = { x: 0, y: 0 };
let driveUpdateTimer = null;
let heartbeatTimer = null;

// DOM Elements
const statusEl = document.getElementById('status');
const joystickXEl = document.getElementById('joystick-x');
const joystickYEl = document.getElementById('joystick-y');
const headTiltEl = document.getElementById('head-tilt');
const headTiltValueEl = document.getElementById('head-tilt-value');
const headPanEl = document.getElementById('head-pan');
const headPanValueEl = document.getElementById('head-pan-value');
const waistEl = document.getElementById('waist');
const waistValueEl = document.getElementById('waist-value');
const stopBtn = document.getElementById('stop-btn');
const lastUpdateEl = document.getElementById('last-update');

// ==================== API Functions ====================

async function apiCall(endpoint, data) {
    try {
        const response = await fetch(API_BASE + endpoint, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify(data),
        });

        if (response.status === 429) {
            console.warn('Rate limited');
            return { success: false, error: 'Rate limited' };
        }

        return await response.json();
    } catch (error) {
        console.error('API error:', error);
        setConnectionStatus(false);
        return { success: false, error: error.message };
    }
}

async function sendDriveCommand(x, y) {
    // Validate locally first
    if (typeof x !== 'number' || typeof y !== 'number') {
        console.error('Invalid drive values');
        return;
    }

    // Clamp values
    x = Math.max(-1, Math.min(1, x));
    y = Math.max(-1, Math.min(1, y));

    const result = await apiCall('/api/drive', { x, y });
    if (result.success) {
        setConnectionStatus(true);
    }
    return result;
}

async function sendHeadTilt(position) {
    // Validate: 0-100 from slider, convert to 0-1
    const normalized = Math.max(0, Math.min(100, position)) / 100;
    return await apiCall('/api/head/tilt', { position: normalized });
}

async function sendHeadPan(position) {
    const normalized = Math.max(0, Math.min(100, position)) / 100;
    return await apiCall('/api/head/pan', { position: normalized });
}

async function sendWaist(position) {
    const normalized = Math.max(0, Math.min(100, position)) / 100;
    return await apiCall('/api/waist', { position: normalized });
}

async function sendSpeak(phraseIndex) {
    return await apiCall('/api/speak', { phrase_index: phraseIndex });
}

async function sendStop() {
    return await apiCall('/api/stop', {});
}

async function sendHeartbeat() {
    try {
        const response = await fetch(API_BASE + '/api/heartbeat', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: '{}',
        });

        if (response.ok) {
            setConnectionStatus(true);
            const data = await response.json();
            updateLastUpdate();
            return data;
        } else {
            setConnectionStatus(false);
            return null;
        }
    } catch (error) {
        setConnectionStatus(false);
        return null;
    }
}

// ==================== UI Functions ====================

function setConnectionStatus(isConnected) {
    connected = isConnected;
    statusEl.textContent = isConnected ? 'Connected' : 'Disconnected';
    statusEl.className = 'status ' + (isConnected ? 'connected' : 'disconnected');
}

function updateLastUpdate() {
    const now = new Date();
    lastUpdateEl.textContent = now.toLocaleTimeString();
}

function updateJoystickDisplay(x, y) {
    joystickXEl.textContent = x.toFixed(2);
    joystickYEl.textContent = y.toFixed(2);
}

// ==================== Joystick Setup ====================

const joystick = new VirtualJoystick('joystick', 'joystick-knob', {
    deadzone: 0.05,

    onMove: (x, y) => {
        updateJoystickDisplay(x, y);
        lastDriveCommand = { x, y };

        // Start sending updates if not already
        if (!driveUpdateTimer) {
            driveUpdateTimer = setInterval(() => {
                sendDriveCommand(lastDriveCommand.x, lastDriveCommand.y);
            }, DRIVE_UPDATE_INTERVAL);

            // Send immediately
            sendDriveCommand(x, y);
        }
    },

    onEnd: () => {
        // Stop the update timer
        if (driveUpdateTimer) {
            clearInterval(driveUpdateTimer);
            driveUpdateTimer = null;
        }

        // Send stop command
        lastDriveCommand = { x: 0, y: 0 };
        updateJoystickDisplay(0, 0);
        sendDriveCommand(0, 0);
    }
});

// ==================== Slider Setup ====================

// Head Tilt
headTiltEl.addEventListener('input', (e) => {
    const value = parseInt(e.target.value);
    headTiltValueEl.textContent = value + '%';
    sendHeadTilt(value);
});

// Head Pan
headPanEl.addEventListener('input', (e) => {
    const value = parseInt(e.target.value);
    headPanValueEl.textContent = value + '%';
    sendHeadPan(value);
});

// Waist
waistEl.addEventListener('input', (e) => {
    const value = parseInt(e.target.value);
    waistValueEl.textContent = value + '%';
    sendWaist(value);
});

// ==================== Voice Buttons Setup ====================

document.querySelectorAll('.voice-btn').forEach(btn => {
    btn.addEventListener('click', () => {
        const index = parseInt(btn.dataset.phraseIndex);
        sendSpeak(index);
    });
});

// ==================== Stop Button ====================

stopBtn.addEventListener('click', () => {
    sendStop();

    // Also stop any ongoing joystick updates
    if (driveUpdateTimer) {
        clearInterval(driveUpdateTimer);
        driveUpdateTimer = null;
    }
    lastDriveCommand = { x: 0, y: 0 };
    updateJoystickDisplay(0, 0);
});

// ==================== Safety: Page Visibility ====================

// Stop robot when page is hidden (tab switch, minimize, etc.)
document.addEventListener('visibilitychange', () => {
    if (document.hidden) {
        console.log('Page hidden - stopping robot');
        sendStop();

        if (driveUpdateTimer) {
            clearInterval(driveUpdateTimer);
            driveUpdateTimer = null;
        }
    }
});

// Stop robot before page unload (refresh, close, navigate away)
window.addEventListener('beforeunload', () => {
    // Use synchronous request for unload
    navigator.sendBeacon(API_BASE + '/api/stop', JSON.stringify({}));
});

// ==================== Heartbeat ====================

function startHeartbeat() {
    // Initial heartbeat
    sendHeartbeat();

    // Regular heartbeat
    heartbeatTimer = setInterval(sendHeartbeat, HEARTBEAT_INTERVAL);
}

// ==================== Initialization ====================

function init() {
    console.log('Robot Control Interface initialized');
    setConnectionStatus(false);
    startHeartbeat();
}

// Start when DOM is ready
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
} else {
    init();
}
