/**
 * Robot Control Interface
 * Handles communication with Flask server and UI updates.
 * Includes dialog engine text input for Project 2.
 */

// Configuration
const API_BASE = '';  // Same origin
const HEARTBEAT_INTERVAL = 200;  // ms
const DRIVE_UPDATE_INTERVAL = 50;  // ms

// State
let connected = false;
let lastDriveCommand = { x: 0, y: 0 };
let driveUpdateTimer = null;
let heartbeatTimer = null;

// DOM Elements
const statusEl = document.getElementById('status');
const engineStateEl = document.getElementById('engine-state');
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

// Dialog elements
const dialogInput = document.getElementById('dialog-input');
const dialogSendBtn = document.getElementById('dialog-send');
const dialogLog = document.getElementById('dialog-log');

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
    if (typeof x !== 'number' || typeof y !== 'number') {
        console.error('Invalid drive values');
        return;
    }

    x = Math.max(-1, Math.min(1, x));
    y = Math.max(-1, Math.min(1, y));

    const result = await apiCall('/api/drive', { x, y });
    if (result.success) {
        setConnectionStatus(true);
    }
    return result;
}

async function sendHeadTilt(position) {
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
            // Update engine state display
            if (data.status && engineStateEl) {
                engineStateEl.textContent = 'State: ' + (data.engine_state || 'N/A');
            }
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

// ==================== Dialog Functions ====================

function addDialogMessage(text, className) {
    const div = document.createElement('div');
    div.className = className;
    div.textContent = text;
    dialogLog.appendChild(div);
    dialogLog.scrollTop = dialogLog.scrollHeight;
}

async function sendDialogInput() {
    const text = dialogInput.value.trim();
    if (!text) return;

    // Show user message
    addDialogMessage('You: ' + text, 'user-msg');
    dialogInput.value = '';

    // Send to server
    const result = await apiCall('/api/dialog', { text: text });

    if (result.success) {
        if (result.response) {
            addDialogMessage('Robot: ' + result.response, 'robot-msg');
        }
        if (result.actions && result.actions.length > 0) {
            addDialogMessage('Actions: ' + result.actions.map(a => '<' + a + '>').join(' '), 'action-msg');
        }
        // Update state display
        if (engineStateEl) {
            engineStateEl.textContent = 'State: ' + (result.state || 'N/A');
        }
    } else {
        addDialogMessage('Error: ' + (result.error || 'Unknown error'), 'system-msg');
    }
}

// Dialog event listeners
dialogSendBtn.addEventListener('click', sendDialogInput);
dialogInput.addEventListener('keydown', (e) => {
    if (e.key === 'Enter') {
        sendDialogInput();
    }
});

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

        if (!driveUpdateTimer) {
            driveUpdateTimer = setInterval(() => {
                sendDriveCommand(lastDriveCommand.x, lastDriveCommand.y);
            }, DRIVE_UPDATE_INTERVAL);

            sendDriveCommand(x, y);
        }
    },

    onEnd: () => {
        if (driveUpdateTimer) {
            clearInterval(driveUpdateTimer);
            driveUpdateTimer = null;
        }

        lastDriveCommand = { x: 0, y: 0 };
        updateJoystickDisplay(0, 0);
        sendDriveCommand(0, 0);
    }
});

// ==================== Slider Setup ====================

headTiltEl.addEventListener('input', (e) => {
    const value = parseInt(e.target.value);
    headTiltValueEl.textContent = value + '%';
    sendHeadTilt(value);
});

headPanEl.addEventListener('input', (e) => {
    const value = parseInt(e.target.value);
    headPanValueEl.textContent = value + '%';
    sendHeadPan(value);
});

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

    if (driveUpdateTimer) {
        clearInterval(driveUpdateTimer);
        driveUpdateTimer = null;
    }
    lastDriveCommand = { x: 0, y: 0 };
    updateJoystickDisplay(0, 0);

    addDialogMessage('EMERGENCY STOP', 'system-msg');
});

// ==================== Safety: Page Visibility ====================

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
function sendStopBeacon() {
    const blob = new Blob(['{}'], { type: 'application/json' });
    navigator.sendBeacon(API_BASE + '/api/stop', blob);
}

window.addEventListener('beforeunload', sendStopBeacon);
window.addEventListener('pagehide', sendStopBeacon);

// ==================== Heartbeat ====================

function startHeartbeat() {
    sendHeartbeat();
    heartbeatTimer = setInterval(sendHeartbeat, HEARTBEAT_INTERVAL);
}

// ==================== Initialization ====================

function init() {
    console.log('Robot Control Interface initialized');
    setConnectionStatus(false);
    startHeartbeat();
}

if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
} else {
    init();
}
