/**
 * Virtual Joystick Implementation
 * Supports both mouse and touch input for mobile devices.
 */

class VirtualJoystick {
    constructor(containerId, knobId, options = {}) {
        this.container = document.getElementById(containerId);
        this.knob = document.getElementById(knobId);

        if (!this.container || !this.knob) {
            console.error('Joystick elements not found');
            return;
        }

        // Configuration
        this.onMove = options.onMove || (() => {});
        this.onEnd = options.onEnd || (() => {});
        this.deadzone = options.deadzone || 0.1;

        // State
        this.active = false;
        this.x = 0;
        this.y = 0;

        // Calculate dimensions
        this.updateDimensions();

        // Bind event handlers
        this.bindEvents();

        // Handle window resize
        window.addEventListener('resize', () => this.updateDimensions());
    }

    updateDimensions() {
        const rect = this.container.getBoundingClientRect();
        this.centerX = rect.width / 2;
        this.centerY = rect.height / 2;
        this.maxDistance = (rect.width / 2) - (this.knob.offsetWidth / 2);
    }

    bindEvents() {
        // Mouse events
        this.container.addEventListener('mousedown', (e) => this.handleStart(e));
        document.addEventListener('mousemove', (e) => this.handleMove(e));
        document.addEventListener('mouseup', (e) => this.handleEnd(e));

        // Touch events
        this.container.addEventListener('touchstart', (e) => this.handleStart(e), { passive: false });
        document.addEventListener('touchmove', (e) => this.handleMove(e), { passive: false });
        document.addEventListener('touchend', (e) => this.handleEnd(e));
        document.addEventListener('touchcancel', (e) => this.handleEnd(e));
    }

    getEventPosition(e) {
        const rect = this.container.getBoundingClientRect();

        if (e.touches && e.touches.length > 0) {
            return {
                x: e.touches[0].clientX - rect.left,
                y: e.touches[0].clientY - rect.top
            };
        }

        return {
            x: e.clientX - rect.left,
            y: e.clientY - rect.top
        };
    }

    handleStart(e) {
        e.preventDefault();
        this.active = true;
        this.updateDimensions();
        this.handleMove(e);
    }

    handleMove(e) {
        if (!this.active) return;

        e.preventDefault();

        const pos = this.getEventPosition(e);

        // Calculate offset from center
        let deltaX = pos.x - this.centerX;
        let deltaY = pos.y - this.centerY;

        // Calculate distance from center
        const distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        // Constrain to circle
        if (distance > this.maxDistance) {
            const angle = Math.atan2(deltaY, deltaX);
            deltaX = Math.cos(angle) * this.maxDistance;
            deltaY = Math.sin(angle) * this.maxDistance;
        }

        // Update knob position
        this.knob.style.left = (this.centerX + deltaX) + 'px';
        this.knob.style.top = (this.centerY + deltaY) + 'px';
        this.knob.style.transform = 'translate(-50%, -50%)';

        // Normalize to -1 to 1 range
        this.x = deltaX / this.maxDistance;
        this.y = -deltaY / this.maxDistance; // Invert Y so up is positive

        // Apply deadzone
        if (Math.abs(this.x) < this.deadzone) this.x = 0;
        if (Math.abs(this.y) < this.deadzone) this.y = 0;

        // Callback
        this.onMove(this.x, this.y);
    }

    handleEnd(e) {
        if (!this.active) return;

        this.active = false;

        // Reset knob to center
        this.knob.style.left = '50%';
        this.knob.style.top = '50%';
        this.knob.style.transform = 'translate(-50%, -50%)';

        // Reset values
        this.x = 0;
        this.y = 0;

        // Callback
        this.onEnd();
    }

    getPosition() {
        return { x: this.x, y: this.y };
    }
}

// Export for use in control.js
window.VirtualJoystick = VirtualJoystick;
