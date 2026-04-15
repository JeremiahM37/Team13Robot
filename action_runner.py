"""
Action Runner - Maps action tag names to robot movement primitives.

Executes actions via a background queue so Flask never blocks.
Enforces bounded-time actions and wheel deadman stop.
"""

import time
import threading
from queue import Queue, Empty

from config import SERVO_LIMITS


# Action time caps (seconds)
ACTION_CAPS = {
    'head_yes': 3.0,
    'head_no': 3.0,
    'arm_raise': 4.0,
    'dance90': 6.0,
}

KNOWN_ACTIONS = set(ACTION_CAPS.keys())


class ActionRunner:
    """
    Background action executor. Accepts action names, maps them to
    robot primitives, and runs them sequentially in a worker thread.
    """

    def __init__(self, robot):
        self.robot = robot
        self.queue = Queue()
        self.running = False
        self.thread = None
        self.interrupted = False  # Flag to cancel current action sequence

    def start(self):
        """Start the background worker thread."""
        self.running = True
        self.interrupted = False
        self.thread = threading.Thread(target=self._worker_loop, daemon=True)
        self.thread.start()
        print("[ActionRunner] Worker started")

    def stop(self):
        """Stop the worker thread."""
        self.running = False
        self.interrupt()
        if self.thread:
            self.thread.join(timeout=2.0)

    def enqueue(self, actions):
        """Add a list of action names to the queue."""
        for action in actions:
            if action == '__stop__':
                self.interrupt()
                return
            if action in KNOWN_ACTIONS:
                self.queue.put(action)
            else:
                print(f"[ActionRunner] WARNING: Unknown action tag <{action}>, ignoring")

    def interrupt(self):
        """Interrupt current action and clear the queue."""
        self.interrupted = True
        # Drain the queue
        while not self.queue.empty():
            try:
                self.queue.get_nowait()
            except Empty:
                break
        # Stop all robot motion immediately
        self.robot.stop_wheels()
        print("[ActionRunner] Interrupted - all actions cancelled, wheels stopped")

    def _worker_loop(self):
        """Main worker loop - pulls actions from queue and executes them."""
        while self.running:
            try:
                action_name = self.queue.get(timeout=0.5)
            except Empty:
                continue

            self.interrupted = False
            print(f"[ACTION] Started: {action_name}")
            start_time = time.time()

            try:
                if action_name == 'head_yes':
                    self._do_head_yes()
                elif action_name == 'head_no':
                    self._do_head_no()
                elif action_name == 'arm_raise':
                    self._do_arm_raise()
                elif action_name == 'dance90':
                    self._do_dance90()
            except Exception as e:
                print(f"[ACTION] Error during {action_name}: {e}")
                # Deadman: always stop wheels on error
                self.robot.stop_wheels()

            elapsed = time.time() - start_time
            print(f"[ACTION] Ended: {action_name} ({elapsed:.1f}s)")

    def _check_interrupt(self):
        """Check if we've been interrupted. Returns True if should abort."""
        return self.interrupted

    def _safe_sleep(self, duration):
        """Sleep in small increments, checking for interrupt."""
        end = time.time() + duration
        while time.time() < end:
            if self._check_interrupt():
                return False
            time.sleep(0.05)
        return True

    # ==================== ACTION IMPLEMENTATIONS ====================

    def _do_head_yes(self):
        """Nod yes: tilt down, up, center."""
        if self._check_interrupt():
            return
        # Tilt down
        self.robot.set_head_tilt(0.2)
        if not self._safe_sleep(0.4):
            return
        # Tilt up
        self.robot.set_head_tilt(0.8)
        if not self._safe_sleep(0.4):
            return
        # Tilt down again
        self.robot.set_head_tilt(0.2)
        if not self._safe_sleep(0.4):
            return
        # Back to center
        self.robot.set_head_tilt(0.5)

    def _do_head_no(self):
        """Shake no: pan left, right, center."""
        if self._check_interrupt():
            return
        # Pan left
        self.robot.set_head_pan(0.2)
        if not self._safe_sleep(0.4):
            return
        # Pan right
        self.robot.set_head_pan(0.8)
        if not self._safe_sleep(0.4):
            return
        # Pan left again
        self.robot.set_head_pan(0.2)
        if not self._safe_sleep(0.4):
            return
        # Back to center
        self.robot.set_head_pan(0.5)

    def _do_arm_raise(self):
        """Raise right arm using shoulder1 (ch5) + shoulder2 (ch6), hold, return."""
        if self._check_interrupt():
            return

        limits1 = SERVO_LIMITS['right_shoulder1']
        center1 = limits1['center']  # 9000
        limits2 = SERVO_LIMITS['right_shoulder2']
        center2 = limits2['center']  # 7000

        # Shoulder1 toward max (inward) + shoulder2 toward max (outward)
        # Combined should lift the arm up
        raise1 = center1 + (limits1['max'] - center1) * 0.4  # 9000 -> ~9400
        raise2 = center2 + (limits2['max'] - center2) * 0.4  # 7000 -> ~8200

        self.robot._set_servo('right_shoulder1', int(raise1))
        self.robot._set_servo('right_shoulder2', int(raise2))

        if not self._safe_sleep(1.5):
            self.robot._set_servo('right_shoulder1', center1)
            self.robot._set_servo('right_shoulder2', center2)
            return

        # Return to neutral
        self.robot._set_servo('right_shoulder1', center1)
        self.robot._set_servo('right_shoulder2', center2)
        self._safe_sleep(0.3)

    def _do_dance90(self):
        """
        Rotate in place left ~90 degrees, then right ~90 degrees,
        then return to starting heading (net zero).
        Uses wheel deadman — always stops wheels even on error.
        """
        try:
            if self._check_interrupt():
                return

            # Spin left (~90 degrees) — left wheel backward, right wheel forward
            self.robot.drive(-0.6, 0.6)
            if not self._safe_sleep(1.2):
                return

            self.robot.stop_wheels()
            if not self._safe_sleep(0.3):
                return

            if self._check_interrupt():
                return

            # Spin right (~90 degrees back, then another 90) — total 180 right
            self.robot.drive(0.6, -0.6)
            if not self._safe_sleep(2.4):
                return

            self.robot.stop_wheels()
            if not self._safe_sleep(0.3):
                return

            if self._check_interrupt():
                return

            # Spin left back to start (~90 degrees)
            self.robot.drive(-0.6, 0.6)
            if not self._safe_sleep(1.2):
                return

        finally:
            # DEADMAN: always stop wheels
            self.robot.stop_wheels()
