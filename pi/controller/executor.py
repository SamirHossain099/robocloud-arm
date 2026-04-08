from queue import Empty, Queue
from threading import Event
from typing import Dict, Optional, Tuple

from pi.config import (
    BASE_MAX,
    BASE_MIN,
    CLAW_MAX,
    CLAW_MIN,
    ELBOW_MAX,
    ELBOW_MIN,
    SHOULDER_MAX,
    SHOULDER_MIN,
    STEP,
    WRIST_MAX,
    WRIST_MIN,
)
from pi.controller.arm import Arm
from pi.logutil import vprint


def _clamp(value: int, minimum: int, maximum: int) -> int:
    return max(minimum, min(maximum, value))


class CommandRouter:
    def __init__(self) -> None:
        self._high_q: Queue = Queue()
        self._low_q: Queue = Queue()
        self.interrupt_event = Event()

    def submit(self, command: Dict) -> None:
        priority = command.get("priority", "low")
        if priority == "high":
            self.interrupt_event.set()
            self._high_q.put(command)
        else:
            self._low_q.put(command)

    def get_next(self, timeout: float = 0.1) -> Optional[Dict]:
        try:
            return self._high_q.get_nowait()
        except Empty:
            pass

        try:
            return self._low_q.get(timeout=timeout)
        except Empty:
            return None


class CommandExecutor:
    def __init__(self, arm: Arm, router: CommandRouter) -> None:
        self.arm = arm
        self.router = router

    def run(self) -> None:
        while True:
            command = self.router.get_next(timeout=0.1)
            if command is None:
                continue

            vprint("Executing:", command)

            # Clear before this command starts executing.
            self.router.interrupt_event.clear()
            self._execute(command)

    def _execute(self, command: Dict) -> None:
        ctype = command.get("type")
        params = command.get("params", {})

        if ctype == "reset":
            self.arm.speak("Resetting position")
            completed = self.arm.reset_pose(self.router.interrupt_event)
            if completed:
                self.arm.speak("Done")
            return

        if ctype == "claw_close":
            self.arm.speak("Closing claw")
            self.arm.send_raw_cmd("close")
            self.arm.speak("Done")
            return

        if ctype == "claw_open":
            self.arm.speak("Opening claw")
            self.arm.send_raw_cmd("open")
            self.arm.speak("Done")
            return

        if ctype == "keyboard_key":
            key = params.get("key", "")
            self._execute_keyboard_key(key)
            return

        if ctype == "vision_base_adjust":
            delta = int(params.get("delta", 0))
            self._execute_vision_base_adjust(delta)
            return

        if ctype == "vision_track_adjust":
            delta_base = int(params.get("delta_base", 0))
            delta_shoulder = int(params.get("delta_shoulder", 0))
            self._execute_vision_track_adjust(delta_base, delta_shoulder)
            return

    def _execute_keyboard_key(self, key: str) -> None:
        base, shoulder, elbow, wrist, claw = self.arm.get_pose()
        new_base, new_shoulder, new_elbow, new_wrist, new_claw = (
            base,
            shoulder,
            elbow,
            wrist,
            claw,
        )

        if key == "a":
            new_base += STEP
        elif key == "d":
            new_base -= STEP
        elif key == "w":
            new_shoulder += STEP
            new_elbow += STEP
            new_wrist -= STEP // 2
        elif key == "s":
            new_shoulder -= STEP
            new_elbow -= STEP
            new_wrist += STEP // 2
        elif key == "f":
            new_shoulder -= STEP
            new_elbow += STEP
            new_wrist += STEP
        elif key == "b":
            new_shoulder += STEP
            new_elbow -= STEP
            new_wrist -= STEP
        elif key == "o":
            new_claw -= STEP * 3
        elif key == "c":
            new_claw += STEP * 3
        elif key == "r":
            self.arm.reset_pose(self.router.interrupt_event)
            return
        else:
            return

        new_base = _clamp(new_base, BASE_MIN, BASE_MAX)
        new_shoulder = _clamp(new_shoulder, SHOULDER_MIN, SHOULDER_MAX)
        new_elbow = _clamp(new_elbow, ELBOW_MIN, ELBOW_MAX)
        new_wrist = _clamp(new_wrist, WRIST_MIN, WRIST_MAX)
        new_claw = _clamp(new_claw, CLAW_MIN, CLAW_MAX)

        self.arm.move_to(
            (new_base, new_shoulder, new_elbow, new_wrist, new_claw),
            interrupt_event=self.router.interrupt_event,
        )

    def _execute_vision_base_adjust(self, delta: int) -> None:
        if delta == 0:
            return

        base, shoulder, elbow, wrist, claw = self.arm.get_pose()
        new_base = _clamp(base + delta, BASE_MIN, BASE_MAX)
        if new_base == base:
            return

        self.arm.move_to(
            (new_base, shoulder, elbow, wrist, claw),
            interrupt_event=self.router.interrupt_event,
        )

    def _execute_vision_track_adjust(self, delta_base: int, delta_shoulder: int) -> None:
        if delta_base == 0 and delta_shoulder == 0:
            return

        base, shoulder, elbow, wrist, claw = self.arm.get_pose()
        new_base = _clamp(base + delta_base, BASE_MIN, BASE_MAX)
        new_shoulder = _clamp(shoulder + delta_shoulder, SHOULDER_MIN, SHOULDER_MAX)

        if new_base == base and new_shoulder == shoulder:
            return

        self.arm.move_to(
            (new_base, new_shoulder, elbow, wrist, claw),
            interrupt_event=self.router.interrupt_event,
        )
