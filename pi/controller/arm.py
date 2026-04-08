from threading import Event, Lock
from typing import Any, Dict, Tuple

from pi.config import (
    BASE_DEFAULT,
    CLAW_DEFAULT,
    ELBOW_DEFAULT,
    SHOULDER_DEFAULT,
    WRIST_DEFAULT,
)
from pi.controller.motion import move_smooth
from pi.logutil import vprint


class Arm:
    def __init__(self, serial_io: Any) -> None:
        self.serial_io = serial_io
        self._lock = Lock()
        self._joints = [
            BASE_DEFAULT,
            SHOULDER_DEFAULT,
            ELBOW_DEFAULT,
            WRIST_DEFAULT,
            CLAW_DEFAULT,
        ]

    def get_pose(self) -> Tuple[int, int, int, int, int]:
        with self._lock:
            return tuple(self._joints)

    def move_to(
        self, target: Tuple[int, int, int, int, int], interrupt_event: Event, delay: float = 0.008
    ) -> bool:
        with self._lock:
            return move_smooth(
                current=self._joints,
                target=target,
                serial_io=self.serial_io,
                interrupt_event=interrupt_event,
                delay_time=delay,
            )

    def reset_pose(self, interrupt_event: Event) -> bool:
        return self.move_to(
            (BASE_DEFAULT, SHOULDER_DEFAULT, ELBOW_DEFAULT, WRIST_DEFAULT, CLAW_DEFAULT),
            interrupt_event=interrupt_event,
            delay=0.015,
        )

    def send_raw_cmd(self, cmd: str) -> None:
        self.serial_io.send_cmd(cmd)

    @staticmethod
    def speak(text: str) -> None:
        import os

        vprint("Robot:", text)
        os.system(f'espeak "{text}"')

    @staticmethod
    def command(command_type: str, params: Dict, priority: str) -> Dict:
        return {"type": command_type, "params": params, "priority": priority}
