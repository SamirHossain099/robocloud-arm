import time
import threading

from pi.controller.arm import Arm
from pi.controller.executor import CommandRouter
from pi.perception.tracker import ColorTracker

_state_lock = threading.Lock()
_vision_state = {
    "tracking": False,
    "cx": None,
    "cy": None,
    "frame_width": None,
    "error": None,
    "delta_cmd": 0,
}


def compute_base_adjust(cx: int, frame_width: int, deadband: int = 20, gain: float = 0.02) -> int:
    center = frame_width // 2
    error = cx - center

    if abs(error) < deadband:
        return 0

    return int(error * gain)


def set_vision_state(**kwargs) -> None:
    with _state_lock:
        _vision_state.update(kwargs)


def get_vision_state() -> dict:
    with _state_lock:
        return dict(_vision_state)


def vision_base_control(camera, router: CommandRouter) -> None:
    tracker = ColorTracker()

    while True:
        frame = camera.get_frame()
        if frame is None:
            set_vision_state(tracking=False, cx=None, cy=None, frame_width=None, error=None, delta_cmd=0)
            time.sleep(0.05)
            continue

        result = tracker.track(frame)
        if not result:
            set_vision_state(
                tracking=False,
                cx=None,
                cy=None,
                frame_width=frame.shape[1],
                error=None,
                delta_cmd=0,
            )
            time.sleep(0.05)
            continue

        cx, cy = result["center"]
        frame_width = frame.shape[1]
        center = frame_width // 2
        error = cx - center
        adjust = compute_base_adjust(cx, frame_width)
        delta_cmd = -adjust

        set_vision_state(
            tracking=True,
            cx=cx,
            cy=cy,
            frame_width=frame_width,
            error=error,
            delta_cmd=delta_cmd,
        )

        if adjust != 0:
            router.submit(
                Arm.command(
                    "vision_base_adjust",
                    {"delta": delta_cmd, "cx": cx, "frame_width": frame_width},
                    "low",
                )
            )

        time.sleep(0.08)
