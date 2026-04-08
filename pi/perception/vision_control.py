import time

from pi.controller.arm import Arm
from pi.controller.executor import CommandRouter
from pi.perception.tracker import ColorTracker


def compute_base_adjust(cx: int, frame_width: int, deadband: int = 20, gain: float = 0.02) -> int:
    center = frame_width // 2
    error = cx - center

    if abs(error) < deadband:
        return 0

    return int(error * gain)


def vision_base_control(camera, router: CommandRouter) -> None:
    tracker = ColorTracker()

    while True:
        frame = camera.get_frame()
        if frame is None:
            time.sleep(0.05)
            continue

        result = tracker.track(frame)
        if not result:
            time.sleep(0.05)
            continue

        cx, _ = result["center"]
        adjust = compute_base_adjust(cx, frame.shape[1])
        if adjust != 0:
            router.submit(
                Arm.command(
                    "vision_base_adjust",
                    {"delta": -adjust, "cx": cx, "frame_width": frame.shape[1]},
                    "low",
                )
            )

        time.sleep(0.08)
