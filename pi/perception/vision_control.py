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


def compute_shoulder_adjust(
    area: float,
    target_area: float = 6000.0,
    area_deadband: float = 1200.0,
    shoulder_step: int = 2,
) -> int:
    area_error = area - target_area

    if abs(area_error) < area_deadband:
        return 0

    # Object looks large (near): move shoulder up.
    if area_error > 0:
        return +shoulder_step

    # Object looks small (far): move shoulder down.
    return -shoulder_step


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
        area = float(result["area"])
        base_adjust = compute_base_adjust(cx, frame.shape[1])
        shoulder_adjust = compute_shoulder_adjust(area)
        if base_adjust != 0 or shoulder_adjust != 0:
            router.submit(
                Arm.command(
                    "vision_track_adjust",
                    {
                        "delta_base": -base_adjust,
                        "delta_shoulder": shoulder_adjust,
                        "cx": cx,
                        "frame_width": frame.shape[1],
                        "area": area,
                    },
                    "low",
                )
            )

        time.sleep(0.08)
