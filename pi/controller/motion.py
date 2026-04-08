import math
import os
import time
from threading import Event
from typing import List, Sequence

from pi.controller.serial_io import SerialIO

# Extra interpolation samples vs max |ΔPWM|. Reduces stair-stepping from round(smoothstep(t)*Δ).
# Total time stays ~ (max_delta * delay_time) because per-step sleep is scaled down.
_DEFAULT_OVERSAMPLE = 3.0


def _smoothstep01(t: float) -> float:
    if t <= 0.0:
        return 0.0
    if t >= 1.0:
        return 1.0
    return t * t * (3.0 - 2.0 * t)


def _motion_max_delta(start: Sequence[int], target: Sequence[int]) -> int:
    m = 0
    for i in range(5):
        d = abs(int(target[i]) - int(start[i]))
        if d > m:
            m = d
    return m


def move_smooth(
    current: List[int],
    target: Sequence[int],
    serial_io: SerialIO,
    interrupt_event: Event,
    delay_time: float = 0.008,
) -> bool:
    """
    Interpolated path with smoothstep easing + oversampling so integer PWM
    does not sit flat then jump (visible steps).
    """
    try:
        oversample = float(os.getenv("ROBOCLOUD_MOTION_OVERSAMPLE", str(_DEFAULT_OVERSAMPLE)))
    except ValueError:
        oversample = _DEFAULT_OVERSAMPLE
    oversample = max(1.0, oversample)

    start = [int(current[i]) for i in range(5)]
    tgt = [int(target[i]) for i in range(5)]
    deltas = [tgt[i] - start[i] for i in range(5)]

    max_delta = _motion_max_delta(start, tgt)
    if max_delta < 1:
        for i in range(5):
            current[i] = tgt[i]
        serial_io.send_all(current[0], current[1], current[2], current[3], current[4])
        return True

    steps = max(1, int(math.ceil(max_delta * oversample)))
    per_step_delay = delay_time / oversample

    for i in range(1, steps + 1):
        if interrupt_event.is_set():
            return False

        t = _smoothstep01(i / float(steps))
        for j in range(5):
            current[j] = start[j] + round(deltas[j] * t)

        serial_io.send_all(current[0], current[1], current[2], current[3], current[4])
        time.sleep(per_step_delay)

    for j in range(5):
        current[j] = tgt[j]
    serial_io.send_all(current[0], current[1], current[2], current[3], current[4])
    return True
