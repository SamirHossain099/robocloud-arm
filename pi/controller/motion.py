import time
from threading import Event
from typing import List, Sequence

from pi.controller.serial_io import SerialIO


def move_smooth(
    current: List[int],
    target: Sequence[int],
    serial_io: SerialIO,
    interrupt_event: Event,
    delay_time: float = 0.008,
) -> bool:
    # Core stepping behavior is intentionally preserved.
    while True:
        if interrupt_event.is_set():
            return False  # immediate exit, executor handles next command

        done = True

        if current[0] < target[0]:
            current[0] += 1
            done = False
        elif current[0] > target[0]:
            current[0] -= 1
            done = False

        if current[1] < target[1]:
            current[1] += 1
            done = False
        elif current[1] > target[1]:
            current[1] -= 1
            done = False

        if current[2] < target[2]:
            current[2] += 1
            done = False
        elif current[2] > target[2]:
            current[2] -= 1
            done = False

        if current[3] < target[3]:
            current[3] += 1
            done = False
        elif current[3] > target[3]:
            current[3] -= 1
            done = False

        if current[4] < target[4]:
            current[4] += 1
            done = False
        elif current[4] > target[4]:
            current[4] -= 1
            done = False

        serial_io.send_all(current[0], current[1], current[2], current[3], current[4])
        time.sleep(delay_time)

        if done:
            break

    return True
