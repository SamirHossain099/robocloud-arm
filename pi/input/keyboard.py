import sys
import termios
import tty

from pi.controller.arm import Arm
from pi.controller.executor import CommandRouter


def getch() -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def keyboard_control(router: CommandRouter) -> None:
    """
    Teleop (see executor keyboard handler for semantics):
      A/D — pan (base 11), W/S — reach (12+13, wrist 14 → default),
      F/B — wrist trim (14), O/C — claw (15), R — reset, Q — quit.
    """
    while True:
        key = getch()

        if key == "q":
            break

        if key in {"a", "d", "w", "s", "f", "b", "o", "c", "r"}:
            router.submit(Arm.command("keyboard_key", {"key": key}, "low"))
