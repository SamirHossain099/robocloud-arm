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
      A/D — pan 11 | 1/2 — shoulder 12 | 3/4 — elbow 13 | W/S — wrist 14 |
      O/C — claw 15 | P — print | R — reset | Q — quit.
    """
    while True:
        key = getch()

        if key == "q":
            break

        if key in {"a", "d", "w", "s", "1", "2", "3", "4", "o", "c", "r", "p"}:
            router.submit(Arm.command("keyboard_key", {"key": key}, "low"))
