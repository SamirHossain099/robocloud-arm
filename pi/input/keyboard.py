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
      A/D — pan 11 | F — fwd 12−/13+, B — back 12+/13− (B capped to home) |
      W/S — up/down 14 | O/C — claw 15 | P — print servos | R — reset | Q — quit.
    """
    while True:
        key = getch()

        if key == "q":
            break

        if key in {"a", "d", "w", "s", "f", "b", "o", "c", "r", "p"}:
            router.submit(Arm.command("keyboard_key", {"key": key}, "low"))
