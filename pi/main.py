import os
import threading
import time

from pi.config import SERIAL_BAUDRATE, SERIAL_PORT
from pi.controller.arm import Arm
from pi.controller.executor import CommandExecutor, CommandRouter
from pi.controller.serial_io import SerialIO
from pi.input.keyboard import keyboard_control
from pi.input.voice import voice_control


def main() -> None:
    os.environ["PYTHONWARNINGS"] = "ignore"

    serial_io = SerialIO(port=SERIAL_PORT, baudrate=SERIAL_BAUDRATE, timeout=1)
    serial_io.connect()

    arm = Arm(serial_io=serial_io)
    router = CommandRouter()
    executor = CommandExecutor(arm=arm, router=router)

    executor_thread = threading.Thread(target=executor.run, daemon=True)
    voice_thread = threading.Thread(target=voice_control, args=(router,), daemon=True)
    keyboard_thread = threading.Thread(
        target=keyboard_control, args=(router,), daemon=True
    )

    executor_thread.start()
    voice_thread.start()
    keyboard_thread.start()

    while True:
        time.sleep(1.0)


if __name__ == "__main__":
    main()
