import os
import sys
import threading
import time
from pathlib import Path

import cv2

if __package__ is None or __package__ == "":
    # Allow running as: python pi/main.py
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from pi.config import SERIAL_BAUDRATE, SERIAL_PORT
from pi.controller.arm import Arm
from pi.controller.executor import CommandExecutor, CommandRouter
from pi.controller.serial_io import SerialIO
from pi.input.keyboard import keyboard_control
from pi.input.voice import voice_control
from pi.perception.camera import Camera


def camera_view(camera):
    while True:
        frame = camera.get_frame()
        if frame is not None:
            cv2.imshow("Live", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break


def main() -> None:
    os.environ["PYTHONWARNINGS"] = "ignore"
    enable_voice = os.getenv("ROBOCLOUD_ENABLE_VOICE", "0") == "1"

    serial_io = SerialIO(port=SERIAL_PORT, baudrate=SERIAL_BAUDRATE, timeout=1)
    serial_io.connect()

    arm = Arm(serial_io=serial_io)
    router = CommandRouter()
    executor = CommandExecutor(arm=arm, router=router)
    camera = Camera()
    camera.start()

    executor_thread = threading.Thread(target=executor.run, daemon=True)
    keyboard_thread = threading.Thread(
        target=keyboard_control, args=(router,), daemon=True
    )
    cam_thread = threading.Thread(target=camera_view, args=(camera,), daemon=True)

    executor_thread.start()
    if enable_voice:
        voice_thread = threading.Thread(target=voice_control, args=(router,), daemon=True)
        voice_thread.start()
        print("Voice thread enabled")
    else:
        print("Voice thread disabled (set ROBOCLOUD_ENABLE_VOICE=1 to enable)")
    keyboard_thread.start()
    cam_thread.start()

    while True:
        time.sleep(1.0)


if __name__ == "__main__":
    main()
