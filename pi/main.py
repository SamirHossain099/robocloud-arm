import os
import sys
import threading
import time
from pathlib import Path

import cv2

if __package__ is None or __package__ == "":
    # Allow running as: python pi/main.py
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from pi.config import (
    CONTROL_TRANSPORT,
    NETWORK_HOST,
    NETWORK_PORT,
    NETWORK_TIMEOUT,
    SERIAL_BAUDRATE,
    SERIAL_PORT,
)
from pi.controller.arm import Arm
from pi.controller.executor import CommandExecutor, CommandRouter
from pi.controller.network_io import NetworkIO
from pi.controller.serial_io import SerialIO
from pi.input.keyboard import keyboard_control
from pi.input.voice import voice_control
from pi.perception.camera import Camera
from pi.perception.stream import start_stream_server
from pi.perception.tracker import ColorTracker


def camera_view(camera):
    tracker = ColorTracker()

    while True:
        frame = camera.get_frame()
        if frame is not None:
            result = tracker.track(frame)

            if result:
                cx, cy = result["center"]
                x, y, w, h = result["bbox"]

                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

            cv2.imshow("Tracking", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break


def main() -> None:
    os.environ["PYTHONWARNINGS"] = "ignore"
    control_transport = os.getenv("ROBOCLOUD_CONTROL_TRANSPORT", CONTROL_TRANSPORT).lower()
    enable_voice = os.getenv("ROBOCLOUD_ENABLE_VOICE", "0") == "1"
    enable_live_feed = os.getenv("ROBOCLOUD_ENABLE_LIVE_FEED", "0") == "1"
    enable_stream = os.getenv("ROBOCLOUD_ENABLE_STREAM", "0") == "1"
    stream_port = int(os.getenv("ROBOCLOUD_STREAM_PORT", "8080"))

    if control_transport == "network":
        network_host = os.getenv("ROBOCLOUD_NETWORK_HOST", NETWORK_HOST)
        network_port = int(os.getenv("ROBOCLOUD_NETWORK_PORT", str(NETWORK_PORT)))
        network_timeout = float(
            os.getenv("ROBOCLOUD_NETWORK_TIMEOUT", str(NETWORK_TIMEOUT))
        )
        io_transport = NetworkIO(
            host=network_host, port=network_port, timeout=network_timeout
        )
        io_transport.connect()
        print(f"Control transport: network ({network_host}:{network_port})")
    else:
        io_transport = SerialIO(port=SERIAL_PORT, baudrate=SERIAL_BAUDRATE, timeout=1)
        io_transport.connect()
        print(f"Control transport: serial ({SERIAL_PORT} @ {SERIAL_BAUDRATE})")

    arm = Arm(serial_io=io_transport)
    router = CommandRouter()
    executor = CommandExecutor(arm=arm, router=router)
    camera = Camera()
    camera.start()

    executor_thread = threading.Thread(target=executor.run, daemon=True)
    keyboard_thread = threading.Thread(
        target=keyboard_control, args=(router,), daemon=True
    )

    executor_thread.start()
    if enable_voice:
        voice_thread = threading.Thread(target=voice_control, args=(router,), daemon=True)
        voice_thread.start()
        print("Voice thread enabled")
    else:
        print("Voice thread disabled (set ROBOCLOUD_ENABLE_VOICE=1 to enable)")
    keyboard_thread.start()
    if enable_live_feed:
        cam_thread = threading.Thread(target=camera_view, args=(camera,), daemon=True)
        cam_thread.start()
        print("Live feed enabled")
    else:
        print("Live feed disabled (set ROBOCLOUD_ENABLE_LIVE_FEED=1 to enable)")
    if enable_stream:
        stream_thread = threading.Thread(
            target=start_stream_server, args=(camera, "0.0.0.0", stream_port), daemon=True
        )
        stream_thread.start()
        print(f"Camera stream enabled at http://0.0.0.0:{stream_port}/stream")
    else:
        print("Camera stream disabled (set ROBOCLOUD_ENABLE_STREAM=1 to enable)")

    while True:
        time.sleep(1.0)


if __name__ == "__main__":
    main()
