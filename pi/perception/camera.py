import os
import platform
import threading
from typing import Optional, Union

import cv2

Source = Union[int, str]


def _parse_camera_source() -> Source:
    """Resolve camera from env: ROBOCLOUD_CAMERA or ROBOCLOUD_CAMERA_INDEX."""
    raw = os.getenv("ROBOCLOUD_CAMERA", "").strip()
    if raw:
        if raw.startswith("/dev/"):
            return raw
        if raw.isdigit() or (raw.startswith("-") and raw[1:].isdigit()):
            return int(raw)
        return raw
    return int(os.getenv("ROBOCLOUD_CAMERA_INDEX", "0"))


def _fourcc_from_string(code: str) -> int:
    code = code.strip().upper().ljust(4)[:4]
    return cv2.VideoWriter_fourcc(*code)


def _configure_capture(cap) -> tuple[int, int, float]:
    """Set width/height/FPS; optional FOURCC (MJPG helps C270 at 720p on USB2)."""
    w = int(os.getenv("ROBOCLOUD_CAMERA_WIDTH", "1280"))
    h = int(os.getenv("ROBOCLOUD_CAMERA_HEIGHT", "720"))
    fps = float(os.getenv("ROBOCLOUD_CAMERA_FPS", "30"))

    fourcc_raw = os.getenv("ROBOCLOUD_CAMERA_FOURCC", "").strip()
    if not fourcc_raw and platform.system() == "Linux":
        fourcc_raw = "MJPG"
    if fourcc_raw and fourcc_raw.lower() not in {"none", "default", "0"}:
        cap.set(cv2.CAP_PROP_FOURCC, _fourcc_from_string(fourcc_raw))

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, fps)

    aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    af = float(cap.get(cv2.CAP_PROP_FPS))
    return aw, ah, af


def _open_capture(source: Source):
    """Open VideoCapture with a backend that works for USB UVC on Linux."""
    system = platform.system()
    if system == "Linux":
        cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
        if cap.isOpened():
            return cap
        cap.release()
        cap = cv2.VideoCapture(source)
        return cap
    return cv2.VideoCapture(source)


class Camera:
    def __init__(self, source: Optional[Source] = None, *, index: Optional[int] = None):
        if index is not None and source is not None:
            raise ValueError("Pass at most one of source or index")
        if index is not None:
            self.source: Source = index
        elif source is not None:
            self.source = source
        else:
            self.source = _parse_camera_source()
        self.cap = _open_capture(self.source)
        self.actual_width = 0
        self.actual_height = 0
        self.actual_fps = 0.0
        if self.cap.isOpened():
            self.actual_width, self.actual_height, self.actual_fps = _configure_capture(self.cap)
        self.frame = None
        self.running = False
        self.lock = threading.Lock()

    def start(self):
        self.running = True
        threading.Thread(target=self._update, daemon=True).start()

    def _update(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.frame = frame

    def get_frame(self):
        with self.lock:
            return self.frame

    def stop(self):
        self.running = False
        self.cap.release()
