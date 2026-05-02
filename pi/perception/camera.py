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
