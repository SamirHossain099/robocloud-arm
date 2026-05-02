import os
import platform
import threading
from typing import Optional, Union

import cv2

try:
    # Cuts noise from benign MJPEG decode quirks on some UVC devices (e.g. C270).
    if os.getenv("ROBOCLOUD_CAMERA_VERBOSE", "").strip() not in {"1", "true", "True"}:
        cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_ERROR)
except Exception:
    pass

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
    """Set width/height/FPS and FOURCC.

    Defaults favor stable UVC capture on USB2 (YUYV 640x480) over high-res MJPEG,
    which often spams corrupt-frame warnings on Pi + C270-class webcams.
    """
    w = int(os.getenv("ROBOCLOUD_CAMERA_WIDTH", "640"))
    h = int(os.getenv("ROBOCLOUD_CAMERA_HEIGHT", "480"))
    fps = float(os.getenv("ROBOCLOUD_CAMERA_FPS", "30"))

    fourcc_raw = os.getenv("ROBOCLOUD_CAMERA_FOURCC", "").strip()
    if not fourcc_raw and platform.system() == "Linux":
        fourcc_raw = "YUYV"
    # UVC: set pixel format before resolution for reliable MJPG negotiation.
    if fourcc_raw and fourcc_raw.lower() not in {"none", "default", "0"}:
        cap.set(cv2.CAP_PROP_FOURCC, _fourcc_from_string(fourcc_raw))

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, fps)

    # Single-frame buffer reduces stale / stitched MJPEG frames (common "Corrupt JPEG" source).
    buf = os.getenv("ROBOCLOUD_CAMERA_BUFFERSIZE", "1").strip()
    if buf and platform.system() == "Linux":
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, int(buf))
        except Exception:
            pass

    aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    af = float(cap.get(cv2.CAP_PROP_FPS))
    return aw, ah, af


def _warmup_capture(cap, n: int) -> None:
    for _ in range(max(0, n)):
        cap.grab()


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
            try:
                warmup = int(os.getenv("ROBOCLOUD_CAMERA_WARMUP_FRAMES", "8"))
            except ValueError:
                warmup = 8
            _warmup_capture(self.cap, warmup)
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
