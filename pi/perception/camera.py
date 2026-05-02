import contextlib
import os
import platform
import sys
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


def _autoprobe_enabled() -> bool:
    """Scan /dev/video* for real capture nodes when no explicit camera env is set."""
    return os.getenv("ROBOCLOUD_CAMERA_AUTOPROBE", "1").strip() not in {"0", "false", "False", "no", "No"}


def _linux_skip_v4l_path(path: str) -> bool:
    """Skip Pi ISP / decoder V4L nodes that are not UVC capture (pispbe, rpi-hevc-dec, etc.).

    On Pi 5, ``v4l2-ctl --list-devices`` often shows dozens of /dev/video20+ pisp nodes; probing them
    spams ffmpeg and is slow. USB webcams usually sit at lower minors (e.g. /dev/video1, video2).
    Set ROBOCLOUD_CAMERA_PROBE_ALL=1 to disable this filter.
    """
    if platform.system() != "Linux":
        return False
    if os.getenv("ROBOCLOUD_CAMERA_PROBE_ALL", "").strip() in {"1", "true", "True"}:
        return False
    vid = os.path.basename(path)
    if not vid.startswith("video"):
        return True
    sysfs = f"/sys/class/video4linux/{vid}/device"
    try:
        r = os.path.realpath(sysfs)
    except Exception:
        return False
    rl = r.lower()
    if "pisp" in rl or "1000880000" in rl:
        return True
    if "rpi-hevc" in rl or "hevc-dec" in rl:
        return True
    return False


def _source_excludes_path(exclude: Optional[Source], path: str) -> bool:
    if exclude is None:
        return False
    try:
        real_p = os.path.realpath(path)
        if isinstance(exclude, str) and exclude.startswith("/dev/"):
            return os.path.realpath(exclude) == real_p
        if isinstance(exclude, int):
            return real_p == os.path.realpath(f"/dev/video{exclude}")
    except Exception:
        pass
    return False


@contextlib.contextmanager
def _quiet_probe_io():
    """Best-effort silence during V4L probing (OpenCV still may log via native code)."""
    try:
        cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_SILENT)
    except Exception:
        pass
    devnull = open(os.devnull, "w")
    old_err = sys.stderr
    try:
        sys.stderr = devnull
        yield
    finally:
        sys.stderr = old_err
        devnull.close()
        try:
            cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_ERROR)
        except Exception:
            pass


def _list_v4l_capture_paths(
    *,
    exclude: Optional[Source] = None,
    max_index: int = 31,
) -> list[str]:
    """Return /dev/videoN paths that open with V4L2 and return at least one frame.

    Many boards expose metadata or ISP nodes that are not capture devices; numeric
    index 0 often hits the wrong node. Probing by path avoids that.
    """
    if platform.system() != "Linux":
        return []
    found: list[str] = []
    for i in range(max(0, max_index) + 1):
        path = f"/dev/video{i}"
        if (
            not os.path.exists(path)
            or _source_excludes_path(exclude, path)
            or _linux_skip_v4l_path(path)
        ):
            continue
        with _quiet_probe_io():
            cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            continue
        # UVC often needs several grabs before the first good frame; a single read()
        # misses working nodes and we fall back to index 0 (often absent on Pi 5).
        _warmup_capture(cap, 6)
        ret = False
        for _ in range(12):
            ret, _ = cap.read()
            if ret:
                break
        cap.release()
        if ret:
            found.append(path)
    return found


def _parse_camera_source() -> Source:
    """Resolve camera from env: ROBOCLOUD_CAMERA, ROBOCLOUD_CAMERA_INDEX, or Linux autoprobes /dev/video*."""
    raw = os.getenv("ROBOCLOUD_CAMERA", "").strip()
    if raw:
        if raw.startswith("/dev/"):
            return raw
        if raw.isdigit() or (raw.startswith("-") and raw[1:].isdigit()):
            return int(raw)
        return raw
    idx_env = os.getenv("ROBOCLOUD_CAMERA_INDEX")
    if idx_env is not None and str(idx_env).strip() != "":
        return int(str(idx_env).strip())
    if platform.system() == "Linux" and _autoprobe_enabled():
        paths = _list_v4l_capture_paths()
        if paths:
            return paths[0]
    return 0


def parse_secondary_camera_source(primary: Optional[Source] = None) -> Optional[Source]:
    """Second UVC device: ROBOCLOUD_CAMERA2 / ROBOCLOUD_CAMERA2_INDEX, or next working V4L path after primary."""
    raw = os.getenv("ROBOCLOUD_CAMERA2", "").strip()
    if raw.lower() in {"none", "off", "disable"}:
        return None
    if raw:
        if raw.startswith("/dev/"):
            return raw
        if raw.isdigit() or (raw.startswith("-") and raw[1:].isdigit()):
            return int(raw)
        return raw
    idx = os.getenv("ROBOCLOUD_CAMERA2_INDEX", "").strip()
    if idx.isdigit() or (idx.startswith("-") and idx[1:].isdigit()):
        return int(idx)
    if platform.system() == "Linux" and _autoprobe_enabled():
        paths = _list_v4l_capture_paths(exclude=primary)
        if paths:
            return paths[0]
    return None


def _fourcc_from_string(code: str) -> int:
    code = code.strip().upper().ljust(4)[:4]
    return cv2.VideoWriter_fourcc(*code)


def _configure_capture(cap, *, role: str = "primary") -> tuple[int, int, float]:
    """Set width/height/FPS and FOURCC.

    role='primary' uses ROBOCLOUD_CAMERA_*; role='secondary' uses ROBOCLOUD_CAMERA2_* with
    fallback to primary env vars for any unset key.
    """
    if role == "secondary":
        w = int(os.getenv("ROBOCLOUD_CAMERA2_WIDTH", os.getenv("ROBOCLOUD_CAMERA_WIDTH", "640")))
        h = int(os.getenv("ROBOCLOUD_CAMERA2_HEIGHT", os.getenv("ROBOCLOUD_CAMERA_HEIGHT", "480")))
        fps = float(os.getenv("ROBOCLOUD_CAMERA2_FPS", os.getenv("ROBOCLOUD_CAMERA_FPS", "30")))
        fourcc_raw = os.getenv("ROBOCLOUD_CAMERA2_FOURCC", "").strip()
        if not fourcc_raw:
            fourcc_raw = os.getenv("ROBOCLOUD_CAMERA_FOURCC", "").strip()
        if not fourcc_raw and platform.system() == "Linux":
            fourcc_raw = "YUYV"
        buf = os.getenv(
            "ROBOCLOUD_CAMERA2_BUFFERSIZE", os.getenv("ROBOCLOUD_CAMERA_BUFFERSIZE", "1")
        ).strip()
    else:
        w = int(os.getenv("ROBOCLOUD_CAMERA_WIDTH", "640"))
        h = int(os.getenv("ROBOCLOUD_CAMERA_HEIGHT", "480"))
        fps = float(os.getenv("ROBOCLOUD_CAMERA_FPS", "30"))
        fourcc_raw = os.getenv("ROBOCLOUD_CAMERA_FOURCC", "").strip()
        if not fourcc_raw and platform.system() == "Linux":
            fourcc_raw = "YUYV"
        buf = os.getenv("ROBOCLOUD_CAMERA_BUFFERSIZE", "1").strip()

    if fourcc_raw and fourcc_raw.lower() not in {"none", "default", "0"}:
        cap.set(cv2.CAP_PROP_FOURCC, _fourcc_from_string(fourcc_raw))

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, fps)

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
    def __init__(
        self,
        source: Optional[Source] = None,
        *,
        index: Optional[int] = None,
        role: str = "primary",
    ):
        if role not in {"primary", "secondary"}:
            raise ValueError("role must be 'primary' or 'secondary'")
        if index is not None and source is not None:
            raise ValueError("Pass at most one of source or index")
        if index is not None:
            self.source: Source = index
        elif source is not None:
            self.source = source
        else:
            self.source = _parse_camera_source()
        self.role = role
        self.cap = _open_capture(self.source)
        self.actual_width = 0
        self.actual_height = 0
        self.actual_fps = 0.0
        if self.cap.isOpened():
            self.actual_width, self.actual_height, self.actual_fps = _configure_capture(
                self.cap, role=role
            )
            try:
                warmup = int(os.getenv("ROBOCLOUD_CAMERA_WARMUP_FRAMES", "8"))
            except ValueError:
                warmup = 8
            _warmup_capture(self.cap, warmup)
        self.frame = None
        self.running = False
        self.lock = threading.Lock()

    def start(self):
        if not self.cap.isOpened():
            return
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
