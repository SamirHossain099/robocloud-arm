import json
import os
import socket
from typing import Optional
import threading
import time
from http import server
from socketserver import ThreadingMixIn

import cv2

from pi.config import (
    BASE_MAX,
    BASE_MIN,
    BASE_DEFAULT,
    SHOULDER_DEFAULT,
    SHOULDER_MIN,
    SHOULDER_MAX,
    ELBOW_DEFAULT,
    ELBOW_MIN,
    ELBOW_MAX,
    WRIST_DEFAULT,
    WRIST_MIN,
    WRIST_MAX,
    CLAW_DEFAULT,
    CLAW_MIN,
    CLAW_MAX,
    SERIAL_BAUDRATE,
    SERIAL_PORT,
)
from pi.controller.serial_io import SerialIO
from pi.perception.camera import Camera, parse_secondary_camera_source


def _clamp(value: int, minimum: int, maximum: int) -> int:
    return max(minimum, min(maximum, value))


class _RawStreamHandler(server.BaseHTTPRequestHandler):
    camera = None
    camera2 = None
    jpeg_quality = 70
    frame_interval = 0.04

    def log_message(self, format, *args):
        return

    def _send_jpeg_frame(self, frame) -> bool:
        ok, jpg = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_quality)],
        )
        if not ok:
            return False
        payload = jpg.tobytes()
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)
        return True

    def do_GET(self):
        if self.path == "/":
            self.send_response(301)
            self.send_header("Location", "/stream")
            self.end_headers()
            return

        if self.path == "/health":
            self.send_response(200)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.end_headers()
            self.wfile.write(b"ok\n")
            return

        if self.path == "/snapshot.jpg":
            frame = self.camera.get_frame() if self.camera else None
            if frame is None:
                self.send_error(503, "No frame available")
                self.end_headers()
                return
            if not self._send_jpeg_frame(frame):
                self.send_error(500, "Encode failed")
                self.end_headers()
            return

        if self.path == "/snapshot2.jpg":
            if not self.camera2:
                self.send_error(404, "Second camera not configured")
                self.end_headers()
                return
            frame = self.camera2.get_frame()
            if frame is None:
                self.send_error(503, "No frame available (camera 2)")
                self.end_headers()
                return
            if not self._send_jpeg_frame(frame):
                self.send_error(500, "Encode failed")
                self.end_headers()
            return

        if self.path == "/stream":
            self._mjpeg_stream(self.camera)
            return

        if self.path == "/stream2":
            if not self.camera2:
                self.send_error(404, "Second camera not configured")
                self.end_headers()
                return
            self._mjpeg_stream(self.camera2)
            return

        self.send_error(404)
        self.end_headers()
        return

    def _mjpeg_stream(self, cam) -> None:
        if not cam:
            self.send_error(503, "Camera not available")
            self.end_headers()
            return
        self.send_response(200)
        self.send_header("Age", 0)
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
        self.end_headers()

        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_quality)]
        while True:
            frame = cam.get_frame()
            if frame is None:
                time.sleep(0.01)
                continue

            ok, jpg = cv2.imencode(".jpg", frame, encode_params)
            if not ok:
                continue

            payload = jpg.tobytes()
            try:
                self.wfile.write(b"--FRAME\r\n")
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)
                self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                break

            if self.frame_interval > 0:
                time.sleep(self.frame_interval)


class _ThreadedHTTPServer(ThreadingMixIn, server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def _run_stream_server(
    camera: Camera,
    camera2: Optional[Camera],
    host: str,
    port: int,
    jpeg_quality: int,
    frame_interval: float,
):
    _RawStreamHandler.camera = camera
    _RawStreamHandler.camera2 = camera2
    _RawStreamHandler.jpeg_quality = jpeg_quality
    _RawStreamHandler.frame_interval = frame_interval
    httpd = _ThreadedHTTPServer((host, port), _RawStreamHandler)
    httpd.serve_forever()


def _udp_control_loop(
    serial_io: SerialIO,
    host: str,
    port: int,
    base_min: int,
    base_max: int,
    shoulder_min: int,
    shoulder_max: int,
    elbow_min: int,
    elbow_max: int,
    wrist_min: int,
    wrist_max: int,
    claw_min: int,
    claw_max: int,
    use_movebase: bool,
    movebase_speed: str,
    use_moveshoulder: bool,
    use_moveelbow: bool,
    use_movewrist: bool,
    use_moveclaw: bool,
):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host, port))
    sock.settimeout(1.0)

    base_value = BASE_DEFAULT
    shoulder_value = SHOULDER_DEFAULT
    elbow_value = ELBOW_DEFAULT
    wrist_value = WRIST_DEFAULT
    claw_value = CLAW_DEFAULT
    serial_io.send_cmd(f"set 11 {base_value}")
    serial_io.send_cmd(f"set 12 {shoulder_value}")
    serial_io.send_cmd(f"set 13 {elbow_value}")
    serial_io.send_cmd(f"set 14 {wrist_value}")
    serial_io.send_cmd(f"set 15 {claw_value}")
    print(f"Control UDP listening on {host}:{port}")

    while True:
        try:
            payload, _addr = sock.recvfrom(4096)
        except socket.timeout:
            continue
        except Exception:
            continue

        try:
            msg = json.loads(payload.decode("utf-8").strip())
        except Exception:
            continue

        updated = False
        shoulder_updated = False
        elbow_updated = False
        wrist_updated = False
        claw_updated = False
        stop_requested = False
        speed = movebase_speed
        if "base" in msg:
            base_value = _clamp(int(msg["base"]), base_min, base_max)
            updated = True
        elif "delta" in msg:
            delta = int(msg["delta"])
            if delta == 0:
                stop_requested = True
            else:
                base_value = _clamp(base_value + delta, base_min, base_max)
                updated = True
        if "wrist" in msg:
            wrist_value = _clamp(int(msg["wrist"]), wrist_min, wrist_max)
            wrist_updated = True
        elif "wrist_delta" in msg:
            wrist_value = _clamp(wrist_value + int(msg["wrist_delta"]), wrist_min, wrist_max)
            wrist_updated = True
        if "shoulder" in msg:
            shoulder_value = _clamp(int(msg["shoulder"]), shoulder_min, shoulder_max)
            shoulder_updated = True
        elif "shoulder_delta" in msg:
            shoulder_value = _clamp(shoulder_value + int(msg["shoulder_delta"]), shoulder_min, shoulder_max)
            shoulder_updated = True
        if "elbow" in msg:
            elbow_value = _clamp(int(msg["elbow"]), elbow_min, elbow_max)
            elbow_updated = True
        elif "elbow_delta" in msg:
            elbow_value = _clamp(elbow_value + int(msg["elbow_delta"]), elbow_min, elbow_max)
            elbow_updated = True
        if "claw" in msg:
            claw_value = _clamp(int(msg["claw"]), claw_min, claw_max)
            claw_updated = True
        elif "claw_delta" in msg:
            claw_value = _clamp(claw_value + int(msg["claw_delta"]), claw_min, claw_max)
            claw_updated = True

        if "stop" in msg and bool(msg["stop"]):
            stop_requested = True

        if "speed" in msg:
            speed = str(msg["speed"]).strip() or movebase_speed

        if stop_requested and (use_movebase or use_moveshoulder or use_moveelbow or use_movewrist or use_moveclaw):
            serial_io.send_cmd("stop")

        if updated:
            if use_movebase:
                serial_io.send_cmd(f"movebase {base_value} {speed}")
            else:
                serial_io.send_cmd(f"set 11 {base_value}")

        if wrist_updated:
            if use_movewrist:
                serial_io.send_cmd(f"movewrist {wrist_value} {speed}")
            else:
                serial_io.send_cmd(f"set 14 {wrist_value}")

        if shoulder_updated:
            if use_moveshoulder:
                serial_io.send_cmd(f"moveshoulder {shoulder_value} {speed}")
            else:
                serial_io.send_cmd(f"set 12 {shoulder_value}")

        if elbow_updated:
            if use_moveelbow:
                serial_io.send_cmd(f"moveelbow {elbow_value} {speed}")
            else:
                serial_io.send_cmd(f"set 13 {elbow_value}")

        if claw_updated:
            if use_moveclaw:
                serial_io.send_cmd(f"moveclaw {claw_value} {speed}")
            else:
                serial_io.send_cmd(f"set 15 {claw_value}")


def main() -> None:
    serial_port = os.getenv("ROBOCLOUD_SERIAL_PORT", SERIAL_PORT)
    serial_baudrate = int(os.getenv("ROBOCLOUD_SERIAL_BAUDRATE", str(SERIAL_BAUDRATE)))

    stream_host = os.getenv("ROBOCLOUD_STREAM_HOST", "0.0.0.0")
    stream_port = int(os.getenv("ROBOCLOUD_STREAM_PORT", "8080"))
    stream_jpeg_quality = int(os.getenv("ROBOCLOUD_STREAM_JPEG_QUALITY", "70"))
    stream_frame_interval = float(os.getenv("ROBOCLOUD_STREAM_FRAME_INTERVAL", "0.04"))

    control_host = os.getenv("ROBOCLOUD_CONTROL_HOST", "0.0.0.0")
    control_port = int(os.getenv("ROBOCLOUD_CONTROL_PORT", "9999"))
    use_movebase = os.getenv("ROBOCLOUD_CONTROL_USE_MOVEBASE", "1").strip() not in {"0", "false", "False"}
    movebase_speed = os.getenv("ROBOCLOUD_MOVEBASE_SPEED", "fast")
    use_moveshoulder = os.getenv("ROBOCLOUD_CONTROL_USE_MOVESHOULDER", "1").strip() not in {"0", "false", "False"}
    use_moveelbow = os.getenv("ROBOCLOUD_CONTROL_USE_MOVEELBOW", "1").strip() not in {"0", "false", "False"}
    use_movewrist = os.getenv("ROBOCLOUD_CONTROL_USE_MOVEWRIST", "1").strip() not in {"0", "false", "False"}
    use_moveclaw = os.getenv("ROBOCLOUD_CONTROL_USE_MOVECLAW", "1").strip() not in {"0", "false", "False"}

    serial_io = SerialIO(port=serial_port, baudrate=serial_baudrate, timeout=1)
    serial_io.connect()

    camera = Camera()
    if not camera.cap.isOpened():
        print(
            "ERROR: Camera did not open. For Logitech C270, try another node, e.g.\n"
            "  v4l2-ctl --list-devices\n"
            "Then: ROBOCLOUD_CAMERA=/dev/video2 ROBOCLOUD_MOVEBASE_SPEED=fast python -m pi.remote_bridge\n"
            "Or: ROBOCLOUD_CAMERA=2 ...  (numeric index)\n"
            "Ensure the user can access video devices (e.g. in the 'video' group)."
        )
    else:
        print(
            f"Camera opened: {camera.source!r} "
            f"({camera.actual_width}x{camera.actual_height} @ {camera.actual_fps:.1f} fps)"
        )

    camera.start()

    camera2 = None
    src2 = parse_secondary_camera_source()
    if src2 is not None:
        camera2 = Camera(source=src2, role="secondary")
        if not camera2.cap.isOpened():
            print(f"WARN: Second camera did not open ({camera2.source!r}); /stream2 disabled.")
            camera2 = None
        else:
            print(
                f"Camera2 opened: {camera2.source!r} "
                f"({camera2.actual_width}x{camera2.actual_height} @ {camera2.actual_fps:.1f} fps)"
            )
            camera2.start()

    stream_thread = threading.Thread(
        target=_run_stream_server,
        args=(camera, camera2, stream_host, stream_port, stream_jpeg_quality, stream_frame_interval),
        daemon=True,
    )
    stream_thread.start()

    print(f"Stream ready: http://{stream_host}:{stream_port}/stream")
    if camera2:
        print(f"Stream2 (overhead): http://{stream_host}:{stream_port}/stream2")
    _udp_control_loop(
        serial_io=serial_io,
        host=control_host,
        port=control_port,
        base_min=BASE_MIN,
        base_max=BASE_MAX,
        shoulder_min=SHOULDER_MIN,
        shoulder_max=SHOULDER_MAX,
        elbow_min=ELBOW_MIN,
        elbow_max=ELBOW_MAX,
        wrist_min=WRIST_MIN,
        wrist_max=WRIST_MAX,
        claw_min=CLAW_MIN,
        claw_max=CLAW_MAX,
        use_movebase=use_movebase,
        movebase_speed=movebase_speed,
        use_moveshoulder=use_moveshoulder,
        use_moveelbow=use_moveelbow,
        use_movewrist=use_movewrist,
        use_moveclaw=use_moveclaw,
    )


if __name__ == "__main__":
    main()
