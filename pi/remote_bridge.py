import json
import os
import socket
import threading
import time
from http import server
from socketserver import ThreadingMixIn

import cv2

from pi.config import BASE_MAX, BASE_MIN, BASE_DEFAULT, SERIAL_BAUDRATE, SERIAL_PORT
from pi.controller.serial_io import SerialIO
from pi.perception.camera import Camera


def _clamp(value: int, minimum: int, maximum: int) -> int:
    return max(minimum, min(maximum, value))


class _RawStreamHandler(server.BaseHTTPRequestHandler):
    camera = None
    jpeg_quality = 70
    frame_interval = 0.04

    def log_message(self, format, *args):
        return

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
            ok, jpg = cv2.imencode(
                ".jpg",
                frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_quality)],
            )
            if not ok:
                self.send_error(500, "Encode failed")
                self.end_headers()
                return
            payload = jpg.tobytes()
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
            return

        if self.path != "/stream":
            self.send_error(404)
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
            frame = self.camera.get_frame() if self.camera else None
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


def _run_stream_server(camera: Camera, host: str, port: int, jpeg_quality: int, frame_interval: float):
    _RawStreamHandler.camera = camera
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
):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host, port))
    sock.settimeout(1.0)

    base_value = BASE_DEFAULT
    serial_io.send_cmd(f"set 11 {base_value}")
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
        if "base" in msg:
            base_value = _clamp(int(msg["base"]), base_min, base_max)
            updated = True
        elif "delta" in msg:
            base_value = _clamp(base_value + int(msg["delta"]), base_min, base_max)
            updated = True

        if updated:
            serial_io.send_cmd(f"set 11 {base_value}")


def main() -> None:
    serial_port = os.getenv("ROBOCLOUD_SERIAL_PORT", SERIAL_PORT)
    serial_baudrate = int(os.getenv("ROBOCLOUD_SERIAL_BAUDRATE", str(SERIAL_BAUDRATE)))

    stream_host = os.getenv("ROBOCLOUD_STREAM_HOST", "0.0.0.0")
    stream_port = int(os.getenv("ROBOCLOUD_STREAM_PORT", "8080"))
    stream_jpeg_quality = int(os.getenv("ROBOCLOUD_STREAM_JPEG_QUALITY", "70"))
    stream_frame_interval = float(os.getenv("ROBOCLOUD_STREAM_FRAME_INTERVAL", "0.04"))

    control_host = os.getenv("ROBOCLOUD_CONTROL_HOST", "0.0.0.0")
    control_port = int(os.getenv("ROBOCLOUD_CONTROL_PORT", "9999"))

    serial_io = SerialIO(port=serial_port, baudrate=serial_baudrate, timeout=1)
    serial_io.connect()

    camera = Camera(index=0)
    camera.start()

    stream_thread = threading.Thread(
        target=_run_stream_server,
        args=(camera, stream_host, stream_port, stream_jpeg_quality, stream_frame_interval),
        daemon=True,
    )
    stream_thread.start()

    print(f"Stream ready: http://{stream_host}:{stream_port}/stream")
    _udp_control_loop(
        serial_io=serial_io,
        host=control_host,
        port=control_port,
        base_min=BASE_MIN,
        base_max=BASE_MAX,
    )


if __name__ == "__main__":
    main()
