from http import server
from socketserver import ThreadingMixIn
import time

import cv2

from pi.perception.tracker import ColorTracker


class _StreamingHandler(server.BaseHTTPRequestHandler):
    camera = None

    def do_GET(self):
        if self.path == "/":
            self.send_response(301)
            self.send_header("Location", "/stream")
            self.end_headers()
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

        tracker = ColorTracker()
        last_log_ts = 0.0

        while True:
            frame = self.camera.get_frame() if self.camera else None
            if frame is None:
                continue

            result = tracker.track(frame)
            output = frame.copy()

            if result:
                cx, cy = result["center"]
                x, y, w, h = result["bbox"]
                area = int(result["area"])

                now = time.time()
                if now - last_log_ts >= 0.5:
                    print(f"TRACK center=({cx}, {cy}) area={area}")
                    last_log_ts = now

                cv2.rectangle(output, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(output, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(
                    output,
                    f"RED FOUND area={area}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
            else:
                cv2.putText(
                    output,
                    "NO RED",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )

            ok, jpg = cv2.imencode(".jpg", output)
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
            except BrokenPipeError:
                break
            except ConnectionResetError:
                break


class _ThreadedHTTPServer(ThreadingMixIn, server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


def start_stream_server(camera, host: str = "0.0.0.0", port: int = 8080) -> None:
    _StreamingHandler.camera = camera
    httpd = _ThreadedHTTPServer((host, port), _StreamingHandler)
    httpd.serve_forever()
