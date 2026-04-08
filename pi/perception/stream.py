from http import server
from socketserver import ThreadingMixIn
import time

import cv2

from pi.perception.tracker import ColorTracker
from pi.perception.vision_control import compute_base_adjust, compute_shoulder_adjust


class _StreamingHandler(server.BaseHTTPRequestHandler):
    camera = None
    
    def log_message(self, format, *args):
        # Keep HTTP logs quiet in long-running stream mode.
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
                self.send_error(503, "No camera frame available")
                self.end_headers()
                return

            ok, jpg = cv2.imencode(".jpg", frame)
            if not ok:
                self.send_error(500, "Failed to encode frame")
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

        tracker = ColorTracker()
        last_log_ts = 0.0

        while True:
            frame = self.camera.get_frame() if self.camera else None
            if frame is None:
                time.sleep(0.01)
                continue

            result = tracker.track(frame)
            output = frame.copy()
            frame_h, frame_w = output.shape[:2]
            frame_center_x = frame_w // 2

            # Reference line for horizontal centering target.
            cv2.line(
                output,
                (frame_center_x, 0),
                (frame_center_x, frame_h - 1),
                (255, 255, 0),
                1,
            )

            if result:
                cx, cy = result["center"]
                x, y, w, h = result["bbox"]
                area = int(result["area"])
                error = cx - frame_center_x
                adjust = compute_base_adjust(cx, frame_w)
                vision_delta = -adjust
                shoulder_delta = compute_shoulder_adjust(float(area))

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
                cv2.putText(
                    output,
                    f"cx={cx} center={frame_center_x} err={error} base_delta={vision_delta}",
                    (10, 58),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    output,
                    f"area={area} shoulder_delta={shoulder_delta}",
                    (10, 84),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 0),
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
