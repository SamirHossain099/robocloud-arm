import argparse
import json
import socket
import threading
import time
import tkinter as tk
from tkinter import ttk

import cv2
import numpy as np
from PIL import Image, ImageTk
from ultralytics import YOLO


def _clamp(value: int, minimum: int, maximum: int) -> int:
    return max(minimum, min(maximum, value))


class YoloFollowApp:
    def __init__(
        self,
        stream_url: str,
        pi_host: str,
        control_port: int,
        model_path: str,
        target_label: str,
    ) -> None:
        self.stream_url = stream_url
        self.pi_host = pi_host
        self.control_port = control_port
        self.target_label = target_label

        self.model = YOLO(model_path)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.running = True
        self.follow_enabled = False
        self.last_command_ts = 0.0

        self.kp = 0.02
        self.deadband = 20
        self.max_delta = 6
        self.command_interval = 0.08

        self.pred_lines = []
        self.frame_bgr = None
        self.display_bgr = None
        self._lock = threading.Lock()

        self.root = tk.Tk()
        self.root.title("RoboCloud YOLO Follow (Cup)")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.video_label = tk.Label(self.root)
        self.video_label.grid(row=0, column=0, padx=8, pady=8)

        side = ttk.Frame(self.root)
        side.grid(row=0, column=1, sticky="ns", padx=8, pady=8)

        self.btn_follow = ttk.Button(side, text="Start Follow", command=self.toggle_follow)
        self.btn_follow.pack(fill="x", pady=(0, 8))

        self.btn_stop = ttk.Button(side, text="Send Stop", command=self.send_stop)
        self.btn_stop.pack(fill="x", pady=(0, 8))

        self.status_var = tk.StringVar(value="Status: initializing...")
        ttk.Label(side, textvariable=self.status_var).pack(anchor="w", pady=(0, 8))

        ttk.Label(side, text="Predictions").pack(anchor="w")
        self.pred_box = tk.Text(side, width=42, height=28)
        self.pred_box.pack(fill="both", expand=True)

        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()

        self.infer_thread = threading.Thread(target=self._infer_loop, daemon=True)
        self.infer_thread.start()

        self.root.after(30, self._ui_tick)

    def toggle_follow(self):
        self.follow_enabled = not self.follow_enabled
        self.btn_follow.config(text="Stop Follow" if self.follow_enabled else "Start Follow")

    def send_stop(self):
        self._send_delta(0)

    def _send_delta(self, delta: int):
        payload = json.dumps({"delta": int(delta)}).encode("utf-8")
        self.sock.sendto(payload, (self.pi_host, self.control_port))

    def _capture_loop(self):
        cap = cv2.VideoCapture(self.stream_url)
        if not cap.isOpened():
            self.status_var.set("Status: stream open failed")
            return

        self.status_var.set("Status: stream connected")
        while self.running:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            with self._lock:
                self.frame_bgr = frame

        cap.release()

    def _infer_loop(self):
        while self.running:
            frame = None
            with self._lock:
                if self.frame_bgr is not None:
                    frame = self.frame_bgr.copy()

            if frame is None:
                time.sleep(0.03)
                continue

            results = self.model(frame, verbose=False)[0]
            names = results.names

            h, w = frame.shape[:2]
            center_x = w // 2

            best = None
            lines = []
            for box in results.boxes:
                cls_idx = int(box.cls.item())
                conf = float(box.conf.item())
                label = names.get(cls_idx, str(cls_idx))
                xyxy = box.xyxy[0].cpu().numpy().astype(int)
                x1, y1, x2, y2 = xyxy.tolist()
                cx = (x1 + x2) // 2
                area = max(0, x2 - x1) * max(0, y2 - y1)
                lines.append(f"{label:12s} conf={conf:.2f} cx={cx:4d} area={area:6d}")

                if label == self.target_label:
                    if best is None or conf > best["conf"]:
                        best = {
                            "conf": conf,
                            "xyxy": (x1, y1, x2, y2),
                            "cx": cx,
                            "area": area,
                        }

            display = frame.copy()
            cv2.line(display, (center_x, 0), (center_x, h - 1), (255, 255, 0), 2)

            if best is not None:
                x1, y1, x2, y2 = best["xyxy"]
                cx = best["cx"]
                error = cx - center_x

                cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(display, (cx, (y1 + y2) // 2), 5, (0, 0, 255), -1)
                cv2.putText(
                    display,
                    f"{self.target_label} conf={best['conf']:.2f} err={error}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

                if self.follow_enabled:
                    now = time.time()
                    if now - self.last_command_ts >= self.command_interval:
                        delta = 0
                        if abs(error) >= self.deadband:
                            delta = _clamp(int(error * self.kp), -self.max_delta, self.max_delta)
                        self._send_delta(delta)
                        self.last_command_ts = now
            else:
                cv2.putText(
                    display,
                    f"No {self.target_label}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )

            with self._lock:
                self.display_bgr = display
                self.pred_lines = lines

            time.sleep(0.02)

    def _ui_tick(self):
        if not self.running:
            return

        display = None
        lines = []
        with self._lock:
            if self.display_bgr is not None:
                display = self.display_bgr.copy()
            lines = list(self.pred_lines)

        if display is not None:
            rgb = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(rgb)
            tk_img = ImageTk.PhotoImage(image=img)
            self.video_label.configure(image=tk_img)
            self.video_label.image = tk_img

        self.pred_box.delete("1.0", tk.END)
        if lines:
            self.pred_box.insert(tk.END, "\n".join(lines))
        else:
            self.pred_box.insert(tk.END, "No detections yet.")

        self.root.after(30, self._ui_tick)

    def on_close(self):
        self.running = False
        self.follow_enabled = False
        try:
            self._send_delta(0)
        except Exception:
            pass
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def parse_args():
    parser = argparse.ArgumentParser(description="YOLO GUI follow app (cup, left/right only).")
    parser.add_argument("--pi-ip", required=True, help="Pi IP address.")
    parser.add_argument("--stream-port", type=int, default=8080, help="Pi stream port.")
    parser.add_argument("--control-port", type=int, default=9999, help="Pi UDP control port.")
    parser.add_argument("--model", default="yolo11n.pt", help="Ultralytics model path.")
    parser.add_argument("--target", default="cup", help="COCO class label to follow.")
    return parser.parse_args()


def main():
    args = parse_args()
    stream_url = f"http://{args.pi_ip}:{args.stream_port}/stream"
    app = YoloFollowApp(
        stream_url=stream_url,
        pi_host=args.pi_ip,
        control_port=args.control_port,
        model_path=args.model,
        target_label=args.target,
    )
    app.run()


if __name__ == "__main__":
    main()
