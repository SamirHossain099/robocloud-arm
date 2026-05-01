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


class DepthProEstimator:
    def __init__(self, enabled: bool) -> None:
        self.enabled = enabled
        self.ready = False
        self.error = ""
        self._pipe = None
        self._last_depth = None
        self._last_ts = 0.0
        self._interval_s = 0.2

        if not self.enabled:
            return

        try:
            from transformers import pipeline
        except Exception as exc:
            self.error = f"DepthPro disabled: transformers import failed ({exc})"
            return

        try:
            self._pipe = pipeline(task="depth-estimation", model="apple/DepthPro-hf")
            self.ready = True
        except Exception as exc:
            self.error = f"DepthPro init failed: {exc}"

    def estimate(self, frame_bgr: np.ndarray) -> np.ndarray | None:
        if not self.ready or self._pipe is None:
            return None

        now = time.time()
        if self._last_depth is not None and (now - self._last_ts) < self._interval_s:
            return self._last_depth

        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        pil_img = Image.fromarray(frame_rgb)
        out = self._pipe(pil_img)
        depth_img = out.get("depth", None)
        if depth_img is None:
            return self._last_depth

        depth_np = np.array(depth_img, dtype=np.float32)
        if depth_np.size == 0:
            return self._last_depth

        d_min = float(np.min(depth_np))
        d_max = float(np.max(depth_np))
        if d_max > d_min:
            depth_np = (depth_np - d_min) / (d_max - d_min)
        else:
            depth_np = np.zeros_like(depth_np, dtype=np.float32)

        self._last_depth = depth_np
        self._last_ts = now
        return self._last_depth


class YoloFollowApp:
    def __init__(
        self,
        stream_url: str,
        pi_host: str,
        control_port: int,
        model_path: str,
        target_label: str,
        use_depthpro: bool,
    ) -> None:
        self.stream_url = stream_url
        self.pi_host = pi_host
        self.control_port = control_port
        self.target_label = target_label

        self.running = True
        self.follow_enabled = False
        self.last_command_ts = 0.0

        self.kp = 0.02
        self.deadband = 20
        self.max_delta = 6
        self.command_interval = 0.08
        self.detect_conf = 0.15

        self.pred_lines = []
        self.frame_bgr = None
        self.display_bgr = None
        self._status_text = "Status: initializing..."
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

        self.status_var = tk.StringVar(value=self._status_text)
        ttk.Label(side, textvariable=self.status_var).pack(anchor="w", pady=(0, 8))

        ttk.Label(side, text="Predictions").pack(anchor="w")
        self.pred_box = tk.Text(side, width=42, height=28)
        self.pred_box.pack(fill="both", expand=True)

        self.model = YOLO(model_path)
        self.depth = DepthProEstimator(enabled=use_depthpro)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        if self.depth.ready:
            self._set_status("Status: stream pending | DepthPro: enabled")
        elif self.depth.enabled:
            self._set_status(f"Status: stream pending | {self.depth.error}")
        else:
            self._set_status("Status: stream pending | DepthPro: disabled")

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

    def _set_status(self, text: str):
        with self._lock:
            self._status_text = text

    def _capture_loop(self):
        cap = cv2.VideoCapture(self.stream_url)
        if not cap.isOpened():
            self._set_status("Status: stream open failed")
            return

        self._set_status("Status: stream connected")
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
            try:
                frame = None
                with self._lock:
                    if self.frame_bgr is not None:
                        frame = self.frame_bgr.copy()

                if frame is None:
                    time.sleep(0.03)
                    continue

                results = self.model(frame, verbose=False, conf=self.detect_conf)[0]
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
                depth_map = self.depth.estimate(frame)

                if best is not None:
                    x1, y1, x2, y2 = best["xyxy"]
                    cx = best["cx"]
                    error = cx - center_x
                    depth_text = "depth=n/a"

                    if depth_map is not None:
                        x1c = _clamp(x1, 0, depth_map.shape[1] - 1)
                        x2c = _clamp(x2, 0, depth_map.shape[1] - 1)
                        y1c = _clamp(y1, 0, depth_map.shape[0] - 1)
                        y2c = _clamp(y2, 0, depth_map.shape[0] - 1)
                        if x2c > x1c and y2c > y1c:
                            roi = depth_map[y1c:y2c, x1c:x2c]
                            rel_depth = float(np.median(roi))
                            best["rel_depth"] = rel_depth
                            depth_text = f"depth~{rel_depth:.3f} (relative)"

                    cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(display, (cx, (y1 + y2) // 2), 5, (0, 0, 255), -1)
                    cv2.putText(
                        display,
                        f"{self.target_label} conf={best['conf']:.2f} err={error} {depth_text}",
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
                                delta = _clamp(
                                    int(error * self.kp), -self.max_delta, self.max_delta
                                )
                                delta = -delta
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
                    if best is not None and "rel_depth" in best:
                        lines.append(f"{self.target_label:12s} relative_depth={best['rel_depth']:.4f}")
                    self.pred_lines = lines
            except Exception as exc:
                self._set_status(f"Status: inference error: {exc}")
                time.sleep(0.2)

            time.sleep(0.02)

    def _ui_tick(self):
        if not self.running:
            return

        display = None
        raw = None
        lines = []
        status = ""
        with self._lock:
            if self.display_bgr is not None:
                display = self.display_bgr.copy()
            if self.frame_bgr is not None:
                raw = self.frame_bgr.copy()
            lines = list(self.pred_lines)
            status = self._status_text

        if display is None:
            display = raw

        if display is not None:
            rgb = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(rgb)
            tk_img = ImageTk.PhotoImage(image=img)
            self.video_label.configure(image=tk_img)
            self.video_label.image = tk_img

        self.status_var.set(status)

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
    parser.add_argument(
        "--use-depthpro",
        action="store_true",
        help="Enable Apple Depth Pro depth estimation (relative depth).",
    )
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
        use_depthpro=args.use_depthpro,
    )
    app.run()


if __name__ == "__main__":
    main()
