# Laptop YOLO Follow GUI

This app:
- reads frames from the Pi MJPEG stream
- runs YOLO on the laptop
- follows a target class left/right by sending base deltas back to the Pi

## Install

```bash
python -m venv .venv
.venv\Scripts\activate
pip install -r laptop/requirements.txt
```

## Run

```bash
python laptop/yolo_follow_gui.py --pi-ip <PI_IP>
```

Optional flags:
- `--model yolo11s.pt`
- `--target cup`
- `--stream-port 8080`
- `--control-port 9999`

## Controls in GUI

- **Start Follow**: enable control sending
- **Stop Follow**: sends `delta=0`
- Close window: stops control and exits
