# Laptop YOLO Follow GUI

This app:
- reads frames from the Pi MJPEG stream
- runs YOLO on the laptop
- follows a target class left/right by sending base deltas back to the Pi
- can run a tool-calling OpenAI "Dummy" agent for prompted sweep-and-report tasks

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
- `--use-depthpro` (Apple Depth Pro relative depth estimate)

## Dummy OpenAI agent GUI (laptop brain)

This GUI keeps all planning/decisions on the laptop. The Pi is only used for:
- snapshots/stream
- base control commands
- optional TTS playback command relay

Set your API key:

```bash
set OPENAI_API_KEY=sk-...
```

Run:

```bash
python laptop/dummy_agent_gui.py
```

Example prompt in the GUI:
- `Hey Dummy, look around and tell me what objects you can find.`

Notes:
- The main GUI controls are now kept simple: Pi IP/ports, LLM model, YOLO model/conf, scan step, nudge step, target frames, and a motion profile.
- Advanced timing/tuning fields are hidden behind **Show advanced**.
- Additional capture/sweep fields are included: camera HFOV, base sweep degrees, target overlap, blur threshold, and narration interval.
- Use **Target frames** to control coverage directly (e.g. 4 means about 4 snapshots across full sweep); this drives sweep step automatically.
- Click **Connect** to open live camera preview.
- Click **Run** to execute a prompt-driven agent task.
- Click **Stop** to halt the running task, and **Reset Arm** to move base to default.
- Settings are persisted to `laptop/dummy_agent_settings.json` for simple relaunch.
- Agent context is constrained to robot arm details, available controls, task/progress, and inference summary.
- Movement intent split:
  - `sweep_step` uses scan step for fast scene coverage
  - `nudge_base` uses nudge step for small adjustments
- Additional generalized movement tools are available for future non-sweep commands:
  - `move_base_to_pwm`
  - `move_base_by_degrees`
- The controller ignores LLM-provided sweep step values and uses your GUI sweep policy, so it won't get stuck at old hardcoded `18` behavior.
- Current motion scope is base-only (left/right).
- `speak(...)` now plays on the laptop speaker directly (Windows SAPI / `espeak` fallback).
- Capture is blur-gated: each step waits for settle and retries snapshots until a minimum sharpness score is reached.
- Sweep step can be auto-derived from HFOV + overlap target to reduce repeated overlapping views.

## Controls in GUI

- **Start Follow**: enable control sending
- **Stop Follow**: sends `delta=0`
- Close window: stops control and exits

## Depth Estimation Notes

When `--use-depthpro` is enabled, the app uses Apple Depth Pro via Hugging Face
to estimate **relative depth** (normalized 0..1) for the target bounding box.
It is useful for near/far ranking and control heuristics, but is not absolute meters
without additional calibration.

## Step-by-step DepthPro GUI (temporary)

This script is separate and button-driven:
1. Capture one frame from Pi
2. Run YOLO banana detection
3. Run DepthPro and estimate banana distance in meters

```bash
python laptop/depthpro_step_gui.py --pi-ip <PI_IP>
```

If using the official `depth_pro` package from Apple repo, install it in your
active environment first (as documented in that repo), then run the GUI.
