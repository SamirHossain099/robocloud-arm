# Raspberry Pi UART pins (TX/RX) typically appear as /dev/serial0.
SERIAL_PORT = "/dev/serial0"
SERIAL_BAUDRATE = 115200

CONTROL_TRANSPORT = "serial"  # "serial" or "network"
NETWORK_HOST = "192.168.1.50"
NETWORK_PORT = 9000
NETWORK_TIMEOUT = 1.0

# ===== SERVO / CHANNEL MAP (PCA9685 channels, matches ESP sketch) =====
# 11 — Base rotation (servo inside turntable)
# 12 — Shoulder pitch (first joint above turntable)
# 13 — Elbow
# 14 — Wrist
# 15 — Claw (open/close macros also use PWM targets on this channel)

# ===== DEFAULT POSE =====
BASE_DEFAULT = 307
SHOULDER_DEFAULT = 440
ELBOW_DEFAULT = 150
WRIST_DEFAULT = 196
CLAW_DEFAULT = 320

STEP = 4

# ===== JOINT LIMITS (match ESP writeJoint: PWM_MIN / PWM_MAX — no extra Pi safety margin) =====
PWM_LIMIT_MIN = 102
PWM_LIMIT_MAX = 512

BASE_MIN = PWM_LIMIT_MIN
BASE_MAX = PWM_LIMIT_MAX
SHOULDER_MIN = PWM_LIMIT_MIN
SHOULDER_MAX = PWM_LIMIT_MAX
ELBOW_MIN = PWM_LIMIT_MIN
ELBOW_MAX = PWM_LIMIT_MAX
WRIST_MIN = PWM_LIMIT_MIN
WRIST_MAX = PWM_LIMIT_MAX
CLAW_MIN = PWM_LIMIT_MIN
CLAW_MAX = PWM_LIMIT_MAX
