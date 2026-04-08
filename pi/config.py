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
WRIST_DEFAULT = 160
CLAW_DEFAULT = 320

STEP = 4

# ===== JOINT LIMITS (raise SHOULDER_MIN / ELBOW_MIN if arm hits bench / floor) =====
BASE_MIN = 180
BASE_MAX = 430
SHOULDER_MIN = 280
SHOULDER_MAX = SHOULDER_DEFAULT
ELBOW_MIN = 100
ELBOW_MAX = 500
WRIST_MIN = 120
WRIST_MAX = 480
CLAW_MIN = 80
CLAW_MAX = 420

# ----- IK CLI (pi.ik_controller, pi.ik_math) -----
L1_MM = L2_MM = L3_MM = 115
HOME_IK_X = 60.0
HOME_IK_Y = 200.0
IK_STEP_MM = 5
IK_WRIST_STEP_DEG = 5
IK_BASE_STEP_PWM = 10
IK_CLAW_STEP_PWM = 12
IK_MOVE_STEPS = 20
IK_MOVE_DELAY_MS = 50
IK_JOYSTICK_SMOOTH_STEPS = 1  # 1 = no interp; try 3 for softer joystick
CLAW_OPEN_PWM = 140
CLAW_CLOSE_PWM = 320
