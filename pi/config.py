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

# ===== INVERSE KINEMATICS (planar 2R: shoulder + elbow in vertical X–Z plane) =====
# X: horizontal reach (mm) from shoulder pivot along "forward", Z: up (mm).
# Calibrate IK_* on your hardware: print poses (P) at known configs, then tune.
IK_LINK_L1_MM = 130.0
IK_LINK_L2_MM = 120.0
# Joint angles (rad) at the PWM home pose below (nominal — tune until FK matches reality).
IK_SHOULDER_HOME_RAD = 1.15
IK_ELBOW_HOME_RAD = -0.55
# Linear map: pwm ≈ DEFAULT + PWM_PER_RAD * (theta - HOME_RAD)
IK_SHOULDER_PWM_PER_RAD = -52.0
IK_ELBOW_PWM_PER_RAD = 42.0
IK_CARTESIAN_STEP_MM = 6.0
IK_ELBOW_UP = True
