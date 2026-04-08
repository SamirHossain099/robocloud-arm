# Raspberry Pi UART pins (TX/RX) typically appear as /dev/serial0.
SERIAL_PORT = "/dev/serial0"
SERIAL_BAUDRATE = 115200

CONTROL_TRANSPORT = "serial"  # "serial" or "network"
NETWORK_HOST = "192.168.1.50"
NETWORK_PORT = 9000
NETWORK_TIMEOUT = 1.0

# ===== DEFAULT POSE =====
BASE_DEFAULT = 307
SHOULDER_DEFAULT = 440
ELBOW_DEFAULT = 150
WRIST_DEFAULT = 160
CLAW_DEFAULT = 320

STEP = 4

# ===== JOINT LIMITS (tune SHOULDER_MIN / ELBOW_MIN if arm still touches floor) =====
BASE_MIN = 180
BASE_MAX = 430
# Raised from 200: lower shoulder values reach farther down; this caps that ROM.
SHOULDER_MIN = 360
SHOULDER_MAX = SHOULDER_DEFAULT
ELBOW_MIN = 140
ELBOW_MAX = 500
WRIST_MIN = 130
WRIST_MAX = 480
CLAW_MIN = 80
CLAW_MAX = 420
