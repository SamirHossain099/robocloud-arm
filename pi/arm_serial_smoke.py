"""Quick Pi→ESP UART check: open serial, send 'stop', then one harmless set. Run on the Pi."""
from __future__ import annotations

import os
import sys
import time

import serial

from pi.config import SERIAL_BAUDRATE, SERIAL_PORT


def main() -> int:
    port = os.getenv("ROBOCLOUD_SERIAL_PORT", SERIAL_PORT)
    baud = int(os.getenv("ROBOCLOUD_SERIAL_BAUDRATE", str(SERIAL_BAUDRATE)))
    print(f"Opening {port!r} @ {baud} ...")
    try:
        ser = serial.Serial(port, baud, timeout=1)
    except (OSError, serial.SerialException) as exc:
        print(f"FAILED: {exc}")
        return 1
    time.sleep(2)
    for line in ("stop\n", "set 11 307\n"):
        ser.write(line.encode())
        print(f"Sent: {line.strip()!r}")
        time.sleep(0.3)
    ser.close()
    print("OK: bytes written. If the arm does not twitch, check TX/RX swap, GND, and ESP GPIO43/44.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
