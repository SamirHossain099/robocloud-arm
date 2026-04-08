import time

import serial


class SerialIO:
    def __init__(self, port: str, baudrate: int, timeout: float = 1.0) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None

    def connect(self) -> None:
        self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        time.sleep(2)

    def send_cmd(self, cmd: str) -> None:
        self._require_connection()
        self.ser.write((cmd + "\n").encode())
        time.sleep(0.2)

    def send_all(
        self, base: int, shoulder: int, elbow: int, wrist: int, claw: int
    ) -> None:
        self._require_connection()
        self.ser.write(f"set 11 {base}\n".encode())
        self.ser.write(f"set 12 {shoulder}\n".encode())
        self.ser.write(f"set 13 {elbow}\n".encode())
        self.ser.write(f"set 14 {wrist}\n".encode())
        self.ser.write(f"set 15 {claw}\n".encode())

    def _require_connection(self) -> None:
        if self.ser is None:
            raise RuntimeError("Serial port is not connected.")
