import socket
import time


class NetworkIO:
    def __init__(self, host: str, port: int, timeout: float = 1.0) -> None:
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock = None

    def connect(self) -> None:
        self.sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        self.sock.settimeout(self.timeout)
        time.sleep(0.2)

    def send_cmd(self, cmd: str) -> None:
        self._require_connection()
        self.sock.sendall((cmd + "\n").encode())
        time.sleep(0.05)

    def send_all(
        self, base: int, shoulder: int, elbow: int, wrist: int, claw: int
    ) -> None:
        self._require_connection()
        self.sock.sendall(f"set 11 {base}\n".encode())
        self.sock.sendall(f"set 12 {shoulder}\n".encode())
        self.sock.sendall(f"set 13 {elbow}\n".encode())
        self.sock.sendall(f"set 14 {wrist}\n".encode())
        self.sock.sendall(f"set 15 {claw}\n".encode())

    def _require_connection(self) -> None:
        if self.sock is None:
            raise RuntimeError("Network socket is not connected.")
