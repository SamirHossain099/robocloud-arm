import socket


class NetworkIO:
    def __init__(self, host: str, port: int, timeout: float = 1.0) -> None:
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock = None

    def connect(self) -> None:
        self.sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        self.sock.settimeout(self.timeout)

    def send_cmd(self, cmd: str) -> None:
        self._require_connection()
        self.sock.sendall((cmd + "\n").encode())

    def send_all(
        self, base: int, shoulder: int, elbow: int, wrist: int, claw: int
    ) -> None:
        self._require_connection()
        self.sock.sendall(
            f"setall {base} {shoulder} {elbow} {wrist} {claw}\n".encode()
        )

    def _require_connection(self) -> None:
        if self.sock is None:
            raise RuntimeError("Network socket is not connected.")
