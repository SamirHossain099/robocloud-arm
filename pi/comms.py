"""Minimal serial transport for IK CLI (setall lines only)."""

from __future__ import annotations

import serial

from pi.config import SERIAL_BAUDRATE, SERIAL_PORT


def _setall_line(pwms: dict) -> str:
    """Format setall line; str.format must not use {11} (parsed as arg index)."""
    return "setall {} {} {} {} {}\n".format(
        int(pwms[11]),
        int(pwms[12]),
        int(pwms[13]),
        int(pwms[14]),
        int(pwms[15]),
    )


def send_setall(
    pwms: dict,
    *,
    port: str = SERIAL_PORT,
    baudrate: int = SERIAL_BAUDRATE,
    ser: serial.Serial | None = None,
) -> None:
    """
    Send one setall line. pwms uses int keys 11..15.
    If ser is provided, use it; otherwise open port for each call (avoid for loops).
    """
    line = _setall_line(pwms)
    own = False
    if ser is None:
        ser = serial.Serial(port, baudrate, timeout=1)
        own = True
    try:
        ser.write(line.encode())
    finally:
        if own:
            ser.close()


def send_setall_on_serial(ser: serial.Serial, pwms: dict) -> None:
    line = _setall_line(pwms)
    ser.write(line.encode())


def assert_pwms_ok(pwms: dict) -> None:
    for ch in (11, 12, 13, 14, 15):
        v = pwms[ch]
        if not (102 <= v <= 512):
            raise ValueError(f"PWM ch{ch}={v} out of [102, 512]")
