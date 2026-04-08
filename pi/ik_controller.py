"""
Interactive IK control: joystick Cartesian, target (x,y), or joint nudge mode.
Run: python -m pi.ik_controller
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

if __package__ is None or __package__ == "":
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import readchar
import serial

from pi import ik_math
from pi.comms import assert_pwms_ok, send_setall_on_serial
from pi.config import (
    BASE_DEFAULT,
    CLAW_CLOSE_PWM,
    CLAW_DEFAULT,
    CLAW_OPEN_PWM,
    ELBOW_DEFAULT,
    HOME_IK_X,
    HOME_IK_Y,
    IK_BASE_STEP_PWM,
    IK_JOYSTICK_SMOOTH_STEPS,
    IK_MOVE_DELAY_MS,
    IK_MOVE_STEPS,
    IK_STEP_MM,
    IK_WRIST_STEP_DEG,
    L1_MM,
    L2_MM,
    L3_MM,
    SERIAL_BAUDRATE,
    SERIAL_PORT,
    SHOULDER_DEFAULT,
    WRIST_DEFAULT,
)


def _home_pwms() -> Dict[int, int]:
    return {
        11: BASE_DEFAULT,
        12: SHOULDER_DEFAULT,
        13: ELBOW_DEFAULT,
        14: WRIST_DEFAULT,
        15: CLAW_DEFAULT,
    }


def move_to_pwms(
    ser: serial.Serial,
    current: Dict[int, int],
    target: Dict[int, int],
    steps: int = IK_MOVE_STEPS,
    delay_ms: float = IK_MOVE_DELAY_MS,
) -> Dict[int, int]:
    """Interpolate each channel 11..15; send setall each step."""
    if steps < 1:
        steps = 1
    for i in range(1, steps + 1):
        interp = {}
        for ch in (11, 12, 13, 14, 15):
            v = int(current[ch] + (target[ch] - current[ch]) * i / steps)
            interp[ch] = ik_math.clamp_pwm(v)
        assert_pwms_ok(interp)
        send_setall_on_serial(ser, interp)
        time.sleep(delay_ms / 1000.0)
    return dict(target)


def solve_send(
    state: Dict[str, Any],
    current_pwms: Dict[int, int],
    ser: serial.Serial,
    smooth_steps: int = IK_JOYSTICK_SMOOTH_STEPS,
) -> Optional[Dict[int, int]]:
    result = ik_math.solve_ik(
        state["x"],
        state["y"],
        state["wrist_tilt"],
        auto_level=state["auto_level"],
        l1=L1_MM,
        l2=L2_MM,
        l3=L3_MM,
    )
    if result is None:
        print("IK unreachable for this (x, y); no move.", flush=True)
        return None
    sh, el, wr = result
    nxt = {
        11: ik_math.clamp_pwm(state["base_pwm"]),
        12: sh,
        13: el,
        14: wr,
        15: ik_math.clamp_pwm(state["claw_pwm"]),
    }
    assert_pwms_ok(nxt)
    if smooth_steps <= 1:
        send_setall_on_serial(ser, nxt)
        final = nxt
    else:
        final = move_to_pwms(ser, current_pwms, nxt, steps=smooth_steps, delay_ms=15)
    print_state(state, final)
    return final


def print_state(state: Dict[str, Any], pwms: Dict[int, int]) -> None:
    print(
        f"x={state['x']:.1f} y={state['y']:.1f} "
        f"base={pwms[11]} tilt°={state['wrist_tilt']:.1f} "
        f"auto_level={state['auto_level']} "
        f"12={pwms[12]} 13={pwms[13]} 14={pwms[14]} 15={pwms[15]}",
        flush=True,
    )


def validation_banner() -> None:
    h = _home_pwms()
    tx, ty = ik_math.forward_kinematics_tip(
        h[12], h[13], h[14], l1=L1_MM, l2=L2_MM, l3=L3_MM
    )
    print("--- IK validation (HOME PWMs) ---", flush=True)
    print(f"HOME tip FK: x={tx:.1f} mm  y={ty:.1f} mm", flush=True)
    print(
        f"Expected rough: x={HOME_IK_X:.0f} y={HOME_IK_Y:.0f} (config / design note)",
        flush=True,
    )
    print(f"HOME setall {h[11]} {h[12]} {h[13]} {h[14]} {h[15]}", flush=True)


def joystick_help() -> None:
    print(
        """
Joystick mode:
  w/s  tip forward/back (∆x)   a/d  base ch11
  r/f  tip up/down (∆y)       q/e  wrist tilt offset (°)
  l    toggle auto-level      o/p  claw open / close
  k    pick-up sequence (bonus)   0  HOME pose
  m    main menu               ESC  exit mode
""",
        flush=True,
    )


def mode_joystick(ser: serial.Serial) -> None:
    joystick_help()
    h = _home_pwms()
    tx, ty = ik_math.forward_kinematics_tip(
        h[12], h[13], h[14], l1=L1_MM, l2=L2_MM, l3=L3_MM
    )
    state: Dict[str, Any] = {
        "x": tx,
        "y": ty,
        "base_pwm": h[11],
        "wrist_tilt": 0.0,
        "auto_level": True,
        "claw_pwm": h[15],
    }
    current = dict(h)
    print("Initial:", flush=True)
    print_state(state, current)

    while True:
        key = readchar.readkey()
        if key == "\x1b":
            return
        if key == "m":
            return

        if key == "w":
            state["x"] += IK_STEP_MM
        elif key == "s":
            state["x"] -= IK_STEP_MM
        elif key == "a":
            state["base_pwm"] += IK_BASE_STEP_PWM
        elif key == "d":
            state["base_pwm"] -= IK_BASE_STEP_PWM
        elif key == "r":
            state["y"] += IK_STEP_MM
        elif key == "f":
            state["y"] -= IK_STEP_MM
        elif key == "e":
            state["wrist_tilt"] += IK_WRIST_STEP_DEG
        elif key == "q":
            state["wrist_tilt"] -= IK_WRIST_STEP_DEG
        elif key == "l":
            state["auto_level"] = not state["auto_level"]
            print(f"auto_level = {state['auto_level']}", flush=True)
            continue
        elif key == "o":
            state["claw_pwm"] = CLAW_OPEN_PWM
            current = {**current, 15: state["claw_pwm"]}
            assert_pwms_ok(current)
            send_setall_on_serial(ser, current)
            print_state(state, current)
            continue
        elif key == "p":
            state["claw_pwm"] = CLAW_CLOSE_PWM
            current = {**current, 15: state["claw_pwm"]}
            assert_pwms_ok(current)
            send_setall_on_serial(ser, current)
            print_state(state, current)
            continue
        elif key == "k":
            out = run_pickup(ser, state, current)
            if out is not None:
                current = out
                state["claw_pwm"] = current[15]
                state["base_pwm"] = current[11]
                state["wrist_tilt"] = 0.0
                state["auto_level"] = True
                state["x"], state["y"] = ik_math.forward_kinematics_tip(
                    current[12], current[13], current[14], l1=L1_MM, l2=L2_MM, l3=L3_MM
                )
            continue
        elif key == "0":
            current = move_to_pwms(
                ser,
                current,
                _home_pwms(),
                steps=IK_MOVE_STEPS,
                delay_ms=IK_MOVE_DELAY_MS,
            )
            h2 = _home_pwms()
            state["base_pwm"] = h2[11]
            state["claw_pwm"] = h2[15]
            state["wrist_tilt"] = 0.0
            tx, ty = ik_math.forward_kinematics_tip(
                h2[12], h2[13], h2[14], l1=L1_MM, l2=L2_MM, l3=L3_MM
            )
            state["x"], state["y"] = tx, ty
            print_state(state, current)
            continue
        else:
            continue

        state["base_pwm"] = ik_math.clamp_pwm(state["base_pwm"])
        state["claw_pwm"] = ik_math.clamp_pwm(state["claw_pwm"])
        nxt = solve_send(state, current, ser, smooth_steps=IK_JOYSTICK_SMOOTH_STEPS)
        if nxt is not None:
            current = nxt


def run_pickup(
    ser: serial.Serial,
    state: Dict[str, Any],
    current: Dict[int, int],
    ground_height_mm: float = -150.0,
) -> Optional[Dict[int, int]]:
    """Bonus pick-up at current X; returns final PWM dict or None on early abort."""
    # 1) hold x,y
    tmp = solve_send(state, current, ser, smooth_steps=3)
    if tmp is None:
        return None
    current = tmp
    # 2) wrist down
    prev_tilt = state["wrist_tilt"]
    prev_auto = state["auto_level"]
    state["auto_level"] = False
    state["wrist_tilt"] = -60.0
    tmp = solve_send(state, current, ser, smooth_steps=8)
    if tmp is None:
        state["wrist_tilt"] = prev_tilt
        state["auto_level"] = prev_auto
        return None
    current = tmp
    # 3) lower
    state["y"] = ground_height_mm
    tmp = solve_send(state, current, ser, smooth_steps=12)
    if tmp is None:
        fy = ik_math.forward_kinematics_tip(
            current[12], current[13], current[14], l1=L1_MM, l2=L2_MM, l3=L3_MM
        )[1]
        state["y"] = fy
        state["wrist_tilt"] = prev_tilt
        state["auto_level"] = prev_auto
        return None
    current = tmp
    # 4) close
    tgt = {**current, 15: CLAW_CLOSE_PWM}
    current = move_to_pwms(ser, current, tgt, steps=10, delay_ms=40)
    state["claw_pwm"] = current[15]
    # 5) raise
    state["y"] = 180.0
    state["auto_level"] = True
    state["wrist_tilt"] = 0.0
    tmp = solve_send(state, current, ser, smooth_steps=15)
    if tmp is not None:
        current = tmp
    print("pick-up sequence done.", flush=True)
    return current


def _clamp_tip_target(
    tx: float, ty: float, wrist_tilt_deg: float = 0.0
) -> Tuple[float, float, bool]:
    """
    If the wrist joint lies inside the 2-link inner workspace, push (tx,ty) outward
    along the same direction from the shoulder until reachable. Returns (tx', ty', clamped?).
    """
    tilt = math.radians(wrist_tilt_deg)
    wx = tx - L3_MM * math.cos(tilt)
    wy = ty - L3_MM * math.sin(tilt)
    d = math.hypot(wx, wy)
    min_reach = abs(L1_MM - L2_MM) + 1.0
    max_reach = L1_MM + L2_MM - 1.0
    clamped = False
    if d < min_reach:
        clamped = True
        if d < 1e-9:
            wx, wy = min_reach, 0.0
        else:
            s = min_reach / d
            wx, wy = wx * s, wy * s
    elif d > max_reach:
        clamped = True
        s = max_reach / d
        wx, wy = wx * s, wy * s
    tx2 = wx + L3_MM * math.cos(tilt)
    ty2 = wy + L3_MM * math.sin(tilt)
    return tx2, ty2, clamped


def mode_target(ser: serial.Serial, cur: Dict[int, int]) -> Dict[int, int]:
    cur = dict(cur)
    raw = input("Enter target x y (mm from shoulder), e.g. 200 100: ").strip()
    parts = raw.split()
    if len(parts) < 2:
        print("Need two numbers.", flush=True)
        return cur
    tx, ty = float(parts[0]), float(parts[1])
    tx, ty, did_clamp = _clamp_tip_target(tx, ty, 0.0)
    if did_clamp:
        print(
            f"Target adjusted to nearest reachable tip pose: x={tx:.1f} y={ty:.1f}",
            flush=True,
        )
    if ik_math.solve_ik(tx, ty, 0.0, True, l1=L1_MM, l2=L2_MM, l3=L3_MM) is None:
        print("IK still unreachable after clamp; no move.", flush=True)
        return cur
    yn = input("Execute smooth move? (y/n): ").strip().lower()
    if yn != "y":
        return cur
    result = ik_math.solve_ik(
        tx, ty, 0.0, True, l1=L1_MM, l2=L2_MM, l3=L3_MM
    )
    if result is None:
        print("IK failed at execute step.", flush=True)
        return cur
    sh, el, wr = result
    tgt = {
        **cur,
        12: sh,
        13: el,
        14: wr,
    }
    cur = move_to_pwms(ser, cur, tgt, steps=IK_MOVE_STEPS, delay_ms=IK_MOVE_DELAY_MS)
    print("Done.", cur, flush=True)
    return cur


def mode_sliders(ser: serial.Serial) -> None:
    channels = [11, 12, 13, 14, 15]
    labels = {
        11: "Base",
        12: "Shoulder",
        13: "Elbow",
        14: "Wrist",
        15: "Claw",
    }
    pwms = _home_pwms()
    idx = 0
    step = 4
    print(
        "Sliders: Tab next joint  +/- nudge  Enter confirm/send  m menu  ESC exit",
        flush=True,
    )
    while True:
        print(
            f"  [{labels[channels[idx]]:8}] ch{channels[idx]} = {pwms[channels[idx]]}",
            flush=True,
        )
        k = readchar.readkey()
        if k == "\x1b" or k == "m":
            return
        if k == "\t":
            idx = (idx + 1) % len(channels)
        elif k == "+" or k == "=":
            ch = channels[idx]
            pwms[ch] = ik_math.clamp_pwm(pwms[ch] + step)
            assert_pwms_ok(pwms)
            send_setall_on_serial(ser, pwms)
        elif k == "-" or k == "_":
            ch = channels[idx]
            pwms[ch] = ik_math.clamp_pwm(pwms[ch] - step)
            assert_pwms_ok(pwms)
            send_setall_on_serial(ser, pwms)
        elif k in ("\r", "\n"):
            assert_pwms_ok(pwms)
            send_setall_on_serial(ser, pwms)
            print(f"  (sent) setall {pwms[11]} {pwms[12]} {pwms[13]} {pwms[14]} {pwms[15]}", flush=True)


def main() -> None:
    print("=== RoboCloud Arm — IK controller ===", flush=True)
    validation_banner()
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=1)
    last_pwms = _home_pwms()
    try:
        while True:
            print(
                "\nMode: [1] joystick  [2] target x y  [3] joint sliders  [q] quit\n> ",
                end="",
                flush=True,
            )
            choice = input().strip().lower()
            if choice == "q":
                break
            if choice == "1":
                mode_joystick(ser)
            elif choice == "2":
                last_pwms = mode_target(ser, last_pwms)
            elif choice == "3":
                mode_sliders(ser)
            else:
                print("Unknown.", flush=True)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
