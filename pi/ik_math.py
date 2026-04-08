"""
Pure inverse / forward kinematics for 3-link planar arm (no I/O).
Angles internally in radians; PWM mapping per ik_roboticarm spec.
"""

from __future__ import annotations

import math
from typing import Dict, Optional, Tuple

# Defaults — overridden by pi.config when imported from controller
L1 = L2 = L3 = 115  # mm
PWM_MIN = 102
PWM_MAX = 512
PWM_SPAN = PWM_MAX - PWM_MIN  # 410
NEUTRAL_PWM = 307


def clamp_pwm(pwm: int) -> int:
    return max(PWM_MIN, min(PWM_MAX, int(pwm)))


def angle_to_pwm_shoulder(angle_from_vertical_deg: float) -> int:
    """0° = up = 307. Positive forward (from vertical) = decreasing PWM."""
    return int(NEUTRAL_PWM - (angle_from_vertical_deg / 180.0) * PWM_SPAN)


def angle_to_pwm_elbow(bend_deg: float) -> int:
    """0° = straight = 307. More bend = decreasing PWM."""
    return int(NEUTRAL_PWM - (bend_deg / 180.0) * PWM_SPAN)


def angle_to_pwm_wrist(tilt_deg: float) -> int:
    """0° = level/neutral = 307. Positive = tilt up = increasing PWM."""
    return int(NEUTRAL_PWM + (tilt_deg / 180.0) * PWM_SPAN)


def pwm_to_shoulder_angle_deg(pwm: int) -> float:
    return (NEUTRAL_PWM - pwm) / PWM_SPAN * 180.0


def pwm_to_elbow_bend_deg(pwm: int) -> float:
    return (NEUTRAL_PWM - pwm) / PWM_SPAN * 180.0


def pwm_to_wrist_tilt_deg(pwm: int) -> float:
    return (pwm - NEUTRAL_PWM) / PWM_SPAN * 180.0


def solve_ik(
    tx: float,
    ty: float,
    wrist_tilt_deg: float = 0.0,
    auto_level: bool = True,
    l1: float = L1,
    l2: float = L2,
    l3: float = L3,
) -> Optional[Tuple[int, int, int]]:
    """
    Returns (shoulder_pwm, elbow_pwm, wrist_pwm) for channels 12,13,14
    or None if unreachable (inner workspace).
    """
    tilt_rad = math.radians(wrist_tilt_deg)

    wx = tx - l3 * math.cos(tilt_rad)
    wy = ty - l3 * math.sin(tilt_rad)

    d = math.sqrt(wx * wx + wy * wy)
    max_reach = l1 + l2 - 1.0
    min_reach = abs(l1 - l2) + 1.0

    if d > max_reach:
        scale = max_reach / d
        wx *= scale
        wy *= scale
        d = max_reach
    if d < min_reach:
        return None

    cos_elbow = (d * d - l1 * l1 - l2 * l2) / (2 * l1 * l2)
    cos_elbow = max(-1.0, min(1.0, cos_elbow))
    elbow_angle = math.acos(cos_elbow)

    alpha = math.atan2(wx, wy)
    beta = math.acos((d * d + l1 * l1 - l2 * l2) / (2 * d * l1))
    shoulder_angle = alpha + beta

    shoulder_pwm = angle_to_pwm_shoulder(math.degrees(shoulder_angle))
    elbow_pwm = angle_to_pwm_elbow(math.degrees(elbow_angle))

    if auto_level:
        forearm_angle_from_vertical = shoulder_angle - (math.pi - elbow_angle)
        wrist_compensation = -forearm_angle_from_vertical
        wrist_angle_total = wrist_compensation + tilt_rad
        wrist_pwm = angle_to_pwm_wrist(math.degrees(wrist_angle_total))
    else:
        wrist_pwm = angle_to_pwm_wrist(wrist_tilt_deg)

    return (
        clamp_pwm(shoulder_pwm),
        clamp_pwm(elbow_pwm),
        clamp_pwm(wrist_pwm),
    )


def forward_kinematics_tip(
    shoulder_pwm: int,
    elbow_pwm: int,
    wrist_pwm: int,
    l1: float = L1,
    l2: float = L2,
    l3: float = L3,
) -> Tuple[float, float]:
    """
    Tip (tx, ty) in mm from shoulder; wrist third link tilt from wrist PWM.
    """
    theta_s = math.radians(pwm_to_shoulder_angle_deg(shoulder_pwm))
    bend_rad = math.radians(pwm_to_elbow_bend_deg(elbow_pwm))
    phi = math.pi - bend_rad
    forearm_from_vertical = theta_s - (math.pi - phi)
    ex = l1 * math.sin(theta_s)
    ey = l1 * math.cos(theta_s)
    wx = ex + l2 * math.sin(forearm_from_vertical)
    wy = ey + l2 * math.cos(forearm_from_vertical)
    tilt_rad = math.radians(pwm_to_wrist_tilt_deg(wrist_pwm))
    tx = wx + l3 * math.cos(tilt_rad)
    ty = wy + l3 * math.sin(tilt_rad)
    return tx, ty


def forward_kinematics_home_validate(pwms: Dict[int, int]) -> Tuple[float, float]:
    """Tip position from full pose dict (uses ch12,13,14)."""
    return forward_kinematics_tip(
        pwms[12], pwms[13], pwms[14]
    )
