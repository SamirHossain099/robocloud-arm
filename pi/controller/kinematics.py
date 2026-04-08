"""
Planar 2-link inverse kinematics for shoulder (12) + elbow (13).

Convention: shoulder at origin; X horizontal (reach), Z vertical up;
theta1 from +X toward +Z; theta2 is elbow bend relative to link1.
PWM mapping is linear — calibrate in pi.config IK_* constants.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

from pi import config as cfg


def _clamp_acos_arg(c: float) -> Optional[float]:
    if c < -1.0 - 1e-9 or c > 1.0 + 1e-9:
        return None
    return max(-1.0, min(1.0, c))


def solve_ik_2r(
    l1: float, l2: float, x: float, z: float, elbow_up: bool = True
) -> Optional[Tuple[float, float]]:
    d_sq = x * x + z * z
    d = math.sqrt(d_sq)
    if d < 1e-6:
        return None
    reach_max = l1 + l2
    reach_min = abs(l1 - l2)
    if d > reach_max + 1e-6 or d < reach_min - 1e-6:
        return None

    c2 = _clamp_acos_arg((d_sq - l1 * l1 - l2 * l2) / (2.0 * l1 * l2))
    if c2 is None:
        return None
    s2_mag = math.sqrt(max(0.0, 1.0 - c2 * c2))
    s2 = s2_mag if elbow_up else -s2_mag
    t2 = math.atan2(s2, c2)
    t1 = math.atan2(z, x) - math.atan2(l2 * math.sin(t2), l1 + l2 * math.cos(t2))
    return t1, t2


def pwm_to_angles(shoulder_pwm: int, elbow_pwm: int) -> Tuple[float, float]:
    t1 = cfg.IK_SHOULDER_HOME_RAD + (
        (shoulder_pwm - cfg.SHOULDER_DEFAULT) / cfg.IK_SHOULDER_PWM_PER_RAD
    )
    t2 = cfg.IK_ELBOW_HOME_RAD + (
        (elbow_pwm - cfg.ELBOW_DEFAULT) / cfg.IK_ELBOW_PWM_PER_RAD
    )
    return t1, t2


def angles_to_pwm(t1: float, t2: float) -> Tuple[int, int]:
    sh = int(
        round(
            cfg.SHOULDER_DEFAULT
            + cfg.IK_SHOULDER_PWM_PER_RAD * (t1 - cfg.IK_SHOULDER_HOME_RAD)
        )
    )
    el = int(
        round(
            cfg.ELBOW_DEFAULT + cfg.IK_ELBOW_PWM_PER_RAD * (t2 - cfg.IK_ELBOW_HOME_RAD)
        )
    )
    return sh, el


def forward_xz(shoulder_pwm: int, elbow_pwm: int) -> Tuple[float, float]:
    t1, t2 = pwm_to_angles(shoulder_pwm, elbow_pwm)
    l1, l2 = cfg.IK_LINK_L1_MM, cfg.IK_LINK_L2_MM
    x = l1 * math.cos(t1) + l2 * math.cos(t1 + t2)
    z = l1 * math.sin(t1) + l2 * math.sin(t1 + t2)
    return x, z


def inverse_to_pwm(x_mm: float, z_mm: float, elbow_up: Optional[bool] = None) -> Optional[Tuple[int, int]]:
    if elbow_up is None:
        elbow_up = cfg.IK_ELBOW_UP
    sol = solve_ik_2r(
        cfg.IK_LINK_L1_MM, cfg.IK_LINK_L2_MM, x_mm, z_mm, elbow_up=elbow_up
    )
    if sol is None:
        return None
    return angles_to_pwm(sol[0], sol[1])


def nudge_tip(
    shoulder_pwm: int,
    elbow_pwm: int,
    dx_mm: float,
    dz_mm: float,
    elbow_up: Optional[bool] = None,
) -> Optional[Tuple[int, int]]:
    x, z = forward_xz(shoulder_pwm, elbow_pwm)
    return inverse_to_pwm(x + dx_mm, z + dz_mm, elbow_up=elbow_up)
