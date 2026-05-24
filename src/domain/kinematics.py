"""Leg kinematics: foot position <-> joint angles. Pure, deterministic.

inverse_kinematics is coord2polar ported; the body is condensed but the
contract is final: one foot in, three angles out, no side effects.
"""

from __future__ import annotations

import math

from src.domain.constants import LEG_A, LEG_B, LEG_C, ALPHA_LIMITS, BETA_LIMITS, GAMMA_LIMITS
from src.domain.coordinates import FootPosition, JointAngles, AllLegAngles


def inverse_kinematics(foot: FootPosition) -> JointAngles:
    """Foot position [x, y, z] -> joint angles (alpha, beta, gamma)."""
    x, y, z = foot
    a, b, c = LEG_A, LEG_B, LEG_C
    w = math.sqrt(x * x + y * y)
    v = w - c
    u = max(30.0, min(91.58, math.sqrt(z * z + v * v)))
    beta = math.degrees(math.acos((b * b + a * a - u * u) / (2 * b * a)))
    alpha = math.degrees(
        math.atan2(z, v) + math.acos((a * a + u * u - b * b) / (2 * a * u))
    )
    gamma = math.degrees(math.atan2(y, x))
    return 90.0 - alpha, beta - 90.0, -(gamma - 45.0)


def clamp_angles(angles: JointAngles) -> tuple[bool, JointAngles]:
    """Clamp to joint limits. Returns (was_clamped, clamped_angles)."""
    a, b, g = angles
    ca = min(max(a, ALPHA_LIMITS[0]), ALPHA_LIMITS[1])
    cb = min(max(b, BETA_LIMITS[0]), BETA_LIMITS[1])
    cg = min(max(g, GAMMA_LIMITS[0]), GAMMA_LIMITS[1])
    clamped = (ca, cb, cg)
    return (clamped != angles, clamped)


def apply_offsets(ideal: JointAngles, leg_offsets: JointAngles) -> JointAngles:
    """Add per-servo calibration offset to one leg's ideal angles.

    The pure bridge from IDEAL angles (what the math says) to PHYSICAL angles
    (what this particular robot's servos must be sent, given how the horns sit):
        physical = ideal + offset
    No I/O, no state — just the correction. leg_offsets is this leg's three
    offsets in (alpha, beta, gamma) order.
    """
    return (
        ideal[0] + leg_offsets[0],
        ideal[1] + leg_offsets[1],
        ideal[2] + leg_offsets[2],
    )


def apply_offsets_all(
        ideal: AllLegAngles,
        offsets: tuple[float, ...],
) -> AllLegAngles:
    """Apply offsets to all four legs. `offsets` is the flat 12-float array in
    PIN_LIST/leg order (leg i -> offsets[i*3 : i*3+3]); this wrapper handles the
    slicing so callers never index the flat array by hand.

    Used on the hot path (locomotion -> physical angles) and anywhere the full
    four-leg correction is needed in one shot.
    """
    if len(offsets) != 12:
        raise ValueError(f"expected 12 offsets, got {len(offsets)}")
    out = []
    for leg in range(4):
        o = leg * 3
        out.append(apply_offsets(ideal[leg], (offsets[o], offsets[o + 1], offsets[o + 2])))
    return out[0], out[1], out[2], out[3]
