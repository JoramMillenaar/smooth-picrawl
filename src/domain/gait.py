"""Gait: the pure math of continuous locomotion, and the composition seam.

gait_foot_targets is a pure function of (phase, velocity) -> foot positions. It
does NOT own the clock; phase is an input. The use-case advances phase and calls
this each tick. Body condensed; contract final.

compose_foot_targets is THE SEAM: sum gait positions + posture offsets in
foot-space, before IK.
"""

from __future__ import annotations

from src.domain.constants import DEFAULT_STANCE
from src.domain.coordinates import BodyVelocity, FootPosition


def gait_foot_targets(phase: float, velocity: BodyVelocity) -> tuple[FootPosition, ...]:
    """(phase in [0, 1), desired velocity) -> four foot POSITIONS, leg order 0..3.

    Each leg has an offset in the cycle; swing legs arc to the next foothold,
    stance legs push along. Condensed stub below; real impl derives stride from
    velocity and computes swing-arc / stance-line per leg from phase.
    """
    stride = velocity.vx * 20.0
    return tuple((x, y + stride, z) for (x, y, z) in DEFAULT_STANCE)


def compose_foot_targets(
    gait: tuple[FootPosition, ...],
    posture: tuple[FootPosition, ...],
) -> tuple[FootPosition, ...]:
    """THE SEAM. gait = absolute positions, posture = offsets. Vector add."""
    return tuple(
        (gx + px, gy + py, gz + pz)
        for (gx, gy, gz), (px, py, pz) in zip(gait, posture)
    )
