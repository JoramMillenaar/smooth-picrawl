"""Trajectory shaping: keep motion within what the servo can physically deliver.

slew_limit caps per-joint angular velocity below the servo ceiling so a large
setpoint jump can't command a speed the hardware can't hit (which would lose the
smooth path). Pure: state is passed in, not held here.
"""

from __future__ import annotations

from src.domain.constants import MAX_JOINT_DEG_PER_S
from src.domain.coordinates import JointAngles


def slew_limit(
    target: JointAngles,
    last: JointAngles | None,
    dt: float,
) -> JointAngles:
    """Cap each joint's change to MAX_JOINT_DEG_PER_S * dt. last=None -> passthrough."""
    if last is None:
        return target
    max_step = MAX_JOINT_DEG_PER_S * dt
    out = []
    for t, l in zip(target, last):
        d = max(-max_step, min(max_step, t - l))
        out.append(l + d)
    return out[0], out[1], out[2]
