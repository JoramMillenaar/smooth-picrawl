"""Posture: stateless body attitude -> per-leg foot-position OFFSETS (deltas).

These deltas are summed with gait output in foot-space (the seam contract).
Ported from rotate_body_* / move_body_absolute. Body condensed; contract final.
"""

from __future__ import annotations

from src.domain.coordinates import BodyPose, FootPosition


def posture_offsets(pose: BodyPose) -> tuple[FootPosition, ...]:
    """Body pose -> four (dx, dy, dz) offsets, leg order 0..3."""
    # Condensed: real impl composes pitch/roll/yaw/height into per-leg deltas
    # via the rotate_body_absolute_x/y and move_body_absolute geometry.
    h = pose.height
    return tuple((0.0, 0.0, h) for _ in range(4))
