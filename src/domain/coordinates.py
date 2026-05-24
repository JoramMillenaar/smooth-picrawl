"""Value objects: the vocabulary every layer speaks in.

Pure data, no behavior beyond construction. Foot positions are (x, y, z) in mm
(z negative = down); joint angles are (alpha, beta, gamma) in degrees.
"""

from __future__ import annotations
from dataclasses import dataclass

FootPosition = tuple[float, float, float]
JointAngles = tuple[float, float, float]

# All-leg quantities are 4-tuples, leg order 0..3.
AllLegFeet = tuple[FootPosition, FootPosition, FootPosition, FootPosition]
AllLegAngles = tuple[JointAngles, JointAngles, JointAngles, JointAngles]


@dataclass(frozen=True)
class BodyVelocity:
    """Desired body motion. vx/vy in -1..1 (forward/right), yaw_rate in -1..1."""
    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0


@dataclass(frozen=True)
class BodyPose:
    """Desired body attitude relative to the planted feet."""
    pitch: float = 0.0
    roll: float = 0.0
    yaw: float = 0.0
    height: float = 0.0
