"""Stage 2 -- body pose target.

Turns the `pointing` intent vector into a desired body attitude
(roll, pitch, yaw), applies the intent-level limits HERE (not in joint
space), including the coupled limit where moving fast reduces how far the
robot can look up.

The output is a rotation matrix the gait/IK stages apply to foot targets.
"""

from __future__ import annotations
import math
from dataclasses import dataclass

from crawler import config as cfg
from . import vmath
from .vmath import Vec3, Mat3


@dataclass
class BodyPose:
    rotation: Mat3
    pitch: float    # kept for telemetry/debugging
    yaw: float
    roll: float


def _pitch_yaw_from_pointing(pointing: Vec3) -> tuple[float, float]:
    """Interpret `pointing` as a gaze direction in the body frame.

    +X is forward. Pitch is elevation above the XY plane; yaw is heading
    in the XY plane. A zero/!tiny vector means "look straight ahead".
    """
    x, y, z = pointing
    horiz = math.hypot(x, y)
    if horiz < 1e-9 and abs(z) < 1e-9:
        return 0.0, 0.0
    pitch = math.atan2(z, horiz)        # +z -> look up
    yaw = math.atan2(y, x)              # +y -> look left
    return pitch, yaw


def compute_body_pose(pointing: Vec3, movement: Vec3) -> BodyPose:
    """Build the body attitude target with all intent-level limits applied."""
    pitch, yaw = _pitch_yaw_from_pointing(pointing)

    # --- COUPLED LIMIT: speed eats into the available look-up range ---
    speed = vmath.clamp(vmath.magnitude(movement) / cfg.SPEED_MAX, 0.0, 1.0)
    pitch_up_max = cfg.PITCH_STATIC_MAX * (1.0 - cfg.PITCH_COUPLE_K * speed)
    pitch = vmath.clamp(pitch, cfg.PITCH_STATIC_MIN, pitch_up_max)

    # --- yaw gaze limit (independent of speed here; couple it if you want) ---
    yaw = vmath.clamp(yaw, -cfg.YAW_MAX, cfg.YAW_MAX)

    roll = 0.0  # extend later: bank into turns from movement yaw-rate.

    rot = vmath.euler_to_mat(roll, pitch, yaw)
    return BodyPose(rotation=rot, pitch=pitch, yaw=yaw, roll=roll)
