"""Final stage -- map internal IK angles to hardware servo angles.

Three things happen here, in order:
  1. Clamp internal angles to the internal joint limits (a backstop; the
     real intent-level limits already live in `pose.py`).
  2. Convert internal radians -> hardware degrees via the per-joint affine
     map, applying the L/R hip mirror.
  3. Return the [hip, femur, tibia] triplet in hardware degrees.

The affine constants live in config and were calibrated so the rest-pose
internal solution maps EXACTLY to the IO layer's [+-45, 78, -147].
"""

from __future__ import annotations
import math

from crawler import config as cfg
from . import vmath
from crawler.config import LegSpec


def to_hardware(internal: tuple[float, float, float], leg: LegSpec) -> list[float]:
    hip, femur, tibia = internal

    # 1. internal-limit backstop (radians)
    hip = vmath.clamp(hip, cfg.HIP_MIN, cfg.HIP_MAX)
    femur = vmath.clamp(femur, cfg.FEMUR_MIN, cfg.FEMUR_MAX)
    tibia = vmath.clamp(tibia, cfg.TIBIA_MIN, cfg.TIBIA_MAX)

    # 2. radians -> degrees, then affine map + hip side mirror
    hip_deg = math.degrees(hip)
    femur_deg = math.degrees(femur)
    tibia_deg = math.degrees(tibia)

    hw_hip = leg.side_sign * (cfg.HIP_SPLAY_OFFSET + hip_deg)
    hw_femur = cfg.FEMUR_HW_SCALE * femur_deg + cfg.FEMUR_HW_OFFSET
    hw_tibia = cfg.TIBIA_HW_SCALE * tibia_deg + cfg.TIBIA_HW_OFFSET

    return [hw_hip, hw_femur, hw_tibia]
