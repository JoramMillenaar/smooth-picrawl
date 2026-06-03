"""Static configuration: robot geometry, joint limits, per-leg calibration,
and gait tuning. Everything here is a hand-set constant you calibrate once.

IMPORTANT CONVENTIONS
---------------------
Leg order matches the IO layer's hardcoded frame:  R, L, BR, BL
    R  = front-right    L  = front-left
    BR = back-right     BL = back-left
The 12-float output is concatenated in that order, each leg [hip, femur, tibia].

Two angle worlds exist and are kept strictly separate:
  * INTERNAL angles  -- clean radians used by the IK/FK math.
                        femur 0 = horizontal, + = up.
                        tibia 0 = collinear with femur, - = knee folded.
  * HARDWARE angles  -- degrees the servos actually want, matching the IO
                        layer's rest pose [+-45, 78, -147].
The conversion is a per-joint affine map (scale, offset) plus a hip side-sign.
It was solved numerically so that the internal rest solution maps EXACTLY to
the IO layer's hardcoded standing pose. See `calibrate.py` to re-derive it.
"""

from __future__ import annotations
from dataclasses import dataclass, field
import math

# ----------------------------------------------------------------------
# Link geometry (millimetres; only ratios matter for angles)
# ----------------------------------------------------------------------
L_FEMUR = 60.0
L_TIBIA = 80.0
HIP_OFFSET = 15.0          # lateral gap: hip yaw axis -> femur root

# ----------------------------------------------------------------------
# Rest pose, in the leg's local frame (foot position relative to hip).
# This is the standing foot target that reproduces the IO rest angles.
# outboard ~56mm, slightly below the hip.
# ----------------------------------------------------------------------
REST_REACH = 56.144        # horizontal distance hip->foot at rest
REST_DOWN = -16.0          # vertical foot offset at rest (foot below hip)

# ----------------------------------------------------------------------
# Internal joint limits (radians). These bound the IK math.
# Generous defaults; tighten to your servos' real travel.
# ----------------------------------------------------------------------
HIP_MIN, HIP_MAX = math.radians(-60), math.radians(60)
FEMUR_MIN, FEMUR_MAX = math.radians(20), math.radians(110)
TIBIA_MIN, TIBIA_MAX = math.radians(-150), math.radians(-5)

# ----------------------------------------------------------------------
# Hardware mapping per joint:  hw_deg = scale * internal_deg + offset
# Derived in calibrate.py so the rest solution -> [45, 78, -147].
# femur offset is ~0, tibia offset is the big convention constant.
# ----------------------------------------------------------------------
# Hip: hardware = side_sign * (internal_deg + HIP_SPLAY_OFFSET).
# At rest the leg points straight out along its mount direction (internal 0);
# the physical 45deg splay is this offset, mirrored per side.
HIP_SPLAY_OFFSET = 45.0
FEMUR_HW_SCALE, FEMUR_HW_OFFSET = 1.0, 0.0          # internal femur ~= hw femur
TIBIA_HW_SCALE, TIBIA_HW_OFFSET = 1.0, -114.0       # internal -33 -> hw -147

# ----------------------------------------------------------------------
# Body-pose limits (radians) and the speed-coupling for "look up".
# ----------------------------------------------------------------------
PITCH_STATIC_MAX = math.radians(30)    # max look-up when standing still
PITCH_STATIC_MIN = math.radians(-30)   # max look-down
PITCH_COUPLE_K = 0.7                    # fraction of up-range eaten at full speed
YAW_MAX = math.radians(35)              # body gaze yaw range
SPEED_MAX = 1.0                         # movement-vector magnitude treated as "full"

# ----------------------------------------------------------------------
# Gait tuning
# ----------------------------------------------------------------------
STRIDE_LENGTH = 40.0       # peak fore/aft foot travel during stance (mm)
LIFT_HEIGHT = 25.0         # swing arc apex (mm)
DUTY_FACTOR = 0.75         # fraction of cycle in stance (0.75 = crawl, 3 feet down)
CYCLE_FREQ_MAX = 1.6       # gait cycles/sec at full speed (Hz)
TURN_GAIN = 35.0           # mm of stride added per unit yaw-rate at the body edge

# Phase offsets within the [0,1) cycle. Crawl gait: one foot swings at a time.
# Order keyed by leg id. A diagonal-ish sequence keeps the support polygon stable.
PHASE_OFFSET = {
    "R": 0.0,
    "BL": 0.25,
    "L": 0.5,
    "BR": 0.75,
}


@dataclass(frozen=True)
class LegSpec:
    """Everything per-leg the pipeline needs.

    anchor:     neutral foot position in BODY frame (mm). Where the foot rests.
    hip_origin: position of the hip yaw axis in BODY frame (mm).
    mount_yaw:  direction (rad) the leg's local +X (outward) points in body frame.
    side_sign:  +1 / -1 applied to the hip hardware angle for L/R mirroring.
    pin_index:  starting index of this leg's triplet in the 12-float output.
    """
    id: str
    anchor: tuple[float, float, float]
    hip_origin: tuple[float, float, float]
    mount_yaw: float
    side_sign: float
    pin_index: int


# Body is roughly square. Hips sit at the four corners. +X forward, +Y left, +Z up.
# Each leg MOUNTS pointing diagonally outward (mount_yaw = its rest direction),
# so at rest the foot sits straight out along the leg's local +X => internal hip 0.
# The visible 45deg splay is the HIP_SPLAY_OFFSET applied in output.py.
_HIP_X = 45.0
_HIP_Y = 40.0


LEGS: list[LegSpec] = [
    # FRONT-RIGHT: mounts toward forward-right (-45 deg)
    LegSpec(
        id="R",
        hip_origin=(_HIP_X, -_HIP_Y, 0.0),
        anchor=(_HIP_X + REST_REACH * math.cos(math.radians(-45)),
                -_HIP_Y + REST_REACH * math.sin(math.radians(-45)),
                REST_DOWN),
        mount_yaw=math.radians(-45),
        side_sign=-1.0,
        pin_index=0,
    ),
    # FRONT-LEFT: mounts toward forward-left (+45 deg)
    LegSpec(
        id="L",
        hip_origin=(_HIP_X, +_HIP_Y, 0.0),
        anchor=(_HIP_X + REST_REACH * math.cos(math.radians(45)),
                +_HIP_Y + REST_REACH * math.sin(math.radians(45)),
                REST_DOWN),
        mount_yaw=math.radians(+45),
        side_sign=+1.0,
        pin_index=3,
    ),
    # BACK-RIGHT: mounts toward back-right (-135 deg)
    LegSpec(
        id="BR",
        hip_origin=(-_HIP_X, -_HIP_Y, 0.0),
        anchor=(-_HIP_X + REST_REACH * math.cos(math.radians(-135)),
                -_HIP_Y + REST_REACH * math.sin(math.radians(-135)),
                REST_DOWN),
        mount_yaw=math.radians(-135),
        side_sign=-1.0,
        pin_index=6,
    ),
    # BACK-LEFT: mounts toward back-left (+135 deg)
    LegSpec(
        id="BL",
        hip_origin=(-_HIP_X, +_HIP_Y, 0.0),
        anchor=(-_HIP_X + REST_REACH * math.cos(math.radians(135)),
                +_HIP_Y + REST_REACH * math.sin(math.radians(135)),
                REST_DOWN),
        mount_yaw=math.radians(+135),
        side_sign=+1.0,
        pin_index=9,
    ),
]
