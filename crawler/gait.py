"""Stage 3 -- gait engine.

Owns the cyclic gait clock and produces, for each leg, a target foot
position in the BODY frame given the current movement intent.

The clock is the only mutable state in the whole pipeline. It freezes when
speed is zero, so the robot cleanly stands still instead of marching in place.
"""

from __future__ import annotations
import math

from . import config as cfg
from . import vmath
from .vmath import Vec3
from .config import LegSpec


class GaitClock:
    """Advances a normalized phase in [0, 1) at a speed-dependent frequency."""

    def __init__(self) -> None:
        self.phase = 0.0
        self._speed = 0.0   # last commanded normalized speed (for telemetry)

    def advance(self, movement: Vec3, dt: float) -> None:
        speed = vmath.clamp(vmath.magnitude(movement) / cfg.SPEED_MAX, 0.0, 1.0)
        self._speed = speed
        freq = speed * cfg.CYCLE_FREQ_MAX
        self.phase = (self.phase + freq * dt) % 1.0

    def reset(self) -> None:
        self.phase = 0.0


def _stride_vector_for_leg(leg: LegSpec, movement: Vec3) -> Vec3:
    """Per-leg ground-plane stride direction & length in the BODY frame.

    Combines translation (everyone strides the same way) with a turning
    component (yaw-rate rotates each foot about the body center, so outer
    legs naturally take longer strides for arc turns / spins).
    """
    speed = vmath.clamp(vmath.magnitude(movement) / cfg.SPEED_MAX, 0.0, 1.0)

    # translation component: horizontal travel direction scaled by speed
    horiz = (movement[0], movement[1], 0.0)
    trans = vmath.scale(vmath.normalize(horiz), cfg.STRIDE_LENGTH * speed)

    # turning component: movement[2] doubles as yaw-rate command (wz).
    # tangential direction at the leg's anchor = perp of the anchor's XY.
    wz = movement[2]
    ax, ay, _ = leg.anchor
    tangent = vmath.normalize((-ay, ax, 0.0))   # 90deg CCW perp
    turn = vmath.scale(tangent, wz * cfg.TURN_GAIN)

    return vmath.add(trans, turn)


def foot_target_body(leg: LegSpec, clock: GaitClock, movement: Vec3) -> Vec3:
    """Target foot position for one leg, in the BODY frame, this tick."""
    phi = (clock.phase + cfg.PHASE_OFFSET[leg.id]) % 1.0
    stride = _stride_vector_for_leg(leg, movement)

    if phi < cfg.DUTY_FACTOR:
        # ---- STANCE: foot planted, slides backward relative to body ----
        s = phi / cfg.DUTY_FACTOR                  # 0..1 through stance
        ground = vmath.scale(stride, 0.5 - s)      # front -> back
        lift = 0.0
    else:
        # ---- SWING: foot lifts and arcs forward to next stance start ----
        s = (phi - cfg.DUTY_FACTOR) / (1.0 - cfg.DUTY_FACTOR)
        ground = vmath.scale(stride, -0.5 + s)     # back -> front
        lift = cfg.LIFT_HEIGHT * math.sin(math.pi * s)

    gx, gy, _ = ground
    ax, ay, az = leg.anchor
    return (ax + gx, ay + gy, az + lift)
