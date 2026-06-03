"""Stages 4 & 5 -- frame composition and analytic inverse kinematics.

Stage 4 (`to_leg_frame`): take a foot target in the BODY frame, apply the
inverse body attitude (so tilting the body shifts all feet correctly), then
express it in the leg's own base frame (translate to the hip, rotate by the
leg's mount yaw).

Stage 5 (`solve_ik`): closed-form 3-DOF IK. Returns INTERNAL-convention
angles in radians. No iterative solver, no NaNs (reachability is clamped).
"""

from __future__ import annotations
import math

from crawler import config as cfg
from . import vmath
from .vmath import Vec3, Mat3
from crawler.config import LegSpec


def to_leg_frame(foot_body: Vec3, leg: LegSpec, body_rot: Mat3) -> Vec3:
    """Express a body-frame foot target in the leg's local base frame."""
    # 1. Undo body attitude: where does this world-ish target sit relative to
    #    a body that is tilted by body_rot? Apply inverse (transpose) rotation.
    p = vmath.mat_vec(vmath.mat_transpose(body_rot), foot_body)
    # 2. Translate so the leg's hip is the origin.
    p = vmath.sub(p, leg.hip_origin)
    # 3. Rotate so the leg's outward (+X) axis is aligned: undo mount yaw.
    p = vmath.rot_z(p, -leg.mount_yaw)
    return p


def solve_ik(p: Vec3) -> tuple[float, float, float]:
    """Analytic 3-DOF IK in the leg base frame.

    Returns (hip, femur, tibia) in INTERNAL radians:
        hip   : yaw about +Z, 0 = pointing along +X
        femur : from horizontal, + = up
        tibia : relative to femur, 0 = straight, - = knee folded under
    """
    x, y, z = p

    # --- hip yaw rotates the leg plane to face the target ---
    hip = math.atan2(y, x)

    # --- reduce to the 2D leg plane (r = horizontal reach, z = vertical) ---
    r = math.hypot(x, y) - cfg.HIP_OFFSET
    d = math.hypot(r, z)

    # --- reachability guard: never feed acos an out-of-range value ---
    d = min(d, cfg.L_FEMUR + cfg.L_TIBIA - 1e-6)

    # --- two pitch joints via law of cosines ---
    cos_knee = (d * d - cfg.L_FEMUR**2 - cfg.L_TIBIA**2) / (2 * cfg.L_FEMUR * cfg.L_TIBIA)
    knee = math.acos(vmath.clamp(cos_knee, -1.0, 1.0))    # interior angle [0, pi]

    alpha = math.atan2(z, r)
    cos_beta = (d * d + cfg.L_FEMUR**2 - cfg.L_TIBIA**2) / (2 * cfg.L_FEMUR * d)
    beta = math.acos(vmath.clamp(cos_beta, -1.0, 1.0))

    # knee-down configuration (foot tucks under the body) matches the rest pose.
    femur = alpha + beta
    tibia = -(math.pi - knee)

    return hip, femur, tibia
