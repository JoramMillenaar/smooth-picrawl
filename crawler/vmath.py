"""Minimal 3D vector + rotation helpers. Pure stdlib, no numpy, so the
pipeline drops into any environment the websocket host already runs in.

Vectors are plain (x, y, z) tuples. Rotations are 3x3 row-major tuples.
"""

from __future__ import annotations
import math

Vec3 = tuple[float, float, float]
Mat3 = tuple[Vec3, Vec3, Vec3]


def add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(a: Vec3, s: float) -> Vec3:
    return (a[0] * s, a[1] * s, a[2] * s)


def magnitude(a: Vec3) -> float:
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def normalize(a: Vec3) -> Vec3:
    m = magnitude(a)
    if m < 1e-12:
        return (0.0, 0.0, 0.0)
    return (a[0] / m, a[1] / m, a[2] / m)


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def rot_z(v: Vec3, theta: float) -> Vec3:
    """Rotate a vector about the Z axis by theta radians."""
    c, s = math.cos(theta), math.sin(theta)
    return (c * v[0] - s * v[1], s * v[0] + c * v[1], v[2])


def euler_to_mat(roll: float, pitch: float, yaw: float) -> Mat3:
    """Intrinsic Z(yaw) * Y(pitch) * X(roll) rotation matrix.
    +pitch tips the nose up, +yaw turns left, +roll banks right-down.
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # R = Rz * Ry * Rx
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp,     cp * sr,                cp * cr),
    )


def mat_vec(m: Mat3, v: Vec3) -> Vec3:
    return (
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    )


def mat_transpose(m: Mat3) -> Mat3:
    """For a rotation matrix the transpose is the inverse."""
    return (
        (m[0][0], m[1][0], m[2][0]),
        (m[0][1], m[1][1], m[2][1]),
        (m[0][2], m[1][2], m[2][2]),
    )
