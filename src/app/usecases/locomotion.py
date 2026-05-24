"""Locomotion use-case: the core tick. step(intent, dt) -> 12 angles.

Factory pre-injects deps via closure; the returned `step` takes only pure params.
Owns gait-phase state in the closure. Does NOT loop and does NOT touch servos —
the container feeds it dt and pushes its output to the servo port. The clock is
deliberately NOT a dep: being handed dt keeps step deterministic and testable.
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Callable, TypedDict

from src.app.ports.input import Intent
from src.domain.coordinates import AllLegAngles
from src.domain.gait import gait_foot_targets, compose_foot_targets
from src.domain.kinematics import clamp_angles, inverse_kinematics
from src.domain.posture import posture_offsets
from src.domain.trajectory import slew_limit

Step = Callable[[Intent, float], AllLegAngles]


@dataclass
class _GaitState:
    """Mutable closure state for the stepping loop."""
    phase: float = 0.0
    last_angles: AllLegAngles | None = None


class LocomotionDeps(TypedDict, total=False):
    """Config-only; this use-case is pure computation and needs no ports.
    total=False -> all keys optional, defaults applied below."""
    gait_hz: float  # full gait cycles per second (default 1.5)


def make_locomotion(deps: LocomotionDeps) -> Step:
    gait_hz = float(deps.get("gait_hz", 1.5))

    state = _GaitState()

    def step(intent: Intent, dt: float) -> AllLegAngles:
        velocity, pose, mode = intent

        # 1. advance the phase clock (only while walking)
        if mode == "walk":
            state.phase = (state.phase + gait_hz * dt) % 1.0

        # 2. domain: gait positions + posture offsets
        gait = gait_foot_targets(state.phase, velocity)
        posture = posture_offsets(pose)

        # 3. THE SEAM: sum in foot-space, before IK
        feet = compose_foot_targets(gait, posture)

        # 4. domain: IK once per leg, then clamp to joint limits
        clamped = [clamp_angles(inverse_kinematics(f))[1] for f in feet]

        # 5. domain: slew-limit against last commanded angles
        last = state.last_angles
        if last is None:
            smoothed: AllLegAngles = (clamped[0], clamped[1], clamped[2], clamped[3])
        else:
            sm = [slew_limit(c, l, dt) for c, l in zip(clamped, last)]
            smoothed = (sm[0], sm[1], sm[2], sm[3])
        state.last_angles = smoothed

        return smoothed

    return step
