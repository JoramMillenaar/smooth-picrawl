"""The orchestrator.

`CrawlerController` is the single object the IO layer consumes. It owns the
only piece of mutable state (the gait clock) and runs every tick through the
full pipeline:

    pointing ┐
             ├─ pose ─┐
    movement ┘        ├─ gait ─ foot targets ─ to_leg_frame ─ IK ─ to_hardware ─ 12 floats
             └────────┘

Public API:
    ctrl = CrawlerController()
    angles = ctrl.step(pointing, movement, dt)   # -> list[float] length 12
    angles = ctrl.rest_angles()                  # the standing pose, no motion
"""

from __future__ import annotations
import time

from . import config as cfg
from . import pose as pose_stage
from . import gait as gait_stage
from . import kinematics as kin
from . import output as out
from .vmath import Vec3


class CrawlerController:
    def __init__(self) -> None:
        self._clock = gait_stage.GaitClock()
        self._last_t: float | None = None   # for wall-clock dt if caller omits it

    # ------------------------------------------------------------------
    def step(
        self,
        pointing: Vec3,
        movement: Vec3,
        dt: float | None = None,
    ) -> list[float]:
        """Run one control tick. Returns 12 hardware-degree floats in
        leg order R, L, BR, BL, each [hip, femur, tibia]."""
        dt = self._resolve_dt(dt)

        # Stage 3 clock: advance the gait phase first.
        self._clock.advance(movement, dt)

        # Stage 2: body attitude target (intent-level limits applied here).
        body = pose_stage.compute_body_pose(pointing, movement)

        angles: list[float] = [0.0] * 12
        for leg in cfg.LEGS:
            # Stage 3: where this foot wants to be, in the body frame.
            foot_body = gait_stage.foot_target_body(leg, self._clock, movement)
            # Stage 4: express it in the leg's local frame given body attitude.
            foot_local = kin.to_leg_frame(foot_body, leg, body.rotation)
            # Stage 5: analytic IK -> internal angles.
            internal = kin.solve_ik(foot_local)
            # Final: -> hardware degrees, mirrored & limited.
            triplet = out.to_hardware(internal, leg)
            angles[leg.pin_index : leg.pin_index + 3] = triplet

        return angles

    # ------------------------------------------------------------------
    def rest_angles(self) -> list[float]:
        """The standing pose with zero intent. Should reproduce the IO
        layer's hardcoded rest frame. Does NOT advance the clock."""
        body = pose_stage.compute_body_pose((1.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        angles: list[float] = [0.0] * 12
        for leg in cfg.LEGS:
            foot_body = leg.anchor   # neutral, no gait offset
            foot_local = kin.to_leg_frame(foot_body, leg, body.rotation)
            internal = kin.solve_ik(foot_local)
            triplet = out.to_hardware(internal, leg)
            angles[leg.pin_index : leg.pin_index + 3] = triplet
        return angles

    def reset(self) -> None:
        self._clock.reset()
        self._last_t = None

    # ------------------------------------------------------------------
    def _resolve_dt(self, dt: float | None) -> float:
        now = time.monotonic()
        if dt is not None:
            self._last_t = now
            return max(0.0, dt)
        if self._last_t is None:
            self._last_t = now
            return 0.0
        out_dt = now - self._last_t
        self._last_t = now
        # guard against huge first-frame / paused-tab dt spikes
        return min(out_dt, 0.1)
