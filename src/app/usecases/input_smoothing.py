"""Input-smoothing use-case: raw (jerky) intent -> smoothly-varying intent.

update(raw_intent, dt) -> smoothed_intent. Slew-rate limiter on the velocity
vector; pose/mode pass through. Keyboard is binary, so this is what ramps
0 -> full smoothly instead of snapping. Filtered value held in the closure.
"""

from __future__ import annotations

from typing import Callable, TypedDict

from src.app.ports.input import Intent
from src.domain.coordinates import BodyVelocity

Update = Callable[[Intent, float], Intent]


class SmoothingDeps(TypedDict, total=False):
    """Config-only. total=False -> optional with defaults below."""
    ramp_per_s: float  # how fast targets approach raw input (default 4.0)


def make_input_smoothing(deps: SmoothingDeps) -> Update:
    ramp = float(deps.get("ramp_per_s", 4.0))
    state = {"v": BodyVelocity()}

    def _approach(cur: float, tgt: float, max_delta: float) -> float:
        return cur + max(-max_delta, min(max_delta, tgt - cur))

    def update(raw: Intent, dt: float) -> Intent:
        raw_v, pose, mode = raw
        cur = state["v"]
        md = ramp * dt
        nv = BodyVelocity(
            vx=_approach(cur.vx, raw_v.vx, md),
            vy=_approach(cur.vy, raw_v.vy, md),
            yaw_rate=_approach(cur.yaw_rate, raw_v.yaw_rate, md),
        )
        state["v"] = nv
        return nv, pose, mode  # pose/mode pass through un-smoothed

    return update
