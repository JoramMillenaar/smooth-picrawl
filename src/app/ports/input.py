"""Operator-intent boundary."""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from src.domain.coordinates import BodyVelocity, BodyPose

# (velocity, pose, mode). mode is a gait/command selector: "walk"/"stand"/"sit".
Intent = tuple[BodyVelocity, BodyPose, str]


@runtime_checkable
class InputPort(Protocol):
    """Hides keyboard/gamepad/network. Returns RAW (un-smoothed) intent;
    smoothing is an app use-case, not the adapter's job."""

    def read_intent(self) -> Intent: ...
