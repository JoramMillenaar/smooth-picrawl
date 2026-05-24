"""Actuation boundary."""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from src.domain.coordinates import AllLegAngles


@runtime_checkable
class ServoOutputPort(Protocol):
    """The ONLY thing the app knows about moving servos.

    Implementations apply pin mapping + direction sign + calibration offset and
    write PWM. The app hands it logical joint angles, leg order 0..3.
    """

    def set_joint_angles(self, angles: AllLegAngles) -> None: ...
