"""In-memory servo simulation for off-device runs and tests.

Lets the entire stack — control loop, adapter, physicalisation — run on a dev
machine with no Robot HAT. Records the last commanded physical angle per channel
so tests can assert on what would have been sent to hardware.
"""

from __future__ import annotations

import logging

logger = logging.getLogger("picrawler.infra.sim")


class SimServo:
    """Mimics robot_hat.Servo.angle(); stores the last value."""

    def __init__(self, channel: int) -> None:
        self.channel = channel
        self.last_angle: float | None = None

    def angle(self, angle: float) -> None:
        self.last_angle = angle


class SimServoFactory:
    """Builds SimServos and keeps a handle to each for inspection."""

    def __init__(self) -> None:
        self.servos: dict[int, SimServo] = {}

    def create(self, channel: int) -> SimServo:
        servo = SimServo(channel)
        self.servos[channel] = servo
        return servo

    def commanded(self) -> dict[int, float | None]:
        """channel -> last commanded physical angle, for assertions/inspection."""
        return {ch: s.last_angle for ch, s in self.servos.items()}
