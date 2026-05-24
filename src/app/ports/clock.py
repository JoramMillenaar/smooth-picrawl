"""Time boundary. A port so the loop is deterministic under a fake clock in
tests and monotonic + jitter-free in production."""

from __future__ import annotations
from typing import Protocol, runtime_checkable


@runtime_checkable
class ClockPort(Protocol):
    def now(self) -> float:
        """Monotonic seconds."""
        ...

    def sleep_until(self, t: float) -> None:
        """Sleep until monotonic time t (holds the container's tick rate)."""
        ...
