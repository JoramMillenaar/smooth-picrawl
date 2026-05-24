"""Persistence boundary for per-servo zero offsets (the .config file).

Offsets are 12 floats, one per servo, in PIN_LIST order. The set-once persisted
half of calibration; direction-sign lives in the servo adapter as wiring fact.
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable


@runtime_checkable
class CalibrationPort(Protocol):
    def load_offsets(self) -> tuple[float, ...]: ...

    def save_offsets(self, offsets: tuple[float, ...]) -> None: ...
