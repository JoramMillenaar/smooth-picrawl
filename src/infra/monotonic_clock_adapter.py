"""Monotonic clock adapter — implements ClockPort.

Production loop timing wants two things stock time.sleep can't give alone:
  - MONOTONIC time, immune to wall-clock adjustments (NTP steps, DST).
  - accurate wake-up at a target instant. time.sleep under-/over-shoots by
    OS-scheduler granularity (often 1-15 ms), which shows up as gait jitter.
    We sleep for most of the interval, then busy-wait the final sub-ms tail.

sleep_until returns nothing but logs when it wakes LATE (couldn't keep the tick),
which is the early-warning signal that the loop is overrunning its budget.
"""

from __future__ import annotations

import logging
import time

logger = logging.getLogger("picrawler.infra.clock")

# Below this many seconds remaining, busy-wait instead of sleeping, to beat
# scheduler granularity. 1 ms is a safe tail on a Pi.
_BUSY_WAIT_TAIL_S = 0.001


class MonotonicClockAdapter:
    """ClockPort backed by time.monotonic with accurate sleep_until."""

    def __init__(self, *, warn_late_after_s: float = 0.002) -> None:
        """warn_late_after_s: log a warning if we wake this far past target."""
        self._warn_late_after_s = warn_late_after_s

    def now(self) -> float:
        return time.monotonic()

    def sleep_until(self, t: float) -> None:
        """Block until monotonic time >= t. If already past t, return at once
        (and warn — that means the previous tick overran)."""
        remaining = t - time.monotonic()
        if remaining <= 0:
            lateness = -remaining
            if lateness > self._warn_late_after_s:
                logger.warning(
                    "loop overrun: woke %.2f ms past tick target",
                    lateness * 1000.0,
                )
            return

        # coarse sleep for the bulk, leaving the busy-wait tail
        coarse = remaining - _BUSY_WAIT_TAIL_S
        if coarse > 0:
            time.sleep(coarse)

        # busy-wait the remainder for tight accuracy
        while time.monotonic() < t:
            pass
