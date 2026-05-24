"""Input backend seam.

The InputPort is poll-based (the 50 Hz loop asks once per tick), but real
keyboards/mice are event-driven (keys go down and stay down). These backends
bridge that: a background listener updates a thread-safe snapshot of current
device state, which the adapter reads synchronously when polled.

Backends are swappable because the device-reading MECHANISM is environment
specific — pynput needs an X server (won't work headless/SSH), so a Pi running
Pi OS Lite needs a different backend or the sim. Keeping this a Protocol lets
the fusion adapter stay identical across all of them.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Protocol, runtime_checkable


@dataclass
class KeyboardState:
    """Thread-safe snapshot of which logical keys are currently held."""
    _held: set[str] = field(default_factory=set)
    _lock: threading.Lock = field(default_factory=threading.Lock)

    def press(self, key: str) -> None:
        with self._lock:
            self._held.add(key)

    def release(self, key: str) -> None:
        with self._lock:
            self._held.discard(key)

    def is_held(self, key: str) -> bool:
        with self._lock:
            return key in self._held

    def snapshot(self) -> frozenset[str]:
        with self._lock:
            return frozenset(self._held)


@dataclass
class MouseState:
    """Thread-safe snapshot of accumulated mouse motion since last drain.

    Heading control is naturally relative (mouse deltas), so we accumulate
    dx/dy and let the adapter drain-and-integrate each tick."""
    _dx: float = 0.0
    _dy: float = 0.0
    _lock: threading.Lock = field(default_factory=threading.Lock)

    def add_motion(self, dx: float, dy: float) -> None:
        with self._lock:
            self._dx += dx
            self._dy += dy

    def drain(self) -> tuple[float, float]:
        """Return accumulated (dx, dy) and reset to zero. Called once per tick."""
        with self._lock:
            dx, dy = self._dx, self._dy
            self._dx = 0.0
            self._dy = 0.0
            return dx, dy


@runtime_checkable
class KeyboardBackend(Protocol):
    """A device reader that drives a KeyboardState. start() begins listening;
    stop() halts and releases resources."""
    state: KeyboardState
    def start(self) -> None: ...
    def stop(self) -> None: ...


@runtime_checkable
class MouseBackend(Protocol):
    state: MouseState
    def start(self) -> None: ...
    def stop(self) -> None: ...
