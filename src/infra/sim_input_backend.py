"""Sim input backends: drive device state programmatically for tests and
headless runs. No hardware, no pynput, no X server.

Tests/scripts call hold()/release()/move() to simulate input; the fusion
adapter reads the resulting state exactly as it would from a real backend.
"""

from __future__ import annotations

from picrawler.infra.input_backends import KeyboardState, MouseState


class SimKeyboardBackend:
    def __init__(self) -> None:
        self.state = KeyboardState()

    def start(self) -> None:  # nothing to listen to
        pass

    def stop(self) -> None:
        pass

    # test/script controls
    def hold(self, key: str) -> None:
        self.state.press(key)

    def release(self, key: str) -> None:
        self.state.release(key)


class SimMouseBackend:
    def __init__(self) -> None:
        self.state = MouseState()

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def move(self, dx: float, dy: float) -> None:
        self.state.add_motion(dx, dy)
