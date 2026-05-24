"""pynput-backed keyboard and mouse backends.

pynput is imported LAZILY here and nowhere else, so the package imports on a
machine without it (or without an X server). Instantiating these backends is
what pulls pynput in; off-device code uses the sim backend instead.

PLATFORM NOTE: pynput on Linux requires a running X server with $DISPLAY set and
will not work over a bare SSH session. For a headless Pi, either run an X
session, or supply a different backend (e.g. an evdev-based one) behind the same
KeyboardBackend/MouseBackend protocol.
"""

from __future__ import annotations

import logging
from typing import Any

from src.infra.input_backends import KeyboardState, MouseState

logger = logging.getLogger("picrawler.infra.input.pynput")


def _key_to_str(key: Any) -> str | None:
    """Normalise a pynput key event to a lowercase logical name.
    Letters -> the char; arrows/space -> a stable name; else None (ignored)."""
    # Lazy import so module import doesn't require pynput.
    from pynput import keyboard  # type: ignore[import-not-found]

    if isinstance(key, keyboard.KeyCode) and key.char is not None:
        return key.char.lower()
    if isinstance(key, keyboard.Key):
        mapping = {
            keyboard.Key.up: "up",
            keyboard.Key.down: "down",
            keyboard.Key.left: "left",
            keyboard.Key.right: "right",
            keyboard.Key.space: "space",
            keyboard.Key.esc: "esc",
        }
        return mapping.get(key)
    return None


class PynputKeyboardBackend:
    """KeyboardBackend using pynput's listener thread."""

    def __init__(self) -> None:
        self.state = KeyboardState()
        self._listener: Any | None = None

    def start(self) -> None:
        from pynput import keyboard  # lazy

        def on_press(key: Any) -> None:
            name = _key_to_str(key)
            if name:
                self.state.press(name)

        def on_release(key: Any) -> None:
            name = _key_to_str(key)
            if name:
                self.state.release(name)

        self._listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        self._listener.start()
        logger.info("pynput keyboard listener started")

    def stop(self) -> None:
        if self._listener is not None:
            self._listener.stop()
            self._listener = None
            logger.info("pynput keyboard listener stopped")


class PynputMouseBackend:
    """MouseBackend using pynput's mouse listener; accumulates motion deltas."""

    def __init__(self) -> None:
        self.state = MouseState()
        self._listener: Any | None = None
        self._last: tuple[int, int] | None = None

    def start(self) -> None:
        from pynput import mouse  # lazy

        def on_move(x: int, y: int) -> None:
            if self._last is not None:
                self.state.add_motion(x - self._last[0], y - self._last[1])
            self._last = (x, y)

        self._listener = mouse.Listener(on_move=on_move)
        self._listener.start()
        logger.info("pynput mouse listener started")

    def stop(self) -> None:
        if self._listener is not None:
            self._listener.stop()
            self._listener = None
            logger.info("pynput mouse listener stopped")
