"""Keyboard + mouse fusion adapter — implements InputPort.

Combines a KeyboardBackend (-> velocity + yaw + mode) and a MouseBackend
(-> body-pose heading) into the single device-agnostic Intent the app expects.
This is the ONLY place key bindings and mouse sensitivity live; swapping to a
gamepad later means a new adapter, not changes anywhere upstream.

read_intent() is the poll: it reads the current key snapshot and drains the
mouse motion accumulated since the last tick, then translates to Intent. Raw
(un-smoothed) — the app's input-smoothing use-case does the ramping.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field

from src.app.ports.input import Intent
from src.domain.coordinates import BodyVelocity, BodyPose
from src.infra.input_backends import KeyboardBackend, MouseBackend

logger = logging.getLogger("picrawler.infra.input")


@dataclass(frozen=True)
class KeyMap:
    """Logical key names for each control. Defaults: WASD move, QE yaw,
    space/c/x toggle modes. Override to rebind."""
    forward: str = "w"
    backward: str = "s"
    left: str = "a"
    right: str = "d"
    yaw_left: str = "q"
    yaw_right: str = "e"
    mode_walk: str = "x"
    mode_stand: str = "space"
    mode_sit: str = "c"


@dataclass
class InputConfig:
    keymap: KeyMap = field(default_factory=KeyMap)
    # mouse pixels -> body-pose radians-ish scaling. Tuned conservatively;
    # the app smoothing layer handles ramp, this only sets raw sensitivity.
    mouse_yaw_per_px: float = 0.0015
    mouse_pitch_per_px: float = 0.0015
    # clamp accumulated heading so a fast mouse fling can't peg the pose.
    max_pose_yaw: float = 0.5
    max_pose_pitch: float = 0.4


class KeyboardMouseInputAdapter:
    """InputPort fusing keyboard + mouse backends."""

    def __init__(
        self,
        keyboard_backend: KeyboardBackend,
        mouse_backend: MouseBackend,
        config: InputConfig | None = None,
    ) -> None:
        self._kb = keyboard_backend
        self._mouse = mouse_backend
        self._cfg = config or InputConfig()
        # body-pose heading integrates mouse motion across ticks; held here.
        self._yaw = 0.0
        self._pitch = 0.0
        self._mode = "stand"  # safe default until told otherwise
        self._started = False

    def start(self) -> None:
        self._kb.start()
        self._mouse.start()
        self._started = True
        logger.info("input adapter started")

    def stop(self) -> None:
        self._kb.stop()
        self._mouse.stop()
        self._started = False
        logger.info("input adapter stopped")

    # --- InputPort ---------------------------------------------------------

    def read_intent(self) -> Intent:
        km = self._cfg.keymap
        held = self._kb.state.snapshot()

        # velocity from held keys: opposing keys cancel (e.g. w+s -> 0).
        vx = (km.forward in held) - (km.backward in held)
        vy = (km.right in held) - (km.left in held)
        yaw_rate = (km.yaw_right in held) - (km.yaw_left in held)
        velocity = BodyVelocity(vx=float(vx), vy=float(vy), yaw_rate=float(yaw_rate))

        # mode: latch on press (last-pressed-wins among the three this tick).
        if km.mode_walk in held:
            self._mode = "walk"
        elif km.mode_sit in held:
            self._mode = "sit"
        elif km.mode_stand in held:
            self._mode = "stand"

        # body-pose heading: integrate drained mouse motion, then clamp.
        dx, dy = self._mouse.state.drain()
        self._yaw = _clamp(
            self._yaw + dx * self._cfg.mouse_yaw_per_px, self._cfg.max_pose_yaw
        )
        self._pitch = _clamp(
            self._pitch - dy * self._cfg.mouse_pitch_per_px, self._cfg.max_pose_pitch
        )
        pose = BodyPose(pitch=self._pitch, yaw=self._yaw)

        return (velocity, pose, self._mode)


def _clamp(v: float, limit: float) -> float:
    return min(max(v, -limit), limit)
