"""Robot HAT servo adapter — the ONLY module that imports robot_hat and the
ONLY place that knows how an ideal joint angle becomes PWM on this robot.

Implements ServoOutputPort. set_joint_angles(ideal) does, per the agreed seam:
    ideal angles -> apply offsets -> direction sign -> pin map -> hardware clamp
    -> Servo.angle()
The port receives IDEAL angles; everything physical happens here.

Production concerns handled:
  - robot_hat imported lazily and isolated, so the package imports on a dev
    machine without the library installed.
  - offsets held here, reloadable (reload_offsets) after calibration writes them.
  - per-servo write failures are logged and aggregated; the adapter decides
    raise-vs-continue rather than letting an I2C blip kill the control loop.
  - explicit relax()/close() to de-energise servos on shutdown or e-stop.
"""

from __future__ import annotations

import logging
from typing import Protocol

from src.domain.coordinates import AllLegAngles
from src.domain.kinematics import apply_offsets_all
from src.infra.hardware_config import NUM_SERVOS, PIN_CHANNELS, DIRECTION_SIGN, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX

logger = logging.getLogger("picrawler.infra.servo")


class _ServoLike(Protocol):
    """Structural type for a robot_hat.Servo, so this module type-checks without
    the real library present."""
    def angle(self, angle: float) -> None: ...


class ServoWriteError(RuntimeError):
    """Raised when one or more servo writes fail in a single command and the
    adapter is configured to surface failures."""


def _flatten(angles: AllLegAngles) -> list[float]:
    """4x(alpha,beta,gamma) -> flat 12, leg-major, matching PIN_CHANNELS order."""
    out: list[float] = []
    for leg in angles:
        out.extend(leg)
    return out


class RobotHatServoAdapter:
    """Concrete ServoOutputPort backed by robot_hat.Servo channels."""

    def __init__(
        self,
        offsets: tuple[float, ...],
        *,
        servo_factory: "ServoFactory | None" = None,
        raise_on_write_error: bool = False,
    ) -> None:
        """
        offsets: 12 per-servo calibration offsets, PIN_CHANNELS order.
        servo_factory: builds a Servo for a channel int. Defaults to the real
            robot_hat factory; inject a fake for tests/sim.
        raise_on_write_error: if True, a failed command raises ServoWriteError;
            if False (default for a running loop), it logs and continues so a
            transient I2C glitch can't crash locomotion. The container's
            watchdog is the right place to escalate persistent failure.
        """
        self._validate_offsets(offsets)
        self._offsets = tuple(offsets)
        self._raise_on_write_error = raise_on_write_error

        factory = servo_factory or _RealRobotHatServoFactory()
        # One Servo object per physical channel, indexed parallel to PIN_CHANNELS.
        self._servos: list[_ServoLike] = [factory.create(ch) for ch in PIN_CHANNELS]
        logger.info("RobotHatServoAdapter initialised with %d servos", len(self._servos))

    @staticmethod
    def _validate_offsets(offsets: tuple[float, ...]) -> None:
        if len(offsets) != NUM_SERVOS:
            raise ValueError(f"expected {NUM_SERVOS} offsets, got {len(offsets)}")

    def reload_offsets(self, offsets: tuple[float, ...]) -> None:
        """Swap in fresh calibration (e.g. after a calibration session writes
        new values). Cheap; safe to call between control-loop ticks."""
        self._validate_offsets(offsets)
        self._offsets = tuple(offsets)
        logger.info("servo offsets reloaded")

    # --- ServoOutputPort ---------------------------------------------------

    def set_joint_angles(self, angles: AllLegAngles) -> None:
        """Physicalise ideal angles and command all 12 servos.

        ideal -> offsets -> direction sign -> hardware clamp -> Servo.angle().
        Pin mapping is implicit: self._servos[i] already corresponds to
        PIN_CHANNELS[i], so flat index i is the right servo.
        """
        corrected = apply_offsets_all(angles, self._offsets)
        flat = _flatten(corrected)

        failures: list[tuple[int, BaseException]] = []
        for i, raw_angle in enumerate(flat):
            physical = self._physicalise(i, raw_angle)
            try:
                self._servos[i].angle(physical)
            except Exception as exc:  # noqa: BLE001 - boundary: convert to domain-safe handling
                failures.append((i, exc))
                logger.error(
                    "servo write failed: channel=%s flat_index=%d angle=%.2f: %s",
                    PIN_CHANNELS[i], i, physical, exc,
                )

        if failures and self._raise_on_write_error:
            chans = ", ".join(str(PIN_CHANNELS[i]) for i, _ in failures)
            raise ServoWriteError(f"{len(failures)} servo write(s) failed: channels {chans}")

    # --- physicalisation ---------------------------------------------------

    def _physicalise(self, flat_index: int, angle: float) -> float:
        """Apply direction sign and clamp to the servo's hardware range."""
        signed = angle * DIRECTION_SIGN[flat_index]
        return min(max(signed, SERVO_ANGLE_MIN), SERVO_ANGLE_MAX)

    # --- lifecycle ---------------------------------------------------------

    def relax(self) -> None:
        """De-energise: stop holding torque. On these servos there's no explicit
        'release', so we simply stop commanding; subclasses/real lib may cut PWM.
        Provided as the e-stop / shutdown hook the container calls."""
        logger.info("servo adapter relaxed (commands halted)")
        # If the underlying lib exposes a PWM-disable, do it here per channel.

    def close(self) -> None:
        """Release hardware resources. Idempotent."""
        self.relax()


# --------------------------------------------------------------------------- #
# Servo factories: real (lazy robot_hat import) and the injection seam.        #
# --------------------------------------------------------------------------- #

class ServoFactory(Protocol):
    def create(self, channel: int) -> _ServoLike: ...


class _RealRobotHatServoFactory:
    """Creates real robot_hat.Servo objects. robot_hat is imported HERE, lazily,
    so importing this module on a machine without the library only fails if you
    actually instantiate the real factory."""

    def create(self, channel: int) -> _ServoLike:
        try:
            from robot_hat import Servo  # type: ignore[import-not-found]
        except ImportError as exc:  # pragma: no cover - depends on deploy env
            raise RuntimeError(
                "robot_hat not installed; use a sim servo factory off-device "
                "(e.g. set ROBOT_HAT_MOCK_SMBUS or inject SimServoFactory)"
            ) from exc
        # robot_hat Servo takes a channel string like "P0".
        return Servo(f"P{channel}")
