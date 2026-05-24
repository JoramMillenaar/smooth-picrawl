"""File-backed calibration store — implements CalibrationPort.

Persists the 12 per-servo offsets to a JSON file. Production concerns:
  - ATOMIC writes: write to a temp file in the same dir, fsync, then os.replace
    (atomic on POSIX). A crash mid-write can never leave a half-written, corrupt
    calibration that would make the robot lurch on next boot.
  - VALIDATION on load: wrong length / non-numeric / unparseable -> fall back to
    a safe default (all zeros) and log loudly, rather than crashing or
    propagating garbage into servo commands.
  - the parent directory is created if missing.

Format: {"version": 1, "offsets": [f0, ..., f11]}
"""

from __future__ import annotations

import json
import logging
import os
import tempfile
from pathlib import Path

logger = logging.getLogger("picrawler.infra.calibration")

_NUM_OFFSETS = 12
_SCHEMA_VERSION = 1
_SAFE_DEFAULT: tuple[float, ...] = tuple(0.0 for _ in range(_NUM_OFFSETS))


class FileCalibrationAdapter:
    """CalibrationPort over a JSON file at `path`."""

    def __init__(self, path: str | os.PathLike[str]) -> None:
        self._path = Path(path)

    def load_offsets(self) -> tuple[float, ...]:
        """Return persisted offsets, or the safe default if the file is missing,
        unreadable, or fails validation. Never raises for bad data."""
        if not self._path.exists():
            logger.warning(
                "calibration file %s missing; using zero offsets", self._path
            )
            return _SAFE_DEFAULT
        try:
            raw = self._path.read_text(encoding="utf-8")
            data = json.loads(raw)
            offsets = self._parse_and_validate(data)
            logger.info("loaded calibration from %s", self._path)
            return offsets
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            logger.error(
                "calibration file %s unreadable/invalid (%s); using zero offsets",
                self._path, exc,
            )
            return _SAFE_DEFAULT

    def save_offsets(self, offsets: tuple[float, ...]) -> None:
        """Validate then atomically persist. Raises ValueError on bad input
        (a programming error worth surfacing), but never leaves a partial file."""
        self._validate(offsets)
        self._path.parent.mkdir(parents=True, exist_ok=True)

        payload = json.dumps(
            {"version": _SCHEMA_VERSION, "offsets": list(offsets)},
            indent=2,
        )
        # temp file in the SAME directory so os.replace is a same-filesystem
        # atomic rename.
        fd, tmp_name = tempfile.mkstemp(
            dir=str(self._path.parent), prefix=".calib-", suffix=".tmp"
        )
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                f.write(payload)
                f.flush()
                os.fsync(f.fileno())  # durability before the rename
            os.replace(tmp_name, self._path)  # atomic swap
            logger.info("saved calibration to %s", self._path)
        except BaseException:
            # clean up the temp file on any failure
            try:
                os.unlink(tmp_name)
            except OSError:
                pass
            raise

    # --- validation --------------------------------------------------------

    @staticmethod
    def _validate(offsets: tuple[float, ...]) -> None:
        if len(offsets) != _NUM_OFFSETS:
            raise ValueError(f"expected {_NUM_OFFSETS} offsets, got {len(offsets)}")
        for i, v in enumerate(offsets):
            if not isinstance(v, (int, float)) or isinstance(v, bool):
                raise ValueError(f"offset[{i}] is not numeric: {v!r}")

    @classmethod
    def _parse_and_validate(cls, data: object) -> tuple[float, ...]:
        if not isinstance(data, dict) or "offsets" not in data:
            raise ValueError("missing 'offsets' field")
        raw = data["offsets"]
        if not isinstance(raw, list):
            raise ValueError("'offsets' is not a list")
        offsets = tuple(float(v) for v in raw)
        cls._validate(offsets)
        return offsets
