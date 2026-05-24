"""Calibration use-case: nudge one leg and persist its offsets via ports.

The procedure resolves the gap between two coordinate truths:
  - IDEAL: pure IK. "default stance -> these joint angles."
  - PHYSICAL: the real robot, where servo horns sit at slightly wrong rotations.
The OFFSET per servo bridges them: physical = ideal + offset (domain.apply_offsets).

Calibration discovers the offset by eye: command the default-stance pose (with
current offsets applied), nudge a foot in foot-space until the leg PHYSICALLY
looks correct, then commit -> the new offset is the deviation you dialed in.

Note the division of labour: APPLYING an offset is pure domain math
(apply_offsets); COMPUTING a new offset is calibration's own job and lives here.
This is the one use-case that reasons about offsets explicitly — everything else
just consumes them. Persistence (CalibrationPort) is dumb storage of 12 floats;
the live in-progress pose lives in the closure, not in the port.
"""

from __future__ import annotations

from typing import Callable, TypedDict

from src.app.ports.calibration import CalibrationPort
from src.app.ports.servo_output import ServoOutputPort
from src.domain.constants import DEFAULT_STANCE
from src.domain.coordinates import FootPosition, JointAngles, AllLegAngles
from src.domain.kinematics import (
    inverse_kinematics, clamp_angles, apply_offsets,
)

CalibrateLeg = Callable[[int, str, bool], None]

# direction -> (foot-space axis index, sign). x=0 (left/right), y=1 (up/down),
# z=2 (high/low).
_NUDGE = {
    "up": (1, +1), "down": (1, -1),
    "left": (0, -1), "right": (0, +1),
    "high": (2, +1), "low": (2, -1),
}


class CalibrationDeps(TypedDict, total=False):
    """servo and calib are REQUIRED ports; step_mm is optional config.

    (TypedDict can't mark a subset required while another is optional in one
    declaration cleanly, so callers must supply servo + calib; step_mm defaults.)
    """
    servo: ServoOutputPort
    calib: CalibrationPort
    step_mm: float


def make_calibration(deps: CalibrationDeps) -> CalibrateLeg:
    servo: ServoOutputPort = deps["servo"]
    calib: CalibrationPort = deps["calib"]
    step_mm = float(deps.get("step_mm", 0.2))

    # IDEAL reference angles for the default stance, per leg — what commit
    # measures the achieved pose against. Computed once.
    ideal_stance: list[JointAngles] = [clamp_angles(inverse_kinematics(c))[1] for c in DEFAULT_STANCE]

    # Live, in-progress calibration pose in FOOT-space (closure state, NOT the
    # port). Starts at the default stance for every leg.
    coords: list[list[float]] = [list(c) for c in DEFAULT_STANCE]

    # Current persisted corrections, loaded once (12 floats, PIN_LIST order).
    offsets: list[float] = list(calib.load_offsets())

    def _leg_offsets(leg: int) -> JointAngles:
        o = leg * 3
        return offsets[o], offsets[o + 1], offsets[o + 2]

    def _command_all() -> None:
        """Drive every leg to its working foot position, offset-corrected, so
        the robot shows the PHYSICAL pose while you nudge."""
        out: list[JointAngles] = []
        for leg, c in enumerate(coords):
            foot: FootPosition = (c[0], c[1], c[2])
            out.append(clamp_angles(apply_offsets(inverse_kinematics(foot), _leg_offsets(leg)))[1])
        angles: AllLegAngles = (out[0], out[1], out[2], out[3])
        servo.set_joint_angles(angles)

    def calibrate_leg(leg: int, direction: str, commit: bool) -> None:
        # 1. nudge this leg in foot-space and drive the servos there.
        axis, sign = _NUDGE[direction]
        coords[leg][axis] += step_mm * sign
        _command_all()

        if not commit:
            return

        # 2. COMMIT: the leg now PHYSICALLY looks correct. The new offset is the
        #    deviation between the IDEAL angles for where the foot now sits and
        #    the IDEAL stance reference, folded into the existing offset.
        foot: FootPosition = (coords[leg][0], coords[leg][1], coords[leg][2])
        achieved = clamp_angles(inverse_kinematics(foot))[1]
        ref = ideal_stance[leg]
        o = leg * 3
        for j in range(3):
            offsets[o + j] = (achieved[j] - ref[j]) + offsets[o + j]

        # 3. persist all 12 (only this leg's 3 changed) and reset its working
        #    coord so the next leg starts clean.
        calib.save_offsets(tuple(offsets))
        coords[leg] = list(DEFAULT_STANCE[leg])

    return calibrate_leg
