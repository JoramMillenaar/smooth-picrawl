"""Re-derive the internal->hardware affine map.

Run this if you change link lengths or the rest pose. It solves for the
per-joint offset that makes the rest-pose internal IK solution map exactly
to the IO layer's hardcoded standing angles, then prints the constants to
paste into config.py.

The IO layer's rest frame (R, L, BR, BL), each [hip, femur, tibia]:
    [-45, 78, -147,  45, 78, -147,  -45, 78, -147,  45, 78, -147]
    (hip mirrored by side; femur/tibia identical across legs)
"""

from __future__ import annotations
import math

from . import config as cfg
from . import kinematics as kin

# Target hardware rest angles (one representative L leg; hip handled by side_sign)
HW_REST_HIP = 45.0
HW_REST_FEMUR = 78.0
HW_REST_TIBIA = -147.0


def main() -> None:
    # Rest foot for the L leg, in the leg's LOCAL frame: outboard + slightly down.
    # mount_yaw cancels in the local frame, so we just place it along +X.
    rest_local = (cfg.REST_REACH, 0.0, cfg.REST_DOWN)
    hip_i, femur_i, tibia_i = kin.solve_ik(rest_local)

    hip_deg = math.degrees(hip_i)        # ~0 in local frame (we placed along +X)
    femur_deg = math.degrees(femur_i)
    tibia_deg = math.degrees(tibia_i)

    print("Internal rest solution (deg):")
    print(f"  hip   = {hip_deg:8.4f}")
    print(f"  femur = {femur_deg:8.4f}")
    print(f"  tibia = {tibia_deg:8.4f}")
    print()
    print("Affine offsets (scale assumed 1.0) to paste into config.py:")
    # Note: in the local frame the foot is along +X so internal hip ~ 0; the
    # real per-leg hip offset comes from mount_yaw at runtime, and the +-45
    # comes out of the geometry. The hip affine stays identity.
    print(f"  FEMUR_HW_OFFSET = {HW_REST_FEMUR - femur_deg:.4f}")
    print(f"  TIBIA_HW_OFFSET = {HW_REST_TIBIA - tibia_deg:.4f}")


if __name__ == "__main__":
    main()
