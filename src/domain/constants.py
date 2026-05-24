"""Physical truths about the leg and body. Single source for the magic numbers
ported from the stock code, so nothing else hardcodes them."""

from __future__ import annotations

from src.domain.coordinates import FootPosition

# Link lengths (mm), from the stock code: A=48, B=78, C=33.
LEG_A = 48.0
LEG_B = 78.0
LEG_C = 33.0

# Joint angle clamps (deg), from limit_angle in the stock code.
ALPHA_LIMITS = (-90.0, 90.0)
BETA_LIMITS = (-10.0, 90.0)
GAMMA_LIMITS = (-60.0, 60.0)

# Max angular velocity ceiling (deg/s), set below the ~400 deg/s SF006 spec
# with load margin. Trajectory shaping must respect this.
MAX_JOINT_DEG_PER_S = 300.0

# Neutral resting foot position per leg.
DEFAULT_STANCE: tuple[FootPosition, ...] = (
    (60.0, 0.0, -30.0), (60.0, 0.0, -30.0),
    (60.0, 0.0, -30.0), (60.0, 0.0, -30.0),
)
