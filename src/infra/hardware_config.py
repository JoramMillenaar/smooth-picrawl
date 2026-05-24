"""Physical wiring facts for this specific PiCrawler build.

These are hardware truths — which Robot HAT PWM channel each joint is soldered
to, and which way each servo is mounted — NOT domain logic. They live in infra
because swapping to a differently-wired robot changes only this file.

Ported from the stock Picrawler class:
  PIN_LIST   : 12 channels, leg-major order (leg i -> indices i*3 .. i*3+2),
               each joint as (alpha_ch, beta_ch, gamma_ch).
  DIRECTION  : per-servo rotation sign (+1/-1); left/right legs are mirror
               mounted, so the "same" logical motion needs opposite servo turns.
"""

from __future__ import annotations

# Robot HAT PWM channels, in leg-major (alpha, beta, gamma) order, legs 0..3.
# Stock order: [9,10,11, 3,4,5, 0,1,2, 6,7,8].
PIN_CHANNELS: tuple[int, ...] = (
    9, 10, 11,  # leg 0
    3, 4, 5,  # leg 1
    0, 1, 2,  # leg 2
    6, 7, 8,  # leg 3
)

# Per-servo rotation sign, same order as PIN_CHANNELS.
# Stock: [1,1,-1, 1,1,1, 1,1,-1, 1,1,1].
DIRECTION_SIGN: tuple[int, ...] = (
    1, 1, -1,  # leg 0
    1, 1, 1,  # leg 1
    1, 1, -1,  # leg 2
    1, 1, 1,  # leg 3
)

# SunFounder Servo.angle() accepts -90..90 degrees. The adapter clamps to this
# hardware range as the final guard before commanding, independent of the
# (tighter, per-joint) domain limits.
SERVO_ANGLE_MIN: float = -90.0
SERVO_ANGLE_MAX: float = 90.0

NUM_SERVOS: int = 12

assert len(PIN_CHANNELS) == NUM_SERVOS
assert len(DIRECTION_SIGN) == NUM_SERVOS
