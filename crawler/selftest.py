"""Diagnostic harness. Run:  python -m crawler.selftest

Verifies:
  1. The rest pose reproduces the IO layer's hardcoded angles.
  2. The pipeline never emits NaN / out-of-range for swept intents.
  3. Telemetry sanity: gait clock advances under movement, freezes at rest.
"""

from __future__ import annotations
import math

from crawler.controller import CrawlerController
from crawler import config as cfg

# IO layer's hardcoded rest frame, R L BR BL, each [hip, femur, tibia]
IO_REST = [-45, 78, -147, 45, 78, -147, -45, 78, -147, 45, 78, -147]


def _approx(a, b, tol=1.0):
    return abs(a - b) <= tol


def test_rest_pose():
    ctrl = CrawlerController()
    rest = ctrl.rest_angles()
    print("rest_angles():")
    for i, leg in enumerate(cfg.LEGS):
        trip = rest[leg.pin_index : leg.pin_index + 3]
        print(f"  {leg.id:>2}: hip={trip[0]:7.2f}  femur={trip[1]:7.2f}  tibia={trip[2]:7.2f}")
    ok = all(_approx(rest[i], IO_REST[i]) for i in range(12))
    print(f"  matches IO hardcoded rest? {ok}")
    print(f"  expected: {IO_REST}")
    print(f"  got     : {[round(v, 1) for v in rest]}")
    return ok


def test_no_nan_sweep():
    ctrl = CrawlerController()
    bad = 0
    checked = 0
    for pitch_z in [-1.0, -0.3, 0.0, 0.3, 1.0]:
        for mx in [-1.0, 0.0, 1.0]:
            for my in [-1.0, 0.0, 1.0]:
                for wz in [-1.0, 0.0, 1.0]:
                    ctrl.reset()
                    pointing = (1.0, my * 0.5, pitch_z)
                    movement = (mx, my, wz)
                    for _ in range(30):  # ~1 gait cycle worth of ticks
                        ang = ctrl.step(pointing, movement, dt=1 / 30)
                        checked += 1
                        for a in ang:
                            if math.isnan(a) or math.isinf(a) or abs(a) > 360:
                                bad += 1
    print(f"swept {checked} frames; bad outputs: {bad}")
    return bad == 0


def test_clock_behavior():
    ctrl = CrawlerController()
    # standing still: clock should not move
    for _ in range(20):
        ctrl.step((1, 0, 0), (0, 0, 0), dt=1 / 30)
    still_phase = ctrl._clock.phase
    # moving: clock should advance
    for _ in range(20):
        ctrl.step((1, 0, 0), (1, 0, 0), dt=1 / 30)
    moved_phase = ctrl._clock.phase
    ok = _approx(still_phase, 0.0, 1e-6) and moved_phase > 0.0
    print(f"clock still={still_phase:.4f} moved={moved_phase:.4f} ok={ok}")
    return ok


def test_coupled_pitch_limit():
    """Looking up should be limited more when moving fast."""
    from . import pose
    up = (1.0, 0.0, 5.0)  # strongly pitched-up gaze
    standing = pose.compute_body_pose(up, (0, 0, 0))
    running = pose.compute_body_pose(up, (1, 0, 0))
    ok = running.pitch < standing.pitch
    print(f"pitch standing={math.degrees(standing.pitch):.2f} "
          f"running={math.degrees(running.pitch):.2f} (coupled limit) ok={ok}")
    return ok


def main():
    results = {
        "rest pose": test_rest_pose(),
        "no NaN sweep": test_no_nan_sweep(),
        "clock behavior": test_clock_behavior(),
        "coupled pitch limit": test_coupled_pitch_limit(),
    }
    print("\n=== summary ===")
    for k, v in results.items():
        print(f"  {'PASS' if v else 'FAIL'}  {k}")
    return all(results.values())


if __name__ == "__main__":
    import sys
    sys.exit(0 if main() else 1)
