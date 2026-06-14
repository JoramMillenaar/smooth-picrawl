#!/usr/bin/env python3
"""
Smooth keyboard control for PiCrawler.

Why this exists
---------------
keyboard_control.py drives the robot with crawler.do_action(), which plays each
gait keyframe as a single discrete target. The servos then slew toward that
target as fast as the hardware allows, and the next keyframe arrives the moment
the previous one finishes. Large per-command angle steps + max-rate slew = foot
slip and visible jerk.

This script keeps the *exact same* gaits but raises the temporal resolution of
the trajectory: every keyframe-to-keyframe transition is split into SUBSTEPS
small interpolated poses, each commanded in turn with a short gap. We are not
inventing new motion -- we are walking the existing keyframe path in many small
increments instead of one big jump, so each servo delta is tiny and the overall
motion is gradual.

Two orthogonal smoothness knobs:
  * speed     -> per-step servo speed passed through to do_step  (- / +)
  * substeps  -> interpolation resolution per keyframe (primary)  ( [ / ] )
"""
from picrawler import Picrawler
from time import sleep
import readchar

crawler = Picrawler()

SPEED_MIN, SPEED_MAX = 50, 90
SUBSTEPS_MIN, SUBSTEPS_MAX = 1, 30

DEFAULT_SPEED = 70
DEFAULT_SUBSTEPS = 8     # micro-poses per gait keyframe; higher = smoother, slower
MICRO_GAP = 0.012        # delay between micro-poses (seconds)
SETTLE_GAP = 0.05        # brief pause after a full action completes

# Actions for which do_action toggles the gait phase. Mirror it so alternating
# presses step the correct diagonal pair, exactly like the stock controller.
GAIT_ACTIONS = ["forward", "backward", "turn left", "turn right",
                "turn left angle", "turn right angle"]

manual = """
Smooth Keyboard Control - PiCrawler

Movement:
  W: Forward
  A: Turn left
  S: Backward
  D: Turn right

Speed Control (per-step servo speed):
  + / = : Increase speed
  -     : Decrease speed

Smoothness (interpolation resolution -- the main knob):
  ] : More substeps (smoother, slower)
  [ : Fewer substeps (snappier)

Other:
  Space  : Stop (no action)
  Ctrl+C : Quit (auto sit)
"""


def clamp(value, min_value, max_value):
    """Limit value within a specified range."""
    return max(min_value, min(max_value, value))


def get_action_keyframes(motion_name):
    """Fetch a gait's keyframes while replicating do_action's bookkeeping.

    do_action sets move_list.stand_position before reading the gait property and
    toggles the phase for locomotion gaits. The gait properties are stateful
    (check_stand prepends a stand ramp, normal_action swaps legs by phase), so we
    must reproduce that here rather than reach past it.
    """
    move_list = crawler.move_list
    move_list.stand_position = crawler.stand_position
    if motion_name in GAIT_ACTIONS:
        crawler.stand_position = (crawler.stand_position + 1) & 1
    try:
        return list(move_list[motion_name])
    except AttributeError:
        # Custom action registered via add_action(), if any.
        return list(crawler.move_list_add.get(motion_name, []))


def lerp_pose(start, target, t):
    """Linear blend of two 4-leg [x, y, z] poses at fraction t in [0, 1]."""
    return [[s + (g - s) * t for s, g in zip(start_leg, target_leg)]
            for start_leg, target_leg in zip(start, target)]


def send_pose(pose, speed):
    """Command one interpolated pose. A single bad IK frame shouldn't kill the gait."""
    try:
        crawler.do_step(pose, speed)
    except Exception:
        pass


def play_smooth(motion_name, speed, substeps):
    """Play an action, subdividing every keyframe transition into `substeps` poses."""
    keyframes = get_action_keyframes(motion_name)
    for target in keyframes:
        start = crawler.current_step_all_leg_value()  # last commanded pose
        for i in range(1, substeps + 1):
            send_pose(lerp_pose(start, target, i / substeps), speed)
            sleep(MICRO_GAP)
    sleep(SETTLE_GAP)


def show_info(speed, substeps):
    """Clear terminal and display control instructions and current state."""
    print("\033[H\033[J", end="")  # Clear terminal screen
    print(manual)
    print(f"Speed:      {speed}  (range {SPEED_MIN}-{SPEED_MAX})")
    print(f"Smoothness: {substeps} substeps/keyframe  "
          f"(range {SUBSTEPS_MIN}-{SUBSTEPS_MAX})")
    print(f"Micro gap:  {MICRO_GAP:.3f}s")


def safe_sit(speed, substeps):
    """Smoothly sit before exit, falling back to a plain sit if anything fails."""
    try:
        play_smooth("sit", clamp(speed, SPEED_MIN, SPEED_MAX), max(substeps, 6))
        sleep(0.5)
    except Exception:
        try:
            crawler.do_step("sit", 40)
            sleep(1.0)
        except Exception:
            pass


def main():
    speed = DEFAULT_SPEED
    substeps = DEFAULT_SUBSTEPS

    # Establish a known pose (and sync current_coord) with a smooth stand so the
    # first locomotion command interpolates from where the robot actually is.
    play_smooth("stand", speed, substeps)
    show_info(speed, substeps)

    try:
        while True:
            key = readchar.readkey()
            k = key.lower()

            if k == "w":
                play_smooth("forward", speed, substeps)
            elif k == "s":
                play_smooth("backward", speed, substeps)
            elif k == "a":
                play_smooth("turn left", speed, substeps)
            elif k == "d":
                play_smooth("turn right", speed, substeps)

            # Speed
            elif k in ("+", "="):
                speed = clamp(speed + 5, SPEED_MIN, SPEED_MAX)
            elif k == "-":
                speed = clamp(speed - 5, SPEED_MIN, SPEED_MAX)

            # Smoothness (interpolation resolution)
            elif k == "]":
                substeps = clamp(substeps + 1, SUBSTEPS_MIN, SUBSTEPS_MAX)
            elif k == "[":
                substeps = clamp(substeps - 1, SUBSTEPS_MIN, SUBSTEPS_MAX)

            # Stop (no movement)
            elif k == " ":
                pass

            # Quit
            elif key == readchar.key.CTRL_C:
                print("\nQuit.")
                break

            show_info(speed, substeps)
            sleep(0.02)

    except KeyboardInterrupt:
        print("\nQuit (KeyboardInterrupt).")

    finally:
        safe_sit(speed, substeps)


if __name__ == "__main__":
    main()