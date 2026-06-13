#!/usr/bin/env python3
"""
Manual single-leg controller in JOINT-ANGLE space (companion to do_single_leg.py).

do_single_leg.py nudges a leg in Cartesian X/Y/Z. This script nudges the three
joint angles of a leg directly, then lets the library's IK/FK relate that back
to a coordinate.

Angle convention follows what Picrawler.set_angle() expects per leg, i.e. the
[beta, alpha, gamma] triple that do_step builds from coord2polar:

    J1 = beta   (in-plane joint, "upper")   limit [-90, 90]
    J2 = alpha  (in-plane joint, "lower")   limit [-10, 90]
    J3 = gamma  (hip yaw / rotation)        limit [-60, 60]

set_angle() moves all four legs at once, so -- exactly like do_single_leg() in
the Cartesian script -- we hold the other three legs at their current angles and
only change the selected leg.
"""
from picrawler import Picrawler
from time import sleep
import readchar

crawler = Picrawler()

SPEED = 70
STEP_SIZE = 2  # degrees per keypress

# Per-joint limits, in the order set_angle() / limit_angle() use them.
ANGLE_LIMITS = [(-90, 90), (-10, 90), (-60, 60)]  # (J1, J2, J3)

manual = '''
----- PiCrawler Joint-Angle Controller -----
       .......          .......
    <=|   2   |┌-┌┐┌┐-┐|   1   |=>
       ``````` ├      ┤ ```````
       ....... ├      ┤ .......
    <=|   3   |└------┘|   4   |=>
       ```````          ```````
    1: Select right front leg
    2: Select left front leg
    3: Select left rear leg
    4: Select right rear leg

    Q: J1++   (upper)      W: J2++  (lower)
    A: J1--                S: J2--

    E: J3++   (hip yaw)    Z: Reset selected leg
    D: J3--                Ctrl+C: Quit
'''

legs_list = ['right front', 'left front', 'left rear', 'right rear']
joint_names = ['J1 beta (upper)', 'J2 alpha (lower)', 'J3 gamma (yaw)']

# key -> (joint index, delta in degrees)
move_map = {
    'q': (0, +STEP_SIZE),  # J1 ++
    'a': (0, -STEP_SIZE),  # J1 --
    'w': (1, +STEP_SIZE),  # J2 ++
    's': (1, -STEP_SIZE),  # J2 --
    'e': (2, +STEP_SIZE),  # J3 ++
    'd': (2, -STEP_SIZE),  # J3 --
}


def clear_screen():
    print("\033[H\033[J", end='')


def clamp_joint(joint_index, value):
    lo, hi = ANGLE_LIMITS[joint_index]
    return max(lo, min(hi, value))


def coord_to_angles(coord):
    """Cartesian [x, y, z] -> set_angle-order joint triple [J1, J2, J3]."""
    alpha, beta, gamma = crawler.coord2polar(coord)
    return [beta, alpha, gamma]


def angles_to_coord(joint_angles):
    """set_angle-order joint triple [J1, J2, J3] -> approximate FK [x, y, z]."""
    j1, j2, j3 = joint_angles
    # polar2coord expects coord2polar naming [alpha, beta, gamma] = [J2, J1, J3]
    return crawler.polar2coord([j2, j1, j3])


def apply_angles(angles):
    """Push all four legs' joint angles to the servos."""
    crawler.set_angle([list(leg) for leg in angles], SPEED)


def show_info(selected_leg, angles):
    clear_screen()
    print(manual)
    print(f"Selected leg: {selected_leg + 1} - {legs_list[selected_leg]}")
    print()
    for i, leg in enumerate(angles):
        marker = ">" if i == selected_leg else " "
        triple = "[" + ", ".join(f"{a:7.2f}" for a in leg) + "]"
        print(f"  {marker} leg {i + 1} {legs_list[i]:<12} {triple}")
    print()
    sel = angles[selected_leg]
    for name, value in zip(joint_names, sel):
        print(f"    {name:<18} {value:7.2f} deg")
    fk = angles_to_coord(sel)
    print(f"\n    approx FK coord [x, y, z]: "
          f"[{fk[0]:.2f}, {fk[1]:.2f}, {fk[2]:.2f}]")


def main():
    selected_leg = 0

    # Seed joint-angle state from the current pose. Like do_single_leg.py, the
    # first commanded move will sweep every leg from this state, so the initial
    # readout should reflect where the robot actually is.
    current_coord = crawler.current_step_all_leg_value()
    angles = [coord_to_angles(coord) for coord in current_coord]
    home_angles = [list(leg) for leg in angles]  # snapshot for reset

    try:
        show_info(selected_leg, angles)

        while True:
            key = readchar.readkey().lower()

            # Select leg
            if key in ('1', '2', '3', '4'):
                selected_leg = int(key) - 1
                show_info(selected_leg, angles)

            # Move a joint of the selected leg
            elif key in move_map:
                joint_index, delta = move_map[key]
                new_value = angles[selected_leg][joint_index] + delta
                angles[selected_leg][joint_index] = clamp_joint(joint_index, new_value)

                apply_angles(angles)
                sleep(0.1)
                show_info(selected_leg, angles)

            # Reset the selected leg to its starting angles
            elif key == 'z':
                angles[selected_leg] = list(home_angles[selected_leg])
                apply_angles(angles)
                sleep(0.1)
                show_info(selected_leg, angles)

            sleep(0.05)

    except KeyboardInterrupt:
        print("\nExiting safely...")

    finally:
        # Return to sitting position on exit
        try:
            crawler.do_step('sit', 40)
            sleep(1)
        except Exception:
            pass

        print("Robot is now sitting. Program ended.")


if __name__ == "__main__":
    main()