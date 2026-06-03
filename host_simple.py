#!/usr/bin/env python3
"""Step-based controller using Picrawler-style discrete gait tables.

IO
--
WebSocket on port 8767 (host.py uses 8766, server.py uses 8765).
In:  {"type":"input", "k":[x,y,z], ...}  — same message format as the UI sends
Out: {"type":"legs", "angles":[...12...]} — same format the UI already understands

How it differs from host.py
----------------------------
host.py runs a continuous gait clock and smooth IK every frame.
This file works like keyboard_control.py / Picrawler.do_action(): it plays
through a pre-computed sequence of discrete foot-position "steps", one per
tick.  The keyboard vector k is quantised into a named action (forward /
backward / turn_left / turn_right / stand / sit), and the controller walks
through that action's step table.

The foot positions come straight from the Picrawler MoveList.  They are fed
into the existing crawler IK (solve_ik → to_hardware) which produces the same
12-float degree format the UI already understands.

Connecting
----------
In the UI, change the WebSocket URL to ws://localhost:8767 and click LINK.
WASD control the robot:  W = forward, S = backward, A = turn left, D = right.
"""

import asyncio
import json
import math

try:
    import websockets
except ImportError:
    raise SystemExit("pip install websockets")

from crawler import config as cfg
from crawler import kinematics as kin
from crawler import output as out


# ─────────────────────────────────────────────────────────────────────────────
# Gait constants  (Picrawler MoveList)
# ─────────────────────────────────────────────────────────────────────────────

X  = 45     # X_DEFAULT  — nominal outboard reach
XT = 70     # X_TURN     — wider reach used during turns / lifts
Y  = 20     # Y_DEFAULT  — fore/aft hip swing amplitude (kept small so hw
            #               angles stay in the visually-valid range for the UI)
YS = 0      # Y_START    — neutral (leg pointing straight out)
ZD = -50    # Z_DEFAULT  — standing foot height
ZU = -30    # Z_UP       — lifted foot height

# Turn geometry — reproduced from MoveList class-level computed attributes.
_LS  = 77.0                                          # LENGTH_SIDE
_TA  = math.sqrt((2*X + _LS)**2 + Y**2)
_TB  = 2*Y + _LS
_TC  = math.sqrt((2*X + _LS)**2 + (Y + _LS)**2)
_ALP = math.acos((_TA**2 + _TB**2 - _TC**2) / (2*_TA*_TB))
TX1  = (_TA - _LS) / 2
TY1  = Y / 2.0
TX0  = TX1 - _TB * math.cos(_ALP)
TY0  = _TB * math.sin(_ALP) - TY1 - _LS


# ─────────────────────────────────────────────────────────────────────────────
# Step tables
#
# Each action is a list of steps.  Each step is [leg0, leg1, leg2, leg3]
# where each entry is an [x, y, z] foot target in the leg's own local frame.
#
# Leg order matches cfg.LEGS: R (front-right), L (front-left),
#                              BR (back-right), BL (back-left).
#
# normal_action parity:
#   mode-0 (forward / backward): swap leg0↔leg1 and leg2↔leg3 for parity-1
#   mode-1 (turn):               swap leg0↔leg2 and leg1↔leg3 for parity-1
# Concatenating both parities gives one complete, repeating walking cycle.
# ─────────────────────────────────────────────────────────────────────────────

def _parity_mode0(steps: list) -> list:
    return [[s[1], s[0], s[3], s[2]] for s in steps]


def _parity_mode1(steps: list) -> list:
    return [[s[2], s[3], s[0], s[1]] for s in steps]


_STAND = [
    [[X,  Y,  ZD], [X,  YS, ZD], [X,  YS, ZD], [X,  Y,  ZD]],
]

_SIT = [
    [[X,  Y,  ZU], [XT, YS, ZU], [XT, YS, ZU], [X,  Y,  ZU]],
]

_FWD_BASE = [
    [[X,  Y,   ZD], [XT, YS,  ZU], [X,  YS, ZD], [X,  Y,   ZD]],
    [[X,  Y,   ZD], [X,  Y*2, ZU], [X,  YS, ZD], [X,  Y,   ZD]],
    [[X,  Y,   ZD], [X,  Y*2, ZD], [X,  YS, ZD], [X,  Y,   ZD]],
    [[X,  YS,  ZD], [X,  Y,   ZD], [X,  Y,  ZD], [X,  Y*2, ZD]],
    [[X,  YS,  ZD], [X,  Y,   ZD], [X,  Y,  ZD], [X,  Y*2, ZU]],
    [[X,  YS,  ZD], [X,  Y,   ZD], [X,  Y,  ZD], [XT, YS,  ZU]],
    [[X,  YS,  ZD], [X,  Y,   ZD], [X,  Y,  ZD], [X,  YS,  ZD]],
]

_BWD_BASE = [
    [[X,  Y,   ZD], [X,  YS,  ZD], [XT, YS,  ZU], [X,  Y,   ZD]],
    [[X,  Y,   ZD], [X,  YS,  ZD], [X,  Y*2, ZU], [X,  Y,   ZD]],
    [[X,  Y,   ZD], [X,  YS,  ZD], [X,  Y*2, ZD], [X,  Y,   ZD]],
    [[X,  Y*2, ZD], [X,  Y,   ZD], [X,  Y,   ZD], [X,  YS,  ZD]],
    [[X,  Y*2, ZU], [X,  Y,   ZD], [X,  Y,   ZD], [X,  YS,  ZD]],
    [[XT, YS,  ZU], [X,  Y,   ZD], [X,  Y,   ZD], [X,  YS,  ZD]],
    [[X,  YS,  ZD], [X,  Y,   ZD], [X,  Y,   ZD], [X,  YS,  ZD]],
]

_TL_BASE = [
    [[X,   Y,   ZD], [X,   YS,  ZD], [XT,  YS,  ZU], [X,   Y,   ZD]],
    [[TX1, TY1, ZD], [TX1, TY1, ZD], [TX0, TY0, ZU], [TX0, TY0, ZD]],
    [[TX1, TY1, ZD], [TX1, TY1, ZD], [TX0, TY0, ZD], [TX0, TY0, ZD]],
    [[TX1, TY1, ZD], [TX1, TY1, ZD], [TX0, TY0, ZD], [TX0, TY0, ZU]],
    [[X,   YS,  ZD], [X,   Y,   ZD], [X,   Y,   ZD], [XT,  YS,  ZU]],
    [[X,   YS,  ZD], [X,   Y,   ZD], [X,   Y,   ZD], [X,   YS,  ZD]],
]

_TR_BASE = [
    [[X,   Y,   ZD], [XT,  YS,  ZU], [X,   YS,  ZD], [X,   Y,   ZD]],
    [[TX0, TY0, ZD], [TX0, TY0, ZU], [TX1, TY1, ZD], [TX1, TY1, ZD]],
    [[TX0, TY0, ZD], [TX0, TY0, ZD], [TX1, TY1, ZD], [TX1, TY1, ZD]],
    [[TX0, TY0, ZU], [TX0, TY0, ZD], [TX1, TY1, ZD], [TX1, TY1, ZD]],
    [[XT,  YS,  ZU], [X,   Y,   ZD], [X,   Y,   ZD], [X,   YS,  ZD]],
    [[X,   YS,  ZD], [X,   Y,   ZD], [X,   Y,   ZD], [X,   YS,  ZD]],
]

ACTIONS: dict[str, list] = {
    "stand":      _STAND,
    "sit":        _SIT,
    "forward":    _FWD_BASE + _parity_mode0(_FWD_BASE),
    "backward":   _BWD_BASE + _parity_mode0(_BWD_BASE),
    "turn_left":  _TL_BASE  + _parity_mode1(_TL_BASE),
    "turn_right": _TR_BASE  + _parity_mode1(_TR_BASE),
}


# ─────────────────────────────────────────────────────────────────────────────
# Simple controller
# ─────────────────────────────────────────────────────────────────────────────

class SimpleController:
    """Plays through a discrete step sequence and returns 12 hardware-degree angles."""

    def __init__(self) -> None:
        self._action = "stand"
        self._idx = 0

    def set_action(self, name: str) -> None:
        if name not in ACTIONS or name == self._action:
            return
        self._action = name
        self._idx = 0

    def tick(self) -> list[float]:
        steps = ACTIONS[self._action]
        coords = steps[self._idx % len(steps)]
        self._idx = (self._idx + 1) % len(steps)
        return _coords_to_angles(coords)

    def rest_angles(self) -> list[float]:
        # Symmetric neutral pose: all legs straight out, standing height.
        neutral = [[X, YS, ZD], [X, YS, ZD], [X, YS, ZD], [X, YS, ZD]]
        return _coords_to_angles(neutral)


def _coords_to_angles(coords: list) -> list[float]:
    """4 leg-local [x,y,z] positions → 12 hardware-degree floats (UI format)."""
    angles = [0.0] * 12
    for leg, coord in zip(cfg.LEGS, coords):
        internal = kin.solve_ik(tuple(coord))
        triplet = out.to_hardware(internal, leg)
        angles[leg.pin_index : leg.pin_index + 3] = triplet
    return angles


# ─────────────────────────────────────────────────────────────────────────────
# Input → action  (keyboard vector from the UI)
# ─────────────────────────────────────────────────────────────────────────────

_THRESH = 0.3


def _action_from_input(msg: dict) -> str:
    """Quantise the UI keyboard vector k into a discrete Picrawler action name.

    k[2] = W - S  (forward / backward)
    k[0] = D - A  (turn right / turn left)
    """
    k = msg.get("k", [0, 0, 0])
    kz = float(k[2])   # W-S
    kx = float(k[0])   # D-A

    if kz > _THRESH:
        return "forward"
    if kz < -_THRESH:
        return "backward"
    if kx > _THRESH:
        return "turn_right"
    if kx < -_THRESH:
        return "turn_left"
    return "stand"


# ─────────────────────────────────────────────────────────────────────────────
# WebSocket handler
# ─────────────────────────────────────────────────────────────────────────────

TICK_INTERVAL = 0.12   # seconds per step advance (~8 steps/s)


async def handle(ws):
    peer = getattr(ws, "remote_address", "?")
    print(f"[+] client connected: {peer}")

    ctrl = SimpleController()

    await ws.send(json.dumps({"type": "legs", "angles": ctrl.rest_angles()}))

    async def ticker():
        while True:
            angles = ctrl.tick()
            try:
                await ws.send(json.dumps({"type": "legs", "angles": angles}))
            except websockets.ConnectionClosed:
                return
            await asyncio.sleep(TICK_INTERVAL)

    tick_task = asyncio.create_task(ticker())

    try:
        async for raw in ws:
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue
            if msg.get("type") != "input":
                continue
            ctrl.set_action(_action_from_input(msg))
    except websockets.ConnectionClosed:
        pass
    finally:
        tick_task.cancel()
        print(f"[-] client disconnected: {peer}")


async def main(host: str = "localhost", port: int = 8767) -> None:
    print(f"simple-step host  ws://{host}:{port}")
    print("In the UI set the WebSocket URL to that address and click LINK.")
    print("WASD: forward / backward / turn-left / turn-right\n")
    async with websockets.serve(handle, host, port, max_queue=8):
        await asyncio.Future()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nshutting down.")
