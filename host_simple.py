#!/usr/bin/env python3
"""Step-based controller using Picrawler-style discrete gait tables.

IO
--
WebSocket on port 8767 (host.py uses 8766, server.py uses 8765).
In:  {"type":"input", "k":[x,y,z], ...}  — same message format as the UI sends
Out: {"type":"legs", "angles":[...12...]} — same format the UI understands

This controller uses the Picrawler's own coord2polar IK and MoveList foot
positions.  The output angles are expressed in the HTML renderer's frame:
  angles[L*3+0] = coxa  (hip yaw,    applied as coxaPivot.rotation.y)
  angles[L*3+1] = femur (femur lift, applied as femurPivot.rotation.z)
  angles[L*3+2] = tibia (knee bend,  applied as tibiaPivot.rotation.z)

All values are in DEGREES (the UI converts to radians with THREE.degToRad).

Leg order sent matches the Picrawler gait-slot order, which the HTML
renderer now also uses:  0=RF, 1=LF, 2=LR, 3=RR  (counter-clockwise from
the front/camera edge).  Forward travel is toward the RF/LF edge (+Z).

Connecting
----------
In the UI set the WebSocket URL to ws://localhost:8767 and click LINK.
WASD:  W = forward, S = backward, A = turn left, D = turn right.
Release all keys → stand.
"""

import asyncio
import json
import math

try:
    import websockets
except ImportError:
    raise SystemExit("pip install websockets")


# ─────────────────────────────────────────────────────────────────────────────
# Picrawler geometry constants
# ─────────────────────────────────────────────────────────────────────────────

_A = 48.0   # femur link length (mm)
_B = 78.0   # tibia link length (mm)
_C = 33.0   # coxa / hip-offset length (mm)


def coord2polar(x: float, y: float, z: float):
    """Picrawler inverse kinematics.

    Input : foot position in the leg's local frame (mm).
            x = outboard reach, y = fore/aft swing, z = height (negative = down).
    Output: (alpha_hw, beta_hw, gamma_hw) in hardware degrees.
            gamma = hip yaw, alpha = femur pitch, beta = tibia/knee angle.
    """
    # reachability clamp
    L = math.sqrt(x*x + y*y + z*z)
    if L == 0:
        L = 0.1
    if L < _C:
        f = _C / L;  x, y, z = f*x, f*y, f*z
    elif L > (_A + _B + _C):
        f = (_A + _B + _C) / L;  x, y, z = f*x, f*y, f*z

    w = math.sqrt(x*x + y*y)
    v = w - _C
    u = math.sqrt(z*z + v*v)
    u = max(30.0, min(91.58, u))

    cos1 = (_B*_B + _A*_A - u*u) / (2*_B*_A)
    beta_rad = math.acos(max(-1.0, min(1.0, cos1)))

    angle1 = math.atan2(z, v)
    cos2   = (_A*_A + u*u - _B*_B) / (2*_A*u)
    angle2 = math.acos(max(-1.0, min(1.0, cos2)))
    alpha_rad = angle2 + angle1

    gamma_rad = math.atan2(y, x)

    alpha_hw = 90.0 - math.degrees(alpha_rad)
    beta_hw  = math.degrees(beta_rad) - 90.0
    gamma_hw = -(math.degrees(gamma_rad) - 45.0)

    return alpha_hw, beta_hw, gamma_hw


def picrawler_to_html_angles(alpha_hw: float, beta_hw: float, gamma_hw: float):
    """Convert Picrawler hardware degrees to the HTML renderer's angle convention.

    HTML conventions (each value in DEGREES, converted to rad by the UI):
      coxa  (rotation.y) : 0 = leg points along its mount diagonal.
                           Positive = leg swings forward (toward nose).
      femur (rotation.z) : 0 = femur inline with coxa (horizontal).
                           Positive = femur lifts upward.
      tibia (rotation.z) : 0 = tibia inline with femur (fully extended).
                           Negative = knee folds (foot tucks toward body).

    Derivation (see notebook):
      coxa_deg  = 45 - gamma_hw          (inverts the Picrawler hw offset)
      femur_deg = 90 - alpha_hw          (converts to elevation-from-horizontal)
      tibia_deg = beta_hw - 90           (converts interior knee angle to bend)
    """
    coxa_deg  =  45.0 - gamma_hw
    femur_deg =  90.0 - alpha_hw
    tibia_deg = beta_hw - 90.0
    return coxa_deg, femur_deg, tibia_deg


# ─────────────────────────────────────────────────────────────────────────────
# Gait constants  (Picrawler MoveList)
# ─────────────────────────────────────────────────────────────────────────────

X  = 45.0   # X_DEFAULT  — nominal outboard reach
XT = 70.0   # X_TURN     — wider reach during turns / lifts
Y  = 45.0   # Y_DEFAULT  — fore/aft hip swing
YS = 0.0    # Y_START    — neutral (leg pointing straight out)
ZD = -50.0  # Z_DEFAULT  — standing foot height
ZU = -30.0  # Z_UP       — lifted foot height

# Turn geometry — reproduced from MoveList class-level computed attributes.
_LS  = 77.0
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
# Each action is a list of steps.
# Each step is [leg0, leg1, leg2, leg3] in Picrawler order: [FR, FL, BR, BL].
# Each entry is [x, y, z] foot target in the leg's local frame (mm).
#
# normal_action parity:
#   mode-0 (forward / backward): swap leg0↔leg1, leg2↔leg3 for parity-1
#   mode-1 (turn):               swap leg0↔leg2, leg1↔leg3 for parity-1
# ─────────────────────────────────────────────────────────────────────────────

def _parity_mode0(steps):
    return [[s[1], s[0], s[3], s[2]] for s in steps]

def _parity_mode1(steps):
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

# Per-leg hip-yaw sign, taken straight from the Picrawler `direction` array
# (the gamma/3rd element of each leg triplet): leg0=-1, leg1=+1, leg2=-1, leg3=+1.
# Slots are in Picrawler order:  0=RF, 1=LF, 2=LR, 3=RR.
# On the physical robot this just compensates for mirror-mounted servos; in the
# simulation (no servos) it is what makes each diagonal pair yaw oppositely so
# the gait travels straight forward instead of crabbing sideways.
_COXA_SIGN = [-1.0, +1.0, -1.0, +1.0]

# The HTML renderer's leg slots are in the SAME order as the Picrawler gait
# slots (0=RF, 1=LF, 2=LR, 3=RR), so the output index is the slot index.


# ─────────────────────────────────────────────────────────────────────────────
# IK: convert one step's 4 foot positions → 12 HTML-degree angles
# ─────────────────────────────────────────────────────────────────────────────

def _step_to_angles(step: list) -> list[float]:
    """4 Picrawler foot coords → 12 HTML-degree angles, leg order RF,LF,LR,RR."""
    out = [0.0] * 12
    for slot, (x, y, z) in enumerate(step):
        alpha_hw, beta_hw, gamma_hw = coord2polar(x, y, z)
        coxa_d, femur_d, tibia_d    = picrawler_to_html_angles(alpha_hw, beta_hw, gamma_hw)
        coxa_d *= _COXA_SIGN[slot]
        out[slot*3 + 0] = coxa_d
        out[slot*3 + 1] = femur_d
        out[slot*3 + 2] = tibia_d
    return out


def _rest_angles() -> list[float]:
    rest_coord = [X, YS, ZD]
    a, b, g = coord2polar(*rest_coord)
    c, f, t = picrawler_to_html_angles(a, b, g)
    return [c, f, t] * 4


# ─────────────────────────────────────────────────────────────────────────────
# Simple controller
# ─────────────────────────────────────────────────────────────────────────────

class SimpleController:
    def __init__(self) -> None:
        self._action = "stand"
        self._idx    = 0

    def set_action(self, name: str) -> None:
        if name not in ACTIONS or name == self._action:
            return
        self._action = name
        self._idx    = 0

    def tick(self) -> list[float]:
        steps  = ACTIONS[self._action]
        coords = steps[self._idx % len(steps)]
        self._idx = (self._idx + 1) % len(steps)
        return _step_to_angles(coords)

    def rest_angles(self) -> list[float]:
        return _rest_angles()


# ─────────────────────────────────────────────────────────────────────────────
# Input → action
# ─────────────────────────────────────────────────────────────────────────────

_THRESH = 0.3

def _action_from_input(msg: dict) -> str:
    k  = msg.get("k", [0, 0, 0])
    kz = float(k[2])   # W - S
    kx = float(k[0])   # D - A
    if kz >  _THRESH:  return "forward"
    if kz < -_THRESH:  return "backward"
    if kx >  _THRESH:  return "turn_right"
    if kx < -_THRESH:  return "turn_left"
    return "stand"


# ─────────────────────────────────────────────────────────────────────────────
# WebSocket handler
# ─────────────────────────────────────────────────────────────────────────────

TICK_INTERVAL = 0.12   # seconds per step (~8 steps/s)


async def handle(ws):
    peer = getattr(ws, "remote_address", "?")
    print(f"[+] client connected: {peer}")

    ctrl = SimpleController()
    await ws.send(json.dumps({"type": "legs", "angles": ctrl.rest_angles()}))

    async def ticker():
        while True:
            try:
                await ws.send(json.dumps({"type": "legs", "angles": ctrl.tick()}))
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
    print("Set the UI WebSocket URL to that address and click LINK.")
    print("WASD: forward / backward / turn-left / turn-right\n")
    async with websockets.serve(handle, host, port, max_queue=8):
        await asyncio.Future()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nshutting down.")
