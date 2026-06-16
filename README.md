# Crawler locomotion pipeline

Turns two intent vectors (`pointing` + `movement`) into 12 servo angles for a
4-leg, 3-DOF spider crawler. Deterministic, closed-form, pure stdlib.

## Layout

```
host.py                 # your websocket IO layer, wired to the controller
crawler/
  __init__.py           # exports CrawlerController
  config.py             # geometry, limits, calibration, gait tuning  <- tune here
  vmath.py              # tiny vector/rotation helpers (no numpy)
  pose.py        (st.2)  # pointing -> body attitude, with coupled speed limits
  gait.py        (st.3)  # cyclic clock + per-leg foot trajectories
  kinematics.py  (st.4/5)# frame composition + analytic 3-DOF IK
  output.py      (final) # internal radians -> hardware degrees, mirror + limits
  controller.py          # orchestrator; owns gait state; single public API
  calibrate.py           # re-derive the hardware affine map if geometry changes
  selftest.py            # diagnostics; proves rest pose == IO hardcoded frame
```

## Public API

```python
from crawler import CrawlerController
ctrl = CrawlerController()
angles = ctrl.step(pointing, movement, dt)  # 12 floats, R L BR BL, [hip,femur,tibia]
angles = ctrl.rest_angles()                 # standing pose, no motion
```

## Run

```
python -m crawler.selftest     # verify everything
python -m crawler.calibrate    # re-derive affine constants
python host.py                 # start the websocket host
```

## Conventions (the bits that bite)

- Output order **R, L, BR, BL**, each `[hip, femur, tibia]`, in **degrees**.
- Two angle worlds: INTERNAL radians (clean IK math) vs HARDWARE degrees
  (matches your rest frame). Conversion is a per-joint affine map in output.py,
  calibrated so the rest solution maps EXACTLY to `[+-45, 78, -147]`.
- Hip rest splay (the +-45) is `side_sign * (HIP_SPLAY_OFFSET + internal_hip)`.
- `movement[2]` doubles as a yaw-rate command for turning/spinning in place.
- The gait clock freezes at zero speed (robot stands instead of marching).
