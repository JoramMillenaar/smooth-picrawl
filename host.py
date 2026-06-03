#!/usr/bin/env python3
"""Websocket IO layer, wired to the crawler locomotion pipeline.

This is your original host file with one change: instead of a hardcoded
angle list, it instantiates a CrawlerController and runs each input frame
through the full pipeline. The controller is created PER CONNECTION so each
client gets its own gait-clock state.
"""

import asyncio
import json
import time

try:
    import websockets
except ImportError:
    raise SystemExit(
        "Missing dependency. Install it with:\n\n    pip install websockets\n"
    )

from crawler import CrawlerController


async def handle(ws):
    peer = getattr(ws, "remote_address", "?")
    print(f"[+] client connected: {peer}")

    ctrl = CrawlerController()            # per-connection gait state
    last_t_ms = None                      # client timestamp for dt

    # Send an immediate neutral frame so the mesh poses even before input moves.
    await ws.send(json.dumps({"type": "legs", "angles": ctrl.rest_angles()}))

    try:
        async for raw in ws:
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue
            if msg.get("type") != "input":
                continue

            k = msg.get("k", [0, 0, 0])          # pointing vector
            m = msg.get("m", [0, 0, 1])          # movement vector
            yaw = float(msg.get("yaw", 0.0))     # (still accepted; unused here)
            pitch = float(msg.get("pitch", 0.0))
            t = float(msg.get("t", time.time() * 1000.0))

            # derive dt (seconds) from the client timestamp stream
            if last_t_ms is None:
                dt = None                        # controller falls back to wall clock
            else:
                dt = max(0.0, (t - last_t_ms) / 1000.0)
            last_t_ms = t

            pointing = (float(k[0]), float(k[1]), float(k[2]))
            movement = (float(m[0]), float(m[1]), float(m[2]))

            # R, L, BR, BL  -- 12 floats, [hip, femur, tibia] each
            angles = ctrl.step(pointing, movement, dt)

            await ws.send(json.dumps({"type": "legs", "angles": angles}))
            await asyncio.sleep(0.1)
    except websockets.ConnectionClosed:
        pass
    finally:
        print(f"[-] client disconnected: {peer}")


async def main(host="localhost", port=8766):
    print(f"crawler host listening on ws://{host}:{port}")
    print("open crawler-link.html and click LINK to connect.\n")
    async with websockets.serve(handle, host, port, max_queue=8):
        await asyncio.Future()  # run forever


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nshutting down.")
