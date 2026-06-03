#!/usr/bin/env python3

import asyncio
import json
import time

try:
    import websockets
except ImportError:
    raise SystemExit(
        "Missing dependency. Install it with:\n\n    pip install websockets\n"
    )


async def handle(ws):
    peer = getattr(ws, "remote_address", "?")
    print(f"[+] client connected: {peer}")
    # Send an immediate neutral frame so the mesh poses even before input moves.
    try:
        async for raw in ws:
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue
            if msg.get("type") != "input":
                continue

            k = msg.get("k", [0, 0, 0])
            m = msg.get("m", [0, 0, 1])
            yaw = float(msg.get("yaw", 0.0))
            pitch = float(msg.get("pitch", 0.0))
            t = float(msg.get("t", time.time() * 1000.0))

            # R, L, BR, BL
            angles = [-45,78,-147,-45,78,-147,45,78,-147,45,78,-147]

            await ws.send(json.dumps({"type": "legs", "angles": angles}))
    except websockets.ConnectionClosed:
        pass
    finally:
        print(f"[-] client disconnected: {peer}")


async def main(host="localhost", port=8765):
    print(f"crawler host listening on ws://{host}:{port}")
    print("open crawler-link.html and click LINK to connect.\n")
    async with websockets.serve(handle, host, port, max_queue=8):
        await asyncio.Future()  # run forever


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nshutting down.")
