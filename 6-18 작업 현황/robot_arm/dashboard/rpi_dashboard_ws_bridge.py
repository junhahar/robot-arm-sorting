#!/usr/bin/env python3
"""
Raspberry Pi -> Sambo dashboard WebSocket bridge.

Reads SocketCAN frames from can0 and broadcasts them to the dashboard at ws://<pi-ip>:8765.
The dashboard understands raw CAN frames:
  0x201: STM32 telemetry [id, angle_lo, angle_hi, tempC, load_lo, load_hi, flags]
  0x202: STM32 current   [id, cur_lo, cur_hi, ok]

Install on Raspberry Pi:
  python3 -m pip install python-can websockets

Run:
  sudo ip link set can0 up type can bitrate 125000 sample-point 0.625
  python3 rpi_dashboard_ws_bridge.py
"""

import asyncio
import json
import time

import can
import websockets


CAN_INTERFACE = "can0"
WS_HOST = "0.0.0.0"
WS_PORT = 8765

clients = set()
can_health = {
    "received": 0,
    "errors": 0,
    "delay_ms": 0,
    "last_rx_ms": 0,
}
last_rx_time = time.monotonic()


def frame_to_payload(msg):
    data = list(msg.data)
    return {
        "system": {
            "server_connected": True,
            "can_status": "OK",
        },
        "can_health": can_health,
        "digital_twin": {
            "source": "Raspberry Pi SocketCAN -> WebSocket",
            "cmd_id": "0x100",
            "telemetry_id": "0x201",
            "current_id": "0x202",
            "latency_ms": can_health["last_rx_ms"],
        },
        "can_frame": {
            "id": f"0x{msg.arbitration_id:03X}",
            "data": data,
            "dlc": msg.dlc,
            "timestamp": msg.timestamp,
        },
    }


async def broadcast(payload):
    if not clients:
        return
    text = json.dumps(payload, ensure_ascii=False)
    dead = []
    for ws in clients:
        try:
            await ws.send(text)
        except Exception:
            dead.append(ws)
    for ws in dead:
        clients.discard(ws)


async def ws_handler(websocket):
    clients.add(websocket)
    print(f"[WS] dashboard connected: {websocket.remote_address}")
    try:
        async for message in websocket:
            # Dashboard command messages are logged only here.
            # Keep real motor control in the verified Raspberry Pi control code.
            print(f"[WS RX] {message}")
    finally:
        clients.discard(websocket)
        print("[WS] dashboard disconnected")


async def can_loop():
    global last_rx_time
    bus = can.interface.Bus(channel=CAN_INTERFACE, interface="socketcan")
    print(f"[CAN] listening on {CAN_INTERFACE}")
    while True:
        msg = await asyncio.to_thread(bus.recv, 0.05)
        if msg is None:
            await asyncio.sleep(0.01)
            continue
        now = time.monotonic()
        can_health["received"] += 1
        can_health["last_rx_ms"] = int((now - last_rx_time) * 1000)
        can_health["delay_ms"] = can_health["last_rx_ms"]
        last_rx_time = now
        if msg.arbitration_id in (0x200, 0x201, 0x202):
            await broadcast(frame_to_payload(msg))


async def main():
    print("Sambo Dashboard WebSocket Bridge")
    print(f"WebSocket: ws://0.0.0.0:{WS_PORT}")
    print(f"CAN      : {CAN_INTERFACE} / 125 kbps / sample-point 62.5%")
    async with websockets.serve(ws_handler, WS_HOST, WS_PORT):
        await can_loop()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nbye")
