"""
디지털 트윈 WebSocket 서버

실제 서보 피드백을 읽어 대시보드로 중계한다.
  RPi5: python ws_twin_server.py
  PC:   python ws_twin_server.py --dry-run   (시뮬레이션)

대시보드는 ws://localhost:8765 로 연결.
"""
import asyncio
import json
import argparse
import time
import math

CLIENTS = set()


async def broadcast(msg):
    data = json.dumps(msg)
    for ws in list(CLIENTS):
        try:
            await ws.send(data)
        except Exception:
            CLIENTS.discard(ws)


async def handler(ws):
    CLIENTS.add(ws)
    print(f"[WS] 클라이언트 연결 ({len(CLIENTS)}개)")
    try:
        async for _ in ws:
            pass
    finally:
        CLIENTS.discard(ws)
        print(f"[WS] 클라이언트 해제 ({len(CLIENTS)}개)")


async def servo_loop(driver, hz=20):
    period = 1.0 / hz
    while True:
        t0 = time.monotonic()
        angles = driver.read_all()
        if angles:
            msg = {k: round(v, 1) for k, v in angles.items()}
            msg["source"] = "servo"
            await broadcast(msg)
        elapsed = time.monotonic() - t0
        await asyncio.sleep(max(0, period - elapsed))


async def sim_loop(hz=20):
    """dry-run: sin 기반 데모 데이터 송출"""
    from config import HOME
    period = 1.0 / hz
    t = 0.0
    while True:
        t += period
        msg = {
            "J1": round(HOME["J1"] + math.sin(t * 0.4) * 20, 1),
            "J2": round(HOME["J2"] + math.sin(t * 0.3) * 12, 1),
            "J3": round(HOME["J3"] + math.sin(t * 0.35) * 10, 1),
            "J4": round(HOME["J4"] + math.sin(t * 0.45) * 8, 1),
            "J5": round(HOME["J5"] + math.sin(t * 0.5) * 15, 1),
            "source": "sim",
        }
        await broadcast(msg)
        await asyncio.sleep(period)


async def main(dry_run=False, host="0.0.0.0", port=8765, hz=20):
    import websockets

    driver = None
    if not dry_run:
        from servo_driver import ServoDriver
        driver = ServoDriver(dry_run=False)
        if driver.dry_run:
            print("[TWIN] CAN 연결 실패 → 시뮬레이션 모드")

    mode = "sim" if (dry_run or (driver and driver.dry_run)) else "servo"
    print(f"[TWIN] 모드: {mode} | ws://{host}:{port} | {hz}Hz")

    async with websockets.serve(handler, host, port):
        if mode == "servo":
            await servo_loop(driver, hz)
        else:
            await sim_loop(hz)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Digital Twin WS Server")
    parser.add_argument("--dry-run", action="store_true", help="시뮬레이션 모드")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--hz", type=int, default=20)
    args = parser.parse_args()
    asyncio.run(main(args.dry_run, args.host, args.port, args.hz))
