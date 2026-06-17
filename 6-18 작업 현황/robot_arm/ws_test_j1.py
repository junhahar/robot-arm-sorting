"""
J1 단독 테스트 — 대시보드 디지털 트윈 검증용

모드:
  python ws_test_j1.py --sweep         J1을 0→270→0 반복 스윕
  python ws_test_j1.py --servo         실제 서보 J1 읽기 (RPi5)
  python ws_test_j1.py --manual        터미널에서 각도 직접 입력

대시보드에서 Running 모드로 전환 후 3D 모델이 따라오는지 확인.
"""
import asyncio
import json
import argparse
import math
import time

CLIENTS = set()
HOME = {"J1": 90.0, "J2": 120.0, "J3": 150.0, "J4": 80.0, "J5": 90.0}


async def broadcast(angles, source="test"):
    msg = {k: round(v, 1) for k, v in angles.items()}
    msg["source"] = source
    data = json.dumps(msg)
    for ws in list(CLIENTS):
        try:
            await ws.send(data)
        except Exception:
            CLIENTS.discard(ws)


async def handler(ws):
    CLIENTS.add(ws)
    print(f"[WS] 대시보드 연결됨 ({len(CLIENTS)})")
    try:
        async for _ in ws:
            pass
    finally:
        CLIENTS.discard(ws)
        print(f"[WS] 대시보드 해제 ({len(CLIENTS)})")


async def sweep_mode(hz=20):
    """J1을 0→270→0 반복 스윕 (나머지 HOME 고정)"""
    period = 1.0 / hz
    t = 0.0
    print("[SWEEP] J1 스윕 시작 (0° ↔ 270°)")
    while True:
        t += period
        j1 = 135.0 + 135.0 * math.sin(t * 0.5)
        angles = dict(HOME)
        angles["J1"] = j1
        await broadcast(angles, "sweep")
        if int(t * hz) % (hz * 2) == 0:
            print(f"  J1 = {j1:.1f}°")
        await asyncio.sleep(period)


async def servo_mode(hz=20):
    """실제 서보에서 J1만 읽기"""
    from servo_driver import ServoDriver
    driver = ServoDriver(dry_run=False)
    if driver.dry_run:
        print("[ERROR] CAN 연결 실패")
        return
    period = 1.0 / hz
    print("[SERVO] 실제 J1 서보 읽기 시작")
    while True:
        t0 = time.monotonic()
        val = driver.read_joint("J1")
        if val is not None:
            angles = dict(HOME)
            angles["J1"] = val
            await broadcast(angles, "servo")
            print(f"\r  J1 = {val:.1f}°", end="", flush=True)
        elapsed = time.monotonic() - t0
        await asyncio.sleep(max(0, period - elapsed))


async def manual_mode():
    """터미널에서 각도 직접 입력"""
    print("[MANUAL] 각도 입력 (0~270, q=종료)")
    loop = asyncio.get_event_loop()
    while True:
        raw = await loop.run_in_executor(None, lambda: input("  J1 각도> "))
        if raw.strip().lower() == "q":
            break
        try:
            val = float(raw.strip())
            val = max(0, min(270, val))
            angles = dict(HOME)
            angles["J1"] = val
            await broadcast(angles, "manual")
            print(f"  → J1 = {val:.1f}° 전송")
        except ValueError:
            print("  숫자를 입력하세요")


async def main(mode, host="0.0.0.0", port=8765, hz=20):
    import websockets

    print(f"[TEST] ws://{host}:{port} | 모드: {mode}")
    print("[TEST] 대시보드에서 MODE → RUNNING 전환 후 확인")

    async with websockets.serve(handler, host, port):
        if mode == "sweep":
            await sweep_mode(hz)
        elif mode == "servo":
            await servo_mode(hz)
        elif mode == "manual":
            await manual_mode()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="J1 Digital Twin Test")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--sweep", action="store_true", help="J1 자동 스윕 (기본)")
    group.add_argument("--servo", action="store_true", help="실제 서보 읽기")
    group.add_argument("--manual", action="store_true", help="각도 직접 입력")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--hz", type=int, default=20)
    args = parser.parse_args()

    mode = "servo" if args.servo else "manual" if args.manual else "sweep"
    asyncio.run(main(mode, port=args.port, hz=args.hz))
