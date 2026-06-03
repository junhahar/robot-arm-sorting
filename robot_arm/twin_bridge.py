"""
디지털 트윈 브리지 — 로컬 STM32 펌웨어(0x201 텔레메트리)용

대상 펌웨어: sts3215/src/main.cpp
  - CAN 0x201 텔레메트리(DLC 7):
      [motor_id, angle_x10_lo, angle_x10_hi, tempC, load_lo, load_hi, flags]
  - angle_x10 = 서보 스텝(0~4095)을 0~3600(0.0~360.0°)으로 매핑한 값.
    펌웨어 stepToAngleX10(): angle_x10 = round(step * 3600 / 4095), 중앙 2048 ≈ 1801(180.1°).
  - flags bit0 = 각도 읽기 OK, bit1 = 온도 OK, bit2 = 부하 OK.

이 브리지는 0x201을 읽어 대시보드(robot_viewer.html / robot_dashboard.html)가
기대하는 J1~J5 절대각으로 변환해 ws://0.0.0.0:8765 로 중계한다.
변환은 servo_driver._calc_angle 과 동일 규약(home 기준, 중앙=home)이라
대시보드는 코드 수정 없이 그대로 동작한다.

실행:
  RPi5: python twin_bridge.py             # 실제 CAN (can0, 125kbps)
  PC:   python twin_bridge.py --dry-run   # 사인파 시뮬레이션 (하드웨어 불필요)

대시보드: ws://localhost:8765 로 연결.
"""
import asyncio
import json
import argparse
import math

from config import (
    JOINTS, SERVO_CENTER, SERVO_STEPS_PER_DEG, HOME,
    CAN_CHANNEL, CAN_BUSTYPE, CAN_BITRATE,
)

CAN_ID_TELEMETRY = 0x201  # 로컬 펌웨어 텔레메트리 ID
FLAG_ANGLE_OK = 0x01

CLIENTS = set()


# ── motor_id(서보 ID) → (joint, dir, home) ──
#   J2는 servo 2/3 듀얼이므로 대표로 servo 2(dir +1)만 표시에 사용하고 servo 3은 무시.
#   (servo_driver.read_joint 도 동일하게 servo_id[0]/dir[0] 만 읽는다.)
def _build_motor_map():
    m = {}
    for joint, cfg in JOINTS.items():
        sids = cfg["servo_id"]
        dirs = cfg["dir"]
        sid = sids[0] if isinstance(sids, list) else sids
        d = dirs[0] if isinstance(dirs, list) else dirs
        m[sid] = (joint, d, cfg["home"])
    return m


MOTOR_MAP = _build_motor_map()


# ── 각도 변환 ──

def angle_x10_to_step(angle_x10):
    """펌웨어 stepToAngleX10 의 역변환: angle_x10(0~3600) → 서보 스텝(0~4095)."""
    step = int(round(angle_x10 * 4095.0 / 3600.0))
    return max(0, min(4095, step))


def step_to_joint_angle(step, servo_dir, home):
    """servo_driver._calc_angle 과 동일: 스텝 → 관절 절대각(deg)."""
    offset = (step - SERVO_CENTER) / SERVO_STEPS_PER_DEG / servo_dir
    return home + offset


def parse_telemetry(data):
    """0x201 프레임(bytes) → (joint, angle_deg, tempC, load) 또는 None."""
    if len(data) < 7:
        return None
    motor_id = data[0]
    if motor_id not in MOTOR_MAP:
        return None  # servo 3(J2 미러) 등은 표시에서 제외
    flags = data[6]
    if not (flags & FLAG_ANGLE_OK):
        return None  # 각도 읽기 실패
    angle_x10 = data[1] | (data[2] << 8)
    tempC = data[3]
    load = data[4] | (data[5] << 8)
    joint, servo_dir, home = MOTOR_MAP[motor_id]
    step = angle_x10_to_step(angle_x10)
    angle = step_to_joint_angle(step, servo_dir, home)
    return joint, round(angle, 1), tempC, load


# ── WebSocket ──

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


# ── 실제 CAN 수신 루프 ──

async def can_loop(bus):
    print("[TWIN] 0x201 텔레메트리 수신 시작")
    loop = asyncio.get_running_loop()
    angles = dict(HOME)
    temps, loads = {}, {}
    while True:
        # bus.recv 는 블로킹이므로 executor 에서 돌려 이벤트 루프를 막지 않는다.
        msg = await loop.run_in_executor(None, bus.recv, 0.5)
        if msg is None or msg.arbitration_id != CAN_ID_TELEMETRY:
            continue
        parsed = parse_telemetry(bytes(msg.data))
        if not parsed:
            continue
        joint, angle, tempC, load = parsed
        angles[joint] = angle
        temps[joint] = tempC
        loads[joint] = load

        out = dict(angles)
        out["source"] = "servo"
        out["temps"] = temps
        out["loads"] = loads
        await broadcast(out)


# ── dry-run 시뮬레이션 ──

async def sim_loop(hz=20):
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


async def main(dry_run, host, port, channel, bitrate):
    import websockets

    runner = None
    if not dry_run:
        try:
            import can
            bus = can.interface.Bus(
                channel=channel, bustype=CAN_BUSTYPE, bitrate=bitrate,
            )
            print(f"[TWIN] CAN 연결: {channel} ({bitrate}bps)")
            runner = can_loop(bus)
        except Exception as e:
            print(f"[TWIN] CAN 연결 실패 → 시뮬레이션 모드: {e}")

    mode = "servo(0x201)"
    if runner is None:
        runner = sim_loop()
        mode = "sim"

    print(f"[TWIN] ws://{host}:{port} | 모드: {mode}")

    async with websockets.serve(handler, host, port):
        await runner


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Digital Twin Bridge (local 0x201 firmware)")
    parser.add_argument("--dry-run", action="store_true", help="사인파 시뮬레이션")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--channel", default=CAN_CHANNEL)
    parser.add_argument("--bitrate", type=int, default=CAN_BITRATE)
    args = parser.parse_args()
    asyncio.run(main(args.dry_run, args.host, args.port, args.channel, args.bitrate))
