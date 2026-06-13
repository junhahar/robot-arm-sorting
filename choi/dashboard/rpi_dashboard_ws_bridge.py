#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sambo dashboard WS bridge v3 = v2(스냅샷 텔레메트리) + 모션 명령.

추가: 대시보드 WS 명령 {"type":"COMMAND","command":...} 수신 시 모터 이동.
  AUTO_START → 스캔자세로 이동
  AUTO_STOP  → 스캔자세 복귀 후 원점(전 모터 180°)
  HALT       → 즉시정지(진행 이동 선점 + 현재 실측각에 토크 홀드)
  PING       → 무동작(수신 경로 검증용)

이동은 ik_server.move_axis 와 동일한 S-curve 동기 이동(AXIS_SPEED 20°/s).
새 명령이 오면 진행 중 이동을 선점(취소)하고 교체.

⚠️ 브리지가 CAN 명령자가 되므로 ik_server.py / 키보드제어(v1_11)와 동시 실행 금지.
"""
import asyncio
import json
import struct
import threading
import time

import can
import websockets

try:
    from sambo_vision_state import load_latest_runtime_state
except Exception as e:
    print(f"[VISION] sambo_vision_state disabled: {e}")

    def load_latest_runtime_state():
        return {"vision": {"target": "NONE", "confidence": 0.0, "stable_frames": 0,
                           "bbox": None, "correction": {"dx_mm": 0.0, "dy_mm": 0.0}},
                "gripper": {"state": "OPEN", "sg90_angle": 60.0, "fresh": False}}

CAN_INTERFACE = "can0"
WS_HOST = "0.0.0.0"
WS_PORT = 8765
SEND_HZ = 10
STALE_S = 1.5
NAMES = {1: "J1", 2: "J2", 3: "J3", 4: "J4", 5: "J5", 6: "J6"}

# ── 모션 파라미터 (ik_server 와 동일) ──
AXIS_SPEED = 20.0          # deg/s
MIN_DURATION = 0.8
DT = 1.0 / 40.0
STREAM_MOVE_MS = 80
SCAN_POSE = {1: 180.0, 2: 104.8, 3: 248.2, 4: 112.7, 5: 140.8, 6: 180.0}
HOME_POSE = {i: 180.0 for i in range(1, 7)}
MANUAL_MIN_DURATION = 0.3   # 수동 jog/적용 최소 소요시간[s] (작은 각도도 부드럽게)

# ── 관절 소프트 한계 (raw 명령각, rpi_keyboard_control_v1_11 MOTOR_LIMITS와 동일) ──
# 수동 시험에서 명령각이 이 범위를 넘지 않게 끝에서 자른다(clamp). 하드스톱 충돌 방지.
MOTOR_LIMITS = {
    1: (40.0, 320.0),
    2: (60.0, 185.0),
    3: (175.0, 300.0),
    4: (80.0, 183.0),
    5: (115.0, 250.0),
    6: (80.0, 280.0),
}


def clamp_limit(mid, deg):
    lo, hi = MOTOR_LIMITS.get(mid, (0.0, 360.0))
    return max(lo, min(hi, deg))


# ── MG90 그리퍼 (STM32 0x100 하위cmd 0x06, PA6 PWM) ──
CMD_GRIPPER = 0x06
GRIPPER_MIN, GRIPPER_MAX = 0, 180

clients = set()
lock = threading.Lock()
motors = {i: {"current": None, "temp": None, "load": None, "target": None,
              "flags": 0, "current_ma": None, "rx": 0.0, "cmd_rx": 0.0}
          for i in range(1, 7)}
health = {"received": 0, "last_rx_ms": 0}

# ── 모션 상태 (generation 기반 선점) ──
mlock = threading.Lock()
motion = {"seq": None, "gen": 0, "label": "", "min_dur": MIN_DURATION}

# ── 그리퍼 요청 상태 (가장 최근 각도만 송신, gen 으로 갱신 감지) ──
glock = threading.Lock()
gripper_req = {"angle": None, "gen": 0}

# MG90S 그리퍼는 위치 피드백이 없다(STS3215처럼 현재각을 못 읽음).
# 따라서 '마지막으로 STM에 내린 명령각'을 그대로 현재각으로 보고한다(open-loop). lock 으로 보호.
gripper_state = {"angle": None}


def can_reader():
    while True:
        try:
            bus = can.interface.Bus(channel=CAN_INTERFACE, interface="socketcan")
        except Exception as e:
            print("[CAN] open fail:", e); time.sleep(2); continue
        print("[CAN] reading", CAN_INTERFACE)
        last = time.monotonic()
        try:
            while True:
                msg = bus.recv(1.0)
                if msg is None:
                    continue
                now = time.monotonic()
                d = bytes(msg.data)
                aid = msg.arbitration_id
                with lock:
                    health["received"] += 1
                    health["last_rx_ms"] = int((now - last) * 1000)
                    if aid == 0x201 and len(d) >= 7:
                        mid, ax10, tempC, load_raw, flags = struct.unpack("<BHBHB", d[0:7])
                        m = motors.get(mid)
                        if m:
                            if flags & 0x01:
                                m["current"] = ax10 / 10.0
                            if flags & 0x02:
                                m["temp"] = tempC
                            if flags & 0x04:
                                m["load"] = round((load_raw & 0x03FF) * 0.1, 1)
                            m["flags"] = flags
                            m["rx"] = now
                            if m["cmd_rx"] == 0.0 and m["target"] is None and m["current"] is not None:
                                pass  # 목표각 기본값(=current) 제거: 명령(0x100) 없으면 target 안 보냄
                    elif aid == 0x202 and len(d) >= 4:
                        mid = d[0]
                        m = motors.get(mid)
                        if m:
                            if d[3]:
                                m["current_ma"] = round(((d[1] | (d[2] << 8)) & 0x7FFF) * 6.5)
                            m["rx"] = now
                    elif aid == 0x100 and len(d) >= 4 and d[0] == 0x03:
                        mid = d[1]
                        m = motors.get(mid)
                        if m:
                            m["target"] = ((d[3] << 8) | d[2]) / 10.0
                            m["cmd_rx"] = now
                last = now
        except Exception as e:
            print("[CAN] error:", e); time.sleep(1)
        finally:
            try:
                bus.shutdown()
            except Exception:
                pass


# ── 모션 ──
def smoothstep(t):
    return t * t * (3.0 - 2.0 * t)


def _send_angle(bus, mid, deg, move_ms=STREAM_MOVE_MS):
    deg = max(0.0, min(360.0, deg))
    ax10 = int(round(deg * 10))
    mt = int(move_ms) & 0xFFFF
    data = [0x03, mid, ax10 & 0xFF, (ax10 >> 8) & 0xFF,
            mt & 0xFF, (mt >> 8) & 0xFF, 0, 0]
    try:
        bus.send(can.Message(arbitration_id=0x100, data=data, is_extended_id=False))
    except can.CanError:
        pass


def _send_gripper(bus, angle):
    a = int(max(GRIPPER_MIN, min(GRIPPER_MAX, round(angle))))
    data = [CMD_GRIPPER, a & 0xFF, 0, 0, 0, 0, 0, 0]
    try:
        bus.send(can.Message(arbitration_id=0x100, data=data, is_extended_id=False))
    except can.CanError:
        pass


def request_motion(seq, label, min_dur=MIN_DURATION):
    """seq: [(targets_dict, sub_label), ...]. 새 요청은 진행 중 이동을 선점."""
    with mlock:
        motion["seq"] = seq
        motion["gen"] += 1
        motion["label"] = label
        motion["min_dur"] = min_dur
        return motion["gen"]


def request_gripper(angle):
    """가장 최근 그리퍼 목표각으로 교체(이전 요청 선점)."""
    a = float(angle)
    with glock:
        gripper_req["angle"] = a
        gripper_req["gen"] += 1
    # 명령하는 즉시 현재각으로 기록(피드백이 없으므로 명령각 = 현재각).
    with lock:
        gripper_state["angle"] = int(max(GRIPPER_MIN, min(GRIPPER_MAX, round(a))))


def _move_to(bus, targets, my_gen, min_dur=MIN_DURATION):
    with lock:
        seeds = {m: (motors[m]["current"] if motors[m]["current"] is not None else 180.0)
                 for m in targets}
    maxd = max(abs(targets[m] - seeds[m]) for m in targets)
    dur = max(maxd / AXIS_SPEED, min_dur)
    steps = max(1, int(dur / DT))
    for i in range(steps + 1):
        with mlock:
            if motion["gen"] != my_gen:
                return False  # 선점됨
        s = smoothstep(i / steps)
        for m in targets:
            _send_angle(bus, m, seeds[m] + (targets[m] - seeds[m]) * s)
        time.sleep(DT)
    for m in targets:
        _send_angle(bus, m, targets[m])
    return True


def motion_worker():
    bus = None
    while True:
        with mlock:
            seq = motion["seq"]; gen = motion["gen"]; label = motion["label"]
            min_dur = motion["min_dur"]
            motion["seq"] = None
        if seq is None:
            time.sleep(0.03); continue
        if bus is None:
            try:
                bus = can.interface.Bus(channel=CAN_INTERFACE, interface="socketcan")
            except Exception as e:
                print("[MOVE] send bus fail:", e); time.sleep(1); continue
        print(f"[MOVE] start: {label}")
        for targets, sub in seq:
            print(f"[MOVE]   -> {sub}")
            if not _move_to(bus, targets, gen, min_dur):
                print(f"[MOVE]   preempted at {sub}")
                break
        else:
            print(f"[MOVE] done: {label}")


def gripper_worker():
    """대시보드 그리퍼 명령을 별도 버스로 즉시 송신(관절 이동과 독립)."""
    bus = None
    last_gen = 0
    while True:
        with glock:
            ang = gripper_req["angle"]; gen = gripper_req["gen"]
        if gen == last_gen or ang is None:
            time.sleep(0.03); continue
        if bus is None:
            try:
                bus = can.interface.Bus(channel=CAN_INTERFACE, interface="socketcan")
            except Exception as e:
                print("[GRIP] send bus fail:", e); time.sleep(1); continue
        last_gen = gen
        _send_gripper(bus, ang)
        print(f"[GRIP] {ang:.0f}°")


def derive_can_status(comms):
    """모터별 comm(WAIT/STALE/PARTIAL/OK)을 CAN 버스 전체 상태로 집계.
    OK=전 노드 정상, WARN=일부만 수신/플래그 부분, STALE=버스 침묵 또는 수신 끊김."""
    if all(c == "WAIT" for c in comms):
        return "STALE"          # 한 번도 수신 없음 = 버스 침묵(끊김)
    if any(c == "STALE" for c in comms):
        return "STALE"          # 수신하다 끊긴 모터 존재
    if any(c in ("WAIT", "PARTIAL") for c in comms):
        return "WARN"           # 일부만 수신 / 텔레메트리 플래그 부분
    return "OK"


def build_snapshot():
    now = time.monotonic()
    joints = []
    comms = []
    with lock:
        for i in range(1, 7):
            m = motors[i]
            if not m["rx"]:
                comm = "WAIT"
            elif (now - m["rx"]) > STALE_S:
                comm = "STALE"
            elif (m["flags"] & 0x07) != 0x07:
                comm = "PARTIAL"
            else:
                comm = "OK"
            comms.append(comm)
            j = {"id": NAMES[i], "comm": comm}
            for k in ("current", "temp", "load", "target", "current_ma"):
                if m[k] is not None:
                    j[k] = m[k]
            joints.append(j)
        hp = dict(health)
        ga = gripper_state["angle"]
    runtime = load_latest_runtime_state()
    vision = runtime.get("vision", {})
    # 그리퍼(MG90S)는 피드백이 없다. 브리지가 마지막에 STM으로 내린 명령각을 현재각으로 덮어쓴다.
    gripper = dict(runtime.get("gripper", {}))
    if ga is not None:
        gripper["sg90_angle"] = ga
    system = {"server_connected": True, "can_status": derive_can_status(comms)}
    if vision.get("fresh"):
        system.update({"camera_status": "OK", "ai_status": "RUNNING"})
    return {"robot": {"joints": joints},
            "system": system,
            "vision": vision,
            "gripper": gripper,
            "can_health": hp}


async def broadcast(text):
    dead = []
    for ws in list(clients):
        try:
            await asyncio.wait_for(ws.send(text), timeout=0.25)
        except Exception:
            dead.append(ws)
    for ws in dead:
        clients.discard(ws)


async def broadcaster():
    period = 1.0 / SEND_HZ
    while True:
        await asyncio.sleep(period)
        if clients:
            await broadcast(json.dumps(build_snapshot(), ensure_ascii=False))


async def ws_handler(ws):
    clients.add(ws)
    print("[WS] connect", ws.remote_address)
    try:
        async for message in ws:
            try:
                d = json.loads(message)
            except Exception:
                continue
            if d.get("type") == "COMMAND":
                cmd = d.get("command")
                payload = d.get("payload") or {}
                if cmd == "AUTO_START":
                    request_motion([(SCAN_POSE, "스캔자세")], "START → 스캔자세")
                    print("[CMD] AUTO_START → 스캔자세 이동")
                elif cmd == "AUTO_STOP":
                    request_motion([(SCAN_POSE, "스캔복귀"), (HOME_POSE, "원점180")],
                                   "STOP → 스캔 후 원점")
                    print("[CMD] AUTO_STOP → 스캔 후 원점 이동")
                elif cmd == "HALT":
                    # 즉시정지: 진행 중 이동을 선점하고 각 모터를 현재 실측각에 짧게 홀드(토크 유지).
                    with lock:
                        hold = {m: motors[m]["current"] for m in range(1, 7)
                                if motors[m]["current"] is not None}
                    if hold:
                        request_motion([(hold, "HALT 현재각 홀드")],
                                       "HALT 즉시정지", min_dur=0.05)
                        print(f"[CMD] HALT → 현재각 홀드 {sorted(hold)}")
                    else:
                        request_motion([], "HALT 선점")  # 현재각 미수신: 진행 이동만 선점
                        print("[CMD] HALT: 현재각 없음 — 진행 이동만 선점")
                elif cmd == "MANUAL_MOVE":
                    # payload.targets: [{motor_id, angle_deg(raw)}, ...]  여러 모터 동시 S-curve.
                    targets = {}
                    for t in payload.get("targets", []):
                        try:
                            mid = int(t.get("motor_id"))
                            deg = float(t.get("angle_deg"))
                        except (TypeError, ValueError):
                            continue
                        if mid in MOTOR_LIMITS:
                            targets[mid] = clamp_limit(mid, deg)
                    if targets:
                        sub = " ".join(f"M{m}={targets[m]:.1f}" for m in sorted(targets))
                        request_motion([(targets, sub)], f"수동 {sub}",
                                       min_dur=MANUAL_MIN_DURATION)
                        print(f"[CMD] MANUAL_MOVE → {sub}")
                    else:
                        print("[CMD] MANUAL_MOVE: 유효 타깃 없음")
                elif cmd == "SET_JOINT_ANGLE":
                    # 단일 관절(하위호환). 여러 관절은 MANUAL_MOVE 권장.
                    try:
                        mid = int(payload.get("motor_id"))
                        deg = float(payload.get("angle_deg"))
                    except (TypeError, ValueError):
                        mid = None
                    if mid in MOTOR_LIMITS:
                        deg = clamp_limit(mid, deg)
                        request_motion([({mid: deg}, f"M{mid}={deg:.1f}")],
                                       f"수동 M{mid}", min_dur=MANUAL_MIN_DURATION)
                        print(f"[CMD] SET_JOINT_ANGLE → M{mid}={deg:.1f}")
                    else:
                        print("[CMD] SET_JOINT_ANGLE: 잘못된 motor_id")
                elif cmd == "GRIPPER":
                    ang = payload.get("sg90_angle_deg", payload.get("angle_deg"))
                    if ang is not None:
                        request_gripper(ang)
                        print(f"[CMD] GRIPPER → {float(ang):.0f}°")
                    else:
                        print("[CMD] GRIPPER: 각도 없음")
                elif cmd == "PING":
                    print("[CMD] PING (무동작, 수신확인용)")
                else:
                    print("[CMD] unknown:", cmd)
    finally:
        clients.discard(ws)
        print("[WS] disconnect")


async def main():
    threading.Thread(target=can_reader, daemon=True).start()
    threading.Thread(target=motion_worker, daemon=True).start()
    threading.Thread(target=gripper_worker, daemon=True).start()
    print(f"Sambo WS Bridge v4 — ws://0.0.0.0:{WS_PORT}, snapshot {SEND_HZ}Hz, "
          f"motion+manual+gripper+halt enabled")
    async with websockets.serve(ws_handler, WS_HOST, WS_PORT):
        await broadcaster()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nbye")
