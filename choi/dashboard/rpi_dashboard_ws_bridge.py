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
import math
import socket
import struct
import threading
import time
from pathlib import Path
from datetime import datetime

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

# ── PICK 두뇌 (pick_runner → ik_pose → ik) : 같은 폴더에 있어야 import 됨 ──
try:
    import pick_runner
except Exception as e:
    print(f"[PICK] pick_runner disabled: {e}")
    pick_runner = None
try:
    import auto_runner_workflow
except Exception as e:
    print(f"[AUTO] auto_runner_workflow disabled: {e}")
    auto_runner_workflow = None

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
ARRIVE_TOL = 3.0            # 도착 판정 허용오차[deg] — 닫힌루프(open-loop "덜 도착" 보정). 너무 빡세면 정착못해 덜덜
ARRIVE_TIMEOUT = 2.0        # 도착 대기 최대[s] — 초과시 미달 경고 후 진행(토크부족/기계)
ARRIVE_DT = 0.05           # 도착 폴링/목표 재전송 주기[s]

# ── M1 등속(스캔/스윕) + 5005 (ik_server 대체) ──
CMD_SET_ANGLE_SPEED = 0x07   # STS3215 등속(위치+속도) — 틱틱 방지
SCAN_SPEED_REG = 250         # 스캔/센터링 등속 속도(reg)
SWEEP_SPEED = 20.0           # 스윕 M1 회전 속도(°/s). 15는 저속 cog(덜덜) → 20. REG와 동기 필수
SWEEP_SPEED_REG = 230        # 스윕 모터 reg. SWEEP_SPEED와 비례 동기(28↔320 → 20↔230). desync 방지
SWEEP_LEFT, SWEEP_RIGHT = 120.0, 240.0
M1_SCAN_MIN, M1_SCAN_MAX = 120.0, 240.0   # M1 스캔 안전범위(180±60)
IK_PORT = 5005               # detect의 scan/home/sweep/stop 수신

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
tof = {"mm": None, "status": None, "rx": 0.0, "dbg_t": 0.0}   # 그리퍼 ToF(VL53L0X) 파지판정. 나노→CAN 0x300(CAN_프로토콜_정리.md §7)
TOF_DEBUG = True                                             # 보정중 콘솔에 ToF mm 출력 → 보정 끝나면 False
grip_result = {"state": None, "mm": None, "t": 0.0}          # 파지판정 결과(대시보드 tof-judge 표시용)
# LED+부저 Nano 출력 echo(나노→CAN 0x411, cmd 0x21). 실제 LED/부저 출력 미러링용 → 대시보드 cnc.output로 전달.
led_buzzer_output = {"state": 255, "mask": 0, "red": False, "green": False, "blue": False, "buzzer": False, "rack_count": 0, "rack_capacity": 2, "remaining_s": 0, "seq": 0, "rx": 0.0}
# CNC 패널 표시용(옵션B): auto_runner의 set_cnc_busy로 가공 시작/종료를 추적 → 나노 없이 가공중 카운트다운 표시. 모션/CAN 무관.
cnc_track = {"busy": False, "start": 0.0}
CNC_TIMER_S = 20.0   # auto_runner_workflow.CNC_TIMER_S와 동일값 유지(다르면 카운트다운만 어긋남)
rack_track = {"count": 0}   # CNC 패널 "적재 n/2" — auto_runner가 set_rack로 갱신(세션 적재 수, 시작 시 0)

def _set_rack(n):
    try:
        rack_track["count"] = int(n)
    except Exception:
        pass

# ── 작업현황(Work Status) 1일 누적 — 파일 저장, 날짜 바뀔 때만 0(시작/종료/재시작으론 안 바뀜) ──
WORK_STATS_PATH = Path(__file__).resolve().parent / "work_stats.json"

def _today():
    return datetime.now().strftime("%Y-%m-%d")

def _work_load():
    try:
        d = json.loads(WORK_STATS_PATH.read_text(encoding="utf-8"))
    except Exception:
        d = {}
    if d.get("date") != _today():            # 날짜 다르면(새 날/첫 실행) 0부터
        d = {"date": _today(), "done": 0, "sum_s": 0.0, "last": None}
    d.setdefault("done", 0); d.setdefault("sum_s", 0.0); d.setdefault("last", None)
    return d

work_stats = _work_load()                    # 브리지 시작 시 그날 값 복원

def _work_save():
    try:
        WORK_STATS_PATH.write_text(json.dumps(work_stats, ensure_ascii=False), encoding="utf-8")
    except Exception:
        pass

def _work_rollover():                         # 자정 넘어가면 자동 0
    if work_stats.get("date") != _today():
        work_stats.update({"date": _today(), "done": 0, "sum_s": 0.0, "last": None})
        _work_save()

def _report_work(cycle_s=None):               # auto_runner가 적재 완료마다 호출(하루치 +1)
    _work_rollover()
    work_stats["done"] += 1
    if cycle_s is not None:
        work_stats["sum_s"] += float(cycle_s)
    work_stats["last"] = {"cycle_s": cycle_s, "at": datetime.now().strftime("%H:%M")}
    _work_save()

# ── DB 로거(자동운전 중에만 활성; 텔레메트리 log_motor와 공유) ──
try:
    from robot_logger import RobotLogger
except Exception:
    RobotLogger = None
db_log = None

LIFE_USAGE_SCHEMA = "sambo_servo_life_usage_v1"
LIFE_USAGE_PATH = Path(__file__).resolve().parent / "servo_life_usage.json"
LIFE_JOINT_IDS = tuple(NAMES.values())
LIFE_DEFAULT_DESIGN_EQ_CYCLES = 100000.0
LIFE_USAGE_UNIT = "official_life_cycle_120deg"
LIFE_DEG_PER_EQ_CYCLE = 120.0
LIFE_LEGACY_DEG_PER_EQ_CYCLE = 120.0
life_lock = threading.Lock()


def _finite_float(value, default=0.0):
    try:
        v = float(value)
    except (TypeError, ValueError):
        return default
    return v if math.isfinite(v) else default


def _blank_life_usage():
    return {
        "schema": LIFE_USAGE_SCHEMA,
        "usageUnit": LIFE_USAGE_UNIT,
        "degreesPerEquivalentCycle": LIFE_DEG_PER_EQ_CYCLE,
        "designEquivalentCycles": LIFE_DEFAULT_DESIGN_EQ_CYCLES,
        "totalEquivalentCycles": {joint_id: 0.0 for joint_id in LIFE_JOINT_IDS},
        "fatigueAdjustedEquivalentCycles": {joint_id: 0.0 for joint_id in LIFE_JOINT_IDS},
        "healthState": {},
        "updatedAt": None,
        "storedOn": "raspberry_pi",
    }


def _normalize_health_state(payload):
    if not isinstance(payload, dict):
        return {}
    result = {}
    for joint_id in LIFE_JOINT_IDS:
        raw = payload.get(joint_id)
        if not isinstance(raw, dict):
            continue
        stress = _finite_float(raw.get("stressFactor", raw.get("factor", 1.0)), 1.0)
        result[joint_id] = {
            "severity": int(_finite_float(raw.get("severity"), 0.0)),
            "label": str(raw.get("label") or "정상"),
            "stressFactor": max(1.0, min(3.0, stress)),
            "score": _finite_float(raw.get("score"), 100.0),
            "confidence": str(raw.get("confidence") or ""),
            "reasons": raw.get("reasons") if isinstance(raw.get("reasons"), list) else [],
        }
    return result


def _normalize_life_usage(payload):
    data = _blank_life_usage()
    if not isinstance(payload, dict):
        return data

    design = _finite_float(payload.get("designEquivalentCycles"), LIFE_DEFAULT_DESIGN_EQ_CYCLES)
    data["designEquivalentCycles"] = design if design > 0 else LIFE_DEFAULT_DESIGN_EQ_CYCLES
    source_deg = _finite_float(payload.get("degreesPerEquivalentCycle"), LIFE_LEGACY_DEG_PER_EQ_CYCLE)
    scale = source_deg / LIFE_DEG_PER_EQ_CYCLE if source_deg > 0 else 1.0

    totals = payload.get("totalEquivalentCycles") or {}
    if isinstance(totals, dict):
        for joint_id in LIFE_JOINT_IDS:
            value = _finite_float(totals.get(joint_id), 0.0)
            data["totalEquivalentCycles"][joint_id] = max(0.0, value * scale)

    adjusted = payload.get("fatigueAdjustedEquivalentCycles") or {}
    if isinstance(adjusted, dict):
        for joint_id in LIFE_JOINT_IDS:
            fallback = _finite_float((totals if isinstance(totals, dict) else {}).get(joint_id), 0.0)
            value = _finite_float(adjusted.get(joint_id), fallback)
            data["fatigueAdjustedEquivalentCycles"][joint_id] = max(0.0, value * scale)
    else:
        data["fatigueAdjustedEquivalentCycles"] = dict(data["totalEquivalentCycles"])

    health_state = payload.get("healthState")
    if isinstance(health_state, dict):
        data["healthState"] = _normalize_health_state(health_state)

    updated_at = payload.get("updatedAt")
    if isinstance(updated_at, str) and updated_at:
        data["updatedAt"] = updated_at
    return data


def _load_life_usage():
    try:
        if LIFE_USAGE_PATH.exists():
            raw = json.loads(LIFE_USAGE_PATH.read_text(encoding="utf-8"))
            normalized = _normalize_life_usage(raw)
            if (
                raw.get("usageUnit") != LIFE_USAGE_UNIT
                or raw.get("degreesPerEquivalentCycle") != LIFE_DEG_PER_EQ_CYCLE
                or raw.get("healthState") != normalized.get("healthState")
            ):
                tmp_path = LIFE_USAGE_PATH.with_name(LIFE_USAGE_PATH.name + ".tmp")
                tmp_path.write_text(json.dumps(normalized, ensure_ascii=False, indent=2), encoding="utf-8")
                tmp_path.replace(LIFE_USAGE_PATH)
            return normalized
    except Exception as e:
        print("[LIFE] usage load failed:", e)
    return _blank_life_usage()


life_usage = _load_life_usage()


def life_usage_snapshot():
    with life_lock:
        return {
            "schema": life_usage.get("schema", LIFE_USAGE_SCHEMA),
            "usageUnit": life_usage.get("usageUnit", LIFE_USAGE_UNIT),
            "degreesPerEquivalentCycle": life_usage.get("degreesPerEquivalentCycle", LIFE_DEG_PER_EQ_CYCLE),
            "designEquivalentCycles": life_usage.get("designEquivalentCycles", LIFE_DEFAULT_DESIGN_EQ_CYCLES),
            "totalEquivalentCycles": dict(life_usage.get("totalEquivalentCycles", {})),
            "fatigueAdjustedEquivalentCycles": dict(life_usage.get("fatigueAdjustedEquivalentCycles", life_usage.get("totalEquivalentCycles", {}))),
            "healthState": dict(life_usage.get("healthState", {})),
            "updatedAt": life_usage.get("updatedAt"),
            "storedOn": "raspberry_pi",
        }


def save_life_usage(payload):
    global life_usage
    normalized = _normalize_life_usage(payload)
    normalized["updatedAt"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    tmp_path = LIFE_USAGE_PATH.with_name(LIFE_USAGE_PATH.name + ".tmp")
    with life_lock:
        life_usage = normalized
        tmp_path.write_text(json.dumps(life_usage, ensure_ascii=False, indent=2), encoding="utf-8")
        tmp_path.replace(LIFE_USAGE_PATH)
        return {
            "schema": life_usage.get("schema", LIFE_USAGE_SCHEMA),
            "usageUnit": life_usage.get("usageUnit", LIFE_USAGE_UNIT),
            "degreesPerEquivalentCycle": life_usage.get("degreesPerEquivalentCycle", LIFE_DEG_PER_EQ_CYCLE),
            "designEquivalentCycles": life_usage.get("designEquivalentCycles", LIFE_DEFAULT_DESIGN_EQ_CYCLES),
            "totalEquivalentCycles": dict(life_usage.get("totalEquivalentCycles", {})),
            "fatigueAdjustedEquivalentCycles": dict(life_usage.get("fatigueAdjustedEquivalentCycles", life_usage.get("totalEquivalentCycles", {}))),
            "healthState": dict(life_usage.get("healthState", {})),
            "updatedAt": life_usage.get("updatedAt"),
            "storedOn": "raspberry_pi",
        }

# ── 모션 상태 (generation 기반 선점) ──
mlock = threading.Lock()
motion = {"seq": None, "gen": 0, "label": "", "min_dur": MIN_DURATION,
          "speed": AXIS_SPEED, "completed_gen": 0}
_last_cmd = {i: None for i in range(1, 7)}   # 모터별 마지막 "명령값"(=슬라이더로 보낸 값). 티칭 저장은 이걸 씀(deadband 이중적용 방지)

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
                            if db_log is not None:        # 자동운전 중에만 DB 기록(텔레메트리 → motor_sample)
                                try:
                                    db_log.log_motor(mid, cur_angle=m.get("current"), tgt_angle=m.get("target"),
                                                     temp=m.get("temp"), load=m.get("load"), current=m.get("current_ma"))
                                except Exception:
                                    pass
                            if m["cmd_rx"] == 0.0 and m["target"] is None and m["current"] is not None:
                                pass  # 목표각 기본값(=current) 제거: 명령(0x100) 없으면 target 안 보냄
                    elif aid == 0x202 and len(d) >= 4:
                        mid = d[0]
                        m = motors.get(mid)
                        if m:
                            if d[3]:
                                m["current_ma"] = round(((d[1] | (d[2] << 8)) & 0x7FFF) * 6.5)
                            m["rx"] = now
                    elif aid == 0x300 and len(d) >= 3:
                        # 그리퍼 ToF(VL53L0X, 나노→0x300): d[0..1]=mm(LE), d[2]=status(0=OK/1=범위초과/2=센서오류)
                        if d[2] != 2:   # 센서오류(2)만 버림. 0=정상, 1=범위초과(빈 그리퍼=유효정보)
                            tof["mm"] = d[0] | (d[1] << 8)
                            tof["status"] = d[2]
                            tof["rx"] = now
                            if TOF_DEBUG and (now - tof["dbg_t"]) > 0.7:
                                tof["dbg_t"] = now
                                print(f"[ToF] {tof['mm']} mm (st={d[2]})")
                    elif aid == 0x411 and len(d) >= 3 and d[0] == 0x21:
                        # LED+부저 Nano 출력 echo: d[1]=state, d[2]=output_mask(빨0x01/초0x02/파0x04/부저0x08)
                        mask = int(d[2])
                        led_buzzer_output.update({
                            "state": int(d[1]),
                            "mask": mask,
                            "red": bool(mask & 0x01),
                            "green": bool(mask & 0x02),
                            "blue": bool(mask & 0x04),
                            "buzzer": bool(mask & 0x08),
                            "rack_count": int(d[3]) if len(d) >= 4 else 0,
                            "rack_capacity": int(d[4]) if len(d) >= 5 else 2,
                            "remaining_s": int(d[5]) if len(d) >= 6 else 0,
                            "seq": int(d[6]) if len(d) >= 7 else 0,
                            "rx": now,
                        })
                    elif aid == 0x100 and len(d) >= 4 and d[0] == 0x03:
                        mid = d[1]
                        m = motors.get(mid)
                        if m:
                            m["target"] = ((d[3] << 8) | d[2]) / 10.0
                            m["cmd_rx"] = now
                    elif aid == 0x100 and len(d) >= 2 and d[0] == CMD_GRIPPER:
                        gripper_state["angle"] = int(max(GRIPPER_MIN, min(GRIPPER_MAX, d[1])))
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
    # 확실한 S-curve = 5차 smootherstep: 6t^5 - 15t^4 + 10t^3.
    # 양 끝에서 속도=0 AND 가속도=0 → 저크 스파이크 없음(3차 smoothstep보다 시작/끝이 더 매끈).
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0)


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


def request_motion(seq, label, min_dur=MIN_DURATION, speed=AXIS_SPEED, compensate=True):
    """seq: [(targets_dict, sub_label), ...]. 새 요청은 진행 중 이동을 선점.
    speed: deg/s (수동 기본 20, pick 30). M1 스윕 중이면 먼저 정지(0x03↔0x07 충돌 방지).
    compensate: deadband 보상(도착확인) 적용 여부. 고정자세(스캔/원점)·PICK은 False(움찔/오버슈트 방지)."""
    stop_sweep()
    with mlock:
        motion["seq"] = seq
        motion["gen"] += 1
        motion["label"] = label
        motion["min_dur"] = min_dur
        motion["speed"] = speed
        motion["compensate"] = compensate
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


def _move_to(bus, targets, my_gen, min_dur=MIN_DURATION, speed=AXIS_SPEED, compensate=True):
    with lock:
        seeds = {m: (motors[m]["current"] if motors[m]["current"] is not None else 180.0)
                 for m in targets}
    maxd = max(abs(targets[m] - seeds[m]) for m in targets)
    dur = max(maxd / speed, min_dur)
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
    with lock:                                      # ★마지막 "명령값" 기록 — 티칭 저장은 telemetry(실제) 대신 이걸 씀.
        for m in targets:                           #   조그=슬라이더 명령값 그대로 저장 → 재명령시 deadband 한 번만 먹혀 일관(볼트너트 방식).
            _last_cmd[m] = targets[m]
    return True                                     # (보상/도착확인 제거: deadband는 명령값 저장으로 흡수, 움찔·오버슈트 없음)


def motion_worker():
    bus = None
    while True:
        with mlock:
            seq = motion["seq"]; gen = motion["gen"]; label = motion["label"]
            min_dur = motion["min_dur"]; speed = motion["speed"]
            compensate = motion.get("compensate", True)
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
            if not _move_to(bus, targets, gen, min_dur, speed, compensate):
                print(f"[MOVE]   preempted at {sub}")
                break
        else:
            with mlock:
                motion["completed_gen"] = gen     # submit_and_wait가 done 판정
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


# ── PICK (단일 잡기) : pick_runner 엔진 (계약 B) ───────────────────────────
# 대시보드 S/PICK → 게이트 통과 → 스레드로 pick_runner.run(api). auto_runner(자동루프)는 안 엮음.
PICK_CONF_TH = 0.5         # 신뢰도 임계
PICK_STABLE_N = 5          # 연속 안정 프레임 임계
PICK_TTL_MS = 2500         # alive 판정(age_ms < TTL)
PICK_SPEED = 30.0          # 잡기 속도(수동 20과 분리, M4 발열↓)
PICK_TIMEOUT_S = 30.0      # submit_and_wait 한 phase 최대 대기(무응답 안전망)

pick_state = {"status": "SCAN_READY", "step": ""}   # SCAN_READY | BUSY
pick_lock = threading.Lock()
run_flags = {"estop": False, "stop_after_cycle": False}   # E-Stop 래치 / 종료(사이클 후 원점)

# ── 안전: 온도/부하 감시 (온도→graceful 종료 / 부하→즉시 정지+홀드) ──
TEMP_LIMIT = 58.0          # °C — 초과시 현재작업 끝내고 원점(graceful 종료)
TEMP_SUSTAIN_N = 10        # 온도 노이즈 스파이크 무시: 연속 N회(×0.1s=1s) 초과해야 정지
LOAD_EACH_LIMIT = 95.0     # % — 한 축이라도 지속 초과시 즉시정지(박힘/고장). ★정상 peak 관찰 후 튜닝
LOAD_TOTAL_LIMIT = 300.0   # % — 6축 합 지속 초과시
LOAD_SUSTAIN_S = 0.5       # 부하 초과가 이만큼 지속돼야 발동(정상 이동 순간 스파이크 무시)

# ── 대시보드 이벤트 로그 알림 버퍼(워커 스레드 → 스냅샷이 d.events로 1회씩 전달) ──
_events_lock = threading.Lock()
pending_events = []        # [{message, level}]

def _push_event(message, level="info"):
    with _events_lock:
        pending_events.append({"message": str(message), "level": level})
        if len(pending_events) > 30:
            del pending_events[:-30]

def _drain_events():
    with _events_lock:
        evs = pending_events[:]
        pending_events.clear()
        return evs


def safety_monitor():
    """모터 온도/부하 감시. 온도≥임계→graceful 종료(stop_after_cycle). 부하≥임계(지속)→즉시정지+현재각 홀드(사람 확인 후 END)."""
    temp_tripped = False
    temp_hot_count = 0
    load_over_since = None
    load_tripped = False
    while True:
        time.sleep(0.1)
        with lock:
            temps = [motors[m]["temp"] for m in range(1, 7)]
            loads = [motors[m]["load"] for m in range(1, 7)]
            curs = {m: motors[m]["current"] for m in range(1, 7)}
        # 온도 → graceful 종료(현재작업 끝→원점)
        hot = [(i + 1, t) for i, t in enumerate(temps) if t is not None and t >= TEMP_LIMIT]
        if hot:
            temp_hot_count += 1                      # ★연속 카운트 — 스파이크 1~2회는 무시
            if temp_hot_count >= TEMP_SUSTAIN_N and not temp_tripped:
                temp_tripped = True
                with pick_lock:
                    run_flags["stop_after_cycle"] = True
                print(f"[SAFETY] ⚠온도 {hot} ≥{TEMP_LIMIT}°C {TEMP_SUSTAIN_N}회 연속 → 현재작업 끝나고 원점(종료)")
                _push_event(f"⚠과열 {max(t for _, t in hot):.0f}°C (M{[m for m, _ in hot]}) ≥{TEMP_LIMIT:.0f}°C — 현재작업 끝내고 원점(종료)", "err")
        else:
            temp_hot_count = 0                       # 한 번이라도 정상이면 카운트 리셋(연속만 인정)
            temp_tripped = False
        # 부하 → 즉시 정지+홀드 (지속 확인 — 순간 스파이크 무시)
        valid = [x for x in loads if x is not None]
        over_each = [(i + 1, l) for i, l in enumerate(loads) if l is not None and l >= LOAD_EACH_LIMIT]
        total = sum(valid)
        if over_each or total >= LOAD_TOTAL_LIMIT:
            if load_over_since is None:
                load_over_since = time.time()
            elif (time.time() - load_over_since) >= LOAD_SUSTAIN_S and not load_tripped:
                load_tripped = True
                hold = {m: curs[m] for m in range(1, 7) if curs[m] is not None}
                if hold:
                    request_motion([(hold, "부하정지 홀드")], "SAFETY 부하 정지")   # 그 자리 정지+홀드(후퇴 안 함)
                with pick_lock:
                    run_flags["stop_after_cycle"] = True
                print(f"[SAFETY] ⚠부하 초과(각:{over_each} 합:{total:.0f}%) {LOAD_SUSTAIN_S}s 지속 → 즉시 정지+홀드. 확인 후 END로 풀거나 해결.")
                _push_event(f"⚠부하 초과 (M{[m for m, _ in over_each]} 합 {total:.0f}%) — 즉시 정지+홀드. 확인 후 END", "err")
        else:
            load_over_since = None
            load_tripped = False
BUSY_FLAG = Path(__file__).resolve().parent / "robot_busy.flag"   # 있으면 detect가 인식 정지
CNC_FLAG = Path(__file__).resolve().parent / "cnc_busy.flag"      # 있으면 detect가 파이프 제외(가공중)
RUN_FLAG = Path(__file__).resolve().parent / "run.flag"          # 있으면 detect가 자동탐색(sweep+센터링)
GRAB_DATA = Path(__file__).resolve().parent / "grab_data.jsonl"  # 잡기마다 어깨부하 프로파일 기록(파지실패 분석용)
TEACH_FILE = Path(__file__).resolve().parent / "teach_poses.json"  # 티칭 포즈 영속(수동모드→TEACH_SAVE→pick_runner가 읽음)
TEACH_NAMES = ("cnc_infeed", "cnc_grab", "rack_v0", "rack_v1")


def _set_busy(b):
    """잡기/이동 중 깃발. detect가 이 파일이 있으면 인식·기록을 멈춤(스캔 정지 상태에서만 인식)."""
    try:
        if b:
            BUSY_FLAG.write_text("1")
        else:
            BUSY_FLAG.unlink(missing_ok=True)
    except Exception:
        pass


def _set_cnc_busy(b):
    """파이프 CNC 가공 중 깃발. detect가 있으면 파이프 제외(볼트/너트만). (계약 7c)"""
    cnc_track["busy"] = bool(b)              # 옵션B: CNC 패널 카운트다운용(나노 없이 가공상태 표시). 모션 무관.
    if b:
        cnc_track["start"] = time.monotonic()
    try:
        if b:
            CNC_FLAG.write_text("1")
        else:
            CNC_FLAG.unlink(missing_ok=True)
    except Exception:
        pass


def _set_run(b):
    """탐색 신호. detect가 run.flag 있으면 자동 sweep+센터링(스스로 M1 몰음)."""
    try:
        if b:
            RUN_FLAG.write_text("1")
        else:
            RUN_FLAG.unlink(missing_ok=True)
    except Exception:
        pass


def _teach_read_doc():
    try:
        return json.loads(TEACH_FILE.read_text(encoding="utf-8"))
    except Exception:
        return {"version": 1, "poses": {}}


def _teach_save(name):
    """마지막 "명령값"(슬라이더로 보낸 값)을 name 포즈로 저장. telemetry(실제) 대신 명령값 → deadband 한 번만 먹혀 재현 일관.
    (명령 없는 모터는 telemetry 폴백)"""
    if name not in TEACH_NAMES:
        return None
    with lock:
        angles = {str(m): (_last_cmd[m] if _last_cmd[m] is not None else motors[m]["current"])
                  for m in range(1, 7)}
    if any(v is None for v in angles.values()):
        return None                                    # 명령·telemetry 둘 다 없음 → 저장 안 함
    doc = _teach_read_doc()
    doc.setdefault("version", 1)
    doc.setdefault("poses", {})
    doc["poses"][name] = {"angles": angles, "saved_at": time.strftime("%Y-%m-%d %H:%M")}
    try:
        TEACH_FILE.write_text(json.dumps(doc, ensure_ascii=False, indent=2), encoding="utf-8")
    except Exception:
        return None
    return doc["poses"][name]


def _teach_list():
    saved = _teach_read_doc().get("poses") or {}
    taught = [n for n in TEACH_NAMES if saved.get(n)]
    missing = [n for n in TEACH_NAMES if not saved.get(n)]
    poses = {}                                         # 이름→각도(평탄) — 새로고침 후 표시·[이동] 복원용
    for n in taught:
        e = saved.get(n)
        a = e.get("angles") if isinstance(e, dict) else None
        if a:
            poses[n] = a
    return taught, missing, poses


def _label_last_grab(success):
    """방금 잡기의 성공/실패 라벨을 grab_data.jsonl 마지막 줄 success에 채움(수동 표시)."""
    try:
        lines = GRAB_DATA.read_text(encoding="utf-8").splitlines()
        if not lines:
            return
        rec = json.loads(lines[-1])
        rec["success"] = bool(success)
        lines[-1] = json.dumps(rec, ensure_ascii=False)
        GRAB_DATA.write_text("\n".join(lines) + "\n", encoding="utf-8")
        print(f"[GRAB_LABEL] 마지막 잡기 success={bool(success)}")
    except Exception as e:
        print(f"[GRAB_LABEL] 실패: {e}")


def get_gen():
    with mlock:
        return motion["gen"]


def get_vision():
    return load_latest_runtime_state().get("vision", {})


def submit_and_wait(seq, speed=PICK_SPEED, min_dur=MIN_DURATION, timeout=PICK_TIMEOUT_S):
    """seq 제출 후 완료까지 대기 → "done" | "preempted"(HALT 등) | "timeout"(무응답).
    내부에서 request_motion으로 gen 생성, motion_worker의 completed_gen으로 done 판정."""
    with pick_lock:
        if run_flags["estop"]:
            return "estop"
    my_gen = request_motion(seq, "pick", min_dur=min_dur, speed=speed)
    deadline = time.time() + timeout
    while time.time() < deadline:
        with mlock:
            g = motion["gen"]; done = motion["completed_gen"]
        if g != my_gen:
            return "preempted"      # 더 새 요청(HALT 등)이 선점
        if done == my_gen:
            return "done"
        time.sleep(0.01)
    return "timeout"


def read_tof():
    """라이브 ToF 거리(mm). 미수신/오래됨(>1s)이면 None → 파지판정 보류."""
    if tof["rx"] and (time.monotonic() - tof["rx"]) < 1.0:
        return tof["mm"]
    return None


def set_grip_result(state, mm=None):
    """pick_runner 파지판정 결과 보고 → 스냅샷으로 대시보드(tof-judge)에 표시."""
    with lock:
        grip_result["state"] = state          # "성공" | "실패"
        grip_result["mm"] = mm
        grip_result["t"] = time.monotonic()


def should_stop():
    """auto_runner 루프 정지 신호 (END/AUTO_STOP→stop_after_cycle, ESTOP→estop)."""
    with pick_lock:
        return run_flags["estop"] or run_flags["stop_after_cycle"]


def set_step(stage):
    with pick_lock:
        pick_state["step"] = stage


class _PickApi:
    """pick_runner/auto_runner에 주입할 DI 객체 (모듈이 브리지를 import 안 하게)."""
    submit_and_wait = staticmethod(submit_and_wait)
    request_gripper = staticmethod(request_gripper)
    get_gen = staticmethod(get_gen)
    set_cnc_busy = staticmethod(_set_cnc_busy)
    set_busy = staticmethod(_set_busy)
    read_tof = staticmethod(read_tof)
    set_grip_result = staticmethod(set_grip_result)
    should_stop = staticmethod(should_stop)
    get_vision = staticmethod(get_vision)
    set_step = staticmethod(set_step)
    report_work = staticmethod(_report_work)   # 작업현황(적재 완료마다 하루치 +1)
    set_rack = staticmethod(_set_rack)         # CNC 패널 적재 n/2(세션 적재 수)


_pick_api = _PickApi()


# ── 자동 워크플로우(auto_runner) 스레드 제어 ──
_auto_thread = None


def _auto_alive():
    return _auto_thread is not None and _auto_thread.is_alive()


def _start_auto():
    """AUTO_START → auto_runner 워크플로우 루프 시작. ENABLED=False면 run()이 즉시 반환(no-op)."""
    global _auto_thread
    if auto_runner_workflow is None:
        print("[AUTO] auto_runner 모듈 없음"); return
    if _auto_alive():
        print("[AUTO] 이미 실행 중"); return
    with pick_lock:
        run_flags["stop_after_cycle"] = False
        run_flags["estop"] = False

    def _worker():
        global db_log
        try:
            _set_busy(True)       # ★스캔 도착 전 detect 정지(이동중 스테일 비전으로 확 튀는 것 방지). run()이 이후 관리
            if submit_and_wait([(SCAN_POSE, "START scan")], speed=25.0) != "done":
                _set_busy(False); print("[AUTO] scan not reached -> hold"); return
            _set_run(True)        # detect 스윕/센터링 ON (루프가 vision 받게)
            with pick_lock:
                pick_state["status"] = "BUSY"   # ★진행바 게이트: 자동 중 단계바 표시(deriveCurrentStep은 status==BUSY일 때만 step 그림)
            if RobotLogger is not None:
                try:
                    db_log = RobotLogger(mode_default="auto")   # ★DB 로깅 시작(텔레메트리 log_motor도 이 인스턴스 공유)
                    print("[AUTO] DB 로깅 시작")
                except Exception as e:
                    db_log = None; print(f"[AUTO] DB 로깅 비활성: {e}")
            res = auto_runner_workflow.run(_pick_api, log=db_log)
            print(f"[AUTO] 워크플로우 종료: {res}")
            _set_run(False)       # ★복귀 모션 전 detect 스윕 정지 → 복귀 중 M1 0x07/0x03 충돌(반대 툭) 방지
            # 종료(END)/AUTO_STOP으로 멈췄고 E-STOP 아니면 → 사이클 마친 뒤 스캔→원점 복귀
            with pick_lock:
                go_home = run_flags["stop_after_cycle"] and not run_flags["estop"]
                run_flags["stop_after_cycle"] = False
            if go_home:
                request_motion([({1: SCAN_POSE[1]}, "M1 복귀"),
                                (SCAN_POSE, "스캔복귀"), (HOME_POSE, "원점")], "END → 원점", compensate=False)
                print("[AUTO] 종료 → 스캔→원점 복귀")
        except Exception as e:
            print(f"[AUTO] 워크플로우 오류: {e}")
        finally:
            _set_run(False)       # detect OFF
            _set_cnc_busy(False)  # ★중단/종료 시 cnc 플래그 해제 → stuck으로 파이프 영구 제외되는 것 방지
            if db_log is not None:
                try: db_log.close()       # auto_runner.run이 end_session 처리 → 여기선 flush+conn 닫기
                except Exception: pass
                db_log = None
            with pick_lock:
                pick_state["status"] = "SCAN_READY"   # ★자동 종료 → 진행바 HOME/대기 복귀
                pick_state["step"] = ""
    _auto_thread = threading.Thread(target=_worker, daemon=True)
    _auto_thread.start()


def _pick_gate(v):
    """잡기 실행 조건: alive · 물체있음 · 신뢰도 · 안정프레임 · 좌표유효."""
    return (v.get("age_ms", 9e9) < PICK_TTL_MS
            and v.get("target", "NONE") != "NONE"
            and v.get("confidence", 0.0) >= PICK_CONF_TH
            and v.get("stable_frames", 0) >= PICK_STABLE_N
            and v.get("x_mm") is not None and v.get("y_mm") is not None)


def _pick_worker(v):
    """게이트 통과한 vision 스냅샷으로 한 사이클 실행(별도 스레드)."""
    def on_step(stage):
        with pick_lock:
            pick_state["step"] = stage
    # ── 파지 데이터 수집: 어깨 M2·M3 부하/전류를 0.1초마다 기록(잡기 성공/실패 분석용) ──
    _samples = []
    _samp_stop = threading.Event()

    def _sampler():
        while not _samp_stop.is_set():
            with lock:                        # 6축 전부 load+current (어느 채널이 갈리는지 나중에 분석)
                row = ([round(time.time(), 2)]
                       + [motors[m]["load"] for m in range(1, 7)]
                       + [motors[m]["current_ma"] for m in range(1, 7)])
            with pick_lock:
                row.append(pick_state["step"])
            _samples.append(row)
            time.sleep(0.1)

    _samp_t = threading.Thread(target=_sampler, daemon=True)
    _samp_t.start()
    result = ("error", "unknown")
    try:
        result = pick_runner.run(
            _pick_api, [v["x_mm"], v["y_mm"]], v.get("target", "NONE"),
            angle_deg=v.get("angle_deg", 0.0),
            near=v.get("near"), distance_mm=v.get("distance_mm"),
            on_step=on_step)
        print(f"[PICK] result: {result}")
    except Exception as e:
        result = ("error", str(e))
        print(f"[PICK] error: {e}")
    finally:
        _samp_stop.set(); _samp_t.join(timeout=0.5)
        try:                              # 잡기 1회 = 1줄 (어깨부하 프로파일). success는 나중 라벨
            with GRAB_DATA.open("a", encoding="utf-8") as _f:
                _f.write(json.dumps({
                    "ts": round(time.time(), 1), "target": v.get("target"),
                    "x_mm": v.get("x_mm"), "y_mm": v.get("y_mm"),
                    "result": result[0], "success": None,
                    "cols": (["t"] + [f"m{m}_load" for m in range(1, 7)]
                             + [f"m{m}_ma" for m in range(1, 7)] + ["step"]),
                    "samples": _samples,
                }, ensure_ascii=False) + "\n")
        except Exception as _e:
            print(f"[GRAB_DATA] 기록 실패: {_e}")
        _set_busy(False)                  # detect 인식 재개(스캔)
        with pick_lock:
            pick_state["status"] = "SCAN_READY"
            pick_state["step"] = ""
            end = run_flags["stop_after_cycle"]; es = run_flags["estop"]
            if end:
                run_flags["stop_after_cycle"] = False
    # 종료(END) 요청 + 정상 완료 + E-Stop 아님 → 원점 복귀
    if end and not es and result[0] == "done":
        request_motion([(HOME_POSE, "원점")], "END → 원점", compensate=False)
        print("[PICK] END → 원점 복귀")


def handle_pick():
    """대시보드 PICK/S 명령 처리. 게이트 통과 시 스레드로 한 사이클 실행."""
    if pick_runner is None:
        print("[PICK] pick_runner 미탑재"); return "no_module"
    v = get_vision()
    with pick_lock:
        if pick_state["status"] != "SCAN_READY":
            print("[PICK] BUSY — 무시"); return "busy"
        if not _pick_gate(v):
            print("[PICK] 게이트 실패 "
                  f"target={v.get('target')} conf={v.get('confidence')} "
                  f"stable={v.get('stable_frames')} age={v.get('age_ms')}")
            return "gate_fail"
        pick_state["status"] = "BUSY"
        pick_state["step"] = "PRE_GRASP"
    _set_busy(True)                       # detect 인식 정지(이동 중)
    threading.Thread(target=_pick_worker, args=(v,), daemon=True).start()
    print(f"[PICK] start: {v.get('target')} @ ({v.get('x_mm')},{v.get('y_mm')})")
    return "started"


def _pick_snapshot():
    with pick_lock:
        return {"status": pick_state["status"], "step": pick_state["step"]}


# ── M1 스캔/센터링 등속 + 스윕 + 5005 리스너 (ik_server_pose 대체) ──────────
_m1_bus = None
_sweep_stop = threading.Event()
_sweep_thread = None
_sweep_m1 = 180.0


def _m1_can():
    global _m1_bus
    if _m1_bus is None:
        _m1_bus = can.interface.Bus(channel=CAN_INTERFACE, interface="socketcan")
    return _m1_bus


def _send_angle_speed(mid, deg, speed):
    """위치+Goal_Speed(0x07)로 STS3215 등속 회전(위치명령 최고속 튐 방지)."""
    deg = max(0.0, min(360.0, deg))
    ax10 = int(round(deg * 10)); sp = int(speed) & 0xFFFF
    data = [CMD_SET_ANGLE_SPEED, mid, ax10 & 0xFF, (ax10 >> 8) & 0xFF,
            sp & 0xFF, (sp >> 8) & 0xFF, 0, 0]
    try:
        _m1_can().send(can.Message(arbitration_id=0x100, data=data, is_extended_id=False))
    except Exception:
        pass


def rotate_m1(target_deg, speed=SCAN_SPEED_REG):
    """M1을 등속으로 target까지(스캔/센터링). 안전범위 클램프."""
    t = max(M1_SCAN_MIN, min(M1_SCAN_MAX, float(target_deg)))
    _send_angle_speed(1, t, speed)
    return t


def _glide(start, goal):
    """start→goal 등속 한 방 + 시간추정. stop 시 현재 추정각에서 정지."""
    global _sweep_m1
    dist = abs(goal - start)
    if dist < 0.5:
        _sweep_m1 = goal; return goal
    dur = dist / SWEEP_SPEED
    sign = 1.0 if goal > start else -1.0
    _send_angle_speed(1, goal, SWEEP_SPEED_REG)
    t0 = time.time()
    while not _sweep_stop.is_set():
        el = time.time() - t0
        if el >= dur:
            _sweep_m1 = goal; return goal
        _sweep_m1 = start + sign * SWEEP_SPEED * el
        time.sleep(0.03)
    cur = start + sign * SWEEP_SPEED * (time.time() - t0)
    cur = min(max(cur, min(start, goal)), max(start, goal))
    _sweep_m1 = cur
    _send_angle_speed(1, cur, SWEEP_SPEED_REG)
    return cur


def _sweep_worker():
    # 좌→우 1회 훑고 스캔(180) 복귀 후 종료 (무한반복 X — detect가 못 찾으면 스캔 대기)
    _glide(_sweep_m1, SWEEP_LEFT)
    if _sweep_stop.is_set():
        return
    _glide(_sweep_m1, SWEEP_RIGHT)
    if _sweep_stop.is_set():
        return
    _glide(_sweep_m1, 180.0)


def start_sweep():
    global _sweep_thread, _sweep_m1
    stop_sweep()
    with lock:
        a = motors[1]["current"]
    if a is not None:
        _sweep_m1 = a
    _sweep_stop.clear()
    _sweep_thread = threading.Thread(target=_sweep_worker, daemon=True)
    _sweep_thread.start()


def stop_sweep():
    global _sweep_thread
    _sweep_stop.set()
    if _sweep_thread is not None:
        _sweep_thread.join(timeout=1.0)
        _sweep_thread = None
    return _sweep_m1


def ik5005_server():
    """detect의 M1 스캔/홈/스윕/정지 명령(5005) 수신 → 실행. ik_server_pose 대체.
    프로토콜 그대로라 detect 변경 없음. 잡기(pick)는 WS PICK로 처리하므로 여기선 거부."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        srv.bind(("0.0.0.0", IK_PORT)); srv.listen(1)
    except OSError as e:
        print(f"[5005] bind 실패: {e}")
        return
    print(f"[5005] ik 명령 수신 대기 (scan/home/sweep/stop) — ik_server 대체")
    while True:
        try:
            conn, _addr = srv.accept()
        except Exception:
            continue
        with conn:
            try:
                buf = conn.recv(4096)
                if not buf:
                    continue
                req = json.loads(buf.decode())
            except Exception:
                try:
                    conn.sendall(b'{"ok":false,"err":"bad json"}')
                except Exception:
                    pass
                continue
            act = req.get("action")
            with pick_lock:
                _es = run_flags["estop"]
            if _es and act in ("home", "scan", "sweep"):
                try:
                    conn.sendall(json.dumps({"ok": False, "err": "E-STOP"}).encode())
                except Exception:
                    pass
                continue
            try:
                if act == "home":
                    conn.sendall(json.dumps({"ok": True, "action": "home"}).encode())
                    request_motion([(HOME_POSE, "원점(5005 home)")], "5005 HOME", compensate=False)
                elif act == "scan":
                    m1 = rotate_m1(req.get("m1", 180.0))
                    conn.sendall(json.dumps({"ok": True, "m1": round(m1, 1)}).encode())
                elif act == "sweep":
                    if not RUN_FLAG.exists():    # ★run.flag OFF면 거부 — END/STOP 뒤 도착한 detect 스윕이 원점이동 중 재시작하던 레이스 방지
                        conn.sendall(json.dumps({"ok": False, "err": "run.flag OFF"}).encode())
                    else:
                        start_sweep()
                        conn.sendall(json.dumps({"ok": True, "sweeping": True}).encode())
                elif act == "stop":
                    est = stop_sweep()
                    with lock:
                        _rm1 = motors[1]["current"]      # ★실제 M1(telemetry) 우선 — 시간추정값은 desync→위치오차 수십mm 증폭
                    m1 = _rm1 if _rm1 is not None else est
                    conn.sendall(json.dumps({"ok": True, "m1": round(m1, 1)}).encode())
                else:
                    conn.sendall(json.dumps({"ok": False, "err": "pick은 WS PICK 사용"}).encode())
            except Exception as e:
                print(f"[5005] {act} 처리 오류: {e}")


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
        gripper["state"] = "OPEN" if ga < (GRIPPER_MIN + GRIPPER_MAX) / 2 else "CLOSED"
        gripper["source"] = "dashboard_bridge_cmd"
        gripper["fresh"] = True
        gripper["reason"] = "bridge_command"
    if tof["rx"] and (now - tof["rx"]) < 2.0:
        gripper["tof_mm"] = tof["mm"]   # 대시보드 ToF 거리(라이브 0x300) → gripper.tof_mm로 표시
        gripper["tof_fresh"] = True
    else:
        gripper["tof_mm"] = None         # ★2초+ 끊김: 신선값 없음 → 대시보드 "신호없음"(50.2 고정 방지). 0x300 다시 오면 자동 복귀
        gripper["tof_fresh"] = False     # (팀원 추가분 반영)
    if grip_result["state"] and (now - grip_result["t"]) < 10.0:
        gripper["grip_result"] = grip_result["state"]   # 파지 성공/실패(판정 후 10s 표시)
    with pick_lock:
        es = run_flags["estop"]
    system = {"server_connected": True, "can_status": derive_can_status(comms), "estop": es,
              "auto_running": _auto_alive()}   # 대시보드 START 가드 동기화용(워크플로우 자동 종료 시 false)
    if vision.get("fresh"):
        system.update({"camera_status": "OK", "ai_status": "RUNNING"})
    # LED+부저 Nano echo(0x411) → cnc.output. fresh(<=800ms)면 대시보드가 실제 출력 미러링.
    led_output = dict(led_buzzer_output)
    if led_output.get("rx"):
        led_output["age_ms"] = int((now - led_output["rx"]) * 1000)
        led_output["fresh"] = led_output["age_ms"] <= 800
    else:
        led_output["age_ms"] = None
        led_output["fresh"] = False
    led_output.pop("rx", None)
    cnc = {"output": led_output}
    # 옵션B: 나노 없이도 CNC 가공상태 표시(가공중 카운트다운). 나노 echo 신선하면 아래 fresh 블록이 덮음(output 우선).
    if not cnc_track["busy"]:
        _cnc_state, _rem = "idle", 0.0
    else:
        _ela = now - cnc_track["start"]
        _cnc_state, _rem = ("running", round(CNC_TIMER_S - _ela, 1)) if _ela < CNC_TIMER_S else ("done", 0.0)
    cnc.update({"state": _cnc_state, "remaining_s": _rem, "total_s": CNC_TIMER_S, "rack_count": rack_track["count"], "rack_max": 2})
    if led_output.get("fresh"):
        cnc.update({"rack_count": led_output.get("rack_count", 0), "rack_max": led_output.get("rack_capacity", 2), "remaining_s": led_output.get("remaining_s", 0)})
    # 작업현황(하루 누적): done/평균/직전 사이클
    _work_rollover()
    _wdone = work_stats["done"]
    _wlast = work_stats.get("last")
    work = {"done": _wdone,
            "avg_s": round(work_stats["sum_s"] / _wdone, 1) if _wdone else None,
            "cycle_s": (_wlast or {}).get("cycle_s"), "last": _wlast}
    return {"robot": {"joints": joints},
            "system": system,
            "vision": vision,
            "gripper": gripper,
            "pick": _pick_snapshot(),
            "cnc": cnc,
            "work": work,
            "events": _drain_events(),
            "life_usage": life_usage_snapshot(),
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
            msg_type = d.get("type")
            if msg_type == "LIFE_USAGE_GET":
                await ws.send(json.dumps({"type": "LIFE_USAGE_STATE", "life_usage": life_usage_snapshot()},
                                         ensure_ascii=False))
                continue
            if msg_type == "LIFE_USAGE_SAVE":
                try:
                    saved = save_life_usage(d.get("payload") or {})
                    await ws.send(json.dumps({"type": "LIFE_USAGE_SAVED", "life_usage": saved},
                                             ensure_ascii=False))
                    print("[LIFE] usage saved")
                except Exception as e:
                    await ws.send(json.dumps({"type": "LIFE_USAGE_ERROR", "message": str(e)},
                                             ensure_ascii=False))
                    print("[LIFE] usage save failed:", e)
                continue
            if msg_type == "TEACH_SAVE":
                name = d.get("name")
                if BUSY_FLAG.exists():
                    await ws.send(json.dumps({"type": "TEACH_ERROR", "name": name,
                                              "message": "잡기/이동 중 — 수동모드에서 저장하세요"}, ensure_ascii=False))
                    continue
                saved = _teach_save(name)
                if saved is None:
                    await ws.send(json.dumps({"type": "TEACH_ERROR", "name": name,
                                              "message": "저장 실패(이름 오류 또는 각도 미수신)"}, ensure_ascii=False))
                    print(f"[TEACH] 저장 실패: {name}")
                else:
                    await ws.send(json.dumps({"type": "TEACH_SAVED", "name": name,
                                              "angles": saved["angles"], "saved_at": saved["saved_at"]},
                                             ensure_ascii=False))
                    print(f"[TEACH] {name} 저장: {saved['angles']}")
                continue
            if msg_type == "TEACH_LIST":
                taught, missing, poses = _teach_list()
                await ws.send(json.dumps({"type": "TEACH_STATE", "taught": taught,
                                          "missing": missing, "poses": poses},
                                         ensure_ascii=False))
                continue
            if msg_type == "TEACH_GOTO":
                name = d.get("name")
                with pick_lock:
                    _es = run_flags["estop"]
                e = (_teach_read_doc().get("poses") or {}).get(name)
                pose = e.get("angles") if isinstance(e, dict) else None
                if _es:
                    await ws.send(json.dumps({"type": "TEACH_ERROR", "name": name, "message": "E-STOP 중"}, ensure_ascii=False))
                elif BUSY_FLAG.exists():
                    await ws.send(json.dumps({"type": "TEACH_ERROR", "name": name, "message": "잡기/이동 중"}, ensure_ascii=False))
                elif not pose:
                    await ws.send(json.dumps({"type": "TEACH_ERROR", "name": name, "message": "미티칭 자세"}, ensure_ascii=False))
                else:
                    p = {int(k): float(v) for k, v in pose.items()}
                    request_motion([({1: p[1]}, "이동 M1"),                              # ★M1 먼저
                                    ({m: p[m] for m in (2, 3, 4, 5, 6)}, "이동 M2~6")],  # 나머지 한번에 (동시이동 덜덜 방지)
                                   "TEACH_GOTO %s" % name)
                    await ws.send(json.dumps({"type": "TEACH_MOVED", "name": name}, ensure_ascii=False))
                    print(f"[TEACH] 이동 → {name}")
                continue
            if msg_type == "COMMAND":
                cmd = d.get("command")
                payload = d.get("payload") or {}
                with pick_lock:
                    _es = run_flags["estop"]
                # E-Stop 중: 로봇을 움직일 수 있는 자동/모션 명령 전부 차단(START·STOP·END·PICK·SWEEP).
                # → 수동(조그·관절·그리퍼·HALT)·ESTOP_RESET만 허용. 손으로 정리 후 RESET으로 풀고 재개.
                _AUTO_CMDS = ("AUTO_START", "AUTO_STOP", "END", "PICK", "S", "VISION_SEND", "SWEEP")
                if _es and cmd in _AUTO_CMDS:
                    print(f"[CMD] {cmd} 거부 (E-STOP 중 — 자동 정지. 수동은 가능, ESTOP_RESET 으로 재개)")
                    continue
                if cmd == "AUTO_START":
                    request_motion([(SCAN_POSE, "스캔자세")], "START → 스캔자세", speed=25.0, compensate=False)   # 6축 한방 이동(30°/s) + 보상 생략
                    _start_auto()                       # 자동 워크플로우 루프(ENABLED=True일 때만 실동작)
                    print("[CMD] AUTO_START → 스캔자세" + (" + 자동루프" if _auto_alive() else " (auto 비활성/단일PICK 운전)"))
                elif cmd == "AUTO_STOP":
                    _set_run(False)                     # 자동탐색 끄기(정지 후 detect가 계속 센터링 방지)
                    with pick_lock:
                        run_flags["stop_after_cycle"] = True   # 자동 루프 graceful 종료 신호
                    if _auto_alive():
                        print("[CMD] AUTO_STOP → 현재 사이클 끝나고 종료(graceful)")
                    else:
                        request_motion([(SCAN_POSE, "스캔복귀"), (HOME_POSE, "원점180")],
                                       "STOP → 스캔 후 원점", compensate=False)
                        print("[CMD] AUTO_STOP → 스캔 후 원점 이동")
                elif cmd == "HALT":
                    _set_run(False)                     # 자동탐색 끄기
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
                        if len(targets) == 6:    # ★전체 자세 이동([이동] 버튼) → 자동사이클과 동일 충돌방지 순서: M1 → M5·6 → M2·3·4 하강
                            legs = [({1: targets[1]}, sub + " M1"),
                                    ({m: targets[m] for m in (5, 6)}, sub + " M5·6(방향,들고)"),
                                    ({m: targets[m] for m in (2, 3, 4)}, sub + " M2·3·4 하강")]
                            request_motion(legs, f"수동 {sub}", min_dur=MANUAL_MIN_DURATION)
                        else:                    # 조그(슬라이더 1~2축) → 한번에(즉시 반응)
                            request_motion([(targets, sub)], f"수동 {sub}", min_dur=MANUAL_MIN_DURATION)
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
                elif cmd == "SWEEP":
                    _set_run(True)                      # detect가 run.flag 읽고 sweep+센터링(M1 직접 안 몲)
                    print("[CMD] SWEEP → run.flag ON (detect 자동탐색)")
                elif cmd == "SWEEP_STOP":
                    _set_run(False)                     # 탐색 끄기
                    request_motion([({1: SCAN_POSE[1]}, "스윕정지 M1복귀")], "SWEEP_STOP → 스캔M1", compensate=False)  # stop_sweep 포함 + M1 복귀
                    print("[CMD] SWEEP_STOP → run.flag OFF, M1 스캔복귀")
                elif cmd == "GRAB_LABEL":
                    _label_last_grab(payload.get("success"))   # 수동 파지 성공/실패 라벨(pick 로직 무관)
                elif cmd in ("PICK", "S", "VISION_SEND"):
                    res = handle_pick()
                    print(f"[CMD] PICK → {res}")
                    if res != "started":   # 게이트실패/작업중/모듈없음 → 사유를 대시보드로(먹통처럼 보이는 것 방지)
                        _msg = {"gate_fail": "잡을 물체 없음 — 검출 불안정/신뢰도 낮음",
                                "busy": "작업 중 — 잠시 후 다시",
                                "no_module": "PICK 모듈 없음"}.get(res, "PICK 실패: %s" % res)
                        await ws.send(json.dumps({"type": "PICK_RESULT", "result": res, "message": _msg},
                                                 ensure_ascii=False))
                elif cmd == "ESTOP":
                    _set_run(False)                     # 자동탐색 끄기
                    with pick_lock:
                        run_flags["estop"] = True       # 자동루프 정지(should_stop)+자동 재시작 차단. 수동은 허용.
                    # 그 자리 즉시정지: 진행 중 이동 선점 + 각 모터를 현재 실측각에 토크 홀드(후퇴 안 함)
                    with lock:
                        hold = {m: motors[m]["current"] for m in range(1, 7)
                                if motors[m]["current"] is not None}
                    if hold:
                        request_motion([(hold, "E-STOP 현재각 홀드")], "E-STOP 즉시정지", min_dur=0.05)
                        print(f"[CMD] ESTOP → 자동정지·현재각 홀드 {sorted(hold)} (수동 가능)")
                    else:
                        request_motion([], "E-STOP 선점")  # 현재각 미수신: 진행 이동만 선점
                        print("[CMD] ESTOP → 자동정지·진행이동 선점 (수동 가능)")
                elif cmd == "ESTOP_RESET":
                    with pick_lock:
                        run_flags["estop"] = False
                    _set_cnc_busy(False)                # 새 사이클 깨끗이 시작(가공중 플래그 초기화)
                    def _estop_resume():
                        # 복구: J1 제외 나머지(M2~6) 한방으로 스캔 올린 뒤 → J1 스캔. S-curve 30°/s. 완료 후 자동 재개.
                        others = {m: SCAN_POSE[m] for m in (2, 3, 4, 5, 6)}
                        my_gen = request_motion([(others, "RESET M2~6 스캔(들기)"),
                                                 ({1: SCAN_POSE[1]}, "RESET J1 스캔")],
                                                "ESTOP_RESET 복구", speed=25.0, compensate=False)
                        deadline = time.time() + 20.0
                        while time.time() < deadline:
                            with mlock:
                                g = motion["gen"]; done = motion["completed_gen"]
                            if g != my_gen:
                                print("[CMD] ESTOP_RESET 복구 선점됨(수동개입?) → 자동 재개 보류")
                                return
                            if done == my_gen:
                                break
                            time.sleep(0.02)
                        _start_auto()                   # 스캔부터 자동 분류 사이클 재시작
                        print("[CMD] ESTOP_RESET → 복구완료 · 자동 재개")
                    threading.Thread(target=_estop_resume, daemon=True).start()
                    print("[CMD] ESTOP_RESET → 해제·복구 시작(M2~6→J1 스캔, 30°/s S-curve)")
                elif cmd == "END":
                    _set_run(False)                     # 자동탐색 끄기(종료 후 책상틈 잡으러 가는 것 방지)
                    with pick_lock:
                        busy = pick_state["status"] == "BUSY"
                        run_flags["stop_after_cycle"] = True
                    if _auto_alive():
                        # 자동운전 중: 선점 안 하고 현재 사이클 마칠 때까지 둠 → 루프 끝나면 _worker가 스캔→원점.
                        print("[CMD] END → 자동: 현재 사이클 끝까지 하고 스캔→원점 (대기)")
                    elif not busy:
                        request_motion([({1: SCAN_POSE[1]}, "M1 복귀"),               # ★M1 먼저 180 (틀어진 채 끌려가던 거 방지)
                                        (SCAN_POSE, "스캔복귀"), (HOME_POSE, "원점")], "END → 원점", compensate=False)
                        with pick_lock:
                            run_flags["stop_after_cycle"] = False
                        print("[CMD] END → (유휴) 원점 복귀")
                    else:
                        print("[CMD] END → 사이클 후 원점 (대기)")
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
    threading.Thread(target=ik5005_server, daemon=True).start()   # detect M1 스캔/홈/스윕 수신(ik_server 대체)
    threading.Thread(target=safety_monitor, daemon=True).start()  # 온도→graceful종료 / 부하→즉시정지+홀드
    _set_busy(False)                      # 시작 시 잔여 깃발 제거(이전 크래시 대비)
    _set_cnc_busy(False)
    _set_run(False)
    print(f"Sambo WS Bridge v4 — ws://0.0.0.0:{WS_PORT}, snapshot {SEND_HZ}Hz, "
          f"motion+manual+gripper+halt enabled")
    async with websockets.serve(ws_handler, WS_HOST, WS_PORT):
        await broadcaster()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nbye")
