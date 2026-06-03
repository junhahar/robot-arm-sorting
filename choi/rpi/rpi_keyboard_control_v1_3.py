#!/usr/bin/env python3
"""
rpi_keyboard_control_v1_3.py — v1_2 + 상태줄에 '목표각 / 실측각' 둘 다 표시
================================================================
v1_2(전부 180° 안전 + 부드러운 홈복귀)와 동일하되, 상태줄을 바꿈:
  · 각 모터를 'M{n}{화살표} 목표/실측' 으로 표시
    - 목표 = RPi가 보내는 명령각 (target)
    - 실측 = STM 0x201 텔레메트리로 받은 실제 모터각 (measured)
  · 둘이 거의 같으면 정상. 차이가 크면 부하/지연/막힘 의심.
  · 값이 아직 없으면 '--'.

키 매핑
  이동:  M1 q/a   M2·M3 w/s(거울)   M4 e/d   M5 r/f   M6 t/g
  영점(전부 180° 중위보정):  z=M1  x=M2·M3  c=M4  v=M5  b=M6
  홈복귀(부드럽게): 1~6 그 모터 180°,  0 전체 180°  (이동키 누르면 취소)
  프리로드(M2·3): ] = +0.5,  [ = -0.5      종료: ESC

CAN 셋업: sudo ip link set can0 type can bitrate 125000 sample-point 0.625
실행:     python3 rpi_keyboard_control_v1_3.py
"""

import sys
import time
import struct
import threading
import termios
import tty
import select
import can

# ── CAN 설정 ──
CAN_INTERFACE = "can0"
CAN_ID_CMD = 0x100
CAN_ID_TELEM = 0x201
CMD_TORQUE = 0x02
CMD_SET_ANGLE = 0x03
CMD_SET_MIDDLE = 0x04

# ── 제어 파라미터 ──
MOTOR_COUNT = 6
ANGLE_MIN = 0.0
ANGLE_MAX = 360.0
DEFAULT_ANGLE = 180.0

HOME_ANGLE = 180.0
RATE_HZ = 40.0
DT = 1.0 / RATE_HZ
STREAM_MOVE_MS = 40
RAMP_RATE = 20.0
MAX_SPEED = 20.0
HOLD_TIMEOUT = 0.5

# ── 부드러운 홈복귀 파라미터 ──
HOME_MAX_SPEED = 20.0      # 홈복귀 최대 속도[deg/s] (멀 때). 일반 이동과 동일.
HOME_APPROACH_GAIN = 0.08  # 감속 세기: 매 틱 (남은거리 × 이 값) → 가까울수록 느려짐
HOME_DEADBAND = 0.5        # 목표 ±이 각도 안이면 멈춤 [deg]

# M2·M3 거울쌍 부하 분담용 프리로드[deg]. 실행 중 [ ] 키로 즉석 조정.
MIRROR_PRELOAD = 0.0
PRELOAD_STEP = 0.5
PRELOAD_MAX = 20.0

# ── 논리 축 정의 ── members = ((motor_id, sign), ...)
AXES = [
    {"label": "M1",   "fwd": "q", "back": "a", "calib": "z", "gain": 1.0, "preload": 0.0,            "members": ((1, +1),)},
    {"label": "M2·3", "fwd": "w", "back": "s", "calib": "x", "gain": 1.0, "preload": MIRROR_PRELOAD, "members": ((2, +1), (3, -1))},
    {"label": "M4",   "fwd": "e", "back": "d", "calib": "c", "gain": 5.0, "preload": 0.0,            "members": ((4, +1),)},
    {"label": "M5",   "fwd": "r", "back": "f", "calib": "v", "gain": 1.0, "preload": 0.0,            "members": ((5, +1),)},
    {"label": "M6",   "fwd": "t", "back": "g", "calib": "b", "gain": 1.0, "preload": 0.0,            "members": ((6, +1),)},
]

MOVE_KEYS = {k for ax in AXES for k in (ax["fwd"], ax["back"])}
CALIB_KEYS = {ax["calib"]: ai for ai, ax in enumerate(AXES)}
MOTOR_AXIS = {mid: (ai, sign) for ai, ax in enumerate(AXES) for (mid, sign) in ax["members"]}
KEY_AXIS = {ax["fwd"]: ai for ai, ax in enumerate(AXES)}
KEY_AXIS.update({ax["back"]: ai for ai, ax in enumerate(AXES)})
MIRROR_AXIS_IDX = next(i for i, ax in enumerate(AXES) if len(ax["members"]) > 1)


class MeasuredAngles(can.Listener):
    def __init__(self):
        self._lock = threading.Lock()
        self._angle = {}

    def on_message_received(self, msg: can.Message) -> None:
        if msg.arbitration_id == CAN_ID_TELEM and len(msg.data) >= 7:
            mid, ax10, _t, _l, flags = struct.unpack("<BHBHB", bytes(msg.data[0:7]))
            if flags & 0x01:
                with self._lock:
                    self._angle[mid] = ax10 / 10.0

    def get(self, motor_id):
        with self._lock:
            return self._angle.get(motor_id)

    def clear(self, motor_id):
        with self._lock:
            self._angle.pop(motor_id, None)


def send_torque(bus, motor_id, on):
    data = [CMD_TORQUE, motor_id, 0, 0, 0, 0, 1 if on else 0, 0]
    try:
        bus.send(can.Message(arbitration_id=CAN_ID_CMD, data=data, is_extended_id=False))
    except can.CanError:
        pass


def send_angle(bus, motor_id, deg, move_ms=0):
    deg = max(ANGLE_MIN, min(ANGLE_MAX, deg))
    ax10 = int(round(deg * 10))
    mt = int(move_ms) & 0xFFFF
    data = [CMD_SET_ANGLE, motor_id, ax10 & 0xFF, (ax10 >> 8) & 0xFF,
            mt & 0xFF, (mt >> 8) & 0xFF, 0, 0]
    try:
        bus.send(can.Message(arbitration_id=CAN_ID_CMD, data=data, is_extended_id=False))
    except can.CanError:
        pass


def send_set_middle(bus, motor_id):
    data = [CMD_SET_MIDDLE, motor_id, 0, 0, 0, 0, 0, 0]
    try:
        bus.send(can.Message(arbitration_id=CAN_ID_CMD, data=data, is_extended_id=False))
    except can.CanError:
        pass


def main():
    try:
        bus = can.interface.Bus(channel=CAN_INTERFACE, interface="socketcan")
    except OSError as e:
        print(f"[ERR] {CAN_INTERFACE} 열기 실패: {e}")
        print("       sudo ip link set can0 type can bitrate 125000 sample-point 0.625")
        return

    measured = MeasuredAngles()
    notifier = can.Notifier(bus, [measured])

    for m in range(1, MOTOR_COUNT + 1):
        send_torque(bus, m, True)
        time.sleep(0.02)
    time.sleep(0.5)

    target = {m: None for m in range(1, MOTOR_COUNT + 1)}
    axis_speed = {ai: 0.0 for ai in range(len(AXES))}
    axis_prev_dir = {ai: 0 for ai in range(len(AXES))}
    last_seen = {}
    homing = set()

    print("키보드 제어 v1_3 시작 (180° 안전 + 부드러운 홈복귀 + 목표/실측 표시)")
    print("  이동:  M1 q/a   M2·3 w/s(거울)   M4 e/d   M5 r/f   M6 t/g")
    print("  영점:  z=M1  x=M2·3  c=M4  v=M5  b=M6   (180° 중위보정)")
    print("  홈복귀: 1~6 그 모터 180°,  0 전체 180°   프리로드: ] [   ESC=종료")
    print("  상태줄: M#화살표 [목표각]/[실측각]  (목표=RPi명령, 실측=STM실제)\n")

    fd = sys.stdin.fileno()
    old_attr = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        next_status = 0.0
        next_hold = 0.0
        home_step_cap = HOME_MAX_SPEED * DT

        while True:
            loop_start = time.monotonic()

            # 1) 입력 처리
            while select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1)
                if ch == "\x1b":
                    raise KeyboardInterrupt
                if ch in MOVE_KEYS:
                    last_seen[ch] = loop_start
                    ai = KEY_AXIS[ch]
                    for mid, _s in AXES[ai]["members"]:
                        homing.discard(mid)
                elif ch == "]":
                    p = min(AXES[MIRROR_AXIS_IDX]["preload"] + PRELOAD_STEP, PRELOAD_MAX)
                    AXES[MIRROR_AXIS_IDX]["preload"] = p
                    sys.stdout.write(f"\n[프리로드] M2·3 = {p:+.1f}°\n")
                    sys.stdout.flush()
                elif ch == "[":
                    p = max(AXES[MIRROR_AXIS_IDX]["preload"] - PRELOAD_STEP, -PRELOAD_MAX)
                    AXES[MIRROR_AXIS_IDX]["preload"] = p
                    sys.stdout.write(f"\n[프리로드] M2·3 = {p:+.1f}°\n")
                    sys.stdout.flush()
                elif ch in CALIB_KEYS:
                    ai = CALIB_KEYS[ch]
                    msgs = []
                    for mid, _sign in AXES[ai]["members"]:
                        if mid > MOTOR_COUNT:
                            continue
                        send_set_middle(bus, mid)
                        target[mid] = None
                        measured.clear(mid)
                        homing.discard(mid)
                        msgs.append(f"M{mid}->180")
                    axis_speed[ai] = 0.0
                    sys.stdout.write("\n[영점] " + "  ".join(msgs) + "\n")
                    sys.stdout.flush()
                elif ch == "0":
                    for hm in range(1, MOTOR_COUNT + 1):
                        homing.add(hm)
                    for ai in axis_speed:
                        axis_speed[ai] = 0.0
                elif ch in "123456":
                    hm = int(ch)
                    if 1 <= hm <= MOTOR_COUNT:
                        homing.add(hm)

            now = loop_start

            # 2) 축별 수동 이동
            for ai, ax in enumerate(AXES):
                fwd = ax["fwd"] in last_seen and (now - last_seen[ax["fwd"]]) < HOLD_TIMEOUT
                back = ax["back"] in last_seen and (now - last_seen[ax["back"]]) < HOLD_TIMEOUT

                direction = 0
                if fwd and not back:
                    direction = +1
                elif back and not fwd:
                    direction = -1

                if direction == 0:
                    axis_speed[ai] = 0.0
                else:
                    if direction != axis_prev_dir[ai]:
                        axis_speed[ai] = 0.0
                    axis_speed[ai] = min(axis_speed[ai] + RAMP_RATE * DT, MAX_SPEED)
                    delta = direction * axis_speed[ai] * DT * ax["gain"]
                    for mid, sign in ax["members"]:
                        if target[mid] is None:
                            a = measured.get(mid)
                            if a is None:
                                continue
                            target[mid] = a
                        target[mid] += sign * delta
                        target[mid] = max(ANGLE_MIN, min(ANGLE_MAX, target[mid]))
                        phys = target[mid] + sign * ax["preload"]
                        send_angle(bus, mid, phys, move_ms=STREAM_MOVE_MS)

                axis_prev_dir[ai] = direction

            # 2.3) 부드러운 홈복귀 — 가까울수록 감속
            for mid in list(homing):
                if target[mid] is None:
                    a = measured.get(mid)
                    if a is None:
                        continue
                    target[mid] = a
                remaining = HOME_ANGLE - target[mid]
                if abs(remaining) <= HOME_DEADBAND:
                    target[mid] = HOME_ANGLE
                    send_angle(bus, mid, HOME_ANGLE, move_ms=STREAM_MOVE_MS)
                    homing.discard(mid)
                    continue
                step = remaining * HOME_APPROACH_GAIN
                if step > home_step_cap:
                    step = home_step_cap
                elif step < -home_step_cap:
                    step = -home_step_cap
                target[mid] += step
                send_angle(bus, mid, target[mid], move_ms=STREAM_MOVE_MS)

            # 2.5) 프리로드 축 정지 중 재송신 (홈복귀 중 제외)
            if now >= next_hold:
                next_hold = now + 0.3
                for ai, ax in enumerate(AXES):
                    if ax["preload"] != 0.0 and axis_prev_dir[ai] == 0:
                        for mid, sign in ax["members"]:
                            if target[mid] is not None and mid not in homing:
                                phys = target[mid] + sign * ax["preload"]
                                send_angle(bus, mid, phys, move_ms=200)

            # 3) 상태 한 줄 — 목표/실측 둘 다
            if now >= next_status:
                next_status = now + 0.2
                parts = []
                for m in range(1, MOTOR_COUNT + 1):
                    if m in homing:
                        arrow = "H"
                    else:
                        ai, sign = MOTOR_AXIS.get(m, (None, 1))
                        d = axis_prev_dir[ai] * sign if ai is not None else 0
                        arrow = "+" if d > 0 else ("-" if d < 0 else " ")
                    tgt = target[m]
                    act = measured.get(m)
                    tgt_s = f"{tgt:.1f}" if tgt is not None else "--"
                    act_s = f"{act:.1f}" if act is not None else "--"
                    parts.append(f"M{m}{arrow}{tgt_s}/{act_s}")
                pre = AXES[MIRROR_AXIS_IDX]["preload"]
                sys.stdout.write("\r" + "  ".join(parts) + f" |pre{pre:+.1f}  ")
                sys.stdout.flush()

            remain = DT - (time.monotonic() - loop_start)
            if remain > 0:
                time.sleep(remain)

    except KeyboardInterrupt:
        print("\n종료.")
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attr)
        notifier.stop()
        bus.shutdown()


if __name__ == "__main__":
    main()
