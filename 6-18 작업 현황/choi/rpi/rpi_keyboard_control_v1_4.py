#!/usr/bin/env python3
"""
rpi_keyboard_control_v1_4.py — v1_3 + '9'번 카메라 자세 축별 순차 이동
================================================================
v1_3(180° 안전 + 부드러운 홈복귀 + 목표/실측 표시)에 추가:
  · '9' 누르면 카메라 촬영 자세(CAMERA_POSE)로 이동.
  · 한 번에 다 가지 않고 '축별로 하나씩 순서대로' 이동:
        M1 → M2·M3 → M4 → M5 → M6
    각 축이 목표에 도착하면(감속) 다음 축으로 넘어감.
  · 이동 키 누르면 자세 이동 취소(수동 우선).

키 매핑
  이동:  M1 q/a   M2·M3 w/s(거울)   M4 e/d   M5 r/f   M6 t/g
  영점:  z=M1  x=M2·M3  c=M4  v=M5  b=M6   (180° 중위보정)
  홈복귀: 1~6 그 모터 180°,  0 전체 180°
  카메라 자세: 9  (축별 순차 이동)
  프리로드(M2·3): ] = +0.5,  [ = -0.5      종료: ESC

CAN 셋업: sudo ip link set can0 type can bitrate 125000 sample-point 0.625
실행:     python3 rpi_keyboard_control_v1_4.py
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

# ── 부드러운 이동(홈복귀/자세) 파라미터 ──
HOME_MAX_SPEED = 20.0      # 최대 속도[deg/s] (멀 때). 낮추면 더 느림
HOME_APPROACH_GAIN = 0.08  # 감속 세기: 매 틱 (남은거리 × 이 값) → 가까울수록 느려짐
HOME_DEADBAND = 0.5        # 목표 ±이 각도 안이면 도착으로 봄 [deg]

# ── 카메라 촬영 자세 (모터별 명령각) ──
CAMERA_POSE = {1: 180.0, 2: 107.4, 3: 245.6, 4: 107.4, 5: 129.2, 6: 180.0}

# M2·M3 거울쌍 부하 분담용 프리로드[deg]. 실행 중 [ ] 키로 즉석 조정.
MIRROR_PRELOAD = 0.0
PRELOAD_STEP = 0.5
PRELOAD_MAX = 20.0

# ── 논리 축 정의 ── members = ((motor_id, sign), ...)
AXES = [
    {"label": "M1",   "fwd": "q", "back": "a", "calib": "z", "gain": 1.0, "preload": 0.0,            "members": ((1, +1),)},
    {"label": "M2·3", "fwd": "w", "back": "s", "calib": "x", "gain": 1.0, "preload": MIRROR_PRELOAD, "members": ((2, +1), (3, -1))},
    {"label": "M4",   "fwd": "e", "back": "d", "calib": "c", "gain": 2.0, "preload": 0.0,            "members": ((4, +1),)},
    {"label": "M5",   "fwd": "r", "back": "f", "calib": "v", "gain": 1.0, "preload": 0.0,            "members": ((5, +1),)},
    {"label": "M6",   "fwd": "t", "back": "g", "calib": "b", "gain": 1.0, "preload": 0.0,            "members": ((6, +1),)},
]

# 카메라 자세 이동 순서 (축 인덱스): M1 → M4 → M2·3 → M5 → M6
POSE_SEQ = [0, 2, 1, 3, 4]

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


def approach(target, mid, goal, send_fn, step_cap):
    """target[mid]을 goal로 감속하며 한 틱 이동. 도착했으면 True 반환."""
    remaining = goal - target[mid]
    if abs(remaining) <= HOME_DEADBAND:
        target[mid] = goal
        send_fn(mid, goal)
        return True
    step = remaining * HOME_APPROACH_GAIN
    if step > step_cap:
        step = step_cap
    elif step < -step_cap:
        step = -step_cap
    target[mid] += step
    send_fn(mid, target[mid])
    return False


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
    pose_idx = None   # 카메라 자세 순차 이동: POSE_SEQ의 현재 단계 (None=비활성)

    print("키보드 제어 v1_4 시작 (180° 안전 + 부드러운 이동 + 카메라 자세 9번)")
    print("  이동:  M1 q/a   M2·3 w/s(거울)   M4 e/d   M5 r/f   M6 t/g")
    print("  영점:  z=M1  x=M2·3  c=M4  v=M5  b=M6   (180° 중위보정)")
    print("  홈복귀: 1~6 그 모터 180°,  0 전체 180°")
    print("  카메라 자세: 9 (M1→M4→M2·3→M5→M6 순차)   프리로드: ] [   ESC=종료")
    print("  상태줄: M#화살표 목표/실측  (H=홈복귀, P=자세이동)\n")

    fd = sys.stdin.fileno()
    old_attr = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        next_status = 0.0
        next_hold = 0.0
        home_step_cap = HOME_MAX_SPEED * DT

        def send_fn(mid, deg):
            send_angle(bus, mid, deg, move_ms=STREAM_MOVE_MS)

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
                    pose_idx = None  # 수동 이동 시 자세이동 취소
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
                elif ch == "9":  # 카메라 자세로 축별 순차 이동 시작
                    pose_idx = 0
                    homing.clear()
                    sys.stdout.write("\n[자세] 카메라 자세로 축별 순차 이동 시작 (M1→M4→M2·3→M5→M6)\n")
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
                    pose_idx = None
                    sys.stdout.write("\n[영점] " + "  ".join(msgs) + "\n")
                    sys.stdout.flush()
                elif ch == "0":
                    for hm in range(1, MOTOR_COUNT + 1):
                        homing.add(hm)
                    for ai in axis_speed:
                        axis_speed[ai] = 0.0
                    pose_idx = None
                elif ch in "123456":
                    hm = int(ch)
                    if 1 <= hm <= MOTOR_COUNT:
                        homing.add(hm)
                        pose_idx = None

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

            # 2.3) 부드러운 홈복귀
            for mid in list(homing):
                if target[mid] is None:
                    a = measured.get(mid)
                    if a is None:
                        continue
                    target[mid] = a
                if approach(target, mid, HOME_ANGLE, send_fn, home_step_cap):
                    homing.discard(mid)

            # 2.4) 카메라 자세 — 현재 축만 이동, 도착하면 다음 축
            posing_motors = set()
            if pose_idx is not None:
                ax = AXES[POSE_SEQ[pose_idx]]
                axis_done = True
                for mid, _sign in ax["members"]:
                    posing_motors.add(mid)
                    if target[mid] is None:
                        a = measured.get(mid)
                        if a is None:
                            axis_done = False
                            continue
                        target[mid] = a
                    if not approach(target, mid, CAMERA_POSE[mid], send_fn, home_step_cap):
                        axis_done = False
                if axis_done:
                    pose_idx += 1
                    if pose_idx >= len(POSE_SEQ):
                        pose_idx = None
                        sys.stdout.write("\n[자세] 카메라 자세 도착 완료\n")
                        sys.stdout.flush()

            # 2.5) 프리로드 축 정지 중 재송신 (홈복귀/자세이동 중 모터는 제외)
            if now >= next_hold:
                next_hold = now + 0.3
                for ai, ax in enumerate(AXES):
                    if ax["preload"] != 0.0 and axis_prev_dir[ai] == 0:
                        for mid, sign in ax["members"]:
                            if (target[mid] is not None and mid not in homing
                                    and mid not in posing_motors):
                                phys = target[mid] + sign * ax["preload"]
                                send_angle(bus, mid, phys, move_ms=200)

            # 3) 상태 한 줄 — 목표/실측
            if now >= next_status:
                next_status = now + 0.2
                parts = []
                for m in range(1, MOTOR_COUNT + 1):
                    if m in posing_motors:
                        arrow = "P"
                    elif m in homing:
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
