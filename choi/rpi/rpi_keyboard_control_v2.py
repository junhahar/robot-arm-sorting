#!/usr/bin/env python3
"""
rpi_keyboard_control_v2.py — 키보드로 '논리 축' 단위 모터 제어
================================================================
M2·M3은 마주보는 거울쌍(한 관절). 각 모터는 '영점값(cal)'과 'sign'을 가진다.

키 매핑
  이동(앞/뒤):
    M1     q / a       (q=각도↑, a=각도↓)
    M2·M3  w / s       (w: M2 360→0 ↓, M3 0→360 ↑ / s: 반대)
    M4     e / d       (e: 360→0 ↓, d: 반대 ↑)
    M5     r / f
    M6     t / g
  영점(현재 위치를 그 값으로, 모터 안 움직임 / 전부 서보 EEPROM 영구):
    z=M1→180   x=M2→360·M3→0   c=M4→360   v=M5→180   b=M6→180
      · 180°(z,v,b): 중위 보정(torque=128)
      · 임의각(x,c): 서보 오프셋 레지스터 직접 기록. 오프셋 범위 ±180°라 0/360은 한계 끝
      · 풀 가동범위를 쓰려면 x/c 누르기 전에 그 관절을 원하는 끝쪽에 물리적으로 둘 것
      · 라벨을 바꿔도 물리 가동범위는 360° 그대로(현재=360이면 아래로만 360°)
  홈이동: 1~6 = 그 모터를 물리 180°로,  0 = 전체 물리 180°로
  프리로드(M2·3 분담): ] = +0.5,  [ = -0.5   (전류 모니터 보며 M2≈M3 맞추기)
  종료: ESC 또는 Ctrl-C

각도 체계
  · logical(사용자가 보는 각도) = physical(서보 실제각) + cal_offset
  · STM에는 physical = logical − cal_offset 을 보낸다 (0~360으로 clamp)

CAN 셋업 (규약 §5.1): sudo ip link set can0 type can bitrate 125000 sample-point 0.625
실행: python3 rpi_keyboard_control_v2.py
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
CMD_SET_MIDDLE = 0x04  # 현재 위치를 180°(중위)로 영점 (EEPROM 영구) — 180 모터용
CMD_SET_REF = 0x05     # 현재 위치를 임의 각도로 영점 (EEPROM 오프셋 직접 기록) — 360/0 등

# ── 제어 파라미터 ──
MOTOR_COUNT = 6
ANGLE_MIN = 0.0
ANGLE_MAX = 360.0
DEFAULT_ANGLE = 180.0  # 텔레메트리 못 받았을 때 임시 현재각

HOME_ANGLE = 180.0  # 홈이동 시 보낼 물리 각도
HOME_MOVE_MS = 800

RATE_HZ = 40.0
DT = 1.0 / RATE_HZ
STREAM_MOVE_MS = 40  # 스트리밍 이동시간(연속 보간용)
RAMP_RATE = 20.0
MAX_SPEED = 20.0
HOLD_TIMEOUT = 0.5

# M2·M3 거울쌍 부하 분담용 프리로드[deg]. 실행 중 [ ] 키로 즉석 조정.
MIRROR_PRELOAD = 0.0
PRELOAD_STEP = 0.5
PRELOAD_MAX = 20.0

# ── 논리 축 정의 ──
# members = ((motor_id, sign, cal_deg), ...)
#   sign: 앞 키(fwd) 눌렀을 때 각도 증가(+1)/감소(-1)
#   cal_deg: 영점 키를 누르면 그 모터의 '현재 위치'를 이 각도로 라벨
# gain: 그 축 속도 배율 (M4=5, 무거워서). preload: 거울쌍 분담(M2·3만).
AXES = [
    {"label": "M1",   "fwd": "q", "back": "a", "calib": "z", "gain": 1.0, "preload": 0.0,            "members": ((1, +1, 180.0),)},
    {"label": "M2·3", "fwd": "w", "back": "s", "calib": "x", "gain": 1.0, "preload": MIRROR_PRELOAD, "members": ((2, +1, 0.0), (3, -1, 360.0))},
    {"label": "M4",   "fwd": "e", "back": "d", "calib": "c", "gain": 5.0, "preload": 0.0,            "members": ((4, -1, 360.0),)},
    {"label": "M5",   "fwd": "r", "back": "f", "calib": "v", "gain": 1.0, "preload": 0.0,            "members": ((5, +1, 180.0),)},
    {"label": "M6",   "fwd": "t", "back": "g", "calib": "b", "gain": 1.0, "preload": 0.0,            "members": ((6, +1, 180.0),)},
]

MOVE_KEYS = {k for ax in AXES for k in (ax["fwd"], ax["back"])}
CALIB_KEYS = {ax["calib"]: ai for ai, ax in enumerate(AXES)}
MOTOR_AXIS = {mid: (ai, sign) for ai, ax in enumerate(AXES) for (mid, sign, _c) in ax["members"]}
MIRROR_AXIS_IDX = next(i for i, ax in enumerate(AXES) if len(ax["members"]) > 1)


# ── 텔레메트리 수신: 현재 physical 각도 ──
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
        # 영점으로 각도 라벨이 바뀌면 옛 측정값을 버린다(새 텔레메트리 올 때까지 None)
        with self._lock:
            self._angle.pop(motor_id, None)


def send_torque(bus, motor_id, on):
    data = [CMD_TORQUE, motor_id, 0, 0, 0, 0, 1 if on else 0, 0]
    try:
        bus.send(can.Message(arbitration_id=CAN_ID_CMD, data=data, is_extended_id=False))
    except can.CanError:
        pass


def send_angle(bus, motor_id, deg, move_ms=0):
    deg = max(ANGLE_MIN, min(ANGLE_MAX, deg))  # physical 0~360으로 clamp
    ax10 = int(round(deg * 10))
    mt = int(move_ms) & 0xFFFF
    data = [CMD_SET_ANGLE, motor_id, ax10 & 0xFF, (ax10 >> 8) & 0xFF,
            mt & 0xFF, (mt >> 8) & 0xFF, 0, 0]
    try:
        bus.send(can.Message(arbitration_id=CAN_ID_CMD, data=data, is_extended_id=False))
    except can.CanError:
        pass


def send_set_middle(bus, motor_id):
    # 현재 위치를 180°(중위)로 영점 — 서보 EEPROM 영구. 모터 안 움직임.
    data = [CMD_SET_MIDDLE, motor_id, 0, 0, 0, 0, 0, 0]
    try:
        bus.send(can.Message(arbitration_id=CAN_ID_CMD, data=data, is_extended_id=False))
    except can.CanError:
        pass


def send_set_ref(bus, motor_id, deg):
    # 현재 위치를 deg로 영점 — 서보 EEPROM 오프셋 직접 기록(영구). 모터 안 움직임.
    deg = max(ANGLE_MIN, min(ANGLE_MAX, deg))
    ax10 = int(round(deg * 10))
    data = [CMD_SET_REF, motor_id, ax10 & 0xFF, (ax10 >> 8) & 0xFF, 0, 0, 0, 0]
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
    time.sleep(0.5)  # 텔레메트리 들어오게 잠깐 대기(영점 시 현재각 필요)

    target = {m: None for m in range(1, MOTOR_COUNT + 1)}        # logical 목표각
    cal_offset = {m: 0.0 for m in range(1, MOTOR_COUNT + 1)}     # logical = physical + offset
    axis_speed = {ai: 0.0 for ai in range(len(AXES))}
    axis_prev_dir = {ai: 0 for ai in range(len(AXES))}
    last_seen = {}

    print("키보드 제어 v2 시작")
    print("  이동:  M1 q/a   M2·3 w/s   M4 e/d   M5 r/f   M6 t/g")
    print("  영점:  z=M1→180  x=M2→360·M3→0  c=M4→360  v=M5→180  b=M6→180  (전부 EEPROM 영구)")
    print("         (x/c는 360/0 → 풀가동 쓰려면 영점 전에 그 관절을 물리적으로 끝쪽에 둘 것)")
    print("  홈이동: 1~6 그 모터 물리180°,  0 전체 물리180°")
    print("  프리로드(M2·3): ] +0.5 / [ -0.5  (전류 모니터로 M2≈M3)")
    print("  누르고 있으면 가속(최대 %.0f°/s), 떼면 정지. ESC=종료\n" % MAX_SPEED)

    fd = sys.stdin.fileno()
    old_attr = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        next_status = 0.0
        next_hold = 0.0

        while True:
            loop_start = time.monotonic()

            # 1) 입력 처리
            while select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1)
                if ch == "\x1b":
                    raise KeyboardInterrupt
                if ch in MOVE_KEYS:
                    last_seen[ch] = loop_start
                elif ch == "]":  # 프리로드 +
                    p = min(AXES[MIRROR_AXIS_IDX]["preload"] + PRELOAD_STEP, PRELOAD_MAX)
                    AXES[MIRROR_AXIS_IDX]["preload"] = p
                    sys.stdout.write(f"\n[프리로드] M2·3 = {p:+.1f}°\n")
                    sys.stdout.flush()
                elif ch == "[":  # 프리로드 -
                    p = max(AXES[MIRROR_AXIS_IDX]["preload"] - PRELOAD_STEP, -PRELOAD_MAX)
                    AXES[MIRROR_AXIS_IDX]["preload"] = p
                    sys.stdout.write(f"\n[프리로드] M2·3 = {p:+.1f}°\n")
                    sys.stdout.flush()
                elif ch in CALIB_KEYS:  # 영점: 현재 위치를 cal_deg로 (서보 EEPROM 영구, 안 움직임)
                    ai = CALIB_KEYS[ch]
                    msgs = []
                    for mid, _sign, cal_deg in AXES[ai]["members"]:
                        if mid > MOTOR_COUNT:
                            continue
                        if abs(cal_deg - 180.0) < 0.01:
                            send_set_middle(bus, mid)        # 180: 중위보정 (검증된 안전 경로)
                        else:
                            send_set_ref(bus, mid, cal_deg)  # 임의각: 오프셋 직접 기록
                        cal_offset[mid] = 0.0                # 서보가 직접 라벨 → SW 오프셋 불필요
                        target[mid] = None                   # ★안전★ 영점 후 실제 측정각에서 다시 시드
                        measured.clear(mid)                  # 옛 라벨 버리고 새 텔레메트리 기다림(점프 방지)
                        msgs.append(f"M{mid}->{cal_deg:.0f}")
                    axis_speed[ai] = 0.0
                    sys.stdout.write("\n[영점/EEPROM] " + "  ".join(msgs) + "\n")
                    sys.stdout.flush()
                elif ch == "0":  # 전체 물리 180°로
                    for hm in range(1, MOTOR_COUNT + 1):
                        target[hm] = HOME_ANGLE + cal_offset[hm]
                        send_angle(bus, hm, HOME_ANGLE, move_ms=HOME_MOVE_MS)
                    for ai in axis_speed:
                        axis_speed[ai] = 0.0
                elif ch in "123456":  # 해당 모터 물리 180°로
                    hm = int(ch)
                    if 1 <= hm <= MOTOR_COUNT:
                        target[hm] = HOME_ANGLE + cal_offset[hm]
                        send_angle(bus, hm, HOME_ANGLE, move_ms=HOME_MOVE_MS)

            # 2) 축별 방향 → 속도 램프 → logical 적분 → physical 송신
            now = loop_start
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
                    for mid, sign, _cal in ax["members"]:
                        if target[mid] is None:
                            a = measured.get(mid)
                            if a is None:
                                continue  # ★안전★ 실제 측정각 받기 전엔 안 움직임(점프 방지)
                            target[mid] = a
                        target[mid] += sign * delta
                        # 물리[0,360]에 대응하는 logical 범위로 제한 (logical = physical + offset)
                        lo = cal_offset[mid] + ANGLE_MIN
                        hi = cal_offset[mid] + ANGLE_MAX
                        target[mid] = max(lo, min(hi, target[mid]))
                        phys = target[mid] + sign * ax["preload"] - cal_offset[mid]
                        send_angle(bus, mid, phys, move_ms=STREAM_MOVE_MS)

                axis_prev_dir[ai] = direction

            # 2.5) 프리로드 축은 정지 중에도 주기적 재송신 → 정적 분담 유지
            if now >= next_hold:
                next_hold = now + 0.3
                for ai, ax in enumerate(AXES):
                    if ax["preload"] != 0.0 and axis_prev_dir[ai] == 0:
                        for mid, sign, _cal in ax["members"]:
                            if target[mid] is not None:
                                phys = target[mid] + sign * ax["preload"] - cal_offset[mid]
                                send_angle(bus, mid, phys, move_ms=200)

            # 3) 상태 한 줄 (logical 각도 + 이동방향)
            if now >= next_status:
                next_status = now + 0.2
                parts = []
                for m in range(1, MOTOR_COUNT + 1):
                    ai, sign = MOTOR_AXIS.get(m, (None, 1))
                    d = axis_prev_dir[ai] * sign if ai is not None else 0
                    arrow = "+" if d > 0 else ("-" if d < 0 else " ")
                    if target[m] is None:
                        cur = measured.get(m)
                        shown = f"{cur:5.1f}" if cur is not None else " --- "
                    else:
                        shown = f"{target[m]:5.1f}"
                    parts.append(f"M{m}{arrow}{shown}")
                pre = AXES[MIRROR_AXIS_IDX]["preload"]
                sys.stdout.write("\r" + "  ".join(parts) + f"  | preload={pre:+.1f}   ")
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
