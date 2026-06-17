#!/usr/bin/env python3
"""
rpi_keyboard_control_v2.py — 키보드로 '논리 축' 단위 모터 제어
================================================================
M2·M3은 서로 마주보고 같은 관절을 함께 구동하는 '거울쌍'이라 한 축으로 묶는다.
  · w(앞) 누르면  M2 정방향(+), M3 반대방향(-) 으로 동시에 돈다 (M3 = 거울).
  · 즉 M2가 '나', M3가 '거울 속의 나'.

키 매핑
  이동(앞/뒤):
    M1     q / a
    M2·M3  w / s     ← 한 축, M3은 반대방향
    M4     e / d
    M5     r / f
    M6     t / g
  영점(현재 위치를 180°로, 모터 안 움직임 / 서보 EEPROM 영구):
    z=M1   x=M2·M3   c=M4   v=M5   b=M6
  홈이동:
    1~6 = 그 모터를 180°로 이동,  0 = 전체 180°로 이동
  종료: ESC 또는 Ctrl-C

원리
    STS3215는 위치 제어 서보라 '속도' 명령이 없다. RPi가 매 틱 축 목표각을
    (방향 × 속도 × dt)만큼 적분해 새 목표를 STM에 보낸다. 축의 각 멤버 모터는
    그 delta에 자기 sign(+1/-1)을 곱해 적용 → 거울쌍이 자동으로 반대로 움직인다.

CAN 셋업 (규약 §5.1 — 125kbps / 62.5%):
    sudo ip link set can0 down
    sudo ip link set can0 type can bitrate 125000 sample-point 0.625
    sudo ip link set can0 up

실행:
    python3 rpi_keyboard_control_v2.py

주의
    · 거울쌍은 '중립' 자세에서 x로 둘 다 180° 영점을 잡아야 대칭으로 움직인다.
    · 이 스크립트는 ACK를 기다리지 않고 위치 명령을 연속 송신한다(fire-and-forget).
    · SSH 터미널은 키 '뗌' 이벤트가 없어, 키 반복이 HOLD_TIMEOUT 동안 안 오면 뗀 것으로 본다.
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
CAN_ID_CMD = 0x100  # RPi → STM
CAN_ID_TELEM = 0x201  # STM → RPi (현재각 시드/표시용)
CMD_TORQUE = 0x02
CMD_SET_ANGLE = 0x03  # 각도×10 명령 (STM이 step 변환 담당)
CMD_SET_MIDDLE = 0x04  # 현재 위치를 180°(중위)로 영점 보정 — STM이 서보 EEPROM에 씀

# ── 제어 파라미터 (여기서 튜닝) ──
MOTOR_COUNT = 6  # 1~6 (6은 STM SERVO_COUNT=6 + ID6 서보 필요)
ANGLE_MIN = 0.0  # 목표각 하한 [deg]
ANGLE_MAX = 360.0  # 목표각 상한 [deg]
DEFAULT_ANGLE = 180.0  # 텔레메트리로 현재각을 못 받았을 때 시작각

# ── 홈(원점) 기능 ──
HOME_ANGLE = 180.0  # 숫자키로 보낼 홈 각도
HOME_MOVE_MS = 800  # 홈까지 이동 시간 [ms] (0이면 최대속도로 확 감). 부드럽게 800ms

RATE_HZ = 40.0  # 제어/송신 루프 주파수
DT = 1.0 / RATE_HZ
# 스트리밍 위치명령의 이동시간[ms]. 루프 주기(DT≈25ms)보다 약간 크게 잡아야
# 서보가 한 목표에 도착하기 전에 다음 목표가 와서 '연속 보간'된다(부드러움).
STREAM_MOVE_MS = 40
RAMP_RATE = 20.0  # 속도 기울기(가속도) [deg/s per second]
MAX_SPEED = 20.0  # 최대 속도 [deg/s]
HOLD_TIMEOUT = (
    0.5  # 키 반복 초기 지연을 넘기도록 크게(안 그러면 램프가 리셋돼 안 움직임).
)

# ── 논리 축(joint) 정의 ──
# 각 축은 앞/뒤 키로 제어되고, 영점 키 하나를 가지며, 여러 모터를 (id, sign)으로 묶는다.
# M2·M3은 마주보는 거울쌍 → 한 축에 (2,+1),(3,-1)로 묶어 M3을 반대방향으로 구동.
# gain = 그 축의 속도 배율. M4는 무거워서 잘 안 움직여 5배(=5배 빠르게/세게 밀어붙임).
AXES = [
    {
        "label": "M1",
        "fwd": "q",
        "back": "a",
        "calib": "z",
        "gain": 1.0,
        "members": ((1, +1),),
    },
    {
        "label": "M2·3",
        "fwd": "w",
        "back": "s",
        "calib": "x",
        "gain": 1.0,
        "members": ((2, +1), (3, -1)),
    },
    {
        "label": "M4",
        "fwd": "e",
        "back": "d",
        "calib": "c",
        "gain": 5.0,
        "members": ((4, +1),),
    },
    {
        "label": "M5",
        "fwd": "r",
        "back": "f",
        "calib": "v",
        "gain": 1.0,
        "members": ((5, +1),),
    },
    {
        "label": "M6",
        "fwd": "t",
        "back": "g",
        "calib": "b",
        "gain": 1.0,
        "members": ((6, +1),),
    },
]

# 키/모터 빠른 조회용 테이블
MOVE_KEYS = {k for ax in AXES for k in (ax["fwd"], ax["back"])}
CALIB_KEYS = {ax["calib"]: ai for ai, ax in enumerate(AXES)}
MOTOR_AXIS = {
    mid: (ai, sign) for ai, ax in enumerate(AXES) for (mid, sign) in ax["members"]
}


# ── 텔레메트리 수신: 시작각 시드 + 화면 표시용 현재각 ──
class MeasuredAngles(can.Listener):
    def __init__(self):
        self._lock = threading.Lock()
        self._angle = {}  # motor_id → deg

    def on_message_received(self, msg: can.Message) -> None:
        if msg.arbitration_id == CAN_ID_TELEM and len(msg.data) >= 7:
            mid, ax10, _t, _l, flags = struct.unpack("<BHBHB", bytes(msg.data[0:7]))
            if flags & 0x01:  # 각도 읽기 OK 비트
                with self._lock:
                    self._angle[mid] = ax10 / 10.0

    def get(self, motor_id):
        with self._lock:
            return self._angle.get(motor_id)


def send_torque(bus, motor_id, on):
    data = [CMD_TORQUE, motor_id, 0, 0, 0, 0, 1 if on else 0, 0]
    try:
        bus.send(
            can.Message(arbitration_id=CAN_ID_CMD, data=data, is_extended_id=False)
        )
    except can.CanError:
        pass


def send_angle(bus, motor_id, deg, move_ms=0):
    deg = max(ANGLE_MIN, min(ANGLE_MAX, deg))
    ax10 = int(round(deg * 10))  # 180.0도 → 1800
    mt = (
        int(move_ms) & 0xFFFF
    )  # 0=최대속도, >0=그 시간에 걸쳐 이동 (STM이 buf[4..5]로 읽음)
    data = [
        CMD_SET_ANGLE,
        motor_id,
        ax10 & 0xFF,
        (ax10 >> 8) & 0xFF,
        mt & 0xFF,
        (mt >> 8) & 0xFF,
        0,
        0,
    ]
    try:
        bus.send(
            can.Message(arbitration_id=CAN_ID_CMD, data=data, is_extended_id=False)
        )
    except can.CanError:
        pass


def send_set_middle(bus, motor_id):
    # 현재 물리 위치를 180°(중위 2048)로 영점 보정. 모터는 안 움직이고
    # 서보 EEPROM 오프셋만 바뀐다(영구). STM이 CMD_SET_MIDDLE을 받아 처리.
    data = [CMD_SET_MIDDLE, motor_id, 0, 0, 0, 0, 0, 0]
    try:
        bus.send(
            can.Message(arbitration_id=CAN_ID_CMD, data=data, is_extended_id=False)
        )
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

    # 토크 ON
    for m in range(1, MOTOR_COUNT + 1):
        send_torque(bus, m, True)
        time.sleep(0.02)
    time.sleep(0.3)  # 텔레메트리 몇 개라도 들어오게 잠깐 대기

    # 모터별 목표각 (첫 입력 때 시드)
    target = {m: None for m in range(1, MOTOR_COUNT + 1)}
    # 축별 램프 속도/직전 방향
    axis_speed = {ai: 0.0 for ai in range(len(AXES))}
    axis_prev_dir = {ai: 0 for ai in range(len(AXES))}
    last_seen = {}  # 키문자 → 마지막으로 본 시각(monotonic)

    print("키보드 제어 v2 시작")
    print("  이동:  M1 q/a   M2·3 w/s   M4 e/d   M5 r/f   M6 t/g   (앞/뒤)")
    print("         (M2·M3은 마주보는 거울쌍 → w/s로 함께, M3은 반대방향)")
    print("  영점:  z=M1  x=M2·3  c=M4  v=M5  b=M6   (현재 위치를 180°로, 안 움직임)")
    print("  홈이동: 1~6 = 그 모터를 180°로 이동,  0 = 전체 180°로 이동")
    print("  누르고 있으면 가속(최대 %.0f°/s), 떼면 정지. ESC=종료\n" % MAX_SPEED)

    fd = sys.stdin.fileno()
    old_attr = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)  # 한 글자씩 즉시 읽기 (Ctrl-C는 살아있음)
        next_status = 0.0

        while True:
            loop_start = time.monotonic()

            # 1) stdin에 쌓인 키를 모두 읽어 처리
            while select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1)
                if ch == "\x1b":  # ESC
                    raise KeyboardInterrupt
                if ch in MOVE_KEYS:  # 이동 키 (누르고 있으면 속도 램프)
                    last_seen[ch] = loop_start
                elif (
                    ch in CALIB_KEYS
                ):  # 영점 보정: 축 멤버 전체를 180°로 (모터 안 움직임)
                    ai = CALIB_KEYS[ch]
                    members = AXES[ai]["members"]
                    for mid, _sign in members:
                        if mid <= MOTOR_COUNT:
                            send_set_middle(bus, mid)
                            target[mid] = (
                                HOME_ANGLE  # 보정 후 현재=180°이므로 목표도 180으로
                            )
                    axis_speed[ai] = 0.0
                    names = ",".join(f"M{mid}" for mid, _s in members)
                    sys.stdout.write(
                        f"\n[영점] {names} 현재 위치를 180°로 설정 (서보 EEPROM 변경)\n"
                    )
                    sys.stdout.flush()
                elif ch == "0":  # 전체 모터를 180°로 이동
                    for hm in range(1, MOTOR_COUNT + 1):
                        target[hm] = HOME_ANGLE
                        send_angle(bus, hm, HOME_ANGLE, move_ms=HOME_MOVE_MS)
                    for ai in axis_speed:
                        axis_speed[ai] = 0.0
                elif ch in "123456":  # 숫자키 = 해당 모터를 180°로 이동
                    hm = int(ch)
                    if 1 <= hm <= MOTOR_COUNT:
                        target[hm] = HOME_ANGLE
                        send_angle(bus, hm, HOME_ANGLE, move_ms=HOME_MOVE_MS)

            # 2) 축별 방향 판정 → 속도 램프 → 멤버 목표각 적분(sign 적용) → 송신
            now = loop_start
            for ai, ax in enumerate(AXES):
                fwd = (
                    ax["fwd"] in last_seen
                    and (now - last_seen[ax["fwd"]]) < HOLD_TIMEOUT
                )
                back = (
                    ax["back"] in last_seen
                    and (now - last_seen[ax["back"]]) < HOLD_TIMEOUT
                )

                direction = 0
                if fwd and not back:
                    direction = +1
                elif back and not fwd:
                    direction = -1

                if direction == 0:
                    axis_speed[ai] = 0.0  # 떼면 즉시 정지
                else:
                    if (
                        direction != axis_prev_dir[ai]
                    ):  # 새로 누르거나 방향 바뀌면 램프 리셋
                        axis_speed[ai] = 0.0
                    axis_speed[ai] = min(axis_speed[ai] + RAMP_RATE * DT, MAX_SPEED)
                    # gain으로 축마다 속도 배율 (M4는 5배). M4는 한 틱에 더 크게 움직여 더 세게 밀어붙임.
                    delta = direction * axis_speed[ai] * DT * ax["gain"]
                    for mid, sign in ax["members"]:
                        if target[mid] is None:  # 첫 입력 시 현재각으로 시드(점프 방지)
                            a = measured.get(mid)
                            target[mid] = a if a is not None else DEFAULT_ANGLE
                        target[mid] += sign * delta  # 거울쌍은 sign=-1로 반대방향
                        target[mid] = max(ANGLE_MIN, min(ANGLE_MAX, target[mid]))
                        send_angle(bus, mid, target[mid], move_ms=STREAM_MOVE_MS)

                axis_prev_dir[ai] = direction

            # 3) 상태 한 줄 표시 (5Hz, 덮어쓰기) — 모터별 실제 이동방향 표시
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
                sys.stdout.write("\r" + "  ".join(parts) + "   ")
                sys.stdout.flush()

            # 4) 루프 주기 유지
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
