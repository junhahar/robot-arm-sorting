#!/usr/bin/env python3
"""
rpi_keyboard_control.py — 키보드로 모터 1~6 속도(증분) 제어
================================================================
키 매핑 (왼쪽 = 각도+, 오른쪽 = 각도-):
    M1: q / a     M2: w / s     M3: e / d
    M4: r / f     M5: t / g     M6: y / h

누르고 있으면 속도가 0에서 선형으로 증가(RAMP_RATE),
MAX_SPEED에서 포화. 떼면 즉시 정지. ESC 또는 Ctrl-C로 종료.

원리:
    STS3215는 위치 제어 서보라 '속도' 명령이 없다. 그래서 RPi가 매 틱
    목표각을 (방향 × 속도 × dt)만큼 적분해서 새 목표각을 STM에 보낸다.
    40Hz로 0.1도 단위씩 갱신 → STM은 새 목표를 따라가며 부드럽게 회전.

CAN 셋업 (규약 §5.1 — 125kbps / 62.5%):
    sudo ip link set can0 down
    sudo ip link set can0 type can bitrate 125000 sample-point 0.625
    sudo ip link set can0 up

실행:
    python3 rpi_keyboard_control.py

주의:
    · 모터 6을 쓰려면 STM 펌웨어 SERVO_COUNT를 6으로 바꾸고 ID=6 서보가 있어야 한다.
      (STM은 현재 1~SERVO_COUNT 밖 ID를 STATUS_ERROR로 거부 → 서보 안 움직임)
    · 이 스크립트는 ACK를 기다리지 않고 위치 명령을 연속 송신한다(fire-and-forget).
      실시간 루프가 ACK 대기로 멈추지 않게 하기 위함.
    · SSH 터미널에서는 키를 '뗀' 이벤트가 없어, 키 반복이 HOLD_TIMEOUT 동안
      안 오면 뗀 것으로 판단한다. 누른 직후 잠깐 멈칫하면 HOLD_TIMEOUT을 키우거나
      로컬 키보드+evdev 방식으로 바꾸면 된다(필요하면 따로 제공).
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
# 0이면 매 스텝을 최대속도로 확 갔다가 멈춤을 반복 → 톡톡 끊겨 보인다.
# 키우면 더 부드럽지만 살짝 지연(≈MAX_SPEED×STREAM_MOVE_MS/1000 도)이 생긴다.
STREAM_MOVE_MS = 40
RAMP_RATE = 20.0  # 속도 기울기(가속도) [deg/s per second] — 요청대로 1
MAX_SPEED = 20.0  # 최대 속도 [deg/s] — 1초에 10도
# 참고: 가속도 1이면 정지→최대(10°/s)까지 약 10초 걸린다(누른 시간에 비례해 천천히 빨라짐).
#       처음부터 더 빠르게 붙이고 싶으면 RAMP_RATE만 키우면 된다.
HOLD_TIMEOUT = (
    0.5  # 키 반복 초기 지연을 넘기도록 크게(안 그러면 램프가 리셋돼 안 움직임).
)
# 단점: 뗀 뒤 이 시간만큼 더 돔(오버슈트 ≈ HOLD_TIMEOUT × 속도).

# 키 → (motor_id, 방향).  왼쪽 = +1(각도 증가), 오른쪽 = -1(각도 감소)
KEYMAP = {
    "q": (1, +1),
    "a": (1, -1),
    "w": (2, +1),
    "s": (2, -1),
    "e": (3, +1),
    "d": (3, -1),
    "r": (4, +1),
    "f": (4, -1),
    "t": (5, +1),
    "g": (5, -1),
    "y": (6, +1),
    "h": (6, -1),
}

# 영점 보정 키 → motor_id.  누르면 그 모터의 '현재 위치 = 180°'로 서보 설정 변경(안 움직임)
CALIB_KEYMAP = {"z": 1, "x": 2, "c": 3, "v": 4, "b": 5, "n": 6}


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
    mt = int(move_ms) & 0xFFFF  # 0=최대속도, >0=그 시간에 걸쳐 이동 (STM이 buf[4..5]로 읽음)
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

    # 모터별 상태
    target = {m: None for m in range(1, MOTOR_COUNT + 1)}  # 목표각 (첫 입력 때 시드)
    speed = {m: 0.0 for m in range(1, MOTOR_COUNT + 1)}  # 현재 램프 속도 [deg/s]
    prev_dir = {m: 0 for m in range(1, MOTOR_COUNT + 1)}  # 직전 방향
    last_seen = {}  # 키문자 → 마지막으로 본 시각(monotonic)

    print("키보드 제어 시작")
    print("  이동:  M1 q/a  M2 w/s  M3 e/d  M4 r/f  M5 t/g  M6 y/h   (왼쪽/오른쪽)")
    print("  영점:  z x c v b n = M1~M6 '현재 위치 수치'를 180°로 설정 (안 움직임, 서보 영구)")
    print("  홈이동: 1~6 = 그 모터를 180°로 이동,  0 = 전체 180°로 이동")
    print("  누르고 있으면 가속(최대 %.0f°/s), 떼면 정지. ESC=종료\n" % MAX_SPEED)

    fd = sys.stdin.fileno()
    old_attr = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)  # 한 글자씩 즉시 읽기 (Ctrl-C는 살아있음)
        next_status = 0.0

        while True:
            loop_start = time.monotonic()

            # 1) stdin에 쌓인 키를 모두 읽어 last_seen 갱신
            while select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1)
                if ch == "\x1b":  # ESC
                    raise KeyboardInterrupt
                if ch in KEYMAP:  # 이동 키 (누르고 있으면 속도 램프)
                    last_seen[ch] = loop_start
                elif ch in CALIB_KEYMAP:  # 영점 보정: 현재 위치를 180°로 (모터 안 움직임)
                    cm = CALIB_KEYMAP[ch]
                    if cm <= MOTOR_COUNT:
                        send_set_middle(bus, cm)
                        target[cm] = HOME_ANGLE  # 보정 후 현재=180°이므로 목표도 180으로
                        speed[cm] = 0.0
                        sys.stdout.write(
                            f"\n[영점] M{cm} 현재 위치를 180°로 설정 (서보 EEPROM 변경)\n"
                        )
                        sys.stdout.flush()
                elif ch == "0":  # 전체 모터를 180°로 이동
                    for hm in range(1, MOTOR_COUNT + 1):
                        target[hm] = HOME_ANGLE
                        speed[hm] = 0.0
                        send_angle(bus, hm, HOME_ANGLE, move_ms=HOME_MOVE_MS)
                elif ch in "123456":  # 숫자키 = 해당 모터를 180°로 이동
                    hm = int(ch)
                    if 1 <= hm <= MOTOR_COUNT:
                        target[hm] = HOME_ANGLE
                        speed[hm] = 0.0
                        send_angle(bus, hm, HOME_ANGLE, move_ms=HOME_MOVE_MS)

            # 2) 모터별로 방향 판정 → 속도 램프 → 목표각 적분 → 송신
            now = loop_start
            for m in range(1, MOTOR_COUNT + 1):
                left = right = False
                for ch, (mm, d) in KEYMAP.items():
                    if mm != m:
                        continue
                    fresh = ch in last_seen and (now - last_seen[ch]) < HOLD_TIMEOUT
                    if fresh and d > 0:
                        left = True
                    elif fresh and d < 0:
                        right = True

                direction = 0
                if left and not right:
                    direction = +1
                elif right and not left:
                    direction = -1

                if direction == 0:
                    speed[m] = 0.0  # 떼면 즉시 정지
                else:
                    if target[m] is None:  # 첫 입력 시 현재각으로 시드(점프 방지)
                        a = measured.get(m)
                        target[m] = a if a is not None else DEFAULT_ANGLE
                    if direction != prev_dir[m]:  # 새로 누르거나 방향 바뀌면 램프 리셋
                        speed[m] = 0.0
                    speed[m] = min(speed[m] + RAMP_RATE * DT, MAX_SPEED)
                    target[m] += direction * speed[m] * DT
                    target[m] = max(ANGLE_MIN, min(ANGLE_MAX, target[m]))
                    # move_ms를 루프 주기보다 약간 크게 줘서 서보가 멈춤 없이 연속 이동
                    send_angle(bus, m, target[m], move_ms=STREAM_MOVE_MS)

                prev_dir[m] = direction

            # 3) 상태 한 줄 표시 (5Hz, 덮어쓰기)
            if now >= next_status:
                next_status = now + 0.2
                parts = []
                for m in range(1, MOTOR_COUNT + 1):
                    arrow = (
                        "L" if prev_dir[m] > 0 else ("R" if prev_dir[m] < 0 else " ")
                    )
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
