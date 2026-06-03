#!/usr/bin/env python3
"""
rpi_current_monitor.py — 각 모터 전류 실시간 모니터 (읽기 전용)
================================================================
STM이 0x202로 보내는 모터별 전류(Present Current, 0x45)를 받아 표로 표시한다.
제어 스크립트(rpi_keyboard_control*.py)와 '다른 터미널'에서 동시에 실행해도 된다.
socketCAN은 브로드캐스트라 여러 프로세스가 같은 can0를 함께 수신할 수 있다.

이 스크립트는 아무것도 송신하지 않는다(읽기 전용) → 제어에 간섭 안 함.

전제:
    STM 펌웨어가 0x202(전류)를 보내야 한다. 안 보내면 표가 계속 '---'로 뜬다.
    (펌웨어에 CAN_ID_CURRENT 0x202 / readServoCurrentRaw 추가본이 올라가 있어야 함)

CAN 셋업 (규약 §5.1):
    sudo ip link set can0 type can bitrate 125000 sample-point 0.625
    sudo ip link set can0 up

실행:
    python3 rpi_current_monitor.py      # Ctrl+C 종료
"""

import time
import can

CAN_INTERFACE = "can0"
CAN_ID_CURRENT = 0x202   # [motor_id, cur_lo, cur_hi, ok]
MOTOR_COUNT = 6

# STS3215 Present Current 스케일. 데이터시트상 약 6.5mA/LSB.
# 실제 전류계와 비교해 안 맞으면 이 값만 보정하면 된다.
CURRENT_MA_PER_LSB = 6.5

DRAW_PERIOD = 0.2        # 화면 갱신 주기 [s]
STALE_AFTER = 1.0        # 이 시간 넘게 갱신 없으면 STALE 표시 [s]
BAR_FULL_MA = 2000.0     # 막대 가득 찰 전류 [mA] (2A)
BAR_WIDTH = 24           # 막대 칸 수


def decode_current(raw):
    # 보통 bit15가 방향, 하위 비트가 크기. 크기만 mA로 환산해 표시.
    magnitude = raw & 0x7FFF
    direction = "-" if (raw & 0x8000) else "+"
    return magnitude * CURRENT_MA_PER_LSB, direction


def bar(ma):
    n = int(min(ma, BAR_FULL_MA) / BAR_FULL_MA * BAR_WIDTH)
    return "#" * n + "." * (BAR_WIDTH - n)


def main():
    try:
        bus = can.interface.Bus(channel=CAN_INTERFACE, interface="socketcan")
    except OSError as e:
        print(f"[ERR] {CAN_INTERFACE} 열기 실패: {e}")
        print("       sudo ip link set can0 type can bitrate 125000 sample-point 0.625")
        return

    # motor_id → dict(ma, direction, ok, last_rx)
    state = {m: {"ma": 0.0, "dir": " ", "ok": 0, "last_rx": 0.0}
             for m in range(1, MOTOR_COUNT + 1)}
    last_draw = 0.0

    print("\033[2J", end="")  # 화면 클리어
    try:
        while True:
            msg = bus.recv(timeout=0.1)
            if msg is not None and msg.arbitration_id == CAN_ID_CURRENT and len(msg.data) >= 4:
                mid = msg.data[0]
                if mid in state:
                    raw = msg.data[1] | (msg.data[2] << 8)
                    ok = msg.data[3]
                    ma, d = decode_current(raw)
                    s = state[mid]
                    s["ma"] = ma if ok else 0.0
                    s["dir"] = d
                    s["ok"] = ok
                    s["last_rx"] = time.time()

            now = time.time()
            if now - last_draw >= DRAW_PERIOD:
                last_draw = now
                lines = []
                lines.append("=== 모터 전류 실시간 (Ctrl+C 종료) ===")
                lines.append(f"  scale={CURRENT_MA_PER_LSB:g} mA/LSB   bar full={BAR_FULL_MA:.0f}mA")
                lines.append("")
                total = 0.0
                for m in range(1, MOTOR_COUNT + 1):
                    s = state[m]
                    age = now - s["last_rx"]
                    if s["last_rx"] == 0.0:
                        lines.append(f"  M{m}   ---  (수신 없음)")
                        continue
                    if not s["ok"]:
                        lines.append(f"  M{m}   NA   (서보 읽기 실패)")
                        continue
                    stale = "  ⚠STALE" if age > STALE_AFTER else ""
                    total += s["ma"]
                    lines.append(
                        f"  M{m} {s['dir']}{s['ma']:6.0f} mA  ({s['ma']/1000:4.2f} A)  "
                        f"[{bar(s['ma'])}]{stale}"
                    )
                lines.append("")
                lines.append(f"  합계: {total:7.0f} mA  ({total/1000:4.2f} A)")
                # 커서 홈으로 이동 후 덮어쓰기 (깜빡임 최소화)
                print("\033[H" + "\n".join(lines) + "\033[J", end="", flush=True)

    except KeyboardInterrupt:
        print("\n종료.")
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
