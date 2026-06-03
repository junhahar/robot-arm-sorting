#!/usr/bin/env python3
"""
rpi_load_monitor.py — 각 모터 '부하(load)' 실시간 모니터 (읽기 전용)
================================================================================
부하(load)를 메인 지표로 보여준다. 부하 = 그 모터가 내는 토크/힘의 세기(0~100%).
  · 0x201 텔레메트리에 들어있어 '항상 잘 읽힌다'. M2·M3 분담/과열 보기에 딱.
전류(mA)는 STS3215 Present Current(0x45)가 지원되는 서보에서만 같이 표시.
  · 0x45 미지원/고장이면 전류는 'NA'로 뜨지만 부하는 정상 표시된다.

제어 스크립트와 '다른 터미널'에서 동시에 실행 OK (socketCAN 브로드캐스트, 읽기 전용).

CAN 셋업: sudo ip link set can0 type can bitrate 125000 sample-point 0.625
실행:     python3 rpi_load_monitor.py      # Ctrl+C 종료
"""

import time
import struct
import can

CAN_INTERFACE = "can0"
CAN_ID_TELEM = 0x201     # [id, ax10_lo, ax10_hi, tempC, load_lo, load_hi, flags]
CAN_ID_CURRENT = 0x202   # [id, cur_lo, cur_hi, ok]
MOTOR_COUNT = 6

CURRENT_MA_PER_LSB = 6.5  # STS3215 Present Current 스케일 (지원 서보에서만)

DRAW_PERIOD = 0.2
STALE_AFTER = 1.5
BAR_WIDTH = 20            # 부하 막대 칸 수 (가득 = 100%)


def decode_load(raw):
    # STS3215 Present Load(0x3C): 하위 10비트 크기, bit10 방향. 0~1000 → %.
    magnitude = raw & 0x03FF
    if magnitude > 1000:
        magnitude = 1000
    direction = "-" if (raw & 0x0400) else "+"
    return magnitude * 0.1, direction   # 0~100 %


def decode_current(raw):
    magnitude = raw & 0x7FFF
    return magnitude * CURRENT_MA_PER_LSB   # mA


def bar(pct):
    n = int(min(max(pct, 0.0), 100.0) / 100.0 * BAR_WIDTH)
    return "#" * n + "." * (BAR_WIDTH - n)


def main():
    try:
        bus = can.interface.Bus(channel=CAN_INTERFACE, interface="socketcan")
    except OSError as e:
        print(f"[ERR] {CAN_INTERFACE} 열기 실패: {e}")
        print("       sudo ip link set can0 type can bitrate 125000 sample-point 0.625")
        return

    st = {m: {"load": None, "ldir": " ", "temp": None,
              "cur": None, "last_rx": 0.0} for m in range(1, MOTOR_COUNT + 1)}
    last_draw = 0.0

    print("\033[2J", end="")
    try:
        while True:
            msg = bus.recv(timeout=0.1)
            if msg is not None:
                d = msg.data
                if msg.arbitration_id == CAN_ID_TELEM and len(d) >= 7:
                    mid, _ax10, tempC, load_raw, flags = struct.unpack("<BHBHB", bytes(d[0:7]))
                    if mid in st:
                        if flags & 0x04:  # 부하 읽기 OK
                            st[mid]["load"], st[mid]["ldir"] = decode_load(load_raw)
                        if flags & 0x02:  # 온도 읽기 OK
                            st[mid]["temp"] = tempC
                        st[mid]["last_rx"] = time.time()
                elif msg.arbitration_id == CAN_ID_CURRENT and len(d) >= 4:
                    mid = d[0]
                    if mid in st:
                        raw = d[1] | (d[2] << 8)
                        ok = d[3]
                        st[mid]["cur"] = decode_current(raw) if ok else None
                        st[mid]["last_rx"] = time.time()

            now = time.time()
            if now - last_draw >= DRAW_PERIOD:
                last_draw = now
                lines = ["=== 모터 부하(load) / 전류 실시간 (Ctrl+C 종료) ===", ""]
                total_load = 0.0
                for m in range(1, MOTOR_COUNT + 1):
                    s = st[m]
                    if s["last_rx"] == 0.0:
                        lines.append(f"  M{m}  (수신 없음)")
                        continue
                    stale = "  ⚠STALE" if (now - s["last_rx"]) > STALE_AFTER else ""
                    lo = s["load"]
                    if lo is None:
                        load_str = " 부하 NA "
                        barstr = "." * BAR_WIDTH
                    else:
                        total_load += lo
                        load_str = f"{s['ldir']}{lo:5.1f}%"
                        barstr = bar(lo)
                    temp_str = f"{s['temp']:2d}C" if s["temp"] is not None else "NA"
                    cur_str = f"{s['cur']:5.0f}mA" if s["cur"] is not None else "전류NA"
                    lines.append(
                        f"  M{m}  부하 {load_str}  [{barstr}]  {temp_str}  {cur_str}{stale}"
                    )
                lines.append("")
                lines.append(f"  부하 합계: {total_load:5.1f}%")
                print("\033[H" + "\n".join(lines) + "\033[J", end="", flush=True)

    except KeyboardInterrupt:
        print("\n종료.")
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
