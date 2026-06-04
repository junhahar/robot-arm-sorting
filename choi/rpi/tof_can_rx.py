#!/usr/bin/env python3
"""
tof_can_rx.py
--------------------------------------------------------------
삼보모터스 5축 로봇암 - VL53L0X ToF 거리 CAN 수신 (RPi5)
--------------------------------------------------------------
체인:  VL53L0X --I2C--> Arduino Nano --SPI--> MCP2515 --CAN--> RPi5(can0)
Nano(nano_tof_can_sender.ino)가 보내는 ID 0x300 / DLC 8 / little-endian
프레임에서 거리(mm)와 status를 뽑아 한 줄로 표시한다.

프레임 0x300 (little-endian):
    byte[0..1]  거리 uint16 (mm),  8190 이상이면 범위초과/실패
    byte[2]     status  0=OK, 1=범위초과, 2=센서타임아웃/초기화실패
    byte[3]     예약(0)
    byte[4..5]  예약(0)
    byte[6..7]  카운터 uint16  (프레임 유실 검출용)

사전 준비 (RPi5, SSH):
    sudo ip link set can0 down
    sudo ip link set can0 type can bitrate 125000 sample-point 0.625
    sudo ip link set can0 up

실행 (RPi5, SSH):
    python3 tof_can_rx.py
    Ctrl+C 로 종료

다른 모듈에서 쓰려면:  from tof_can_rx import read_distance_mm
"""

import struct
import time

import can

CHANNEL   = "can0"
EXPECT_ID = 0x300

ST_OK, ST_OUTRANGE, ST_SENSORERR = 0, 1, 2
_ST_TEXT = {ST_OK: "OK", ST_OUTRANGE: "범위초과", ST_SENSORERR: "센서오류"}


def parse_tof(data: bytes):
    """0x300 페이로드(>=7B)를 (거리_mm, status, counter)로 디코드. little-endian."""
    dist, status = struct.unpack("<HB", bytes(data[0:3]))
    counter = struct.unpack("<H", bytes(data[6:8]))[0] if len(data) >= 8 else 0
    return dist, status, counter


def open_bus():
    try:
        return can.interface.Bus(channel=CHANNEL, interface="socketcan")
    except OSError as e:
        print(f"[ERR] {CHANNEL} 열기 실패: {e}")
        print("       'sudo ip link set can0 type can bitrate 125000 "
              "sample-point 0.625' 후 'up' 했는지 확인")
        return None


def read_distance_mm(timeout: float = 1.0):
    """1회성 헬퍼: 유효(status=OK) ToF 거리 mm를 반환. 실패 시 None.
    체크포인트(GRASP 깊이 보정) 등에서 단발 호출용."""
    bus = open_bus()
    if bus is None:
        return None
    try:
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = bus.recv(timeout=timeout)
            if msg is None or msg.arbitration_id != EXPECT_ID or len(msg.data) < 3:
                continue
            dist, status, _ = parse_tof(msg.data)
            if status == ST_OK:
                return dist
        return None
    finally:
        bus.shutdown()


def main() -> None:
    print(f"CAN RX  channel={CHANNEL}  expect ID=0x{EXPECT_ID:03X} (VL53L0X ToF)")
    print("(Ctrl+C 로 종료)")

    bus = open_bus()
    if bus is None:
        return

    prev_counter = None
    dropped = 0

    try:
        while True:
            msg = bus.recv(timeout=2.0)
            if msg is None:
                print(f"[{time.strftime('%H:%M:%S')}]  ... 2초간 프레임 없음 "
                      "(Nano 전원/배선/비트레이트 확인)")
                continue

            if msg.arbitration_id != EXPECT_ID or len(msg.data) < 3:
                continue

            dist, status, counter = parse_tof(msg.data)

            # 카운터 연속성으로 프레임 유실 추정.
            # gap이 비정상적으로 크면(>1000) 실제 유실이 아니라 Nano 재부팅
            # (카운터 0으로 리셋)으로 간주하고 유실로 세지 않는다.
            if prev_counter is not None:
                gap = (counter - prev_counter) & 0xFFFF
                if gap > 1000:
                    print(f"   (Nano 재시작 감지: cnt {prev_counter}->{counter}, 유실 카운트 유지)")
                elif gap > 1:
                    dropped += gap - 1
            prev_counter = counter

            st_txt = _ST_TEXT.get(status, f"?{status}")
            if status == ST_OK:
                print(f"[수신] 거리 {dist:4d} mm  ({st_txt})  "
                      f"cnt={counter}  유실누적={dropped}")
            else:
                print(f"[수신] ---- mm  ({st_txt})  cnt={counter}  유실누적={dropped}")

    except KeyboardInterrupt:
        print()
        print(f"종료 (총 유실 추정 {dropped} 프레임)")
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
