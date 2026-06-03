#!/usr/bin/env python3
"""
RPi5 → CANable → CAN 버스 → SN65HVD230 → NUCLEO-F103RB → STS3215 x5

실행 전 준비 (규약 §5.1 — 125kbps / 샘플포인트 62.5%):
  sudo ip link set can0 down
  sudo ip link set can0 type can bitrate 125000 sample-point 0.625
  sudo ip link set can0 up
  # ※ 2번째 줄은 반드시 한 줄로. 줄바꿈하면 'sample-point: command not found'
  # ※ gs_usb는 restart-ms 미지원 → Bus-off 시 수동 down/up

실행:
  python3 rpi_motor_control.py
"""

import time
import can

CAN_INTERFACE = "can0"
CAN_ID_CMD = 0x100
CAN_ID_ACK = 0x200

CMD_POSITION = 0x01
CMD_TORQUE = 0x02
CMD_PING = 0x7F


def deg_to_step(deg: float) -> int:
    step = int(round((deg / 360.0) * 4096))
    return max(0, min(4095, step))


def send_frame(bus: can.BusABC, data: list) -> None:
    msg = can.Message(arbitration_id=CAN_ID_CMD, data=data, is_extended_id=False)
    bus.send(msg)


def wait_ack(
    bus: can.BusABC, expected_cmd: int, expected_motor: int, timeout=1.0
) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = bus.recv(timeout=max(0.0, deadline - time.time()))
        if msg is None:
            continue
        if msg.arbitration_id != CAN_ID_ACK or len(msg.data) < 3:
            continue
        cmd, motor_id, status = msg.data[0], msg.data[1], msg.data[2]
        if cmd != expected_cmd or motor_id != expected_motor:
            continue
        if status == 0x00:
            return True
        return False
    print(f"      ACK 없음 (cmd=0x{expected_cmd:02X} motor={expected_motor})")
    return False


def ping(bus: can.BusABC) -> bool:
    send_frame(bus, [CMD_PING, 0, 0, 0, 0, 0, 0, 0])
    return wait_ack(bus, CMD_PING, 0, timeout=2.0)


def set_torque(bus: can.BusABC, motor_id: int, on: bool) -> bool:
    data = [CMD_TORQUE, motor_id, 0, 0, 0, 0, 1 if on else 0, 0]
    send_frame(bus, data)
    return wait_ack(bus, CMD_TORQUE, motor_id)


def send_position(bus: can.BusABC, motor_id: int, deg: float) -> bool:
    step = deg_to_step(deg)
    data = [CMD_POSITION, motor_id, step & 0xFF, (step >> 8) & 0xFF, 0, 0, 0, 0]
    send_frame(bus, data)
    print(f"[TX] motor {motor_id} → {deg:.1f}° (step {step})")
    return wait_ack(bus, CMD_POSITION, motor_id)


def main():
    bus = can.interface.Bus(channel=CAN_INTERFACE, interface="socketcan")
    print(f"CAN 열림: {CAN_INTERFACE}")

    try:
        # 연결 확인
        print("ping 전송...")
        if not ping(bus):
            print("Nucleo 응답 없음. 펌웨어/배선 확인.")
            return
        print("ping OK\n")

        # 전 서보 토크 ON
        for i in range(1, 6):
            set_torque(bus, i, True)
            time.sleep(0.05)

        # 위치 순환: motor 1~5 각각 90° → 270° 반복
        sequence = [(i, deg) for i in range(1, 6) for deg in (90.0, 270.0)]

        cycle = 0
        while True:
            cycle += 1
            print(f"===== cycle {cycle} =====")
            for motor_id, deg in sequence:
                send_position(bus, motor_id, deg)
                time.sleep(2.0)

    except KeyboardInterrupt:
        print("\n중지.")
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
