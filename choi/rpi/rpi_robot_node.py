#!/usr/bin/env python3
"""
rpi_robot_node.py  — RPi5 통합 노드
================================================================
한 CAN 버스에서 동시에:
  · 모터 제어     RPi → STM32   0x100 (명령) / 0x200 (ACK)
  · 모터 상태수신 STM32 → RPi   0x201 (모터별 각도·온도·부하, 모터 1개씩 순환)
  · 센서 수신     Nano → RPi    0x300 (온/습도·조도·카운터, 100ms)

기존 rpi_motor_control.py(모터) + can_rx_demo.py(센서)를 합친 것.
모터 제어는 여러 개를 묶어 '동시에' 구동한다(2개→3개→4개→5개 스윕, 또는 고정 N개씩).

구조
  버스를 읽는 건 Notifier 스레드 1개뿐. 모든 프레임을 받아
    0x200 → ACK 큐로       (메인 스레드가 모터 명령 후 대기)
    0x201 → 모터 상태 저장소로 (STM32가 보내는 각도/온도/부하)
    0x300 → 센서 상태 저장소로
  메인 스레드는 송신(0x100)만 하고 recv는 안 함 → 셋이 프레임 안 뺏음.

CAN 셋업 (규약 §5.1 — 125kbps / 샘플포인트 62.5%):
  sudo ip link set can0 down
  sudo ip link set can0 type can bitrate 125000 sample-point 0.625
  sudo ip link set can0 up
  # ※ 2번째 줄은 반드시 한 줄로. 줄바꿈 시 'sample-point: command not found'

실행:
  python3 rpi_robot_node.py        # Ctrl+C 종료
"""

import time
import struct
import threading
import queue
import can

CAN_INTERFACE = "can0"
SERVO_COUNT   = 5

# ── CAN ID (규약 §3) ──
CAN_ID_CMD    = 0x100   # RPi → STM32   모터 명령
CAN_ID_ACK    = 0x200   # STM32 → RPi   모터 ACK
CAN_ID_TELEM  = 0x201   # STM32 → RPi   모터 상태(각도/온도/부하), 모터 1개씩 순환
CAN_ID_SENSOR = 0x300   # Nano → RPi    센서 데이터

# ── 모터 cmd (규약 §4.1) ──
CMD_POSITION  = 0x01
CMD_TORQUE    = 0x02
CMD_PING      = 0x7F

# ── 동시 구동 데모 설정 ──
POS_A    = 90.0    # 목표 각도 A
POS_B    = 270.0   # 목표 각도 B
SETTLE_S = 2.0     # 한 그룹 이동 후 다 멈출 때까지 대기(이 동안 상태 출력)
# GROUP_SIZE = 0  → 2개→3개→4개→5개로 점점 늘려가며 동시 구동(스윕 데모)
# GROUP_SIZE = N  → 항상 N개씩 묶어서 동시 구동 (1~5)
GROUP_SIZE = 0


def deg_to_step(deg: float) -> int:
    step = int(round((deg / 360.0) * 4096))
    return max(0, min(4095, step))


# ── 센서 상태 저장소 (Notifier 스레드가 쓰고, 메인 스레드가 읽음) ──
class SensorState:
    def __init__(self):
        self._lock = threading.Lock()
        self.temp_c = None
        self.humi_pct = None
        self.lux = None
        self.counter = None
        self.last_rx = 0.0
        self._last_counter = None
        self.drop_count = 0          # 카운터 누락(시퀀스 끊김) 누적

    def update(self, data) -> None:
        # 규약 §4.3, little-endian:  int16 온도, uint16 습도, uint16 조도, uint16 카운터
        temp_raw, humi_raw, lux, counter = struct.unpack("<hHHH", bytes(data[0:8]))
        with self._lock:
            self.temp_c = temp_raw / 100.0
            self.humi_pct = humi_raw / 100.0
            self.lux = lux
            self.counter = counter
            self.last_rx = time.time()
            if self._last_counter is not None:
                gap = (counter - self._last_counter) & 0xFFFF
                if gap != 1:
                    self.drop_count += max(0, gap - 1)
            self._last_counter = counter

    def snapshot(self):
        with self._lock:
            return (self.temp_c, self.humi_pct, self.lux,
                    self.counter, self.last_rx, self.drop_count)


# ── 모터 상태 저장소 (STM32 0x201, 모터 1개씩 순환 수신 → 모터별로 누적) ──
class MotorState:
    def __init__(self, count: int = SERVO_COUNT):
        self._lock = threading.Lock()
        # motor_id(1~count) → dict(angle_deg, temp_c, load_pct, flags, last_rx)
        self._m = {i: {"angle_deg": None, "temp_c": None, "load_pct": None,
                       "flags": 0, "last_rx": 0.0} for i in range(1, count + 1)}

    def update(self, data) -> None:
        # STM32 0x201 프레임 (DLC 7, little-endian):
        #   [0] motor_id  [1..2] angle×10(0~3600)  [3] tempC  [4..5] load raw  [6] flags
        #   flags bit0=각도OK, bit1=온도OK, bit2=부하OK (0이면 STM 쪽 읽기 실패)
        #   ※ STM이 step이 아니라 angle×10으로 보냄 → ÷10 해서 도(°)로 환산
        mid, angle_x10, tempC, load_raw, flags = struct.unpack("<BHBHB", bytes(data[0:7]))
        if mid not in self._m:
            return
        angle = angle_x10 / 10.0 if (flags & 0x01) else None
        temp = tempC if (flags & 0x02) else None
        load = (load_raw & 0x03FF) * 0.1 if (flags & 0x04) else None
        with self._lock:
            self._m[mid].update(angle_deg=angle, temp_c=temp, load_pct=load,
                                flags=flags, last_rx=time.time())

    def snapshot(self):
        with self._lock:
            # 얕은 복사로 반환 (호출자가 안전하게 읽도록)
            return {mid: dict(v) for mid, v in self._m.items()}


# ── RX 라우터: Notifier 스레드에서 프레임마다 호출됨 ──
class RxRouter(can.Listener):
    def __init__(self, ack_queue: queue.Queue, sensor: SensorState, motors: MotorState):
        self.ack_queue = ack_queue
        self.sensor = sensor
        self.motors = motors

    def on_message_received(self, msg: can.Message) -> None:
        if msg.arbitration_id == CAN_ID_ACK and len(msg.data) >= 3:
            self.ack_queue.put((msg.data[0], msg.data[1], msg.data[2]))
        elif msg.arbitration_id == CAN_ID_TELEM and len(msg.data) >= 7:
            self.motors.update(msg.data)
        elif msg.arbitration_id == CAN_ID_SENSOR and len(msg.data) >= 8:
            self.sensor.update(msg.data)
        # 그 외 ID(0x37F ping 등)는 무시


# ── 모터 송신 + ACK 대기 ──
def send_cmd(bus: can.BusABC, data: list) -> None:
    bus.send(can.Message(arbitration_id=CAN_ID_CMD, data=data, is_extended_id=False))


def wait_ack(ack_queue: queue.Queue, expected_cmd: int, expected_motor: int,
             timeout: float = 1.0) -> bool:
    deadline = time.time() + timeout
    while True:
        remaining = deadline - time.time()
        if remaining <= 0:
            print(f"      ACK 없음 (cmd=0x{expected_cmd:02X} motor={expected_motor})")
            return False
        try:
            cmd, motor_id, status = ack_queue.get(timeout=remaining)
        except queue.Empty:
            continue
        if cmd != expected_cmd or motor_id != expected_motor:
            continue                 # 엉뚱한/지난 ACK → 버리고 계속 대기
        return status == 0x00


def ping(bus, ack_queue) -> bool:
    send_cmd(bus, [CMD_PING, 0, 0, 0, 0, 0, 0, 0])
    return wait_ack(ack_queue, CMD_PING, 0, timeout=2.0)


def set_torque(bus, ack_queue, motor_id: int, on: bool) -> bool:
    send_cmd(bus, [CMD_TORQUE, motor_id, 0, 0, 0, 0, 1 if on else 0, 0])
    return wait_ack(ack_queue, CMD_TORQUE, motor_id)


def send_position(bus, ack_queue, motor_id: int, deg: float) -> bool:
    step = deg_to_step(deg)
    send_cmd(bus, [CMD_POSITION, motor_id, step & 0xFF, (step >> 8) & 0xFF, 0, 0, 0, 0])
    print(f"[TX] motor {motor_id} → {deg:.1f}° (step {step})")
    return wait_ack(ack_queue, CMD_POSITION, motor_id)


def make_groups(group_size: int):
    """모터 1..SERVO_COUNT를 group_size개씩 묶는다. 예) size=2 → [[1,2],[3,4],[5]]"""
    ids = list(range(1, SERVO_COUNT + 1))
    return [ids[i:i + group_size] for i in range(0, len(ids), group_size)]


def move_group(bus, ack_queue, sensor, motors, group, deg: float,
               settle: float = SETTLE_S) -> None:
    """group(모터 id 리스트)을 동시에 deg로 이동시킨다.

    명령들을 수 ms 안에 연달아 보내면(각 ACK 왕복 ms 단위) 서보들이 사실상
    같이 출발한다. 모두 명령한 뒤 settle초 동안 함께 움직이는 모습을 출력한다.
    """
    print(f"[GROUP] {len(group)}개 동시 이동 → "
          f"{', '.join(f'M{m}' for m in group)} = {deg:.0f}°")
    for motor_id in group:
        send_position(bus, ack_queue, motor_id, deg)   # 빠르게 연달아 발사
    idle_with_status(sensor, motors, settle)           # 같이 움직이는 동안 대기·표시


def print_sensor_status(sensor: SensorState) -> None:
    t, h, lux, cnt, last_rx, drops = sensor.snapshot()
    if t is None:
        print("[센서] 수신 없음 (Nano 미동작/배선 확인)")
        return
    stale = "  ⚠STALE" if (time.time() - last_rx) > 0.5 else ""   # 100ms 주기인데 0.5s 끊기면 경고
    print(f"[센서] {t:5.1f}°C  {h:4.1f}%  {lux:5d}lux  cnt={cnt}  drop={drops}{stale}")


def print_motor_status(motors: MotorState) -> None:
    snap = motors.snapshot()
    if all(m["last_rx"] == 0.0 for m in snap.values()):
        print("[모터] 상태 수신 없음 (STM32가 0x201 미전송/펌웨어 확인)")
        return
    parts = []
    now = time.time()
    for mid in sorted(snap):
        m = snap[mid]
        if m["last_rx"] == 0.0:
            parts.append(f"M{mid}: --")
            continue
        ang = f"{m['angle_deg']:5.1f}°" if m["angle_deg"] is not None else "  NA "
        tmp = f"{m['temp_c']:2d}C" if m["temp_c"] is not None else "NA"
        ld  = f"{m['load_pct']:4.1f}%" if m["load_pct"] is not None else "  NA"
        stale = "!" if (now - m["last_rx"]) > 1.5 else ""   # 순환주기 고려해 1.5s
        parts.append(f"M{mid}:{ang} {tmp} {ld}{stale}")
    print("[모터] " + " | ".join(parts))


# ── 모터 동작 사이 대기 + 최신 센서/모터 상태 출력 ──
def idle_with_status(sensor: SensorState, motors: MotorState,
                     seconds: float, period: float = 0.5) -> None:
    end = time.time() + seconds
    while time.time() < end:
        print_motor_status(motors)
        print_sensor_status(sensor)
        time.sleep(min(period, max(0.0, end - time.time())))


def main():
    try:
        bus = can.interface.Bus(channel=CAN_INTERFACE, interface="socketcan")
    except OSError as e:
        print(f"[ERR] {CAN_INTERFACE} 열기 실패: {e}")
        print("       먼저 can0를 125kbps로 올렸는지 확인 (규약 §5.1):")
        print("       sudo ip link set can0 type can bitrate 125000 sample-point 0.625")
        return

    print(f"CAN 열림: {CAN_INTERFACE} (125kbps / 62.5%)")

    sensor = SensorState()
    motors = MotorState()
    ack_queue: queue.Queue = queue.Queue()
    notifier = can.Notifier(bus, [RxRouter(ack_queue, sensor, motors)])

    try:
        print("ping 전송...")
        if not ping(bus, ack_queue):
            print("Nucleo 응답 없음. 펌웨어/배선 확인.")
            return
        print("ping OK\n")

        # 전 서보 토크 ON
        for i in range(1, 6):
            set_torque(bus, ack_queue, i, True)
            time.sleep(0.05)

        # 그룹 동시 구동: 한 그룹을 A각도로 같이 → B각도로 같이 이동 반복
        cycle = 0
        while True:
            cycle += 1
            # GROUP_SIZE=0 이면 2→3→4→5개로 스윕, 아니면 고정 크기
            sizes = range(2, SERVO_COUNT + 1) if GROUP_SIZE == 0 else [GROUP_SIZE]
            for n in sizes:
                print(f"\n===== cycle {cycle} | {n}개씩 동시 구동 =====")
                for group in make_groups(n):
                    move_group(bus, ack_queue, sensor, motors, group, POS_A)
                    move_group(bus, ack_queue, sensor, motors, group, POS_B)

    except KeyboardInterrupt:
        print("\n중지.")
    finally:
        notifier.stop()
        bus.shutdown()


if __name__ == "__main__":
    main()
