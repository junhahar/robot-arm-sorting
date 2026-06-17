"""
STS3215 서보 드라이버 — CAN 통신

통신 경로:
  RPi5 --USB--> CANable Pro --CAN bus--> STM32 Nucleo --UART--> STS3215

CAN 프로토콜 (config.py에 ID 정의):
  RPi5 -> STM32:
    0x10  SERVO MOVE  [servo_id:u8, position:u16_be]
    0x11  SERVO READ  [servo_id:u8]
    0x12  GRIPPER     [angle:u8]
    0x13  TOF READ    []
    0xFF  E-STOP      []
  STM32 -> RPi5:
    0x20  SERVO POS   [servo_id:u8, position:u16_be]
    0x21  TOF DIST    [distance:u16_be]
"""
import time
import struct
from config import (
    JOINTS, SERVO_CENTER, SERVO_STEPS_PER_DEG,
    GRIPPER_OPEN, GRIPPER_CLOSE, HOME,
    CAN_CHANNEL, CAN_BUSTYPE, CAN_BITRATE,
    CAN_ID_SERVO_MOVE, CAN_ID_SERVO_READ,
    CAN_ID_GRIPPER, CAN_ID_TOF_READ, CAN_ID_ESTOP,
    CAN_ID_SERVO_POS, CAN_ID_TOF_DIST,
)


class ServoDriver:

    def __init__(self, dry_run=False):
        self.dry_run = dry_run
        self.bus = None
        self._sim_pos = dict(HOME)

        if not dry_run:
            try:
                import can
                self.bus = can.interface.Bus(
                    channel=CAN_CHANNEL,
                    bustype=CAN_BUSTYPE,
                    bitrate=CAN_BITRATE,
                )
                print(f"[SERVO] CAN 연결: {CAN_CHANNEL} ({CAN_BITRATE}bps)")
            except Exception as e:
                print(f"[SERVO] CAN 연결 실패: {e} - dry-run 전환")
                self.dry_run = True

    # ── CAN 송수신 ──

    def _send(self, arb_id, data=b""):
        if self.dry_run:
            return
        import can
        msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
        self.bus.send(msg)

    def _recv(self, expected_id, timeout=0.1):
        if self.dry_run:
            return None
        msg = self.bus.recv(timeout=timeout)
        if msg and msg.arbitration_id == expected_id:
            return msg.data
        return None

    # ── 각도 <-> 서보 위치 변환 ──

    @staticmethod
    def _calc_pos(servo_dir, angle, home):
        offset = angle - home
        return int(SERVO_CENTER + servo_dir * offset * SERVO_STEPS_PER_DEG)

    @staticmethod
    def _calc_angle(servo_dir, pos, home):
        offset = (pos - SERVO_CENTER) / SERVO_STEPS_PER_DEG / servo_dir
        return home + offset

    # ── 이동 명령 ──

    def move_joint(self, joint, angle):
        cfg = JOINTS[joint]
        angle = max(cfg["min"], min(cfg["max"], angle))
        if self.dry_run:
            self._sim_pos[joint] = angle
            return
        servo_ids = cfg["servo_id"]
        dirs = cfg["dir"]
        if isinstance(servo_ids, list):
            for sid, d in zip(servo_ids, dirs):
                pos = self._calc_pos(d, angle, cfg["home"])
                self._send(CAN_ID_SERVO_MOVE, struct.pack(">BH", sid, pos))
        else:
            pos = self._calc_pos(dirs, angle, cfg["home"])
            self._send(CAN_ID_SERVO_MOVE, struct.pack(">BH", servo_ids, pos))

    def move_all(self, angles):
        for joint, angle in angles.items():
            if joint in JOINTS:
                self.move_joint(joint, angle)

    # ── 위치 읽기 ──

    def read_joint(self, joint):
        if self.dry_run:
            return self._sim_pos.get(joint)
        cfg = JOINTS[joint]
        servo_ids = cfg["servo_id"]
        dirs = cfg["dir"]
        sid = servo_ids[0] if isinstance(servo_ids, list) else servo_ids
        d = dirs[0] if isinstance(dirs, list) else dirs
        self._send(CAN_ID_SERVO_READ, struct.pack("B", sid))
        data = self._recv(CAN_ID_SERVO_POS)
        if data and len(data) >= 3:
            _, pos = struct.unpack(">BH", data[:3])
            return self._calc_angle(d, pos, cfg["home"])
        return None

    def read_all(self):
        if self.dry_run:
            return dict(self._sim_pos)
        result = {}
        for joint in JOINTS:
            val = self.read_joint(joint)
            if val is not None:
                result[joint] = val
        return result

    # ── 그리퍼 (SG90) ──

    def gripper(self, open_state):
        angle = GRIPPER_OPEN if open_state else GRIPPER_CLOSE
        if self.dry_run:
            return
        self._send(CAN_ID_GRIPPER, struct.pack("B", angle))

    # ── ToF ──

    def read_tof(self):
        if self.dry_run:
            return 999
        self._send(CAN_ID_TOF_READ)
        data = self._recv(CAN_ID_TOF_DIST)
        if data and len(data) >= 2:
            return struct.unpack(">H", data[:2])[0]
        return None

    # ── E-STOP ──

    def estop(self):
        self._send(CAN_ID_ESTOP)

    # ── 홈 ──

    def home(self):
        self.move_all(HOME)

    def close(self):
        if self.bus:
            self.bus.shutdown()
