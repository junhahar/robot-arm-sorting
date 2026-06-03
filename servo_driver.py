"""
STS3215 서보 드라이버 — CAN 통신 (STM32 프로토콜)

통신 경로:
  RPi5 --USB--> CANable --CAN 125kbps--> STM32 Nucleo --UART--> STS3215

CAN 프로토콜 (STM32 펌웨어 기준):
  RPi5 -> STM32 (0x100):
    0x02  TORQUE     [cmd, id, 0,0,0,0, on_off, 0]
    0x03  SET_ANGLE  [cmd, id, ax10_lo, ax10_hi, mv_lo, mv_hi, 0, 0]
  STM32 -> RPi5:
    0x200  ACK
    0x201  TELEMETRY  [id, ax10_lo, ax10_hi, tempC, load_lo, load_hi, flags]
    0x202  CURRENT    [id, cur_lo, cur_hi, ok]
"""
import time
import struct
import math
import arm_ik

from config import (
    HOME, GRIPPER_OPEN, GRIPPER_CLOSE, GRIP_CLOSE_WAIT,
    CAN_CHANNEL, CAN_BUSTYPE, CAN_BITRATE,
)

# ── CAN IDs (STM32 펌웨어 기준) ──
CAN_ID_CMD   = 0x100
CAN_ID_ACK   = 0x200
CAN_ID_TELEM = 0x201
CAN_ID_CUR   = 0x202

# ── CAN Commands ──
CMD_TORQUE    = 0x02
CMD_SET_ANGLE = 0x03

# ── 기본 이동 시간 (ms) ──
DEFAULT_MOVE_MS = 40
HOME_MOVE_MS    = 800


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
                print(f"[SERVO] CAN: {CAN_CHANNEL} ({CAN_BITRATE}bps)")
            except Exception as e:
                print(f"[SERVO] CAN failed: {e} - dry-run")
                self.dry_run = True

    # ── CAN 송수신 ──

    def _send(self, arb_id, data):
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

    # ── 모터 각도 전송 (CMD_SET_ANGLE) ──

    def _send_motor_angle(self, motor_id, angle_deg, move_ms=DEFAULT_MOVE_MS):
        ax10 = int(round(angle_deg * 10))
        mt = int(move_ms) & 0xFFFF
        data = [
            CMD_SET_ANGLE,
            motor_id,
            ax10 & 0xFF, (ax10 >> 8) & 0xFF,
            mt & 0xFF, (mt >> 8) & 0xFF,
            0, 0,
        ]
        self._send(CAN_ID_CMD, bytes(data))

    # ── 관절 이동 (IK deg -> motor deg -> CAN) ──

    def move_joint(self, joint, angle_deg, move_ms=DEFAULT_MOVE_MS):
        if self.dry_run:
            self._sim_pos[joint] = angle_deg
            return
        q_partial = {joint: math.radians(angle_deg)}
        for j in ["J1", "J2", "J3", "J4", "J5"]:
            if j not in q_partial:
                q_partial[j] = math.radians(self._sim_pos.get(j, 180.0))
        motor_angles = arm_ik.joints_to_motor_angles(q_partial)
        for mid, cal_ref, d, jcal in arm_ik.MOTOR_CAL.get(joint, []):
            if mid in motor_angles:
                self._send_motor_angle(mid, motor_angles[mid], move_ms)
        self._sim_pos[joint] = angle_deg

    def move_all(self, angles_deg, move_ms=DEFAULT_MOVE_MS):
        if self.dry_run:
            self._sim_pos.update(angles_deg)
            return
        q_rad = {}
        for j in ["J1", "J2", "J3", "J4", "J5"]:
            if j in angles_deg:
                q_rad[j] = math.radians(angles_deg[j])
            else:
                q_rad[j] = math.radians(self._sim_pos.get(j, 180.0))
        motor_angles = arm_ik.joints_to_motor_angles(q_rad)
        for mid, adeg in motor_angles.items():
            self._send_motor_angle(mid, adeg, move_ms)
        self._sim_pos.update(angles_deg)

    # ── 위치 읽기 (텔레메트리) ──

    def read_joint(self, joint):
        if self.dry_run:
            return self._sim_pos.get(joint)
        return None

    def read_all(self):
        if self.dry_run:
            return dict(self._sim_pos)
        return None

    # ── 그리퍼 (SG90, STM32에서 PWM) ──

    def gripper(self, open_state):
        angle = GRIPPER_OPEN if open_state else GRIPPER_CLOSE
        if self.dry_run:
            return
        # TODO: 그리퍼 CAN 명령 확정 후 구현

    # ── ToF ──

    def read_tof(self):
        if self.dry_run:
            return 999
        # TODO: ToF CAN 명령 확정 후 구현
        return None

    # ── Torque ──

    def torque_on(self):
        if self.dry_run:
            return
        for mid in range(1, 7):
            data = [CMD_TORQUE, mid, 0, 0, 0, 0, 1, 0]
            self._send(CAN_ID_CMD, bytes(data))
            time.sleep(0.02)
        print("[SERVO] Torque ON")

    def torque_off(self):
        if self.dry_run:
            return
        for mid in range(1, 7):
            data = [CMD_TORQUE, mid, 0, 0, 0, 0, 0, 0]
            self._send(CAN_ID_CMD, bytes(data))
            time.sleep(0.02)
        print("[SERVO] Torque OFF")

    # ── E-STOP ──

    def estop(self):
        self.torque_off()

    # ── 홈 ──

    def home(self):
        self.move_all(HOME, move_ms=HOME_MOVE_MS)

    def close(self):
        if self.bus:
            self.bus.shutdown()
