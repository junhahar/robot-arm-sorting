import struct
from config import CAN_BAUDRATE, CAN_CMD_SERVO, CAN_CMD_GRIPPER, CAN_CMD_HOME, CAN_CMD_ESTOP


class CANBridge:
    """RPi5 MCP2515 SPI → CAN → Arduino Nano 통신"""

    def __init__(self):
        self.bus = None

    def connect(self):
        try:
            import can
            self.bus = can.interface.Bus(
                channel="can0", bustype="socketcan", bitrate=CAN_BAUDRATE,
            )
        except Exception as e:
            print(f"CAN 연결 실패 (하드웨어 없이 테스트 중): {e}")
            self.bus = None

    def send_angles(self, angles: dict):
        if not self.bus:
            print(f"[CAN TX] servo angles: {angles}")
            return
        for joint, angle in angles.items():
            data = struct.pack("Bf", int(joint[1]), angle)
            msg = __import__("can").Message(
                arbitration_id=CAN_CMD_SERVO, data=data, is_extended_id=False,
            )
            self.bus.send(msg)

    def send_gripper(self, open_close: bool):
        if not self.bus:
            state = "open" if open_close else "close"
            print(f"[CAN TX] gripper: {state}")
            return
        data = struct.pack("B", 1 if open_close else 0)
        msg = __import__("can").Message(
            arbitration_id=CAN_CMD_GRIPPER, data=data, is_extended_id=False,
        )
        self.bus.send(msg)

    def send_home(self):
        if not self.bus:
            print("[CAN TX] home command")
            return
        msg = __import__("can").Message(
            arbitration_id=CAN_CMD_HOME, data=[], is_extended_id=False,
        )
        self.bus.send(msg)

    def send_estop(self):
        if not self.bus:
            print("[CAN TX] E-STOP!")
            return
        msg = __import__("can").Message(
            arbitration_id=CAN_CMD_ESTOP, data=[], is_extended_id=False,
        )
        self.bus.send(msg)

    def receive(self):
        if not self.bus:
            return None
        return self.bus.recv(timeout=0.01)

    def close(self):
        if self.bus:
            self.bus.shutdown()
