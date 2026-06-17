"""CNC LED+buzzer status helper for a separate display Nano.

The Raspberry Pi owns the workflow state. The display Nano only renders the
state and sends rack-reset requests.
"""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Any, Iterable

try:
    import can
except ImportError:  # pragma: no cover - dry-run mode does not need python-can.
    can = None  # type: ignore[assignment]


CAN_ID_CNC_STATUS = 0x410
CAN_ID_RACK_RESET = 0x411
CMD_CNC_STATUS = 0x20
CMD_RACK_RESET = 0x31
DEFAULT_CNC_PROCESS_SEC = 20.0
DEFAULT_RACK_CAPACITY = 2
DEFAULT_TX_PERIOD_SEC = 0.2


class CncStatus(IntEnum):
    IDLE = 0
    MACHINING = 1
    DONE_WAIT = 2
    RACK_FULL = 3
    ERROR = 4
    CAN_LOST = 255


def _u8(value: int | float | None) -> int:
    if value is None:
        return 0
    return max(0, min(255, int(value)))


def calc_cnc_status(
    *,
    now: float,
    cnc_busy: bool,
    cnc_start: float | None,
    rack_count: int,
    rack_capacity: int = DEFAULT_RACK_CAPACITY,
    error: bool = False,
    cnc_process_sec: float = DEFAULT_CNC_PROCESS_SEC,
) -> CncStatus:
    """Return the simplified 5-state CNC status for the display Nano."""

    if error:
        return CncStatus.ERROR

    rack_full = rack_capacity > 0 and rack_count >= rack_capacity

    if cnc_busy:
        if cnc_start is None:
            return CncStatus.MACHINING

        elapsed = max(0.0, now - cnc_start)
        if elapsed < cnc_process_sec:
            return CncStatus.MACHINING

        if rack_full:
            return CncStatus.RACK_FULL

        return CncStatus.DONE_WAIT

    if rack_full:
        return CncStatus.RACK_FULL

    return CncStatus.IDLE


def remaining_cnc_seconds(
    *,
    now: float,
    cnc_busy: bool,
    cnc_start: float | None,
    cnc_process_sec: float = DEFAULT_CNC_PROCESS_SEC,
) -> int:
    if not cnc_busy or cnc_start is None:
        return 0
    remaining = max(0.0, cnc_process_sec - max(0.0, now - cnc_start))
    return _u8(math.ceil(remaining))


def build_status_payload(
    state: CncStatus | int,
    *,
    rack_count: int,
    rack_capacity: int,
    remaining_sec: int,
    flags: int = 0,
    seq: int = 0,
) -> bytes:
    return bytes(
        [
            CMD_CNC_STATUS,
            _u8(int(state)),
            _u8(rack_count),
            _u8(rack_capacity),
            _u8(remaining_sec),
            _u8(flags),
            _u8(seq),
            0,
        ]
    )


def build_rack_reset_payload(*, seq: int = 0) -> bytes:
    return bytes([CMD_RACK_RESET, 1, _u8(seq), 0, 0, 0, 0, 0])


def is_rack_reset_request(msg: Any) -> bool:
    data = bytes(getattr(msg, "data", b""))
    return (
        getattr(msg, "arbitration_id", None) == CAN_ID_RACK_RESET
        and len(data) >= 2
        and data[0] == CMD_RACK_RESET
        and data[1] == 1
    )


@dataclass
class CncStatusCanNode:
    """CAN helper for the separate display Nano."""

    bus: Any
    period_s: float = DEFAULT_TX_PERIOD_SEC
    seq: int = 0
    reset_requested: bool = False
    reset_seq: int | None = None
    _last_tx: float = field(default=0.0, init=False)

    def build_status_message(
        self,
        state: CncStatus | int,
        *,
        rack_count: int,
        rack_capacity: int,
        remaining_sec: int,
        flags: int = 0,
    ) -> Any:
        if can is None:
            raise RuntimeError("python-can is required for live CAN sending")

        payload = build_status_payload(
            state,
            rack_count=rack_count,
            rack_capacity=rack_capacity,
            remaining_sec=remaining_sec,
            flags=flags,
            seq=self.seq,
        )
        return can.Message(
            arbitration_id=CAN_ID_CNC_STATUS,
            data=payload,
            is_extended_id=False,
        )

    def send_status(
        self,
        state: CncStatus | int,
        *,
        rack_count: int,
        rack_capacity: int,
        remaining_sec: int,
        flags: int = 0,
    ) -> Any:
        msg = self.build_status_message(
            state,
            rack_count=rack_count,
            rack_capacity=rack_capacity,
            remaining_sec=remaining_sec,
            flags=flags,
        )
        self.bus.send(msg)
        self.seq = (self.seq + 1) & 0xFF
        self._last_tx = time.monotonic()
        return msg

    def send_status_if_due(
        self,
        state: CncStatus | int,
        *,
        rack_count: int,
        rack_capacity: int,
        remaining_sec: int,
        flags: int = 0,
        now: float | None = None,
        force: bool = False,
    ) -> Any | None:
        now = time.monotonic() if now is None else now
        if not force and now - self._last_tx < self.period_s:
            return None
        return self.send_status(
            state,
            rack_count=rack_count,
            rack_capacity=rack_capacity,
            remaining_sec=remaining_sec,
            flags=flags,
        )

    def handle_reset_frame(self, msg: Any) -> bool:
        if not is_rack_reset_request(msg):
            return False
        data = bytes(getattr(msg, "data", b""))
        self.reset_requested = True
        self.reset_seq = data[2] if len(data) >= 3 else None
        return True

    def pop_reset_request(self, *, robot_busy: bool) -> bool:
        """Return True only when a pending reset request may be accepted."""

        if not self.reset_requested:
            return False

        self.reset_requested = False
        if robot_busy:
            return False

        return True


def open_socketcan_bus(interface: str = "can0") -> Any:
    if can is None:
        raise RuntimeError("python-can is not installed")
    return can.Bus(channel=interface, interface="socketcan")


def parse_status(value: str) -> CncStatus:
    raw = value.strip()
    try:
        return CncStatus(int(raw, 0))
    except ValueError:
        pass

    key = raw.upper().replace("-", "_")
    if key.startswith("ST_"):
        key = key[3:]
    try:
        return CncStatus[key]
    except KeyError as exc:
        names = ", ".join(state.name for state in CncStatus)
        raise argparse.ArgumentTypeError(f"unknown state {value!r}; use one of {names}") from exc


def format_frame(can_id: int, payload: bytes, label: str) -> str:
    data = " ".join(f"{byte:02X}" for byte in payload)
    return f"id=0x{can_id:03X} {label} data=[{data}]"


def iter_demo_statuses() -> Iterable[tuple[CncStatus, int, int, int, float]]:
    yield CncStatus.IDLE, 0, DEFAULT_RACK_CAPACITY, 0, 2.0
    yield CncStatus.MACHINING, 0, DEFAULT_RACK_CAPACITY, 15, 3.0
    yield CncStatus.DONE_WAIT, 0, DEFAULT_RACK_CAPACITY, 0, 3.0
    yield CncStatus.RACK_FULL, DEFAULT_RACK_CAPACITY, DEFAULT_RACK_CAPACITY, 0, 4.0
    yield CncStatus.ERROR, DEFAULT_RACK_CAPACITY, DEFAULT_RACK_CAPACITY, 0, 3.0


def send_or_print_status(
    *,
    node: CncStatusCanNode | None,
    dry_run: bool,
    state: CncStatus,
    rack_count: int,
    rack_capacity: int,
    remaining_sec: int,
    seq: int,
) -> int:
    payload = build_status_payload(
        state,
        rack_count=rack_count,
        rack_capacity=rack_capacity,
        remaining_sec=remaining_sec,
        seq=seq,
    )

    if dry_run:
        print(format_frame(CAN_ID_CNC_STATUS, payload, state.name))
        return (seq + 1) & 0xFF

    if node is None:
        raise RuntimeError("node is required when dry_run is false")

    node.seq = seq
    node.send_status(
        state,
        rack_count=rack_count,
        rack_capacity=rack_capacity,
        remaining_sec=remaining_sec,
    )
    print(format_frame(CAN_ID_CNC_STATUS, payload, state.name))
    return node.seq


def listen_for_reset(bus: Any) -> int:
    print(f"listening for rack reset requests on 0x{CAN_ID_RACK_RESET:03X}")
    while True:
        msg = bus.recv(0.5)
        if msg is None:
            continue
        if is_rack_reset_request(msg):
            payload = bytes(msg.data)
            print(format_frame(CAN_ID_RACK_RESET, payload, "RACK_RESET_REQUEST"))


def main() -> int:
    parser = argparse.ArgumentParser(description="Send Sambo CNC LED+buzzer CAN frames")
    parser.add_argument("--interface", default="can0", help="SocketCAN interface, default: can0")
    parser.add_argument("--state", type=parse_status, default=CncStatus.IDLE)
    parser.add_argument("--rack-count", type=int, default=0)
    parser.add_argument("--rack-capacity", type=int, default=DEFAULT_RACK_CAPACITY)
    parser.add_argument("--remaining", type=int, default=0, help="remaining CNC seconds")
    parser.add_argument("--period", type=float, default=DEFAULT_TX_PERIOD_SEC)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--once", action="store_true", help="send one frame and exit")
    parser.add_argument("--cycle", action="store_true", help="cycle through all display states")
    parser.add_argument("--listen-reset", action="store_true", help="print 0x411 reset requests")
    parser.add_argument("--dry-run", action="store_true", help="print frames without opening CAN")
    args = parser.parse_args()

    bus = None if args.dry_run else open_socketcan_bus(args.interface)
    node = None if args.dry_run else CncStatusCanNode(bus, period_s=args.period)
    seq = 0

    try:
        if args.listen_reset:
            if bus is None:
                print(format_frame(CAN_ID_RACK_RESET, build_rack_reset_payload(seq=0), "RACK_RESET_REQUEST"))
                return 0
            return listen_for_reset(bus)

        if args.cycle:
            deadline = time.monotonic() + args.duration
            while time.monotonic() < deadline:
                for state, rack_count, rack_capacity, remaining, hold_s in iter_demo_statuses():
                    state_deadline = min(deadline, time.monotonic() + hold_s)
                    while time.monotonic() < state_deadline:
                        seq = send_or_print_status(
                            node=node,
                            dry_run=args.dry_run,
                            state=state,
                            rack_count=rack_count,
                            rack_capacity=rack_capacity,
                            remaining_sec=remaining,
                            seq=seq,
                        )
                        time.sleep(args.period)
            return 0

        seq = send_or_print_status(
            node=node,
            dry_run=args.dry_run,
            state=args.state,
            rack_count=args.rack_count,
            rack_capacity=args.rack_capacity,
            remaining_sec=args.remaining,
            seq=seq,
        )

        if args.once:
            return 0

        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            time.sleep(args.period)
            seq = send_or_print_status(
                node=node,
                dry_run=args.dry_run,
                state=args.state,
                rack_count=args.rack_count,
                rack_capacity=args.rack_capacity,
                remaining_sec=args.remaining,
                seq=seq,
            )
        return 0
    finally:
        if bus is not None:
            shutdown = getattr(bus, "shutdown", None)
            if shutdown is not None:
                shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
