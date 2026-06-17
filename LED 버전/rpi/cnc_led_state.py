"""CNC LED status helper for the Sambo robot arm dashboard.

This module is intentionally standalone so it can be copied next to
``rpi_dashboard_ws_bridge.py`` and imported by the process that already owns
the CAN bus.
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


CAN_ID_CNC_LED = 0x410
LED_CMD_CNC_STATE = 0x20
DEFAULT_CNC_PROCESS_SEC = 20.0
DEFAULT_RACK_CAPACITY = 2
DEFAULT_LED_TX_PERIOD_SEC = 0.2


class CncLedState(IntEnum):
    IDLE = 0
    LOADING = 1
    MACHINING = 2
    DONE_WAIT_UNLOAD = 3
    UNLOADING = 4
    RACK_FULL = 5
    ERROR = 6
    CAN_LOST = 255


def _u8(value: int | float | None) -> int:
    if value is None:
        return 0
    return max(0, min(255, int(value)))


def _normalized_phase(motion_phase: str | None) -> str:
    return (motion_phase or "").strip().upper().replace("-", "_")


def calc_cnc_led_state(
    *,
    now: float,
    cnc_busy: bool,
    cnc_start: float | None,
    rack_count: int,
    rack_capacity: int = DEFAULT_RACK_CAPACITY,
    motion_phase: str | None = None,
    error: bool = False,
    cnc_process_sec: float = DEFAULT_CNC_PROCESS_SEC,
) -> CncLedState:
    """Return the operator LED state from the current workflow state."""

    if error:
        return CncLedState.ERROR

    phase = _normalized_phase(motion_phase)
    if phase in {"LOADING", "LOAD", "CNC_LOADING"}:
        return CncLedState.LOADING
    if phase in {"UNLOADING", "UNLOAD", "CNC_UNLOADING"}:
        return CncLedState.UNLOADING

    rack_full = rack_capacity > 0 and rack_count >= rack_capacity

    if cnc_busy:
        if cnc_start is None:
            return CncLedState.MACHINING

        elapsed = max(0.0, now - cnc_start)
        if elapsed < cnc_process_sec:
            return CncLedState.MACHINING

        if rack_full:
            return CncLedState.RACK_FULL

        return CncLedState.DONE_WAIT_UNLOAD

    if rack_full:
        return CncLedState.RACK_FULL

    return CncLedState.IDLE


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


def build_cnc_led_payload(
    state: CncLedState | int,
    *,
    rack_count: int,
    rack_capacity: int,
    remaining_sec: int,
    flags: int = 0,
    seq: int = 0,
) -> bytes:
    return bytes(
        [
            LED_CMD_CNC_STATE,
            _u8(int(state)),
            _u8(rack_count),
            _u8(rack_capacity),
            _u8(remaining_sec),
            _u8(flags),
            _u8(seq),
            0,
        ]
    )


@dataclass
class CncLedCanPublisher:
    """Send the CNC LED state frame at a bounded rate."""

    bus: Any
    period_s: float = DEFAULT_LED_TX_PERIOD_SEC
    seq: int = 0
    _last_tx: float = field(default=0.0, init=False)

    def build_message(
        self,
        state: CncLedState | int,
        *,
        rack_count: int,
        rack_capacity: int,
        remaining_sec: int,
        flags: int = 0,
    ) -> Any:
        if can is None:
            raise RuntimeError("python-can is required for live CAN sending")

        payload = build_cnc_led_payload(
            state,
            rack_count=rack_count,
            rack_capacity=rack_capacity,
            remaining_sec=remaining_sec,
            flags=flags,
            seq=self.seq,
        )
        return can.Message(
            arbitration_id=CAN_ID_CNC_LED,
            data=payload,
            is_extended_id=False,
        )

    def send(
        self,
        state: CncLedState | int,
        *,
        rack_count: int,
        rack_capacity: int,
        remaining_sec: int,
        flags: int = 0,
    ) -> Any:
        msg = self.build_message(
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

    def send_if_due(
        self,
        state: CncLedState | int,
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
        return self.send(
            state,
            rack_count=rack_count,
            rack_capacity=rack_capacity,
            remaining_sec=remaining_sec,
            flags=flags,
        )


def open_socketcan_bus(interface: str = "can0") -> Any:
    if can is None:
        raise RuntimeError("python-can is not installed")
    return can.Bus(channel=interface, interface="socketcan")


def parse_state(value: str) -> CncLedState:
    raw = value.strip()
    try:
        return CncLedState(int(raw, 0))
    except ValueError:
        pass

    key = raw.upper().replace("-", "_")
    if key.startswith("LED_"):
        key = key[4:]
    try:
        return CncLedState[key]
    except KeyError as exc:
        names = ", ".join(state.name for state in CncLedState)
        raise argparse.ArgumentTypeError(f"unknown state {value!r}; use one of {names}") from exc


def format_frame(state: CncLedState, payload: bytes) -> str:
    data = " ".join(f"{byte:02X}" for byte in payload)
    return f"id=0x{CAN_ID_CNC_LED:03X} state={state.name} data=[{data}]"


def iter_demo_states() -> Iterable[tuple[CncLedState, int, int, int, float]]:
    yield CncLedState.IDLE, 0, DEFAULT_RACK_CAPACITY, 0, 2.0
    yield CncLedState.LOADING, 0, DEFAULT_RACK_CAPACITY, 0, 2.0
    yield CncLedState.MACHINING, 0, DEFAULT_RACK_CAPACITY, 15, 3.0
    yield CncLedState.DONE_WAIT_UNLOAD, 0, DEFAULT_RACK_CAPACITY, 0, 2.0
    yield CncLedState.UNLOADING, 0, DEFAULT_RACK_CAPACITY, 0, 2.0
    yield CncLedState.RACK_FULL, DEFAULT_RACK_CAPACITY, DEFAULT_RACK_CAPACITY, 0, 3.0
    yield CncLedState.ERROR, DEFAULT_RACK_CAPACITY, DEFAULT_RACK_CAPACITY, 0, 2.0


def send_or_print(
    *,
    publisher: CncLedCanPublisher | None,
    dry_run: bool,
    state: CncLedState,
    rack_count: int,
    rack_capacity: int,
    remaining_sec: int,
    seq: int,
) -> int:
    payload = build_cnc_led_payload(
        state,
        rack_count=rack_count,
        rack_capacity=rack_capacity,
        remaining_sec=remaining_sec,
        seq=seq,
    )

    if dry_run:
        print(format_frame(state, payload))
        return (seq + 1) & 0xFF

    if publisher is None:
        raise RuntimeError("publisher is required when dry_run is false")

    publisher.seq = seq
    publisher.send(
        state,
        rack_count=rack_count,
        rack_capacity=rack_capacity,
        remaining_sec=remaining_sec,
    )
    print(format_frame(state, payload))
    return publisher.seq


def main() -> int:
    parser = argparse.ArgumentParser(description="Send or print Sambo CNC LED CAN frames")
    parser.add_argument("--interface", default="can0", help="SocketCAN interface, default: can0")
    parser.add_argument("--state", type=parse_state, default=CncLedState.IDLE)
    parser.add_argument("--rack-count", type=int, default=0)
    parser.add_argument("--rack-capacity", type=int, default=DEFAULT_RACK_CAPACITY)
    parser.add_argument("--remaining", type=int, default=0, help="remaining CNC seconds")
    parser.add_argument("--period", type=float, default=DEFAULT_LED_TX_PERIOD_SEC)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--once", action="store_true", help="send one frame and exit")
    parser.add_argument("--cycle", action="store_true", help="cycle through all LED states")
    parser.add_argument("--dry-run", action="store_true", help="print frames without opening CAN")
    args = parser.parse_args()

    bus = None if args.dry_run else open_socketcan_bus(args.interface)
    publisher = None if args.dry_run else CncLedCanPublisher(bus, period_s=args.period)
    seq = 0

    try:
        if args.cycle:
            deadline = time.monotonic() + args.duration
            while time.monotonic() < deadline:
                for state, rack_count, rack_capacity, remaining, hold_s in iter_demo_states():
                    state_deadline = min(deadline, time.monotonic() + hold_s)
                    while time.monotonic() < state_deadline:
                        seq = send_or_print(
                            publisher=publisher,
                            dry_run=args.dry_run,
                            state=state,
                            rack_count=rack_count,
                            rack_capacity=rack_capacity,
                            remaining_sec=remaining,
                            seq=seq,
                        )
                        time.sleep(args.period)
            return 0

        seq = send_or_print(
            publisher=publisher,
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
            seq = send_or_print(
                publisher=publisher,
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
