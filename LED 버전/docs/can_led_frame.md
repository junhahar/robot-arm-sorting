# CAN LED Frame

이 문서는 CNC/랙 상태 표시용 LED 프레임만 정의합니다. 기존 로봇 제어 프레임을 바꾸지 않습니다.

## Bus

```text
Bitrate: 125 kbps
Frame: Standard 11-bit
CAN ID: 0x410
DLC: 8
Sender: Raspberry Pi 5, CANable
Receiver: Arduino Nano, MCP2515
Recommended TX rate: 5Hz
```

현재 운용 중인 주요 ID와 겹치지 않게 `0x410`을 사용합니다.

| 기존 ID | 용도 |
|---|---|
| `0x100` | RPi에서 Nano로 서보/그리퍼 명령 |
| `0x201` | Nano에서 RPi로 위치/온도/로드 텔레메트리 |
| `0x202` | Nano에서 RPi로 전류 텔레메트리 |
| `0x300` | Nano에서 RPi로 TOF 거리 상태 |
| `0x410` | RPi에서 Nano로 CNC LED 상태 |

## Payload

| Byte | 이름 | 값 |
|---|---|---|
| `0` | `cmd` | `0x20` 고정 |
| `1` | `state` | 아래 상태 번호 |
| `2` | `rack_count` | 0~255 |
| `3` | `rack_capacity` | 0~255 |
| `4` | `remaining_sec` | CNC 남은 시간, 초 단위 |
| `5` | `flags` | 현재 0 |
| `6` | `seq` | 송신할 때마다 1씩 증가, 0~255 반복 |
| `7` | `reserved` | 현재 0 |

## State Values

| 번호 | 이름 | LED 의미 |
|---|---|---|
| `0` | `IDLE` | 초록 고정, CNC 대기 |
| `1` | `LOADING` | 빨강/초록 빠른 교차, 소재 투입 중 |
| `2` | `MACHINING` | 빨강 고정, CNC 가공 중 |
| `3` | `DONE_WAIT_UNLOAD` | 초록 느린 점멸, 회수 대기 |
| `4` | `UNLOADING` | 빨강/초록 빠른 교차, 제품 회수 중 |
| `5` | `RACK_FULL` | 빨강 두 번 점멸 반복, 랙 만재 |
| `6` | `ERROR` | 빨강/초록 빠른 동시 점멸 |
| `255` | `CAN_LOST` | Nano 내부 timeout 상태, RPi가 직접 보내지 않음 |

`CAN_LOST`는 Nano가 2.5초 이상 `0x410` 프레임을 받지 못했을 때 자체 표시하는 상태입니다.

## Example Frames

```text
IDLE, rack 0/2:
id=0x410 data=[20 00 00 02 00 00 00 00]

MACHINING, 15 seconds remaining:
id=0x410 data=[20 02 00 02 0F 00 01 00]

RACK_FULL, rack 2/2:
id=0x410 data=[20 05 02 02 00 00 02 00]
```
