# CAN 상태 프레임

이 문서는 별도 Arduino Nano + MCP2515가 받는 CNC LED/부저 상태 명령과,
Nano가 다시 Raspberry Pi/대시보드로 보내는 실제 출력 동기화 프레임을 정의합니다.

## 버스

```text
Bitrate: 125 kbps
Frame: Standard 11-bit
RPi -> display Nano: 0x410
display Nano -> RPi/dashboard: 0x411
```

## RPi -> Nano 상태 명령

```text
CAN ID: 0x410
DLC: 8
byte[0] = 0x20
byte[1] = state
byte[2] = rack_count
byte[3] = rack_capacity
byte[4] = remaining_sec
byte[5] = flags
byte[6] = seq
byte[7] = reserved
```

| state | 이름 | 표시 |
|---:|---|---|
| `0` | `CNC_EMPTY` | 파랑 LED ON |
| `1` | `CNC_OCCUPIED` | 빨강 LED ON |
| `2` | `CNC_DONE_WAIT` | 초록 LED 느린 점멸 |
| `3` | `RACK_LOADED` | 파랑 LED ON + 부저 3회 |
| `4` | `RACK_FULL` | LED OFF + 부저 2회 반복 |
| `5` | `ERROR` | 전체 LED 빠른 점멸 + 부저 빠른 반복 |
| `255` | `CAN_LOST` | Nano 내부 timeout 표시 |

## Nano -> RPi/dashboard 출력 동기화

Nano는 실제로 핀에 출력한 LED/부저 상태를 `0x411`로 다시 보냅니다.
대시보드는 이 프레임이 800ms 이내로 들어오면 자체 예측 애니메이션보다
Nano의 실제 출력값을 우선 표시합니다.

```text
CAN ID: 0x411
DLC: 8
byte[0] = 0x21
byte[1] = state_to_show
byte[2] = output_mask
byte[3] = rack_count
byte[4] = rack_capacity
byte[5] = remaining_sec
byte[6] = last_rx_seq
byte[7] = reserved
```

`output_mask` 비트 정의:

| bit | 값 | 의미 |
|---:|---:|---|
| 0 | `0x01` | 빨강 LED ON |
| 1 | `0x02` | 초록 LED ON |
| 2 | `0x04` | 파랑 LED ON |
| 3 | `0x08` | 부저 ON |

## 출력 패턴 기준

Nano 코드와 대시보드 fallback 표시의 시간값은 동일하게 맞춥니다.

| 상태 | 패턴 |
|---|---|
| `CNC_EMPTY` | 파랑 ON |
| `CNC_OCCUPIED` | 빨강 ON |
| `CNC_DONE_WAIT` | 초록 700ms half-period 점멸 |
| `RACK_LOADED` | 파랑 ON, 부저 180ms x 3회 원샷 |
| `RACK_FULL` | LED OFF, 부저 150ms x 2회 / 1400ms 반복 |
| `ERROR` | LED 150ms half-period 점멸, 부저 120ms / 300ms 반복 |
| `CAN_LOST` | 빨강 -> 초록 -> 파랑 순차 점멸 |

## cansend 테스트

```bash
# CNC_EMPTY: 파랑 LED ON
cansend can0 410#2000000200000000

# CNC_OCCUPIED: 빨강 LED ON, 남은 시간 15초
cansend can0 410#200100020F000000

# CNC_DONE_WAIT: 초록 LED 느린 점멸
cansend can0 410#2002000200000000

# RACK_LOADED: rack_count 1, 부저 3회
cansend can0 410#2003010200000000

# RACK_FULL: rack_count 2/2, LED OFF + 부저 반복
cansend can0 410#2004020200000000

# ERROR: 전체 LED 빠른 점멸 + 부저 반복
cansend can0 410#2005000200000000

# Nano 출력 에코 확인
candump can0,411:7FF
```
