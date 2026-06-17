# CAN Status Frame

이 문서는 별도 표시용 Nano에 들어가는 LED + 부저 상태 프레임을 정의합니다. 기존 TOF용 Nano나 로봇 제어 Nano의 펌웨어를 바꾸는 문서가 아닙니다.

## Bus

```text
Bitrate: 125 kbps
Frame: Standard 11-bit
RPi -> display Nano: 0x410
display Nano -> RPi: 0x411
DLC: 8
Recommended status TX rate: 5Hz
```

## RPi -> Nano 상태 프레임

```text
CAN ID: 0x410
byte[0] = 0x20
byte[1] = state
byte[2] = rack_count
byte[3] = rack_capacity
byte[4] = remaining_sec
byte[5] = flags
byte[6] = seq
byte[7] = reserved
```

| state | 이름 | 의미 |
|---|---|---|
| `0` | `IDLE` | CNC 비어 있음 |
| `1` | `MACHINING` | CNC 가공 중 |
| `2` | `DONE_WAIT` | 가공 완료, 회수 대기 |
| `3` | `RACK_FULL` | 랙 만재 |
| `4` | `ERROR` | 오류 |
| `255` | `CAN_LOST` | Nano 내부 timeout 표시용, RPi가 직접 보내지 않음 |

## Nano -> RPi 랙 리셋 요청

```text
CAN ID: 0x411
byte[0] = 0x31
byte[1] = 1
byte[2] = seq
byte[3..7] = 0
```

RPi는 이 요청을 받았다고 바로 `rack_count = 0`으로 바꾸면 안 됩니다. 최소한 `robot_busy == False`일 때만 받아야 합니다.

## cansend 테스트

RPi에서 표시용 Nano 출력만 먼저 확인할 수 있습니다.

```bash
# IDLE: 초록 ON
cansend can0 410#2000000200000000

# MACHINING: 빨강 ON, 남은 시간 15초
cansend can0 410#200100020F000000

# DONE_WAIT: 초록 느린 점멸
cansend can0 410#2002000200000000

# RACK_FULL: 빨강 두 번 점멸 + 부저
cansend can0 410#2003020200000000

# ERROR: 빨강/초록 빠른 점멸 + 부저
cansend can0 410#2004000200000000
```

리셋 버튼을 누르면 RPi `candump can0`에서 아래처럼 보여야 합니다.

```text
411   [8]  31 01 xx 00 00 00 00 00
```
