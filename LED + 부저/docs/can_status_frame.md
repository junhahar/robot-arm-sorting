# CAN Status Frame

이 문서는 표시 전용 Arduino Nano가 수신하는 LED + 부저 상태 프레임을 정의합니다. 기존 TOF용 Nano나 로봇팔 제어 Nano의 펌웨어를 바꾸는 문서가 아닙니다.

## Bus

```text
Bitrate: 125 kbps
Frame: Standard 11-bit
RPi -> display Nano: 0x410
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

| state | 이름 | 표시 |
|---:|---|---|
| `0` | `CNC_EMPTY` | 파란 LED ON |
| `1` | `CNC_OCCUPIED` | 빨간 LED ON |
| `2` | `CNC_DONE_WAIT` | 초록 LED 느린 점멸 |
| `3` | `RACK_LOADED` | 파란 LED ON + 부저 짧게 3번 |
| `4` | `RACK_FULL` | LED OFF + 부저 반복 |
| `5` | `ERROR` | 전체 LED 빠른 점멸 + 부저 |
| `255` | `CAN_LOST` | Nano 내부 timeout 표시. RPi가 직접 보낼 필요 없음 |

## Payload 필드

```text
rack_count    = 현재 적재 개수
rack_capacity = 최대 적재 개수, 현재 기본 2
remaining_sec = CNC 가공 남은 시간. 표시용 Nano는 현재 LED에는 쓰지 않지만 디버깅용으로 보관
flags         = 예비 필드
seq           = 송신 순번
```

## cansend 테스트

```bash
# CNC_EMPTY: 파란 LED ON
cansend can0 410#2000000200000000

# CNC_OCCUPIED: 빨간 LED ON, 남은 시간 15초
cansend can0 410#200100020F000000

# CNC_DONE_WAIT: 초록 LED 느린 점멸
cansend can0 410#2002000200000000

# RACK_LOADED: rack_count 1, 부저 짧게 3번
cansend can0 410#2003010200000000

# RACK_FULL: rack_count 2/2, LED OFF + 부저 반복
cansend can0 410#2004020200000000

# ERROR: 전체 LED 빠른 점멸 + 부저
cansend can0 410#2005000200000000
```

## 상태 전환 기준

```text
CNC_EMPTY:
  홈 대기, 탐색, 집기, CNC로 이동, CNC 입구 도착 전까지

CNC_OCCUPIED:
  CNC 안으로 물체를 넣는 동작을 시작한 순간부터 가공 완료 전까지

CNC_DONE_WAIT:
  CNC 가공시간이 끝난 뒤, 로봇이 CNC에서 가공품을 회수하는 동안

RACK_LOADED:
  적재 위치에서 그리퍼 열기 완료 + 적재함 밖 안전 위치 후퇴 완료

RACK_FULL:
  rack_count >= rack_capacity

ERROR:
  비상 정지, 동작 실패, 통신/상태 이상 등 RPi가 에러로 판단한 상황
```
