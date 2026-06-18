# LED + 부저 최종 적용안

이 폴더는 **별도 Arduino Nano + MCP2515 + CNC 상태 LED 3개 + 압전 부저 1개**로 로봇팔 작업 상태를 표시하는 최종 적용 버전입니다.

중요합니다. 이 Nano는 현재 라즈베리파이에 연결된 TOF용 Nano가 아닙니다. STS3215/URT 제어 Nano도 아닙니다. 이 구성은 LED/부저 표시만 담당하는 별도 Nano입니다.

```text
입력: Raspberry Pi가 CAN 0x410으로 보내는 작업 상태
처리: 표시용 Nano가 상태 번호를 수신해서 LED/부저 패턴 선택
출력: CNC 박스 LED 3개, 적재함/작업자 알림용 부저
필요성: 로봇팔 판단 로직과 표시 장치를 분리해서 상태가 꼬이지 않게 하기 위함
```

## 포함 파일

| 경로 | 용도 |
|---|---|
| `arduino/nano_cnc_status_buzzer_node/nano_cnc_status_buzzer_node.ino` | 표시용 Arduino Nano 최종 코드 |
| `rpi/cnc_status_buzzer.py` | RPi에서 CAN 상태 프레임을 보내는 Python helper |
| `rpi/led_buzzer_config.example.json` | CNC 가공시간, 적재함 용량 설정 예시 |
| `docs/can_status_frame.md` | CAN ID, payload, 상태 번호, `cansend` 테스트 |
| `docs/wiring.md` | Nano, MCP2515, LED, 부저 배선 |
| `현장_적용_체크리스트.md` | 실제 조립/통합 전 확인용 요약 체크리스트 |

## 최종 하드웨어

```text
CNC 박스
- 파란 LED 1개: CNC 비어 있음
- 빨간 LED 1개: CNC 점유/가공 중
- 초록 LED 1개: 가공 완료/회수 대기

적재함 또는 작업자가 잘 듣는 위치
- 압전 부저 1개: 적재 완료, 만재, 에러 알림
```

## 핀맵

| 기능 | Arduino Nano 핀 | 의미 |
|---|---:|---|
| 빨간 LED | D4 | CNC 점유/가공 중 |
| 초록 LED | D5 | 가공 완료/회수 대기 |
| 압전 부저 | D6 | 적재 완료/만재/에러 알림 |
| 파란 LED | D7 | CNC 비어 있음 |
| MCP2515 INT | D2 | CAN 수신 인터럽트 입력 |
| MCP2515 CS | D10 | SPI CS |
| MCP2515 MOSI | D11 | SPI MOSI |
| MCP2515 MISO | D12 | SPI MISO |
| MCP2515 SCK | D13 | SPI SCK |

LED는 반드시 각 LED마다 저항을 넣습니다.

```text
Arduino 핀 -> 220~330옴 저항 -> LED +
LED - -> GND
```

## 상태 정의

| 번호 | 상태 | 파란 LED | 빨간 LED | 초록 LED | 부저 | 의미 |
|---:|---|---|---|---|---|---|
| `0` | `CNC_EMPTY` | ON | OFF | OFF | OFF | CNC가 비어 있고 투입 가능 |
| `1` | `CNC_OCCUPIED` | OFF | ON | OFF | OFF | CNC 안으로 투입 시작, 가공 대기, 가공 중 |
| `2` | `CNC_DONE_WAIT` | OFF | OFF | 느린 점멸 | OFF | 가공 완료, 로봇 회수 대기, CNC 픽업 중 |
| `3` | `RACK_LOADED` | ON | OFF | OFF | 짧게 3번 | 적재함에 1개 적재 완료 |
| `4` | `RACK_FULL` | OFF | OFF | OFF | 반복 | 적재함 2개 만재, 작업자 확인 필요 |
| `5` | `ERROR` | 빠른 점멸 | 빠른 점멸 | 빠른 점멸 | 빠른 반복 | 에러/비상 상태 |
| `255` | `CAN_LOST` | 순차 점멸 | 순차 점멸 | 순차 점멸 | OFF | RPi 상태 프레임 수신 끊김 |

## 실제 작업 흐름

```text
1. 홈 대기, 물체 탐색, 물체 집기, CNC로 이동
   -> CNC_EMPTY
   -> 파란 LED ON

2. 로봇팔이 CNC 안으로 물체를 넣는 동작 시작
   -> CNC_OCCUPIED
   -> 빨간 LED ON

3. 그리퍼 열기, 로봇팔 후퇴, CNC 가공 대기/가공 중
   -> CNC_OCCUPIED 유지
   -> 빨간 LED 유지

4. CNC_PROCESS_SEC 시간이 끝남
   -> CNC_DONE_WAIT
   -> 초록 LED 느린 점멸

5. 로봇이 CNC에서 가공품을 잡고 CNC 밖 안전 위치까지 후퇴 완료
   -> CNC_EMPTY
   -> 파란 LED ON

6. 로봇이 적재함 위치에 가공품을 내려놓고 후퇴 완료
   -> RACK_LOADED
   -> 파란 LED ON + 부저 짧게 3번

7. rack_count가 rack_capacity 이상이 됨
   -> RACK_FULL
   -> LED 전부 OFF + 부저만 반복
   -> 다음 사이클 정지
```

## 전환 기준

파란색에서 빨간색으로 바뀌는 기준은 그리퍼가 열리는 순간이 아닙니다.

```text
로봇팔이 CNC 안으로 물체를 넣는 동작을 시작하는 순간
```

초록색에서 파란색으로 바뀌는 기준은 가공품을 잡은 순간이 아닙니다.

```text
가공품을 들고 CNC 밖 안전 위치까지 완전히 후퇴한 순간
```

적재 완료 기준은 센서 없이 단순하게 잡습니다.

```text
적재 위치 도착
-> 그리퍼 열기 완료
-> 적재함 밖 안전 위치로 후퇴 완료
-> rack_count += 1
```

TOF나 카메라는 현재 1차 구현에는 사용하지 않습니다. 다만 RPi 코드 구조는 나중에 `verify_gripper_has_part()` 또는 `verify_cnc_empty()` 같은 검증 함수를 끼울 수 있게 열어두면 됩니다.

## 설정값

`rpi/led_buzzer_config.example.json`:

```json
{
  "cnc_process_sec": 20,
  "rack_capacity": 2
}
```

`cnc_process_sec`는 기본 20초입니다. 나중에 15초, 30초, 45초로 바뀌어도 뒤 신호가 꼬이지 않게 상태 시작 시각 기준으로 계산합니다.

`rack_capacity`는 현재 2개입니다. `rack_count >= rack_capacity`가 되면 `RACK_FULL`을 보냅니다.

## RPi 적용 예시

```python
import time

from cnc_status_buzzer import (
    CncStatus,
    CncStatusCanNode,
    load_runtime_config,
    open_socketcan_bus,
    remaining_cnc_seconds,
)

config = load_runtime_config("led_buzzer_config.json")
cnc_process_sec = float(config["cnc_process_sec"])
rack_capacity = int(config["rack_capacity"])
rack_count = 0

bus = open_socketcan_bus("can0")
display = CncStatusCanNode(bus)

def send_led_state(state, remaining_sec=0, force=True):
    display.send_status_if_due(
        state,
        rack_count=rack_count,
        rack_capacity=rack_capacity,
        remaining_sec=remaining_sec,
        force=force,
    )

send_led_state(CncStatus.CNC_EMPTY)

search_object()
move_to_pick()
grip_object()
move_to_cnc_approach()

send_led_state(CncStatus.CNC_OCCUPIED)
move_to_cnc_insert_pose()
open_gripper()
move_to_cnc_retreat()

cnc_started_at = time.monotonic()

while time.monotonic() - cnc_started_at < cnc_process_sec:
    remaining = remaining_cnc_seconds(
        now=time.monotonic(),
        cnc_occupied=True,
        cnc_process_started_at=cnc_started_at,
        cnc_process_sec=cnc_process_sec,
    )
    send_led_state(CncStatus.CNC_OCCUPIED, remaining_sec=remaining, force=False)
    do_wait_or_debris_motion()
    time.sleep(0.2)

send_led_state(CncStatus.CNC_DONE_WAIT)

move_to_cnc_pick()
grip_processed_part()
move_to_cnc_clear_pose()

send_led_state(CncStatus.CNC_EMPTY)

move_to_rack_slot(rack_count)
open_gripper()
move_to_rack_clear_pose()

rack_count += 1
send_led_state(CncStatus.RACK_LOADED)
time.sleep(1.2)

if rack_count >= rack_capacity:
    send_led_state(CncStatus.RACK_FULL)
    stop_next_cycle()
```

## 적재함 초기화

버튼은 사용하지 않습니다. 적재함을 비운 뒤에는 RPi 쪽에서 `rack_count = 0`으로 초기화해야 합니다.

추천 방식:

```text
1순위: 대시보드에 "적재함 비움" 버튼 추가
2순위: RPi 콘솔 명령 또는 운영 스크립트에서 rack_count 초기화
```

표시용 Nano는 `rack_count`를 직접 바꾸지 않습니다.

## 테스트 명령

CAN 없이 payload만 확인:

```bash
python3 "LED + 부저/rpi/cnc_status_buzzer.py" --dry-run --state CNC_EMPTY --once
python3 "LED + 부저/rpi/cnc_status_buzzer.py" --dry-run --state CNC_OCCUPIED --remaining 15 --once
python3 "LED + 부저/rpi/cnc_status_buzzer.py" --dry-run --state CNC_DONE_WAIT --once
python3 "LED + 부저/rpi/cnc_status_buzzer.py" --dry-run --state RACK_LOADED --rack-count 1 --once
python3 "LED + 부저/rpi/cnc_status_buzzer.py" --dry-run --state RACK_FULL --rack-count 2 --rack-capacity 2 --once
```

실제 CAN으로 전체 상태 순환:

```bash
python3 "LED + 부저/rpi/cnc_status_buzzer.py" --interface can0 --cycle --duration 25
```

직접 `cansend`:

```bash
# CNC_EMPTY: 파란 LED ON
cansend can0 410#2000000200000000

# CNC_OCCUPIED: 빨간 LED ON, 남은 시간 15초
cansend can0 410#200100020F000000

# CNC_DONE_WAIT: 초록 LED 느린 점멸
cansend can0 410#2002000200000000

# RACK_LOADED: 파란 LED + 부저 3번
cansend can0 410#2003010200000000

# RACK_FULL: LED OFF + 부저 반복
cansend can0 410#2004020200000000

# ERROR: 전체 빠른 점멸 + 부저
cansend can0 410#2005000200000000
```

## Arduino 적용

1. 표시 전용 Arduino Nano를 준비합니다.
2. MCP2515, LED 3개, 압전 부저를 `docs/wiring.md`대로 연결합니다.
3. Arduino IDE에서 `arduino/nano_cnc_status_buzzer_node/nano_cnc_status_buzzer_node.ino`를 엽니다.
4. 보드는 `Arduino Nano`, 프로세서는 보드에 맞게 `ATmega328P` 또는 `ATmega328P Old Bootloader`를 선택합니다.
5. `MCP_CAN_lib` 라이브러리를 설치합니다.
6. MCP2515 크리스털이 8MHz면 그대로 둡니다. 16MHz면 코드의 `MCP_CLOCK`을 `MCP_16MHZ`로 바꿉니다. 사양 확인 필요.
7. 압전 부저면 `PASSIVE_BUZZER 1` 그대로 사용합니다. 능동 부저면 `PASSIVE_BUZZER 0`으로 바꿉니다.
8. 업로드 후 RPi에서 CAN 테스트 명령을 보냅니다.

## 안전 주의

- 표시용 Nano는 안전장치가 아니라 알림장치입니다.
- E-Stop은 반드시 STS3215 +7.4~+7.5V 서보 전원을 물리적으로 차단해야 합니다.
- Raspberry Pi 5는 전용 USB-C 5V 어댑터를 사용합니다.
- STS3215 서보 전원은 Arduino 5V, Raspberry Pi 5V, CANable 5V OUT에서 공급하지 않습니다.
- 표시용 Nano GND, MCP2515 GND, CANable terminal GND는 공통 GND로 묶는 것을 권장합니다.
- 12V/24V 부저나 큰 전류 부저는 Nano 핀에 직접 연결하지 말고 MOSFET/트랜지스터 드라이버를 사용해야 합니다. 사양 확인 필요.
