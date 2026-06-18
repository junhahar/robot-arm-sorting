# 팀원/Claude용 작업 핸드오프 - 대시보드, LED + 부저

이 문서는 팀원이 Claude 같은 AI 코딩 도구로 `lee` 브랜치의 대시보드와 LED + 부저 기능을 이어서 수정할 때 쓰는 전용 안내서다.
일반 README만 보고 수정하면 맥락을 놓칠 수 있어서, 현재까지 반영한 의도, 파일 위치, 수정 금지 범위, 검증 방법을 한 번에 정리했다.

## 1. 반드시 지킬 것

- GitHub 수정은 `lee` 브랜치 기준으로만 한다.
- 라즈베리파이 내부 파일은 건드리지 않는다.
- 이 문서의 목적은 팀원이 GitHub 코드만 보고 수정할 수 있게 하는 것이다.
- 대시보드 수정 대상은 기본적으로 아래 HTML 한 파일이다.
  - `6-18 작업 현황/choi/dashboard/sambo_robot_arm_dashboard.html`
- Raspberry Pi WebSocket/CAN 브리지 수정이 필요할 때만 아래 파일을 건드린다.
  - `6-18 작업 현황/choi/dashboard/rpi_dashboard_ws_bridge.py`
- LED + 부저 실제 적용 코드와 문서는 아래 폴더에 있다.
  - `LED + 부저/`
- 현재 LED + 부저용 Arduino Nano는 TOF용 Arduino Nano가 아니다.
- 현재 LED + 부저용 Arduino Nano는 로봇팔 서보 제어용 Arduino Nano도 아니다.
- LED + 부저용 Arduino Nano는 별도 Nano이며, MCP2515를 통해 CAN 메시지를 받아 표시만 담당한다.
- 안전 정지는 대시보드나 LED + 부저 기능이 아니라 실제 E-Stop 배선으로 처리해야 한다.

## 2. 먼저 읽을 파일

팀원이 Claude에게 작업을 맡기기 전에 아래 파일을 먼저 읽히면 좋다.

1. `팀원_Claude_대시보드_LED부저_핸드오프.md`
2. `LED + 부저/README.md`
3. `LED + 부저/현장_적용_체크리스트.md`
4. `LED + 부저/docs/wiring.md`
5. `LED + 부저/docs/can_status_frame.md`
6. `LED + 부저/arduino/nano_cnc_status_buzzer_node/nano_cnc_status_buzzer_node.ino`
7. `LED + 부저/rpi/cnc_status_buzzer.py`
8. `6-18 작업 현황/choi/dashboard/sambo_robot_arm_dashboard.html`
9. `6-18 작업 현황/choi/dashboard/rpi_dashboard_ws_bridge.py`

## 3. 전체 구조

현재 구조는 역할을 분리하는 방식이다.

```text
Raspberry Pi
  - 로봇 작업 상태 판단
  - CNC 상태 판단
  - 적재 개수 판단
  - CANable로 상태 프레임 송신

Arduino Nano + MCP2515
  - CAN 상태 프레임 수신
  - CNC LED 3개 제어
  - 부저 제어
  - 실제 출력 상태를 CAN으로 echo 송신

Dashboard
  - 로봇/시뮬레이션 표시
  - CNC LED와 부저 상태 표시
  - Nano echo가 있으면 실제 출력 기준으로 표시
  - Nano echo가 없으면 데모용 예측 상태로 표시
```

중요한 점은 대시보드가 LED/부저를 마음대로 가짜로 표시하는 구조가 아니라는 것이다.
실제 Nano가 `0x411`로 보내는 출력 echo가 살아 있으면 그 값을 우선 표시해야 한다.

## 4. LED + 부저 최종 상태 로직

현재 CNC 상태 표시는 CNC 박스 기준으로만 한다.
적재함에는 LED를 달지 않는다. 적재 완료나 만재 알림은 부저로 처리한다.

| 상태 코드 | 상태 이름 | 의미 | LED | 부저 |
| --- | --- | --- | --- | --- |
| `0` | `CNC_EMPTY` | CNC 비어 있음, 로봇이 탐색/이동 중 | 파란색 ON | OFF |
| `1` | `CNC_OCCUPIED` | CNC 안에 물체 있음, 가공/대기/이물 제거 시간 | 빨간색 ON | OFF |
| `2` | `CNC_DONE_WAIT` | 가공 완료, 로봇 회수 대기/픽업 중 | 초록색 느린 점멸 | OFF |
| `3` | `RACK_LOADED` | 적재함에 1개 적재 완료 | 파란색 ON 복귀 | 짧게 3회 |
| `4` | `RACK_FULL` | 적재함 2/2 만재 | LED OFF | 반복 경고 |
| `5` | `ERROR` | 오류/비상 상태 | LED 전체 빠른 점멸 | 빠른 반복 |
| `255` | `CAN_LOST` | CAN 수신 끊김 | timeout 표시 패턴 | 필요 시 경고 |

현재 기본 설정:

- CNC 가공 시간: `20초`
- 적재함 최대 개수: `2개`

가공 시간은 나중에 바뀔 수 있으므로 코드에서 상수 하나만 바꾸면 뒤 상태 흐름이 무너지지 않게 유지해야 한다.

## 5. CAN 프레임 규칙

### 5.1 Raspberry Pi -> Arduino Nano

CAN ID: `0x410`

| byte | 의미 |
| --- | --- |
| `0` | command, 고정값 `0x20` |
| `1` | state |
| `2` | rack_count |
| `3` | rack_capacity |
| `4` | remaining_sec |
| `5` | flags |
| `6` | seq |
| `7` | reserved |

### 5.2 Arduino Nano -> Raspberry Pi/Dashboard

CAN ID: `0x411`

| byte | 의미 |
| --- | --- |
| `0` | command, 고정값 `0x21` |
| `1` | state_to_show |
| `2` | output_mask |
| `3` | rack_count |
| `4` | rack_capacity |
| `5` | remaining_sec |
| `6` | last_rx_seq |
| `7` | reserved |

`output_mask` bit:

| bit | 값 | 의미 |
| --- | --- | --- |
| `0` | `0x01` | 빨간 LED ON |
| `1` | `0x02` | 초록 LED ON |
| `2` | `0x04` | 파란 LED ON |
| `3` | `0x08` | 부저 ON |

대시보드는 가능하면 이 `0x411` echo를 기준으로 LED 깜빡임 속도와 부저 울림 타이밍을 그대로 반영해야 한다.

## 6. Arduino Nano 핀맵

현재 LED + 부저 테스트 및 최종 적용 코드는 아래 핀맵을 기준으로 한다.

| 부품 | Nano 핀 |
| --- | --- |
| CNC 빨간 LED | `D4` |
| CNC 초록 LED | `D5` |
| 부저 | `D6` |
| CNC 파란 LED | `D7` |
| MCP2515 INT | `D2` |
| MCP2515 CS | `D10` |
| MCP2515 MOSI | `D11` |
| MCP2515 MISO | `D12` |
| MCP2515 SCK | `D13` |

MCP2515 모듈 클럭:

- 기본 코드: `MCP_8MHZ`
- 모듈 크리스털이 16MHz면 `MCP_16MHZ`로 바꿔야 한다.

부저 설정:

- 패시브 압전 부저면 `PASSIVE_BUZZER 1`
- 액티브 부저면 `PASSIVE_BUZZER 0`

## 7. 대시보드 수정 포인트

대시보드는 큰 단일 HTML 파일이라서 전체를 대충 고치면 깨지기 쉽다.
아래 키워드로 검색해서 필요한 부분만 좁게 수정한다.

대상 파일:

```text
6-18 작업 현황/choi/dashboard/sambo_robot_arm_dashboard.html
```

주요 검색어:

```text
led-buzzer
ledBuzzer
renderCnc
getLedBuzzerStatus
cncBox
cncState
cncLedBlue
cncLedRed
cncLedGreen
cncBuzzer
cncRack
initSamboCursor
sambo-cursor
```

현재 CNC 상태 패널은 크기를 바꾸지 않고 내부 내용만 맞추는 방향이다.

상태 패널에 반드시 보여야 하는 내용:

- CNC 상태
- 파란/빨간/초록 LED 중 현재 실제 출력
- 부저가 울리는지 여부
- 적재 개수 `0/2`, `1/2`, `2/2`
- 남은 가공 시간이 있을 때만 남은 시간

현재 의도:

- CNC 박스에는 LED 3개가 표시된다.
- 적재함에는 LED를 표시하지 않는다.
- 부저가 울리면 대시보드에서도 부저 ON 상태가 보인다.
- 실제 Nano 출력 echo가 있으면 대시보드 LED/부저도 그 echo와 동기화된다.

## 8. WebSocket/CAN 브리지 수정 포인트

대상 파일:

```text
6-18 작업 현황/choi/dashboard/rpi_dashboard_ws_bridge.py
```

현재 브리지는 Nano가 보내는 `0x411` 출력 echo를 받아서 `cnc.output`에 넣는 역할을 한다.

중요 키워드:

```text
CAN_ID_CNC_OUTPUT
CMD_CNC_OUTPUT
led_buzzer_output
nano_output_echo
output_mask
fresh
age_ms
```

중요한 기준:

- `0x411` echo가 최근 값이면 `fresh: true`
- 현재 기준 fresh window는 약 `800ms`
- fresh 값이 있으면 대시보드는 그 값을 우선 표시
- fresh 값이 없으면 대시보드는 예측/데모 표시로 fallback

대시보드와 Nano 출력 동기화를 망치지 않으려면, `cnc.output` 구조를 임의로 바꾸지 말고 필요한 필드만 추가해야 한다.

## 9. 마우스 포인터 수정 포인트

커스텀 포인터도 대시보드 HTML 안에 있다.

대상 파일:

```text
6-18 작업 현황/choi/dashboard/sambo_robot_arm_dashboard.html
```

주요 검색어:

```text
sambo-cursor
initSamboCursor
sambo-custom-cursor
is-click-pop
is-down
```

현재 사용자가 승인한 방향:

- 이전에 괜찮다고 한 포인터 모양을 유지한다.
- 크기만 줄인 상태다.
- 좌우 대칭형으로 바꾸지 않는다. 사용자가 별로라고 했다.
- 위쪽 동그라미 장식은 없다.
- 클릭하면 살짝 눌렸다가 나오는 느낌이 있다.

현재 기준 크기:

```text
.sambo-cursor: 31px x 38px
.sambo-cursor::before: 25px x 34px
```

주의:

- `pointer-events: none`을 유지한다.
- input, textarea 등 텍스트 입력 영역은 텍스트 커서가 살아 있어야 한다.
- 포인터를 다시 크게 만들지 않는다.
- 캐시 때문에 확인이 꼬일 수 있으니 테스트 URL에는 `?bust=...` 값을 붙인다.

## 10. 시뮬레이션/화면 변경 이력

현재 대시보드는 아래 흐름으로 수정되어 있다.

- CNC와 적재함이 시뮬레이션 안에 배치되어 있다.
- CNC와 적재함은 나란히 보이도록 조정되어 있다.
- 적재함 입구는 로봇팔 쪽을 바라보는 방향으로 조정되어 있다.
- CNC에는 LED가 표시된다.
- 적재함에는 LED를 표시하지 않는다.
- 부저가 울리면 대시보드에서 부저 상태를 보여준다.
- 수동 시험 화면 위쪽의 빈 패널들은 삭제되어 있다.
- ToF/집기 판단 패널은 단순화되어 있다.
- 집기 판단 문구는 `물체를 집음`, `물체를 안집음`처럼 직관적인 표현을 사용한다.

3D 관련 코드를 수정할 때는 아래 검색어를 먼저 사용한다.

```text
draw3D
drawCnc
rack
stock
CNC
```

## 11. 자주 바꿀 수 있는 값

아래 값은 향후 팀원이 바꿀 가능성이 높다.

| 항목 | 현재값 | 위치 |
| --- | --- | --- |
| CNC 가공 시간 | `20초` | `LED + 부저/rpi/led_buzzer_config.example.json`, Arduino/RPi 상태 송신 로직 |
| 적재함 최대 개수 | `2개` | `rack_capacity`, `rack_max` 관련 코드 |
| Nano echo fresh window | 약 `800ms` | `rpi_dashboard_ws_bridge.py` |
| LED 점멸 속도 | 상태별 Arduino 코드 | `nano_cnc_status_buzzer_node.ino` |
| 부저 횟수/길이 | 상태별 Arduino 코드 | `nano_cnc_status_buzzer_node.ino` |
| 커서 크기 | `31x38`, `25x34` | `sambo_robot_arm_dashboard.html` |

가공 시간은 반드시 상태 흐름 전체를 기준으로 바꿔야 한다.
예를 들어 `20초`를 `30초`로 바꾸면 빨간 LED 유지 시간, 남은 시간 표시, 가공 완료 후 초록 점멸 전환 타이밍이 같이 맞아야 한다.

## 12. 검증 명령

### 12.1 HTML 내부 script 파싱 확인

PowerShell에서 repo root 기준:

```powershell
node -e "const fs=require('fs');const p='6-18 작업 현황/choi/dashboard/sambo_robot_arm_dashboard.html';const html=fs.readFileSync(p,'utf8');const scripts=[...html.matchAll(/<script[^>]*>([\s\S]*?)<\/script>/gi)].map(m=>m[1]);for(const s of scripts)new Function(s);console.log('parsed scripts',scripts.length);"
```

### 12.2 Git 공백/충돌 확인

```powershell
git diff --check
```

### 12.3 로컬 대시보드 실행

```powershell
cd "6-18 작업 현황\choi\dashboard"
python -m http.server 8766
```

브라우저:

```text
http://127.0.0.1:8766/sambo_robot_arm_dashboard.html?nosplash=1&bust=handoff
```

### 12.4 RPi 송신 코드 dry-run

Windows에서는 `python`, Raspberry Pi에서는 보통 `python3`를 사용한다.

```powershell
python "LED + 부저/rpi/cnc_status_buzzer.py" --dry-run --state CNC_EMPTY --once
python "LED + 부저/rpi/cnc_status_buzzer.py" --dry-run --state CNC_OCCUPIED --remaining 15 --once
python "LED + 부저/rpi/cnc_status_buzzer.py" --dry-run --state CNC_DONE_WAIT --once
python "LED + 부저/rpi/cnc_status_buzzer.py" --dry-run --state RACK_LOADED --rack-count 1 --once
python "LED + 부저/rpi/cnc_status_buzzer.py" --dry-run --state RACK_FULL --rack-count 2 --rack-capacity 2 --once
```

### 12.5 Arduino IDE 확인

Arduino IDE에서:

- Board: `Arduino Nano`
- Processor: 보통 `ATmega328P`
- 업로드 실패 시 `ATmega328P (Old Bootloader)`도 시도
- Port: 연결된 Nano 포트

업로드 전 확인:

- MCP2515 크리스털이 8MHz인지 16MHz인지 확인
- 패시브/액티브 부저 설정 확인
- LED 극성 확인
- 저항 연결 확인
- GND 공통 확인

## 13. Claude에게 그대로 줄 수 있는 프롬프트

아래 프롬프트를 팀원이 Claude에게 그대로 붙여 넣으면 된다.

```text
너는 robot-arm-sorting 프로젝트의 lee 브랜치만 수정하는 코딩 에이전트다.

반드시 지킬 것:
- 라즈베리파이에 접속하거나 라즈베리파이 내부 파일을 수정하지 마라.
- GitHub lee 브랜치 코드만 수정한다.
- 먼저 아래 파일을 읽고 현재 맥락을 파악해라.
  1. 팀원_Claude_대시보드_LED부저_핸드오프.md
  2. LED + 부저/README.md
  3. LED + 부저/현장_적용_체크리스트.md
  4. LED + 부저/docs/wiring.md
  5. LED + 부저/docs/can_status_frame.md
  6. LED + 부저/arduino/nano_cnc_status_buzzer_node/nano_cnc_status_buzzer_node.ino
  7. LED + 부저/rpi/cnc_status_buzzer.py
  8. 6-18 작업 현황/choi/dashboard/sambo_robot_arm_dashboard.html
  9. 6-18 작업 현황/choi/dashboard/rpi_dashboard_ws_bridge.py

현재 구조:
- RPi가 로봇 상태와 CNC 상태를 판단한다.
- 별도 Arduino Nano + MCP2515가 LED와 부저 표시만 담당한다.
- LED + 부저용 Nano는 TOF용 Nano가 아니다.
- LED + 부저용 Nano는 로봇팔 서보 제어용 Nano도 아니다.
- RPi -> Nano CAN ID는 0x410이다.
- Nano -> RPi/dashboard echo CAN ID는 0x411이다.
- 대시보드는 0x411 actual output echo가 fresh하면 그 값을 우선 표시해야 한다.

LED 상태:
- CNC_EMPTY: 파란 LED ON
- CNC_OCCUPIED: 빨간 LED ON
- CNC_DONE_WAIT: 초록 LED 느린 점멸
- RACK_LOADED: 부저 짧게 3회, 이후 CNC_EMPTY 계열 표시로 복귀
- RACK_FULL: 적재함 2/2, 부저 반복 경고
- ERROR: LED 전체 빠른 점멸 + 부저 빠른 반복
- CAN_LOST: 통신 끊김 fallback 표시

대시보드 조건:
- CNC 상태 패널 크기는 임의로 키우거나 줄이지 마라.
- CNC에는 LED 3개를 표시한다.
- 적재함에는 LED를 넣지 마라.
- 부저가 울릴 때 대시보드에서도 부저 ON이 보여야 한다.
- Nano echo가 있으면 LED 깜빡임 속도와 부저 타이밍도 echo 기준으로 표시한다.

마우스 포인터 조건:
- 현재 승인된 커스텀 포인터는 이전 모양을 작게 줄인 형태다.
- 좌우 대칭형 포인터로 바꾸지 마라.
- 위쪽 동그라미 장식은 다시 넣지 마라.
- 클릭 시 눌렸다가 나오는 효과는 유지한다.
- 현재 기준 크기는 .sambo-cursor 31x38, ::before 25x34다.

수정 방식:
- 큰 리팩터링 금지.
- 관련 있는 부분만 좁게 수정해라.
- 대시보드 HTML에서는 ledBuzzer, renderCnc, getLedBuzzerStatus, initSamboCursor 키워드로 위치를 찾아라.
- 브리지에서는 CAN_ID_CNC_OUTPUT, CMD_CNC_OUTPUT, led_buzzer_output, output_mask, fresh 키워드로 위치를 찾아라.

검증:
- node로 HTML script 파싱을 확인해라.
- git diff --check를 실행해라.
- 가능하면 로컬 http.server로 대시보드를 열어 UI가 깨지지 않는지 확인해라.
- 변경 내용을 commit하고 origin의 lee 브랜치로 push해라.
```

## 14. 수정 금지/주의 사항

- Raspberry Pi 내부 파일 수정 금지.
- 기존 작업 브랜치 외 다른 브랜치에 임의 push 금지.
- 대시보드 전체 구조를 갈아엎는 리팩터링 금지.
- 적재함 LED를 다시 추가하지 말 것.
- Nano echo가 있는데도 대시보드가 자체 예측값만 표시하게 만들지 말 것.
- CNC 상태 패널 크기를 요청 없이 바꾸지 말 것.
- 커서를 다시 크게 만들거나 좌우 대칭형 새 모양으로 바꾸지 말 것.
- STS3215 서보 전원을 Arduino, Raspberry Pi, CANable에서 공급한다고 설명하지 말 것.
- E-Stop을 소프트웨어 정지만으로 설명하지 말 것.

## 15. 작업 완료 기준

팀원이 수정 작업을 끝냈다고 말하려면 아래가 확인되어야 한다.

- `lee` 브랜치 기준으로 변경됨
- 관련 파일만 변경됨
- `git diff --check` 통과
- HTML script 파싱 통과
- 대시보드에서 CNC LED 3개와 부저 상태가 보임
- 적재함 LED가 없음
- Nano echo가 있으면 dashboard 상태가 echo와 동기화됨
- 커서 수정 시 현재 승인된 작고 둥근 기존 포인터 느낌이 유지됨
- commit 후 `origin/lee`로 push 완료
