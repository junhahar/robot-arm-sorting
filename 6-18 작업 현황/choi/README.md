# choi — 5축(6모터) 로봇팔 STM32 + RPi 제어 코드

삼보모터스 5축 로봇팔(STS3215 ×6)의 **STM32 펌웨어 + 라즈베리파이 제어/역기구학** 코드 모음.

```
RPi5 (CANable) ─ CAN 125kbps/62.5% ─ STM32 (NUCLEO-F103RB) ─ UART ─ STS3215 ×6 (ID 1~6)
                                      + Arduino Nano (가상센서, 0x300)
```

---

## 폴더 구조
```
choi/
├── stm32/                       STM32 펌웨어 (PlatformIO)
│   ├── src/main.cpp
│   └── platformio.ini
├── rpi/                         라즈베리파이 파이썬 코드
│   ├── rpi_keyboard_control_v1.py    전부 180° 안전 영점 (기본)
│   ├── rpi_keyboard_control_v1_2.py  v1 + 부드러운 홈복귀(비례감속)
│   ├── rpi_keyboard_control_v1_3.py  v1_2 + 상태줄 '목표/실측' 둘 다 표시
│   ├── rpi_keyboard_control_v1_4.py  v1_3 + '9' 카메라 자세 축별 순차 이동
│   ├── rpi_keyboard_control_v1_5.py  v1_4 + 홈복귀 S-curve
│   ├── rpi_keyboard_control_v1_6.py  v1_5 + '8' 카메라 자세2, 자세이동 S-curve
│   ├── rpi_keyboard_control_v1_7.py  v1_6 + MG90 그리퍼(Nano, 0x310)
│   ├── rpi_keyboard_control_v1_8.py  v1_7 + MG90을 STM32 PWM(0x06)으로 이전
│   ├── rpi_keyboard_control_v1_9.py  v1_8 + IK↔실물 캘리브 수집(k/j/p/l)+선형보정 회귀
│   ├── rpi_keyboard_control_v1_10.py v1_9 + '7' 각도 입력 자세이동 + 관절 한계
│   ├── rpi_keyboard_control_v1_11.py v1_10 + '1~5' 축별 입력이동, M2·3 거울 자동 (★최신 권장)
│   ├── rpi_current_monitor.py        전류 실시간 모니터 (0x202)
│   ├── rpi_load_monitor.py           부하(load) 실시간 모니터 (0x201, 항상 읽힘)
│   └── arm_ik.py                     역기구학 (좌표 → 모터각)
├── vision/                       노트북(PC) 카메라·YOLO·IK
│   ├── detect_and_ik_calib.py        감지 → IK각 계산/표시 (C=출력)
│   ├── ik.py                         역기구학 (캘리 반영본, MOTOR_LIMITS 포함)
│   ├── list_cameras.py               USB 캠 인덱스 찾기
│   └── (best.pt·*.npz 는 대용량/세팅값이라 Git 제외, 별도 공유)
└── docs/
    ├── 로봇팔_작업정리.md            전체 작업 정리 (프로토콜·영점·IK·과열 등)
    └── CAN_통신_규약.md              CAN 통신 규약 (브링업·트러블슈팅)
```

---

## 키보드 제어 버전 비교

| 파일 | 홈복귀 | 추가 기능 |
|---|---|---|
| **v1** | 즉시 명령 | 거울쌍·gain·프리로드 (전부 180° 중위보정) |
| **v1_2** | 부드러운(비례감속) | |
| **v1_3** | 〃 | 상태줄 목표/실측 표시 |
| **v1_4** | 〃 | `9`=카메라 자세 순차 이동 |
| **v1_5** | **S-curve** | `9`=카메라 자세, STREAM 80 |
| **v1_6** | S-curve | `8`=카메라 자세2, 자세이동도 S-curve |
| **v1_7** | S-curve | MG90 그리퍼(Nano, 0x310) `y/h` |
| **v1_8** | S-curve | MG90을 STM32 PWM(0x06)으로 이전 |
| **v1_9** | S-curve | IK↔실물 캘리브 수집 `k/j/p/l` + 회귀(real=a·ik+b) |
| **v1_10** | S-curve | `7`=각도 6개 입력 자세이동 + 관절 한계(수동 clamp/입력 거부) |
| **v1_11** ★ | S-curve | `1~5`=축별 목표각 입력이동, M2·3 거울 자동, 개별 홈복귀 제거(`0`만) |

> **v1_11 권장** (최신). 캘리브는 calib/ik.py 한 곳에서만 — k/j/p는 중복이니 보정 끝났으면 사용 안 함(이중보정 주의).

### 키 매핑 (v1_11 기준)
- 이동(수동, 누르면 가속): M1 `q/a`  M2·M3 `w/s`(거울쌍)  M4 `e/d`  M5 `r/f`  M6 `t/g`
- 영점(현재 위치를 180°로, 모터 안 움직임): `z`=M1 `x`=M2·3 `c`=M4 `v`=M5 `b`=M6
- **축이동(S-curve, 목표각 입력)**: `1`=M1 `2`=M2·3 `3`=M4 `4`=M5 `5`=M6
- **자세입력(S-curve, 각 5개)**: `7` (M1 M2·3 M4 M5 M6 입력, M3은 360−M2 거울 자동)
- 카메라 자세(S-curve): `9` / `8` (M1→M4→M2·3→M5→M6 순차)
- 전체 홈복귀: `0` (전부 180°)
- 그리퍼(MG90/STM32 PWM): `y`=+ `h`=−
- 프리로드(M2·3 부하 분담): `]` `[`,  종료: `ESC`
- 캘리브(보정 끝났으면 사용 안 함): `k`=IK각입력 `j`=실측 저장 `p`=회귀 `l`=샘플수

> M2·M3 거울쌍은 명령각 **360 대칭**(M2+M3=360, 중점 180). 두 모터 모두 180으로 영점을 잡는 전제. 카메라 자세값도 M2+M3=360으로 맞춤.

---

## CAN 프로토콜 (요약)

| ID | 방향 | 용도 |
|---|---|---|
| `0x100` | RPi→STM | 모터 명령 |
| `0x200` | STM→RPi | ACK |
| `0x201` | STM→RPi | 텔레메트리 (각도·온도·부하) |
| `0x202` | STM→RPi | 전류 |
| `0x300` | Nano→RPi | 가상 센서 |

**0x100 명령:** `0x01`=위치(step), `0x03`=각도(angle×10), `0x04`=영점180°(중위보정), `0x05`=영점 임의각(오프셋 기록), `0x02`=토크, `0x06`=MG90 그리퍼 PWM(PA6, 각도 0~180), `0x7F`=ping.

물리: 125 kbps / sample-point 62.5% / PB8=RX, PB9=TX. 자세한 건 `docs/CAN_통신_규약.md`.

---

## 빌드 / 실행

### STM32 (PlatformIO, board: nucleo_f103rb)
```bash
pio run                 # 빌드
pio run -t upload       # ST-Link 업로드
```

### RPi (CAN 셋업 후 실행)
```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 125000 sample-point 0.625
sudo ip link set can0 up

python3 rpi/rpi_keyboard_control_v1_11.py  # 키보드 제어 (권장, 최신)
python3 rpi/rpi_load_monitor.py            # (다른 터미널) 부하 모니터
python3 rpi/arm_ik.py                      # 역기구학 self-test (FK 라운드트립)
```

---

## 주요 특징
- **키보드 제어**: 모터를 논리 축으로 묶음. M2·M3 거울쌍(어깨), M4 gain 2배, 누르면 속도 램프.
- **영점**: 180°(중위보정, 안전) / 임의각(오프셋 직접 기록). 모터 안 움직이고 EEPROM 영구.
- **거울쌍 부하 분담(프리로드)**: M2·M3가 부하를 나눠 들도록 `[` `]` 실시간 튜닝.
- **부하/전류 모니터**: 다른 터미널에서 읽기 전용 실행. 부하는 0x201로 항상 읽힘.
- **역기구학(arm_ik.py)**: 기하학적 2링크 해석해, 모터별 영점기준 변환, FK 0mm 검증.

자세한 설계·실측 캘리브레이션·트러블슈팅은 `docs/로봇팔_작업정리.md` 참고.
