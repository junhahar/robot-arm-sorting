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
│   ├── rpi_keyboard_control_v1_5.py  v1_4 + 홈복귀 S-curve (★최신 권장)
│   ├── rpi_keyboard_control_v2.py    360/0 임의각 영점 버전 (set_ref, 주의)
│   ├── rpi_current_monitor.py        전류 실시간 모니터 (0x202)
│   ├── rpi_load_monitor.py           부하(load) 실시간 모니터 (0x201, 항상 읽힘)
│   └── arm_ik.py                     역기구학 (좌표 → 모터각)
└── docs/
    ├── 로봇팔_작업정리.md            전체 작업 정리 (프로토콜·영점·IK·과열 등)
    └── CAN_통신_규약.md              CAN 통신 규약 (브링업·트러블슈팅)
```

---

## 키보드 제어 버전 비교

| 파일 | 영점 | 홈복귀 | 추가 기능 |
|---|---|---|---|
| **v1** | 전부 180° 중위보정(안전) | 즉시 명령 | 거울쌍·gain·프리로드 |
| **v1_2** | 〃 | 부드러운(비례감속) | |
| **v1_3** | 〃 | 〃 | 상태줄 목표/실측 표시 |
| **v1_4** | 〃 | 〃 | `9`=카메라 자세 순차 이동 |
| **v1_5** ★ | 〃 | **S-curve** | `9`=카메라 자세, STREAM 80 |
| v2 | M2→360·M3→0·M4→360 (set_ref) | 즉시 | 360/0 풀가동 (clamp 튐 주의) |

> **v1_5 권장.** v2의 360/0 EEPROM 영점(set_ref)은 오프셋 ±180° 한계로 끝이 아닌 자세에서 튈 수 있어 주의.

### 공통 키 매핑
- 이동: M1 `q/a`  M2·M3 `w/s`(거울쌍)  M4 `e/d`  M5 `r/f`  M6 `t/g`
- 영점: `z` `x` `c` `v` `b` (현재 위치를 기준각으로, 모터 안 움직임)
- 홈복귀: `1`~`6` 그 모터 / `0` 전체 → 180°
- 카메라 자세(v1_4/v1_5): `9` (M1→M4→M2·3→M5→M6 순차)
- 프리로드(M2·3 부하 분담): `]` `[`,  종료: `ESC`

---

## CAN 프로토콜 (요약)

| ID | 방향 | 용도 |
|---|---|---|
| `0x100` | RPi→STM | 모터 명령 |
| `0x200` | STM→RPi | ACK |
| `0x201` | STM→RPi | 텔레메트리 (각도·온도·부하) |
| `0x202` | STM→RPi | 전류 |
| `0x300` | Nano→RPi | 가상 센서 |

**0x100 명령:** `0x01`=위치(step), `0x03`=각도(angle×10), `0x04`=영점180°(중위보정), `0x05`=영점 임의각(오프셋 기록), `0x02`=토크, `0x7F`=ping.

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

python3 rpi/rpi_keyboard_control_v1_5.py   # 키보드 제어 (권장)
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
